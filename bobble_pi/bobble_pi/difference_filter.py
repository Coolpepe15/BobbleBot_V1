"""Digital IIR filters built from linear difference equations.

This is a direct Python port of the ``Filter`` / ``LowPassFilter`` /
``DerivativeFilter`` / ``IntegralFilter`` C++ classes from the original
bobble_controllers ROS 1 package (see Filter.cpp, LowPassFilter.cpp and
PIDFilters.cpp). The continuous-time transfer functions are converted to
discrete time with the bilinear transform, exactly as in the original.
"""

import math

MAX_FILTER_SIZE = 20


class DifferenceEquationFilter:
    """Generic filter of the form:

    a[0]*y[n] = b[0]*x[n] + b[1]*x[n-1] + ... - a[1]*y[n-1] - a[2]*y[n-2] - ...
    """

    def __init__(self):
        self._input_buffer = [0.0] * MAX_FILTER_SIZE
        self._output_buffer = [0.0] * MAX_FILTER_SIZE
        self._num_in_weights = 0
        self._num_out_weights = 0
        self._input_weights = [0.0] * MAX_FILTER_SIZE
        self._output_weights = [0.0] * MAX_FILTER_SIZE

    def _set_weights(self, input_weights, output_weights):
        self._input_buffer = [0.0] * MAX_FILTER_SIZE
        self._output_buffer = [0.0] * MAX_FILTER_SIZE
        self._input_weights = [0.0] * MAX_FILTER_SIZE
        self._output_weights = [0.0] * MAX_FILTER_SIZE
        self._num_in_weights = len(input_weights)
        self._num_out_weights = len(output_weights)
        for i, w in enumerate(input_weights):
            self._input_weights[i] = w
        for i, w in enumerate(output_weights):
            self._output_weights[i] = w

    def filter(self, input_value):
        for i in range(self._num_in_weights, 0, -1):
            self._input_buffer[i] = self._input_buffer[i - 1]
        self._input_buffer[0] = input_value

        for i in range(self._num_out_weights, 0, -1):
            self._output_buffer[i] = self._output_buffer[i - 1]

        input_contribution = 0.0
        for i in range(self._num_in_weights):
            input_contribution += self._input_weights[i] * self._input_buffer[i]

        output_contribution = 0.0
        for i in range(1, self._num_out_weights):
            output_contribution += self._output_weights[i] * self._output_buffer[i]

        self._output_buffer[0] = (1.0 / self._output_weights[0]) * (
            input_contribution - output_contribution
        )
        return self._output_buffer[0]


class LowPassFilter(DifferenceEquationFilter):
    """Second order low pass filter: 1 / (s^2 + 2*zeta*wc*s + wc^2).

    Note: unlike the original C++ class, a cutoff frequency <= 0 is treated
    here as "no filtering" (pass-through) instead of degenerating to an
    always-zero output. The original only ever hit fc == 0 as a broken
    fallback when a required ROS parameter was missing.
    """

    def __init__(self, ts=0.002, fc=50.0, zeta=1.0):
        super().__init__()
        self.reset_filter_parameters(ts, fc, zeta)

    def reset_filter_parameters(self, ts, fc, zeta):
        if fc <= 0.0:
            self._set_weights([1.0], [1.0])
            return
        wc = 2.0 * math.pi * fc
        input_weights = [ts * ts * wc * wc, 2 * ts * ts * wc * wc, ts * ts * wc * wc]
        output_weights = [
            ts * ts * wc * wc + 4 * ts * wc * zeta + 4,
            2 * ts * ts * wc * wc - 8,
            ts * ts * wc * wc - 4 * ts * wc * zeta + 4,
        ]
        self._set_weights(input_weights, output_weights)


class DerivativeFilter(DifferenceEquationFilter):
    """Filtered derivative: s * Kd * wc / (s + wc)."""

    def __init__(self, ts=0.002, fc=50.0, kd=1.0):
        super().__init__()
        self.reset_filter_parameters(ts, fc, kd)

    def reset_filter_parameters(self, ts, fc, kd):
        wc = 2.0 * math.pi * fc
        input_weights = [2.0 * kd * wc, -2.0 * kd * wc]
        output_weights = [2.0 + wc * ts, wc * ts - 2.0]
        self._set_weights(input_weights, output_weights)


class IntegralFilter(DifferenceEquationFilter):
    """Discrete integrator: Ki / s."""

    def __init__(self, ts=0.002, ki=1.0):
        super().__init__()
        self.reset_filter_parameters(ts, ki)

    def reset_filter_parameters(self, ts, ki):
        input_weights = [ts * ki, ts * ki]
        output_weights = [2.0, -2.0]
        self._set_weights(input_weights, output_weights)
