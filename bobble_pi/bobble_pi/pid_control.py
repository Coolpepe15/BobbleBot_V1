"""Cascade PID controller, ported from the bobble_controllers PidControl
C++ class (PidControl.cpp). Behavior (output filtering, setpoint ramping,
anti-windup, external derivative source) matches the original 1:1."""

from bobble_pi.difference_filter import DerivativeFilter, IntegralFilter


def _clamp(value, lo, hi):
    if value > hi:
        return hi
    if value < lo:
        return lo
    return value


def _bounded(value, lo, hi):
    return lo < value < hi


class PidControl:
    def __init__(self, p=0.0, i=0.0, d=0.0, fc=0.0, sample_time=0.01, f=0.0):
        self.P = p
        self.I = i
        self.D = d
        self.Fc = fc
        self.F = f
        self.Ts = sample_time

        self.derivative_filter = DerivativeFilter(self.Ts, self.Fc, self.D)
        self.integral_filter = IntegralFilter(self.Ts, self.I)

        self.max_i_output = 0.0
        self.max_error = 0.0
        self.error_sum = 0.0
        self.ext_deriv_error = None
        self.use_external_deriv_error = False

        self.max_output = 0.0
        self.min_output = 0.0
        self.setpoint = 0.0
        self.last_actual = 0.0
        self.first_run = True
        self.reversed = False
        self.output_ramp_rate = 0.0
        self.last_output = 0.0
        self.output_filter = 0.0
        self.setpoint_range = 0.0

    def set_pid(self, p, i, d, fc, sample_time, f=0.0):
        self.P, self.I, self.D, self.F = p, i, d, f
        self.Fc, self.Ts = fc, sample_time
        self._check_signs()
        self.derivative_filter.reset_filter_parameters(self.Ts, self.Fc, self.D)
        self.integral_filter.reset_filter_parameters(self.Ts, self.I)

    def set_max_i_output(self, maximum):
        self.max_i_output = maximum
        if self.I != 0:
            self.max_error = self.max_i_output / self.I

    def set_output_limits(self, minimum, maximum=None):
        if maximum is None:
            minimum, maximum = -minimum, minimum
        if maximum < minimum:
            return
        self.max_output = maximum
        self.min_output = minimum
        if self.max_i_output == 0 or self.max_i_output > (maximum - minimum):
            self.set_max_i_output(maximum - minimum)

    def set_direction(self, reversed_):
        self.reversed = reversed_
        self._check_signs()

    def set_external_derivative_error(self, getter):
        """`getter` is a zero-argument callable returning the current
        derivative error (e.g. a lambda reading the latest gyro rate)."""
        self.ext_deriv_error = getter
        self.use_external_deriv_error = getter is not None

    def set_setpoint_range(self, r):
        self.setpoint_range = r

    def set_output_ramp_rate(self, rate):
        self.output_ramp_rate = rate

    def set_output_filter(self, strength):
        if strength == 0 or _bounded(strength, 0, 1):
            self.output_filter = strength

    def reset(self):
        self.first_run = True
        self.error_sum = 0.0

    def get_output(self, desired, actual):
        self.setpoint = desired
        if self.setpoint_range != 0:
            self.setpoint = _clamp(
                self.setpoint, actual - self.setpoint_range, actual + self.setpoint_range
            )

        error = self.setpoint - actual
        f_output = self.F * self.setpoint
        p_output = self.P * error

        if self.first_run:
            self.last_actual = actual
            self.last_output = p_output + f_output
            self.first_run = False

        if self.use_external_deriv_error:
            d_output = -self.D * self.ext_deriv_error()
        else:
            d_output = self.derivative_filter.filter(error)

        i_output = self.integral_filter.filter(error)
        if self.max_i_output != 0:
            i_output = _clamp(i_output, -self.max_i_output, self.max_i_output)

        output = f_output + p_output + i_output + d_output

        if self.min_output != self.max_output and not _bounded(
            output, self.min_output, self.max_output
        ):
            self.error_sum = error
        elif self.output_ramp_rate != 0 and not _bounded(
            output,
            self.last_output - self.output_ramp_rate,
            self.last_output + self.output_ramp_rate,
        ):
            self.error_sum = error
        elif self.max_i_output != 0:
            self.error_sum = _clamp(self.error_sum + error, -self.max_error, self.max_error)
        else:
            self.error_sum += error

        if self.output_ramp_rate != 0:
            output = _clamp(
                output,
                self.last_output - self.output_ramp_rate,
                self.last_output + self.output_ramp_rate,
            )
        if self.min_output != self.max_output:
            output = _clamp(output, self.min_output, self.max_output)
        if self.output_filter != 0.0:
            output = self.last_output * self.output_filter + output * (1.0 - self.output_filter)

        self.last_output = output
        return output

    def _check_signs(self):
        if self.reversed:
            if self.P > 0:
                self.P *= -1
            if self.I > 0:
                self.I *= -1
            if self.D > 0:
                self.D *= -1
            if self.F > 0:
                self.F *= -1
        else:
            if self.P < 0:
                self.P *= -1
            if self.I < 0:
                self.I *= -1
            if self.D < 0:
                self.D *= -1
            if self.F < 0:
                self.F *= -1
