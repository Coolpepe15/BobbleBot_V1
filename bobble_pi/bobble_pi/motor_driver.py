"""L298N dual H-bridge motor driver, plus optional JGY-370 hall encoder
odometry. Built on gpiozero, which on Ubuntu 24.04 (Raspberry Pi 4) should
use the 'lgpio' pin factory:

    pip install gpiozero lgpio

L298N wiring per motor: IN1/IN2 pick direction, ENA/ENB (here: pwm_pin) sets
speed via PWM. Power the L298N's motor supply (the LiPo, through its screw
terminals) separately from the Pi; tie the L298N GND to the Pi GND. Do not
power the L298N logic side from the Pi 5V *and* enable its onboard 5V
regulator at the same time - see the README.
"""

import math
import time

from gpiozero import DigitalInputDevice, DigitalOutputDevice, PWMOutputDevice


class L298nMotor:
    def __init__(self, in1_pin, in2_pin, pwm_pin, pwm_frequency=1000, invert=False):
        self._in1 = DigitalOutputDevice(in1_pin)
        self._in2 = DigitalOutputDevice(in2_pin)
        self._pwm = PWMOutputDevice(pwm_pin, frequency=pwm_frequency)
        self._invert = invert

    def set_effort(self, effort):
        """effort in [-1.0, 1.0]. Positive drives the motor in the
        direction wired to IN1=HIGH/IN2=LOW."""
        if self._invert:
            effort = -effort
        effort = max(-1.0, min(1.0, effort))
        if effort >= 0.0:
            self._in1.on()
            self._in2.off()
        else:
            self._in1.off()
            self._in2.on()
        self._pwm.value = abs(effort)

    def stop(self):
        self._in1.off()
        self._in2.off()
        self._pwm.value = 0.0

    def close(self):
        self.stop()
        self._in1.close()
        self._in2.close()
        self._pwm.close()


class WheelEncoder:
    """Reads pulses from a JGY-370 hall encoder wheel.

    Wire the encoder's signal output(s) through a 3.3V-safe path to a Pi
    GPIO - most JGY-370 encoder boards run their Hall sensors at 3.3-5V;
    if yours outputs 5V logic, use a level shifter or a simple resistor
    divider before the GPIO pin. VCC goes to the Pi's 3V3 pin if the
    encoder board accepts it, otherwise level-shift.

    If your motor only has a single encoder output wire, pass
    channel_b_pin=None: pulses are then counted as unsigned magnitude and
    the caller must supply the direction (e.g. from the last commanded
    motor effort) via read_velocity_rad_s(assumed_direction=...). If you
    have both encoder channels wired, direction is read directly from the
    encoder and assumed_direction is ignored.
    """

    def __init__(self, channel_a_pin, pulses_per_revolution, channel_b_pin=None, invert=False):
        self._pulses_per_rev = float(pulses_per_revolution)
        self._invert = invert
        self._count = 0
        self._last_count = 0
        self._last_time = time.monotonic()
        self._chan_a = DigitalInputDevice(channel_a_pin, pull_up=True)
        self._chan_b = (
            DigitalInputDevice(channel_b_pin, pull_up=True) if channel_b_pin is not None else None
        )
        self._chan_a.when_activated = self._on_pulse

    def _on_pulse(self):
        if self._chan_b is not None:
            self._count += 1 if self._chan_b.is_active else -1
        else:
            self._count += 1

    def read_velocity_rad_s(self, assumed_direction=1.0):
        now = time.monotonic()
        dt = now - self._last_time
        count = self._count
        pulses = count - self._last_count
        self._last_count = count
        self._last_time = now
        if dt <= 0.0:
            return 0.0
        if self._chan_b is None:
            pulses = abs(pulses) * (1.0 if assumed_direction >= 0.0 else -1.0)
        revolutions = pulses / self._pulses_per_rev
        velocity = (revolutions * 2.0 * math.pi) / dt
        return -velocity if self._invert else velocity

    def close(self):
        self._chan_a.close()
        if self._chan_b is not None:
            self._chan_b.close()
