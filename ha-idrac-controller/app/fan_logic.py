# HA-iDRAC/ha-idrac-controller-dev/app/fan_logic.py
"""Small, testable fan-control helpers.

Kept separate from main.py so the threshold/hysteresis behaviour can be unit
tested without importing Flask, paho-mqtt or an IPMI connection.
"""

MIN_FAN_PERCENT = 5
DEFAULT_HYSTERESIS_C = 2.0


def clamp_speed(speed, minimum=MIN_FAN_PERCENT, maximum=100):
    try:
        speed = int(speed)
    except (TypeError, ValueError):
        speed = minimum
    return max(minimum, min(maximum, speed))


class SimpleFanState:
    """3-tier threshold control with hysteresis.

    - manual -> Dell auto at the critical threshold;
    - auto -> manual only once the temperature has fallen by `hysteresis_c`
      below the critical threshold, so the fans do not flap in and out of
      iDRAC control every cycle;
    - base -> high at `low_temp`, high -> base only below
      `low_temp - hysteresis_c`.
    """

    def __init__(self, hysteresis_c=DEFAULT_HYSTERESIS_C):
        self.hysteresis_c = hysteresis_c
        self.in_auto = False
        self.bucket = None  # "base" | "high"

    def should_use_auto(self, temp_c, critical_temp_c):
        if self.in_auto:
            if temp_c <= critical_temp_c - self.hysteresis_c:
                self.in_auto = False
        elif temp_c >= critical_temp_c:
            self.in_auto = True
        return self.in_auto

    def simple_speed(self, temp_c, low_temp_c, base_speed, high_speed):
        if self.bucket == "high":
            if temp_c <= low_temp_c - self.hysteresis_c:
                self.bucket = "base"
        else:
            if temp_c >= low_temp_c:
                self.bucket = "high"
        if self.bucket == "high":
            return clamp_speed(high_speed)
        return clamp_speed(base_speed)
