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


def _int_or(value, default):
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def validate_fan_config(low_temp_c, critical_temp_c, base_speed, high_speed):
    """Return safe, self-consistent fan settings plus warnings to log.

    Nonsensical settings (critical <= low, high < base) are corrected rather
    than silently producing a fan profile that never uses one of its tiers.
    """
    warnings = []
    low = _int_or(low_temp_c, 45)
    critical = _int_or(critical_temp_c, 65)
    base = clamp_speed(base_speed)
    high = clamp_speed(high_speed)

    if critical <= low:
        critical = low + 1
        warnings.append(
            f"critical_temp_threshold ({critical_temp_c}) must be above "
            f"low_temp_threshold ({low_temp_c}); using {critical} C instead."
        )
    if high < base:
        high = base
        warnings.append(
            f"high_temp_fan_speed_percent ({high_speed}) must be at least "
            f"base_fan_speed_percent ({base_speed}); using {high}% instead."
        )

    return {
        "low": low,
        "critical": critical,
        "base": base,
        "high": high,
        "warnings": warnings,
    }
