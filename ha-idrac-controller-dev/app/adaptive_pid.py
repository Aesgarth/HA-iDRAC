import math
import time


class AdaptiveFanController:
    """Self-tuning fan controller.

    Learns the server's thermal resistance R(u) = a + b * u**-gamma (K per watt, u = fan fraction)
    from steady-state samples, uses its inverse as feedforward for the target temperature, and
    trims the remainder with a small leaky integral plus a proportional term scaled by the
    learned sensitivity.
    """

    GAMMA = 0.8
    DEFAULT_POWER_W = 200.0
    DEFAULT_INLET_C = 25.0

    def __init__(self, setpoint, min_speed=15, max_speed=95, deadband=0.75,
                 ti_seconds=300.0, slew_down_per_min=4.0, emergency_margin=8.0):
        self.setpoint = float(setpoint)
        self.min_speed = float(min_speed)
        self.max_speed = float(max_speed)
        self.deadband = float(deadband)
        self.ti = float(ti_seconds)
        self.slew_down_per_s = slew_down_per_min / 60.0
        self.emergency_margin = float(emergency_margin)
        self.theta = [0.09, 0.02]
        self.cov = [[1.0, 0.0], [0.0, 1.0]]
        self.forgetting = 0.995
        self.trim = 0.0
        self.trim_limit = 6.0
        self.trim_leak_per_poll = 0.97
        self.temp_filtered = None
        self.prev_filtered = None
        self.prev_speed = None
        self.speed = float(min_speed)
        self.last_time = None
        self.stable_seconds = 0.0
        self.samples = 0

    def set_gains(self, setpoint=None, min_speed=None, max_speed=None, deadband=None, ti_seconds=None):
        if setpoint is not None:
            self.setpoint = float(setpoint)
        if min_speed is not None:
            self.min_speed = float(min_speed)
        if max_speed is not None:
            self.max_speed = float(max_speed)
        if deadband is not None:
            self.deadband = float(deadband)
        if ti_seconds is not None:
            self.ti = float(ti_seconds)

    def resistance(self, speed):
        return self.theta[0] + self.theta[1] * (max(speed, 5.0) / 100.0) ** -self.GAMMA

    def _learn(self, temp, inlet, power, speed, dt):
        steady = (self.prev_filtered is not None and abs(temp - self.prev_filtered) < 0.4
                  and self.prev_speed == speed)
        self.stable_seconds = self.stable_seconds + dt if steady else 0.0
        self.prev_filtered = temp
        self.prev_speed = speed
        if self.stable_seconds < 180.0 or power < 20.0:
            return
        y = (temp - inlet) / power
        x = [1.0, (speed / 100.0) ** -self.GAMMA]
        px = [self.cov[0][0] * x[0] + self.cov[0][1] * x[1],
              self.cov[1][0] * x[0] + self.cov[1][1] * x[1]]
        den = self.forgetting + x[0] * px[0] + x[1] * px[1]
        gain = [px[0] / den, px[1] / den]
        err = y - (self.theta[0] * x[0] + self.theta[1] * x[1])
        self.theta[0] = min(0.5, max(0.0, self.theta[0] + gain[0] * err))
        self.theta[1] = min(0.3, max(0.002, self.theta[1] + gain[1] * err))
        for i in range(2):
            for j in range(2):
                self.cov[i][j] = (self.cov[i][j] - gain[i] * px[j]) / self.forgetting
        trace = self.cov[0][0] + self.cov[1][1]
        if trace > 10.0:
            scale = 10.0 / trace
            self.cov = [[c * scale for c in row] for row in self.cov]
        self.samples += 1

    def _feedforward(self, inlet, power):
        target_r = (self.setpoint - inlet) / max(power, 20.0)
        a, b = self.theta
        if target_r <= a + b * (self.max_speed / 100.0) ** -self.GAMMA:
            return self.max_speed
        speed = (b / (target_r - a)) ** (1.0 / self.GAMMA) * 100.0
        return min(self.max_speed, max(self.min_speed, speed))

    def update(self, current_temp, inlet_temp=None, power_w=None, now=None):
        now = time.time() if now is None else now
        dt = 30.0 if self.last_time is None else max(1.0, now - self.last_time)
        self.last_time = now
        inlet = self.DEFAULT_INLET_C if inlet_temp is None else float(inlet_temp)
        power = self.DEFAULT_POWER_W if power_w is None else float(power_w)

        if self.temp_filtered is None:
            self.temp_filtered = float(current_temp)
        else:
            self.temp_filtered += 0.5 * (float(current_temp) - self.temp_filtered)
        temp = self.temp_filtered

        self._learn(temp, inlet, power, round(self.speed), dt)

        if current_temp >= self.setpoint + self.emergency_margin:
            self.speed = self.max_speed
            return int(round(self.speed))

        feed = self._feedforward(inlet, power)
        error = temp - self.setpoint
        self.trim *= self.trim_leak_per_poll ** (dt / 30.0)
        at_floor = self.speed <= self.min_speed and error < 0
        at_ceiling = self.speed >= self.max_speed and error > 0
        outside = abs(error) > self.deadband
        excess = error - math.copysign(self.deadband, error) if outside else 0.0
        if outside and not at_floor and not at_ceiling:
            self.trim += excess * dt / self.ti
        self.trim = max(-self.trim_limit, min(self.trim_limit, self.trim))

        slope = power * self.theta[1] * self.GAMMA * (max(self.speed, 5.0) / 100.0) ** (-self.GAMMA - 1.0) / 100.0
        kp = 0.5 / max(slope, 0.05)
        target = feed + self.trim + 0.5 * kp * excess
        target = max(self.min_speed, min(self.max_speed, target))
        if target < self.speed:
            target = max(target, self.speed - self.slew_down_per_s * dt)
        self.speed = target
        return int(round(self.speed))

    def get_state(self):
        return {"theta": list(self.theta), "cov": [list(r) for r in self.cov], "trim": self.trim,
                "speed": self.speed, "samples": self.samples}

    def load_state(self, state):
        if not isinstance(state, dict) or "theta" not in state:
            return
        try:
            theta = [float(state["theta"][0]), float(state["theta"][1])]
            cov = [[float(v) for v in row] for row in state["cov"]]
            if not (0.0 <= theta[0] <= 0.5 and 0.002 <= theta[1] <= 0.3):
                return
            self.theta, self.cov = theta, cov
            self.trim = max(-self.trim_limit, min(self.trim_limit, float(state.get("trim", 0.0))))
            self.speed = max(self.min_speed, min(self.max_speed, float(state.get("speed", self.min_speed))))
            self.samples = int(state.get("samples", 0))
        except (KeyError, TypeError, ValueError, IndexError):
            return

