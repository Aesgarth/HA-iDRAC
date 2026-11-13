import statistics


class FanMonitor:
    """Detects stopped, missing or slow fans from RPM readings alone.

    Each fan's speed is compared with the median of all fans, relative to its own
    learned baseline ratio, so fans of different models are not flagged.
    """

    def __init__(self, poll_seconds=30.0, confirm_seconds=90.0, settle_seconds=60.0,
                 low_ratio=0.65, recover_polls=5):
        self.poll_seconds = float(poll_seconds)
        self.confirm_polls = max(3, int(round(confirm_seconds / max(poll_seconds, 1.0))))
        self.settle_seconds = float(settle_seconds)
        self.low_ratio = float(low_ratio)
        self.recover_polls = int(recover_polls)
        self.baseline = {}
        self.strikes = {}
        self.reasons = {}
        self.clear_count = 0
        self.faulty = {}
        self.prev_median = None
        self.settle_until = -1.0
        self.known_fans = set()

    def set_poll_seconds(self, poll_seconds):
        self.poll_seconds = float(poll_seconds)

    def update(self, now, readings):
        """readings: {fan_name: rpm or None}. Returns {'faulty': {...}, 'ok': bool}."""
        for name in readings:
            self.known_fans.add(name)
        current = {name: readings.get(name) for name in self.known_fans}
        spinning = [v for v in current.values() if v is not None and v > 0]
        median = statistics.median(spinning) if spinning else 0.0

        if self.prev_median and median and abs(median - self.prev_median) / self.prev_median > 0.10:
            self.settle_until = now + self.settle_seconds
        self.prev_median = median if median else self.prev_median
        settling = now < self.settle_until

        anomalies = {}
        for name, rpm in current.items():
            if rpm is None:
                anomalies[name] = "missing"
            elif rpm <= 0 and median > 0:
                anomalies[name] = "stopped"
            elif (not settling and len(spinning) >= 3 and median > 0 and rpm > 0
                  and rpm < self.low_ratio * self.baseline.get(name, 1.0) * median):
                anomalies[name] = "slow"

        for name in current:
            if name in anomalies:
                self.strikes[name] = self.strikes.get(name, 0) + 1
                self.reasons[name] = anomalies[name]
            else:
                self.strikes[name] = 0
        if not anomalies and not settling and len(spinning) >= 3 and median > 0:
            for name, rpm in current.items():
                if rpm and rpm > 0:
                    ratio = rpm / median
                    old = self.baseline.get(name)
                    self.baseline[name] = ratio if old is None else old + 0.02 * (ratio - old)

        confirmed = {name: self.reasons[name] for name, n in self.strikes.items()
                     if n >= self.confirm_polls}
        if confirmed:
            self.clear_count = 0
            self.faulty = confirmed
        elif self.faulty:
            self.clear_count += 1
            if self.clear_count >= self.recover_polls:
                self.faulty = {}
                self.clear_count = 0
        return {"faulty": dict(self.faulty), "ok": not self.faulty}

    def describe(self):
        if not self.faulty:
            return "OK"
        return "; ".join(f"{name}: {reason}" for name, reason in sorted(self.faulty.items()))

