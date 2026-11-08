import math
import random
import pytest
from conftest import load_module

mod = load_module("adaptive_pid", "ha-idrac-controller-dev/app/adaptive_pid.py")
Ctl = mod.AdaptiveFanController


def plant_step(state, speed, power, inlet, r0, r1, tau, dt=30.0):
    resistance = r0 + r1 * (max(speed, 5) / 100.0) ** -0.8
    steady = inlet + power * resistance
    if state["t"] is None:
        state["t"] = steady
    state["t"] += (steady - state["t"]) * (1 - math.exp(-dt / tau))
    return state["t"]


def simulate(ctl, power_fn, r0=0.07, r1=0.03, tau=60.0, inlet=22.0, steps=480, seed=1):
    rnd = random.Random(seed)
    state = {"t": None}
    speed = 25
    log = []
    for k in range(steps):
        t = k * 30.0
        power = power_fn(t)
        temp = plant_step(state, speed, power, inlet, r0, r1, tau)
        speed = ctl.update(round(temp + rnd.gauss(0, 0.1)), inlet, power, now=t)
        log.append((t, temp, speed))
    return log


def test_learns_true_thermal_resistance():
    ctl = Ctl(65)
    simulate(ctl, lambda t: 182 if t < 1000 else 350 if t < 3600 else 220)
    assert ctl.theta[0] == pytest.approx(0.07, abs=0.02)
    assert ctl.theta[1] == pytest.approx(0.03, abs=0.01)


def test_holds_target_under_steady_load():
    ctl = Ctl(65)
    log = simulate(ctl, lambda t: 350, steps=240)
    tail = [temp for _, temp, _ in log[-40:]]
    assert max(abs(temp - 65) for temp in tail) < 2.5


def test_idle_stays_at_minimum_speed():
    ctl = Ctl(65, min_speed=15)
    log = simulate(ctl, lambda t: 100, steps=120)
    assert all(speed == 15 for _, _, speed in log[-20:])


def test_speed_stays_within_limits_and_is_int():
    ctl = Ctl(50, min_speed=20, max_speed=80)
    log = simulate(ctl, lambda t: 500, steps=120)
    assert all(20 <= speed <= 80 and isinstance(speed, int) for _, _, speed in log)


def test_emergency_jumps_to_max():
    ctl = Ctl(50, max_speed=90)
    assert ctl.update(60, 20, 200, now=0) == 90


def test_slew_down_is_limited():
    ctl = Ctl(65)
    ctl.speed = 80.0
    ctl.last_time = 0.0
    ctl.temp_filtered = 40.0
    speed = ctl.update(40, 20, 100, now=30.0)
    assert 78 <= speed <= 80


def test_works_without_inlet_or_power():
    ctl = Ctl(55)
    speeds = [ctl.update(50 + (k % 3), None, None, now=k * 30.0) for k in range(50)]
    assert all(15 <= s <= 95 for s in speeds)


def test_state_roundtrip_and_bad_state_ignored():
    ctl = Ctl(65)
    simulate(ctl, lambda t: 350, steps=200)
    other = Ctl(65)
    other.load_state(ctl.get_state())
    assert other.theta == pytest.approx(ctl.theta)
    clean = Ctl(65)
    clean.load_state({"integral": 12.0})
    clean.load_state({"theta": [9, 9], "cov": [[1, 0], [0, 1]]})
    assert clean.theta == [0.09, 0.02]

