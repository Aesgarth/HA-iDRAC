import json
import os
import subprocess
import sys
import tempfile
import time

HEARTBEAT_FILE = os.environ.get("IDRAC_HEARTBEAT_FILE", "/tmp/idrac_heartbeat.json")
STALE_SECONDS = 180
CHECK_SECONDS = 30
AUTO_ARGS = ["raw", "0x30", "0x30", "0x01", "0x01"]


def write_heartbeat(servers, path=HEARTBEAT_FILE, now=None):
    """servers: list of dicts {id, host, user, password} currently under manual fan control."""
    data = {"time": now if now is not None else time.time(), "servers": servers}
    fd, tmp = tempfile.mkstemp(dir=os.path.dirname(path) or ".")
    with os.fdopen(fd, "w") as f:
        json.dump(data, f)
    os.chmod(tmp, 0o600)
    os.replace(tmp, path)


def stale_servers(path=HEARTBEAT_FILE, now=None, stale_seconds=STALE_SECONDS):
    now = now if now is not None else time.time()
    try:
        with open(path) as f:
            data = json.load(f)
    except (OSError, ValueError):
        return []
    if now - data.get("time", 0) < stale_seconds:
        return []
    return data.get("servers", [])


def hand_back(server, runner=subprocess.run):
    cmd = ["ipmitool", "-I", "lanplus", "-H", server["host"], "-U", server["user"], "-E"] + AUTO_ARGS
    env = dict(os.environ, IPMI_PASSWORD=server["password"])
    return runner(cmd, env=env, capture_output=True, timeout=20).returncode == 0


def clear(path=HEARTBEAT_FILE):
    write_heartbeat([], path)


def run_forever(path=HEARTBEAT_FILE, sleep=time.sleep, runner=subprocess.run, once=False):
    while True:
        stale = stale_servers(path)
        if stale:
            done = [s for s in stale if hand_back(s, runner)]
            for s in done:
                print(f"watchdog: handed {s['host']} back to Dell auto", flush=True)
            if len(done) == len(stale):
                clear(path)
        if once:
            return
        sleep(CHECK_SECONDS)


if __name__ == "__main__":
    run_forever()
    sys.exit(0)

