# HA-iDRAC/ha-idrac-controller-dev/app/web_server.py
from flask import Flask, render_template, request, redirect, flash
from markupsafe import Markup
import os
import json
import re
import subprocess
import tempfile
import threading

try:
    from . import disk_health
except (ImportError, ValueError):
    try:
        from app import disk_health
    except (ImportError, ValueError):
        import importlib.util
        from pathlib import Path
        _dh_path = Path(__file__).parent / "disk_health.py"
        if _dh_path.exists():
            _spec = importlib.util.spec_from_file_location("disk_health", _dh_path)
            disk_health = importlib.util.module_from_spec(_spec)
            _spec.loader.exec_module(disk_health)
        else:
            disk_health = None

app = Flask(__name__)
app.secret_key = os.urandom(24)

# --- Global paths and locks ---
STATUS_FILE = None
SERVERS_CONFIG_FILE = "/data/servers_config.json"
status_lock = None
config_lock = threading.Lock()
global_config = {}
health_check = None
server_manager = None


# --- Helper functions ---
def _to_int(val, default):
    if val is None or val == "":
        return default
    try:
        return int(val)
    except (TypeError, ValueError):
        return default


def _to_float(val, default):
    if val is None or val == "":
        return default
    try:
        return float(val)
    except (TypeError, ValueError):
        return default


def _int_field(form, name, default):
    return _to_int(form.get(name), default)


def _float_field(form, name, default):
    return _to_float(form.get(name), default)


def _parse_poll_interval(val):
    if val is None or (isinstance(val, str) and val.strip() == ""):
        return None
    try:
        return max(5, int(val))
    except (TypeError, ValueError):
        return None


def _parse_on_fan_fault(val):
    if val in ("dell_auto", "raise_fans", "alert_only"):
        return val
    return "dell_auto"


def _parse_min_safe_fan_speed(val):
    try:
        parsed = int(val)
    except (TypeError, ValueError):
        return 20
    return max(5, min(100, parsed))


def _parse_disk_poll_minutes(val):
    if val is None or (isinstance(val, str) and val.strip() == ""):
        return 10
    try:
        minutes = int(val)
        return max(1, min(1440, minutes))
    except (TypeError, ValueError):
        return 10


def _parse_snmp_port(val):
    if val is None or (isinstance(val, str) and val.strip() == ""):
        return 161
    try:
        port = int(val)
        if 1 <= port <= 65535:
            return port
        return 161
    except (TypeError, ValueError):
        return 161


def parse_and_validate_pid_config(form):
    target_val = form.get('target_temp') if form.get('target_temp') is not None else form.get('pid_target_temp')
    target_temp = _to_int(target_val, 50)
    target_temp = max(20, min(90, target_temp))

    min_val = form.get('min_speed') if form.get('min_speed') is not None else form.get('pid_min_speed')
    min_speed = _to_int(min_val, 15)
    min_speed = max(5, min(100, min_speed))

    max_val = form.get('max_speed') if form.get('max_speed') is not None else form.get('pid_max_speed')
    max_speed = _to_int(max_val, 95)
    max_speed = max(5, min(100, max_speed))

    if max_speed < min_speed:
        max_speed = min_speed

    deadband_val = form.get('deadband') if form.get('deadband') is not None else form.get('pid_deadband')
    deadband = _to_float(deadband_val, 0.75)
    if deadband < 0:
        deadband = 0.0

    ti_val = form.get('ti_seconds') if form.get('ti_seconds') is not None else form.get('pid_ti_seconds')
    ti_seconds = _to_float(ti_val, 300.0)
    if ti_seconds <= 0:
        ti_seconds = 300.0

    ramp_down_val = form.get('ramp_down_percent_per_min') if form.get('ramp_down_percent_per_min') is not None else form.get('pid_ramp_down_percent_per_min')
    ramp_down = _to_int(ramp_down_val, 4)
    ramp_down = max(1, min(50, ramp_down))

    return {
        "target_temp": target_temp,
        "min_speed": min_speed,
        "max_speed": max_speed,
        "deadband": deadband,
        "ti_seconds": ti_seconds,
        "ramp_down_percent_per_min": ramp_down,
    }


def load_servers_config():
    with config_lock:
        if not os.path.exists(SERVERS_CONFIG_FILE):
            return []
        try:
            with open(SERVERS_CONFIG_FILE, 'r') as f:
                return json.load(f)
        except (json.JSONDecodeError, IOError):
            return []


def save_servers_config(servers):
    with config_lock:
        try:
            parent = os.path.dirname(SERVERS_CONFIG_FILE) or "."
            os.makedirs(parent, exist_ok=True)
            fd, tmp_path = tempfile.mkstemp(dir=parent, prefix=".servers_config_", suffix=".tmp")
            try:
                with os.fdopen(fd, 'w') as f:
                    json.dump(servers, f, indent=4)
                os.replace(tmp_path, SERVERS_CONFIG_FILE)
            except Exception:
                if os.path.exists(tmp_path):
                    os.unlink(tmp_path)
                raise
            flash("Saved and applied.", "success")
            return True
        except IOError:
            flash("Error: Could not write to config file.", "error")
            return False


def load_all_servers_status():
    if STATUS_FILE and os.path.exists(STATUS_FILE):
        try:
            with open(STATUS_FILE, 'r') as f:
                return json.load(f)
        except (json.JSONDecodeError, IOError):
            pass
    return []


def set_health_check(callback):
    global health_check
    health_check = callback


# --- Routes ---
@app.route('/health')
def health():
    healthy = health_check() if health_check else True
    return ("ok" if healthy else "unhealthy"), (200 if healthy else 503)


@app.route('/')
def index():
    all_statuses = load_all_servers_status()
    all_statuses.sort(key=lambda x: x.get('alias', ''))
    return render_template('index.html', servers=all_statuses)


@app.route('/servers')
def manage_servers():
    servers = load_servers_config()
    return render_template('servers.html', servers=servers, defaults=global_config)


@app.route('/servers/add', methods=['POST'])
def add_server():
    servers = load_servers_config()
    new_alias = (request.form.get('alias') or "").strip()
    if not new_alias:
        flash("Server alias is required.", "error")
        return redirect('../servers')
    if any(s['alias'] == new_alias for s in servers):
        flash(f"Server alias '{new_alias}' already exists.", "error")
        return redirect('../servers')

    poll_int = _parse_poll_interval(request.form.get('poll_interval_seconds'))
    snmp_comm = request.form.get('snmp_community')
    if snmp_comm is not None:
        snmp_comm = snmp_comm.strip()
    else:
        snmp_comm = ""

    new_server = {
        "alias": new_alias,
        "idrac_ip": (request.form.get('idrac_ip') or "").strip(),
        "idrac_username": request.form.get('idrac_username') or "root",
        "idrac_password": request.form.get('idrac_password') or "",
        "enabled": True,
        "fan_control_enabled": request.form.get('fan_control_enabled') == 'true',
        "disable_pcie_cooling_response": request.form.get('disable_pcie_cooling_response') == 'true',
        "on_fan_fault": _parse_on_fan_fault(request.form.get('on_fan_fault')),
        "min_safe_fan_speed": _parse_min_safe_fan_speed(request.form.get('min_safe_fan_speed')),
        "snmp_enabled": request.form.get('snmp_enabled') in ('true', 'on', '1'),
        "snmp_community": snmp_comm,
        "snmp_port": _parse_snmp_port(request.form.get('snmp_port')),
        "disk_poll_minutes": _parse_disk_poll_minutes(request.form.get('disk_poll_minutes')),
        "fan_mode": request.form.get('fan_mode') or "simple",
        "base_fan_speed_percent": _int_field(request.form, 'base_fan_speed_percent', 20),
        "low_temp_threshold": _int_field(request.form, 'low_temp_threshold', 45),
        "high_temp_fan_speed_percent": _int_field(request.form, 'high_temp_fan_speed_percent', 50),
        "critical_temp_threshold": _int_field(request.form, 'critical_temp_threshold', 65),
        "poll_interval_seconds": poll_int,
        "pid_config": parse_and_validate_pid_config(request.form),
        "fan_curve": [],
    }
    if (request.form.get('fan_mode') or "simple") == "target":
        raw_min = request.form.get('min_speed')
        raw_max = request.form.get('max_speed')
        if raw_min not in (None, "") and raw_max not in (None, "") and _to_int(raw_min, 15) > _to_int(raw_max, 95):
            flash("PID minimum fan speed cannot exceed maximum fan speed.", "error")
            return redirect('../servers')

    if server_manager is not None:
        try:
            server_manager.add_server(new_server)
            flash("Saved and applied.", "success")
        except ValueError as e:
            flash(str(e), "error")
        return redirect('../servers')

    servers.append(new_server)
    save_servers_config(servers)
    return redirect('../servers')


@app.route('/servers/edit/<alias>')
def edit_server_form(alias):
    servers = load_servers_config()
    server_to_edit = next((s for s in servers if s['alias'] == alias), None)
    if server_to_edit:
        server_to_edit.setdefault('fan_mode', 'simple')
        server_to_edit.setdefault('fan_curve', [])
        server_to_edit.setdefault('poll_interval_seconds', None)
        pid_cfg = server_to_edit.setdefault('pid_config', {})
        if pid_cfg.get('target_temp') is None or pid_cfg.get('target_temp') == "":
            pid_cfg['target_temp'] = 50
        if pid_cfg.get('min_speed') is None or pid_cfg.get('min_speed') == "":
            pid_cfg['min_speed'] = 15
        if pid_cfg.get('max_speed') is None or pid_cfg.get('max_speed') == "":
            pid_cfg['max_speed'] = 95
        if pid_cfg.get('deadband') is None or pid_cfg.get('deadband') == "":
            pid_cfg['deadband'] = 0.75
        if pid_cfg.get('ti_seconds') is None or pid_cfg.get('ti_seconds') == "":
            pid_cfg['ti_seconds'] = 300.0
        if pid_cfg.get('ramp_down_percent_per_min') is None or pid_cfg.get('ramp_down_percent_per_min') == "":
            pid_cfg['ramp_down_percent_per_min'] = 4
        server_to_edit.setdefault('fan_control_enabled', True)
        server_to_edit.setdefault('disable_pcie_cooling_response', False)
        server_to_edit.setdefault('on_fan_fault', 'dell_auto')
        server_to_edit.setdefault('min_safe_fan_speed', 20)
        server_to_edit.setdefault('snmp_enabled', False)
        server_to_edit.setdefault('snmp_community', '')
        server_to_edit.setdefault('snmp_port', 161)
        server_to_edit.setdefault('disk_poll_minutes', 10)
        return render_template('edit_server.html', server=server_to_edit)
    flash(f"Server '{alias}' not found.", "error")
    return redirect('../../servers')


@app.route('/servers/update/<alias>', methods=['POST'])
def update_server(alias):
    servers = load_servers_config()
    server_to_update = next((s for s in servers if s['alias'] == alias), None)
    if not server_to_update:
        flash(f"Server '{alias}' not found.", "error")
        return redirect('../../servers')

    server_to_update = dict(server_to_update)
    server_to_update['idrac_ip'] = (request.form.get('idrac_ip') or "").strip()
    server_to_update['idrac_username'] = request.form.get('idrac_username') or "root"
    new_password = request.form.get('idrac_password')
    if new_password:
        server_to_update['idrac_password'] = new_password
    server_to_update['enabled'] = request.form.get('enabled') == 'true'
    server_to_update['fan_control_enabled'] = request.form.get('fan_control_enabled') == 'true'
    server_to_update['disable_pcie_cooling_response'] = request.form.get('disable_pcie_cooling_response') == 'true'
    server_to_update['on_fan_fault'] = _parse_on_fan_fault(request.form.get('on_fan_fault'))
    server_to_update['min_safe_fan_speed'] = _parse_min_safe_fan_speed(request.form.get('min_safe_fan_speed'))
    server_to_update['snmp_enabled'] = request.form.get('snmp_enabled') in ('true', 'on', '1')
    new_community = request.form.get('snmp_community')
    if new_community is not None and new_community.strip() != "":
        server_to_update['snmp_community'] = new_community.strip()
    elif 'snmp_community' not in server_to_update:
        server_to_update['snmp_community'] = ""
    server_to_update['snmp_port'] = _parse_snmp_port(request.form.get('snmp_port'))
    server_to_update['disk_poll_minutes'] = _parse_disk_poll_minutes(request.form.get('disk_poll_minutes'))

    server_to_update['fan_mode'] = request.form.get('fan_mode') or 'simple'

    server_to_update['base_fan_speed_percent'] = _int_field(request.form, 'base_fan_speed_percent', 20)
    server_to_update['low_temp_threshold'] = _int_field(request.form, 'low_temp_threshold', 45)
    server_to_update['high_temp_fan_speed_percent'] = _int_field(request.form, 'high_temp_fan_speed_percent', 50)
    server_to_update['critical_temp_threshold'] = _int_field(request.form, 'critical_temp_threshold', 65)

    server_to_update['poll_interval_seconds'] = _parse_poll_interval(request.form.get('poll_interval_seconds'))
    server_to_update['pid_config'] = parse_and_validate_pid_config(request.form)

    fan_curve = []
    i = 0
    while True:
        temp = request.form.get(f'curve_temp_{i}')
        speed = request.form.get(f'curve_speed_{i}')
        if temp and speed:
            try:
                fan_curve.append({'temp': int(temp), 'speed': int(speed)})
            except ValueError:
                flash(f"Invalid fan curve value at point {i + 1}; it was ignored.", "error")
            i += 1
        else:
            break
    server_to_update['fan_curve'] = sorted(fan_curve, key=lambda p: p['temp'])

    if server_to_update['fan_mode'] == "target":
        raw_min = request.form.get('min_speed')
        raw_max = request.form.get('max_speed')
        if raw_min not in (None, "") and raw_max not in (None, "") and _to_int(raw_min, 15) > _to_int(raw_max, 95):
            flash("PID minimum fan speed cannot exceed maximum fan speed.", "error")
            return redirect('../../servers')

    if server_manager is not None:
        try:
            server_manager.apply_settings(alias, server_to_update)
            flash("Saved and applied.", "success")
        except ValueError as e:
            flash(str(e), "error")
        return redirect('../../servers')

    idx = next(i for i, s in enumerate(servers) if s['alias'] == alias)
    servers[idx] = server_to_update
    save_servers_config(servers)
    return redirect('../../servers')


@app.route('/servers/delete/<alias>', methods=['POST'])
def delete_server(alias):
    if server_manager is not None:
        server_manager.remove_server(alias)
        flash("Saved and applied.", "success")
        return redirect('../../servers')

    servers = load_servers_config()
    servers_to_keep = [s for s in servers if s['alias'] != alias]
    if len(servers_to_keep) < len(servers):
        save_servers_config(servers_to_keep)
        try:
            from .main import delete_pid_state
            delete_pid_state(alias)
        except Exception:
            try:
                from app.main import delete_pid_state
                delete_pid_state(alias)
            except Exception:
                pass
    else:
        flash(f"Server '{alias}' not found.", "error")
    return redirect('../../servers')


@app.route('/servers/test_snmp/<alias>', methods=['POST'])
def test_snmp(alias):
    servers = load_servers_config()
    server = next((s for s in servers if s['alias'] == alias), None)
    if not server:
        return {"success": False, "error": f"Server '{alias}' not found.", "disks": [], "raw": ""}, 404

    data = request.get_json(silent=True) or request.form
    host = (data.get('idrac_ip') or server.get('idrac_ip') or "").strip()
    community = data.get('snmp_community')
    if community is None or community.strip() == "":
        community = server.get('snmp_community', '')
    else:
        community = community.strip()
    port = _parse_snmp_port(data.get('snmp_port') if data.get('snmp_port') is not None else server.get('snmp_port', 161))

    if not community:
        return {"success": False, "error": "SNMP community string is not set.", "disks": [], "raw": ""}, 200

    cmd = ["snmpwalk", "-v2c", "-c", community, "-On", "-t", "3", "-r", "1", f"{host}:{port}", "1.3.6.1.4.1.674.10892.5.5.1.20.130.4.1"]
    try:
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
        raw_output = (res.stdout or "") + (res.stderr or "")
        if res.returncode != 0:
            err_msg = (res.stderr or res.stdout).strip().splitlines()
            reason = err_msg[0] if err_msg else f"snmpwalk exited with {res.returncode}"
            return {"success": False, "error": reason, "disks": [], "raw": raw_output}, 200

        disks_dict = disk_health.parse_walk(res.stdout)
        if not disks_dict:
            return {"success": False, "error": "No physical disks returned (SNMP agent off, wrong community, or no RAID controller).", "disks": [], "raw": raw_output}, 200

        disks_list = []
        for disk in disks_dict.values():
            bay_match = re.search(r'Disk\.Bay\.(\d+)', disk.get('fqdn') or '')
            bay = int(bay_match.group(1)) if bay_match else disk['index'] - 1
            d_copy = dict(disk)
            d_copy['bay'] = bay
            disks_list.append(d_copy)

        return {"success": True, "error": None, "disks": disks_list, "raw": raw_output}, 200
    except FileNotFoundError:
        return {"success": False, "error": "snmpwalk is not installed in this container.", "disks": [], "raw": ""}, 200
    except subprocess.TimeoutExpired:
        return {"success": False, "error": "snmpwalk timed out.", "disks": [], "raw": ""}, 200
    except Exception as e:
        return {"success": False, "error": str(e), "disks": [], "raw": ""}, 200
