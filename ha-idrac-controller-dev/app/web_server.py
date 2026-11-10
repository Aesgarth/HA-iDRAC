# HA-iDRAC/ha-idrac-controller-dev/app/web_server.py
from flask import Flask, render_template, request, redirect, flash
from markupsafe import Markup
import os
import json
import logging
import threading

log = logging.getLogger('werkzeug')
app = Flask(__name__)
app.secret_key = os.urandom(24)

# --- Global paths and locks ---
STATUS_FILE = None
SERVERS_CONFIG_FILE = "/data/servers_config.json"
status_lock = None
config_lock = threading.Lock()
global_config = {}
health_check = None


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
            with open(SERVERS_CONFIG_FILE, 'w') as f:
                json.dump(servers, f, indent=4)
            restart_url = "/hassio/dashboard"
            message = Markup(f"Configuration saved! <a href='{restart_url}' target='_top'>Click here to go to the Add-ons dashboard to RESTART</a> for changes to take effect.")
            flash(message, "success")
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
    new_server = {
        "alias": new_alias,
        "idrac_ip": (request.form.get('idrac_ip') or "").strip(),
        "idrac_username": request.form.get('idrac_username') or "root",
        "idrac_password": request.form.get('idrac_password') or "",
        "enabled": True,
        "fan_control_enabled": request.form.get('fan_control_enabled') == 'true',
        "disable_pcie_cooling_response": request.form.get('disable_pcie_cooling_response') == 'true',
        "fan_mode": request.form.get('fan_mode') or "simple",
        "base_fan_speed_percent": _int_field(request.form, 'base_fan_speed_percent', 20),
        "low_temp_threshold": _int_field(request.form, 'low_temp_threshold', 45),
        "high_temp_fan_speed_percent": _int_field(request.form, 'high_temp_fan_speed_percent', 50),
        "critical_temp_threshold": _int_field(request.form, 'critical_temp_threshold', 65),
        "poll_interval_seconds": poll_int,
        "pid_config": parse_and_validate_pid_config(request.form),
        "fan_curve": [],
    }
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

    server_to_update['idrac_ip'] = (request.form.get('idrac_ip') or "").strip()
    server_to_update['idrac_username'] = request.form.get('idrac_username') or "root"
    new_password = request.form.get('idrac_password')
    if new_password:
        server_to_update['idrac_password'] = new_password
    server_to_update['enabled'] = request.form.get('enabled') == 'true'
    server_to_update['fan_control_enabled'] = request.form.get('fan_control_enabled') == 'true'
    server_to_update['disable_pcie_cooling_response'] = request.form.get('disable_pcie_cooling_response') == 'true'

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

    save_servers_config(servers)
    return redirect('../../servers')


@app.route('/servers/delete/<alias>', methods=['POST'])
def delete_server(alias):
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


def run_web_server(port, status_file_path, lock):
    global STATUS_FILE, status_lock
    STATUS_FILE = status_file_path
    status_lock = lock

    host = '0.0.0.0'
    app.run(host=host, port=port, debug=False, use_reloader=False)
