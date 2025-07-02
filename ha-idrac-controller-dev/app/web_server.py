# HA-iDRAC/ha-idrac-controller-dev/app/web_server.py
from flask import Flask, render_template, request, redirect, url_for, flash, Markup
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

# --- Helper functions for config management ---
def load_servers_config():
    with config_lock:
        if not os.path.exists(SERVERS_CONFIG_FILE): return []
        try:
            with open(SERVERS_CONFIG_FILE, 'r') as f: return json.load(f)
        except (json.JSONDecodeError, IOError): return []

def save_servers_config(servers):
    with config_lock:
        try:
            with open(SERVERS_CONFIG_FILE, 'w') as f:
                json.dump(servers, f, indent=4)
            # Use Markup to allow the HTML link in the flashed message
            restart_url = "/hassio/addon/self/info"
            message = Markup(f"Configuration saved! <a href='{restart_url}' target='_parent'>Click here to go to the add-on page to RESTART</a> for changes to take effect.")
            flash(message, "success")
            return True
        except IOError:
            flash("Error: Could not write to config file.", "error")
            return False

def load_all_servers_status():
    if STATUS_FILE and os.path.exists(STATUS_FILE):
        try:
            with open(STATUS_FILE, 'r') as f: return json.load(f)
        except (json.JSONDecodeError, IOError): pass
    return []

# --- Routes ---
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
    new_alias = request.form.get('alias')
    if any(s['alias'] == new_alias for s in servers):
        flash(f"Server alias '{new_alias}' already exists.", "error")
        return redirect('../servers') # Use relative redirect

    new_server = {
        "alias": new_alias,
        "idrac_ip": request.form.get('idrac_ip'),
        "idrac_username": request.form.get('idrac_username'),
        "idrac_password": request.form.get('idrac_password'),
        "enabled": True,
        "base_fan_speed_percent": int(request.form.get('base_fan_speed_percent')),
        "low_temp_threshold": int(request.form.get('low_temp_threshold')),
        "high_temp_fan_speed_percent": int(request.form.get('high_temp_fan_speed_percent')),
        "critical_temp_threshold": int(request.form.get('critical_temp_threshold'))
    }
    servers.append(new_server)
    save_servers_config(servers)
    return redirect('../servers') # Use relative redirect
    
@app.route('/servers/edit/<alias>')
def edit_server_form(alias):
    servers = load_servers_config()
    server_to_edit = next((s for s in servers if s['alias'] == alias), None)
    if server_to_edit:
        return render_template('edit_server.html', server=server_to_edit)
    flash(f"Server '{alias}' not found.", "error")
    return redirect('../servers') # Use relative redirect

@app.route('/servers/update/<alias>', methods=['POST'])
def update_server(alias):
    servers = load_servers_config()
    server_to_update = next((s for s in servers if s['alias'] == alias), None)
    if not server_to_update:
        flash(f"Server '{alias}' not found.", "error")
        return redirect('../servers') # Use relative redirect

    server_to_update['idrac_ip'] = request.form.get('idrac_ip')
    server_to_update['idrac_username'] = request.form.get('idrac_username')
    new_password = request.form.get('idrac_password')
    if new_password:
        server_to_update['idrac_password'] = new_password
    server_to_update['enabled'] = request.form.get('enabled') == 'true'
    server_to_update['base_fan_speed_percent'] = int(request.form.get('base_fan_speed_percent'))
    server_to_update['low_temp_threshold'] = int(request.form.get('low_temp_threshold'))
    server_to_update['high_temp_fan_speed_percent'] = int(request.form.get('high_temp_fan_speed_percent'))
    server_to_update['critical_temp_threshold'] = int(request.form.get('critical_temp_threshold'))
    
    save_servers_config(servers)
    return redirect('../../servers') # Relative redirect from a deeper path

@app.route('/servers/delete/<alias>', methods=['POST'])
def delete_server(alias):
    servers = load_servers_config()
    servers_to_keep = [s for s in servers if s['alias'] != alias]
    if len(servers_to_keep) < len(servers):
        save_servers_config(servers_to_keep)
    else:
        flash(f"Server '{alias}' not found.", "error")
    return redirect('../servers') # Use relative redirect

def run_web_server(port, status_file_path, lock):
    global STATUS_FILE, status_lock
    STATUS_FILE = status_file_path
    status_lock = lock
    
    host = '0.0.0.0'
    app.run(host=host, port=port, debug=False, use_reloader=False)