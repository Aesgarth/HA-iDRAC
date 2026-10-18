#!/bin/bash
echo "[RUN.SH] >>> Add-on execution started at $(date)"

# Default values
#IDRAC_IP_DEFAULT=""
#IDRAC_USERNAME_DEFAULT="root"
#IDRAC_PASSWORD_DEFAULT=""
CHECK_INTERVAL_SECONDS_DEFAULT=60
LOG_LEVEL_DEFAULT="info"
BASE_FAN_SPEED_PERCENT_DEFAULT=20
LOW_TEMP_THRESHOLD_DEFAULT=45
HIGH_TEMP_FAN_SPEED_PERCENT_DEFAULT=50
CRITICAL_TEMP_THRESHOLD_DEFAULT=65
MQTT_HOST_DEFAULT="core-mosquitto"
MQTT_PORT_DEFAULT=1883
MQTT_USERNAME_DEFAULT=""
MQTT_PASSWORD_DEFAULT=""

# Read configuration from /data/options.json if it exists
if [ -f /data/options.json ]; then
    echo "[RUN.SH] Reading configuration from /data/options.json"
    #export IDRAC_IP=$(jq -r '.idrac_ip // empty' /data/options.json)
    #export IDRAC_USERNAME=$(jq -r '.idrac_username // "'"$IDRAC_USERNAME_DEFAULT"'"' /data/options.json)
    #export IDRAC_PASSWORD=$(jq -r '.idrac_password // empty' /data/options.json)
    export CHECK_INTERVAL_SECONDS=$(jq -r '.check_interval_seconds // "'"$CHECK_INTERVAL_SECONDS_DEFAULT"'"' /data/options.json)
    export LOG_LEVEL=$(jq -r '.log_level // "'"$LOG_LEVEL_DEFAULT"'"' /data/options.json)

    export BASE_FAN_SPEED_PERCENT=$(jq -r '.base_fan_speed_percent // "'"$BASE_FAN_SPEED_PERCENT_DEFAULT"'"' /data/options.json)
    export LOW_TEMP_THRESHOLD=$(jq -r '.low_temp_threshold // "'"$LOW_TEMP_THRESHOLD_DEFAULT"'"' /data/options.json)
    export HIGH_TEMP_FAN_SPEED_PERCENT=$(jq -r '.high_temp_fan_speed_percent // "'"$HIGH_TEMP_FAN_SPEED_PERCENT_DEFAULT"'"' /data/options.json)
    export CRITICAL_TEMP_THRESHOLD=$(jq -r '.critical_temp_threshold // "'"$CRITICAL_TEMP_THRESHOLD_DEFAULT"'"' /data/options.json)

    export MQTT_HOST=$(jq -r '.mqtt_host // "'"$MQTT_HOST_DEFAULT"'"' /data/options.json)
    export MQTT_PORT=$(jq -r '.mqtt_port // '$MQTT_PORT_DEFAULT /data/options.json)
    export MQTT_USERNAME=$(jq -r '.mqtt_username // empty' /data/options.json)
    export MQTT_PASSWORD=$(jq -r '.mqtt_password // empty' /data/options.json)
else
    echo "[RUN.SH] WARNING: /data/options.json not found. Using internal defaults."
    export IDRAC_IP="$IDRAC_IP_DEFAULT"
    export IDRAC_USERNAME="$IDRAC_USERNAME_DEFAULT"
    export IDRAC_PASSWORD="$IDRAC_PASSWORD_DEFAULT"
    export CHECK_INTERVAL_SECONDS="$CHECK_INTERVAL_SECONDS_DEFAULT"
    export LOG_LEVEL="$LOG_LEVEL_DEFAULT"
    export BASE_FAN_SPEED_PERCENT="$BASE_FAN_SPEED_PERCENT_DEFAULT"
    export LOW_TEMP_THRESHOLD="$LOW_TEMP_THRESHOLD_DEFAULT"
    export HIGH_TEMP_FAN_SPEED_PERCENT="$HIGH_TEMP_FAN_SPEED_PERCENT_DEFAULT"
    export CRITICAL_TEMP_THRESHOLD="$CRITICAL_TEMP_THRESHOLD_DEFAULT"
    export MQTT_HOST="$MQTT_HOST_DEFAULT"
    export MQTT_PORT="$MQTT_PORT_DEFAULT"
    export MQTT_USERNAME="$MQTT_USERNAME_DEFAULT"
    export MQTT_PASSWORD="$MQTT_PASSWORD_DEFAULT"
fi

echo "[RUN.SH] Effective Configuration:"
echo "[RUN.SH]   IDRAC_IP: ${IDRAC_IP}"
echo "[RUN.SH]   LOG_LEVEL: ${LOG_LEVEL}"
echo "[RUN.SH]   BASE_FAN_SPEED: ${BASE_FAN_SPEED_PERCENT}%"
echo "[RUN.SH]   LOW_TEMP_THRESH: ${LOW_TEMP_THRESHOLD}°C"
echo "[RUN.SH]   HIGH_TEMP_FAN_SPEED: ${HIGH_TEMP_FAN_SPEED_PERCENT}%"
echo "[RUN.SH]   CRITICAL_TEMP_THRESH: ${CRITICAL_TEMP_THRESHOLD}°C"
echo "[RUN.SH]   MQTT_HOST: ${MQTT_HOST}:${MQTT_PORT}"
# Avoid logging username/password directly unless debugging and you know it's safe
# echo "[RUN.SH] MQTT_USERNAME: ${MQTT_USERNAME}"

echo "[RUN.SH] Starting Python application (executing python3 -m app.main from /)..."
cd / 
exec python3 -m app.main

echo "[RUN.SH] !!!!! CRITICAL ERROR: python3 -m app.main failed to start or exited unexpectedly via exec !!!!!" >&2
exit 1