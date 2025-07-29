# HA-iDRAC/ha-idrac-controller-dev/app/mqtt_client.py
import paho.mqtt.client as mqtt
import json
import re

class MqttClient:
    def __init__(self, client_id="ha_idrac_controller"):
        self.client_id = client_id
        self.client = mqtt.Client(client_id=self.client_id, protocol=mqtt.MQTTv311)
        self.broker_address = "core-mosquitto"
        self.port = 1883
        self.username = ""
        self.password = ""
        self.is_connected = False
        self.log_level = "info"
        
        self.base_topic = "ha_idrac_controller"
        self.availability_topic = f"{self.base_topic}/status"
        self.device_info_dict = None
        self.message_callback = None

        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self._on_message

    def _log(self, level, message):
        levels = {"trace": -1, "debug": 0, "info": 1, "warning": 2, "error": 3, "fatal": 4}
        if levels.get(self.log_level, levels["info"]) <= levels.get(level.lower(), levels["info"]):
            print(f"[{level.upper()}] MQTT ({self.client_id}): {message}", flush=True)

    def configure_broker(self, host, port, username, password, log_level="info"):
        self.broker_address = host
        self.port = int(port)
        self.username = username
        self.password = password
        self.log_level = log_level.lower()
        if self.username:
            self.client.username_pw_set(self.username, self.password)

    def set_device_info(self, server_alias, manufacturer, model, ip_address):
        safe_alias = re.sub(r'[^a-zA-Z0-9_-]+', '_', server_alias)
        self.base_topic = f"ha_idrac_controller/{safe_alias}"
        self.availability_topic = f"{self.base_topic}/status"
        self.device_info_dict = {
            "identifiers": [f"idrac_controller_{safe_alias}"],
            "name": f"iDRAC ({server_alias})",
            "model": model or "PowerEdge Server",
            "manufacturer": manufacturer or "DELL",
            "configuration_url": f"http://{ip_address}" if ip_address else None
        }
        self._log("info", f"Device info for MQTT discovery set for '{server_alias}'")

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self._log("info", f"Connected successfully to broker {self.broker_address}:{self.port}")
            self.is_connected = True
        else:
            self._log("error", f"Connection failed with code {rc}")
            self.is_connected = False

    def on_disconnect(self, client, userdata, rc):
        self._log("info", f"Disconnected from broker with result code {rc}.")
        self.is_connected = False

    def _on_message(self, client, userdata, msg):
        if self.message_callback:
            self.message_callback(msg.topic, msg.payload.decode('utf-8'))

    def connect(self):
        if self.is_connected: return
        self._log("info", f"Attempting to connect to broker {self.broker_address}...")
        try:
            self.client.will_set(self.availability_topic, payload="offline", qos=1, retain=True)
            self.client.connect(self.broker_address, self.port, 60)
            self.client.loop_start()
        except Exception as e:
            self._log("error", f"Could not connect to broker: {e}")

    def disconnect(self):
        if not self.is_connected: return
        self.publish(self.availability_topic, "offline", retain=True)
        self.client.loop_stop()
        self.client.disconnect()
        self._log("info", "Gracefully disconnected.")
        self.is_connected = False

    def subscribe(self, topic):
        self._log("info", f"Subscribing to command topic: {topic}")
        self.client.subscribe(topic)

    def publish(self, topic, payload, retain=False, qos=0):
        if not self.is_connected:
            self._log("warning", f"Not connected. Cannot publish to {topic}.")
            return
        try:
            self.client.publish(topic, payload, qos=qos, retain=retain)
        except Exception as e:
            self._log("error", f"Failed to publish to {topic}: {e}")

    def publish_discovery(self, component, slug, name, device_class=None, unit=None, icon=None, cmd_topic=None, val_template=None, state_class=None):
        if not self.device_info_dict:
            return

        unique_id = f"{self.device_info_dict['identifiers'][0]}_{slug}"
        config_topic = f"homeassistant/{component}/{unique_id}/config"
        
        payload = {
            "name": name,
            "unique_id": unique_id,
            "device": self.device_info_dict,
            "availability_topic": self.availability_topic,
        }
        
        if component == 'sensor':
            payload["state_topic"] = f"{self.base_topic}/sensor/{slug}"
            payload["json_attributes_topic"] = f"{self.base_topic}/sensor/{slug}"
            payload["value_template"] = "{{ value_json.state }}"
        elif component == 'binary_sensor':
            payload["state_topic"] = f"{self.base_topic}/binary_sensor/{slug}"
            payload["payload_on"] = "ON"
            payload["payload_off"] = "OFF"
        elif component == 'button':
            payload["command_topic"] = cmd_topic
            payload["payload_press"] = "PRESS"
        
        if device_class: payload["device_class"] = device_class
        if unit: payload["unit_of_measurement"] = unit
        if icon: payload["icon"] = icon
        if state_class: payload["state_class"] = state_class

        self.publish(config_topic, json.dumps(payload), retain=True)

    def publish_state(self, component, slug, state, attributes=None):
        if not self.is_connected:
            return
            
        topic = f"{self.base_topic}/{component}/{slug}"
        if component == "sensor":
            payload = {"state": state}
            if attributes:
                payload.update(attributes)
            self.publish(topic, json.dumps(payload))
        else: # For binary_sensor
            self.publish(topic, state)