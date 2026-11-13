import json
import urllib.request

from flask import Flask
from conftest import load_module

ws = load_module("webserver", "ha-idrac-controller-dev/app/webserver.py")


def test_server_serves_and_stops_cleanly():
    app = Flask(__name__)

    @app.route("/health")
    def health():
        return json.dumps({"ok": True})

    server = ws.WebServer(app, host="127.0.0.1", port=0)
    server.start()
    try:
        body = urllib.request.urlopen(f"http://127.0.0.1:{server.port}/health", timeout=5).read()
        assert json.loads(body) == {"ok": True}
    finally:
        server.stop()
    assert not server._thread.is_alive()

