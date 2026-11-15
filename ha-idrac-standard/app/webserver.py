import logging
import threading

from waitress.server import create_server

logger = logging.getLogger(__name__)


class WebServer:
    """Runs a Flask app on waitress in a background thread, with a clean shutdown."""

    def __init__(self, app, host="0.0.0.0", port=8099, threads=4):
        self.app = app
        self.host = host
        self.port = port
        self.threads = threads
        self._server = None
        self._thread = None

    def start(self):
        self._server = create_server(
            self.app, host=self.host, port=self.port, threads=self.threads,
            ident="ha-idrac", clear_untrusted_proxy_headers=True,
        )
        self.port = self._server.effective_port
        self._thread = threading.Thread(target=self._server.run, name="web", daemon=True)
        self._thread.start()
        logger.info("Web UI listening on %s:%s (waitress)", self.host, self.port)

    def stop(self):
        if self._server is not None:
            try:
                self._server.close()
            except Exception:
                logger.debug("Error closing web server", exc_info=True)
        if self._thread is not None:
            self._thread.join(timeout=5)

