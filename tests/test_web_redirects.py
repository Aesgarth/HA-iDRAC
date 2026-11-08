"""Regression tests for relative redirects under Home Assistant Ingress."""
import json
from urllib.parse import urljoin
import pytest

pytest.importorskip("flask")

from conftest import load_module

web_server = load_module("dev_web_server", "ha-idrac-controller-dev/app/web_server.py")

BASE = "https://x.ui.nabu.casa/api/hassio_ingress/TOKEN//"


@pytest.fixture
def client(tmp_path):
    config_file = str(tmp_path / "servers_config.json")
    with open(config_file, "w") as f:
        json.dump([{"alias": "r720", "idrac_ip": "192.168.1.100", "enabled": True}], f)

    web_server.SERVERS_CONFIG_FILE = config_file
    web_server.global_config = {
        "base_fan_speed_percent": 20,
        "low_temp_threshold": 45,
        "high_temp_fan_speed_percent": 50,
        "critical_temp_threshold": 65,
    }
    web_server.app.config["TESTING"] = True
    with web_server.app.test_client() as test_client:
        yield test_client


def assert_redirect_to_manage_page(response, path):
    assert response.status_code == 302
    location = response.headers.get("Location")
    assert location is not None
    expected = urljoin(BASE, "servers")
    resolved = urljoin(urljoin(BASE, path), location)
    assert resolved == expected


def test_redirect_add_success(client):
    path = "servers/add"
    res = client.post(f"/{path}", data={"alias": "r730", "idrac_ip": "192.168.1.101"})
    assert_redirect_to_manage_page(res, path)


def test_redirect_add_duplicate_alias(client):
    path = "servers/add"
    res = client.post(f"/{path}", data={"alias": "r720", "idrac_ip": "192.168.1.100"})
    assert_redirect_to_manage_page(res, path)


def test_redirect_edit_unknown_alias(client):
    path = "servers/edit/unknown_server"
    res = client.get(f"/{path}")
    assert_redirect_to_manage_page(res, path)


def test_redirect_update_unknown_alias(client):
    path = "servers/update/unknown_server"
    res = client.post(f"/{path}", data={"idrac_ip": "192.168.1.102"})
    assert_redirect_to_manage_page(res, path)


def test_redirect_delete_existing(client):
    path = "servers/delete/r720"
    res = client.post(f"/{path}")
    assert_redirect_to_manage_page(res, path)


def test_redirect_delete_unknown_alias(client):
    path = "servers/delete/unknown_server"
    res = client.post(f"/{path}")
    assert_redirect_to_manage_page(res, path)

