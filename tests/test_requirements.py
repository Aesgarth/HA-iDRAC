import re
from pathlib import Path

from conftest import resolve_addon_path


def test_requirements_lists_runtime_dependencies():
    req_path = resolve_addon_path("ha-idrac-controller-dev/app/requirements.txt")
    lines = [
        line.strip()
        for line in req_path.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.strip().startswith("#")
    ]
    packages = {re.split(r"[=<>!~\[]", line, maxsplit=1)[0].strip().lower() for line in lines}
    for required in ("flask", "paho-mqtt", "waitress"):
        assert required in packages, f"{required} missing from {req_path}"

