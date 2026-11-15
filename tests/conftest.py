"""Load the add-on modules without importing Flask/paho or touching /data."""
import importlib.util
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ADDON_DIR = os.environ.get("ADDON_DIR") or "ha-idrac-controller-dev"
ADDON_PATH = ROOT / ADDON_DIR

if str(ADDON_PATH) not in sys.path:
    sys.path.insert(0, str(ADDON_PATH))


def resolve_addon_path(relative_path):
    parts = Path(relative_path).parts
    if parts and parts[0] == "ha-idrac-controller-dev":
        return ROOT.joinpath(ADDON_DIR, *parts[1:])
    return ROOT / relative_path


def load_module(name, relative_path):
    spec = importlib.util.spec_from_file_location(name, resolve_addon_path(relative_path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module
