# Contributing to HA-iDRAC

## Repository Layout and Workflow

This repository contains three Home Assistant add-on folders:

1. **`ha-idrac-controller-dev/`** (`ha_idrac_controller_development`): The active development add-on where all new features and fixes are implemented and hardware-tested first.
2. **`ha-idrac-standard/`** (`ha_idrac_controller_standard`): The recommended release add-on. Do not edit its `app/` code directly; sync verified changes from Dev using [`tools/sync_dev_to_standard.py`](tools/sync_dev_to_standard.py).
3. **`ha-idrac-controller/`** (`ha_idrac_controller`, `0.1.2`): The legacy single-server add-on. It is frozen and under review—do not modify its code.

### Syncing Dev to Standard

After a change has been verified in `ha-idrac-controller-dev/`:

```bash
# Preview differences between Dev and Standard (ignores identity files)
python tools/sync_dev_to_standard.py --check

# Copy non-identity files from Dev into Standard and rewrite header comments
python tools/sync_dev_to_standard.py
```

The sync script never overwrites identity files (`config.yaml`, `CHANGELOG.md`, `README.md`, `Dockerfile`) or `data/`. Update `ha-idrac-standard/README.md`, `ha-idrac-standard/CHANGELOG.md`, and `ha-idrac-standard/config.yaml` manually when releasing a new Standard version.

## Running the Test Suite

GitHub Actions ([`.github/workflows/tests.yml`](.github/workflows/tests.yml)) runs `pytest` across an `ADDON_DIR` matrix (`ha-idrac-controller-dev` and `ha-idrac-standard`). Before pushing, run the suite in a fresh virtualenv containing only `pytest` and the add-on's `app/requirements.txt`:

```bash
python -m venv .venv
# Linux / macOS:
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install pytest -r ha-idrac-controller-dev/app/requirements.txt
ADDON_DIR=ha-idrac-controller-dev .venv/bin/python -m pytest -q
ADDON_DIR=ha-idrac-standard .venv/bin/python -m pytest -q
```

(Install `pyyaml` in the test environment if you also want to run the optional YAML-parsing tests in `tests/test_docs_yaml.py` and `tests/test_make_dashboard.py`.)

## Versioning and Changelogs

* Home Assistant requires the changelog filename to be uppercase **`CHANGELOG.md`** (`ha-idrac-controller-dev/CHANGELOG.md` and `ha-idrac-standard/CHANGELOG.md`).
* Every code change to an add-on must bump the `version` field in that add-on's `config.yaml` and add a matching entry at the top of its `CHANGELOG.md`.

## Security and Hygiene Rules

* **Never commit secrets:** Do not commit iDRAC passwords, SNMP community strings, tokens, real private IP addresses, or real service tags/serial numbers in code, tests, fixtures, screenshots, or documentation.
* **Never commit runtime or exported files:** Never commit anything under `data/` (`servers_config.json`, `pid_states.json`), exported settings JSON files (`ha-idrac-servers.json`), or local `entities.txt` / generated dashboard YAML files.
* Always check `git status --short` and `git diff` before committing.
