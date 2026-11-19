from conftest import load_module

sync_mod = load_module("sync_dev_to_standard", "tools/sync_dev_to_standard.py")


def _setup_fake_dirs(tmp_path):
    dev = tmp_path / "ha-idrac-controller-dev"
    std = tmp_path / "ha-idrac-standard"
    (dev / "app").mkdir(parents=True)
    (std / "app").mkdir(parents=True)

    for name in ("config.yaml", "CHANGELOG.md", "README.md"):
        (dev / name).write_text(f"dev {name}", encoding="utf-8")
        (std / name).write_text(f"std {name}", encoding="utf-8")

    (dev / "Dockerfile").write_text(
        "# HA-iDRAC/ha-idrac-controller/Dockerfile\nFROM python:3.11\nENV ADDON_ID=ha_idrac_dev\n",
        encoding="utf-8",
    )
    (std / "Dockerfile").write_text(
        "# HA-iDRAC/ha-idrac-standard/Dockerfile\nFROM python:3.11\nENV ADDON_ID=ha_idrac_standard\n",
        encoding="utf-8",
    )
    (dev / "data").mkdir()
    (dev / "data" / "servers_config.json").write_text("[]", encoding="utf-8")

    (dev / "app" / "x.py").write_text(
        "# HA-iDRAC/ha-idrac-controller-dev/app/x.py\nVALUE = 2\n",
        encoding="utf-8",
    )
    (std / "app" / "x.py").write_text(
        "# HA-iDRAC/ha-idrac-standard/app/x.py\nVALUE = 1\n",
        encoding="utf-8",
    )
    (std / "app" / "obsolete.py").write_text("OLD = True\n", encoding="utf-8")
    return dev, std


def test_sync_script_check_and_apply_and_identity_preserved(tmp_path):
    dev, std = _setup_fake_dirs(tmp_path)

    # 1. --check mode changes nothing
    changed, extra = sync_mod.sync(dev, std, check=True, delete=False)
    assert changed == ["app/x.py"]
    assert extra == ["app/obsolete.py"]
    assert "VALUE = 1" in (std / "app" / "x.py").read_text(encoding="utf-8")
    assert (std / "app" / "obsolete.py").exists()

    # 2. Normal sync rewrites header, preserves identity files, skips data/, removes obsolete only with --delete
    sync_mod.sync(dev, std, check=False, delete=False)
    assert (std / "app" / "x.py").read_text(encoding="utf-8") == (
        "# HA-iDRAC/ha-idrac-standard/app/x.py\nVALUE = 2\n"
    )
    assert (std / "app" / "obsolete.py").exists()
    for name in ("config.yaml", "CHANGELOG.md", "README.md"):
        assert (std / name).read_text(encoding="utf-8") == f"std {name}"
    assert "ADDON_ID=ha_idrac_standard" in (std / "Dockerfile").read_text(encoding="utf-8")
    assert not (std / "data").exists()

    # 3. With --delete, obsolete file is removed and second --check reports nothing
    sync_mod.sync(dev, std, check=False, delete=True)
    assert not (std / "app" / "obsolete.py").exists()
    changed_after, extra_after = sync_mod.sync(dev, std, check=True, delete=False)
    assert changed_after == []
    assert extra_after == []
