import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
DEV_AUTOMATIONS = ROOT / "ha-idrac-controller-dev" / "AUTOMATIONS.md"
STD_AUTOMATIONS = ROOT / "ha-idrac-standard" / "AUTOMATIONS.md"
_YAML_BLOCK_RE = re.compile(r"```yaml\s*\r?\n(.*?)```", re.DOTALL)


def test_automations_docs_are_identical_and_have_yaml_blocks():
    dev_text = DEV_AUTOMATIONS.read_text(encoding="utf-8")
    std_text = STD_AUTOMATIONS.read_text(encoding="utf-8")
    assert dev_text == std_text

    blocks = _YAML_BLOCK_RE.findall(dev_text)
    assert len(blocks) == 7


def test_automations_yaml_blocks_parse_cleanly():
    yaml = pytest.importorskip("yaml")
    for path in (DEV_AUTOMATIONS, STD_AUTOMATIONS):
        text = path.read_text(encoding="utf-8")
        blocks = _YAML_BLOCK_RE.findall(text)
        assert len(blocks) == 7
        for i, block in enumerate(blocks, start=1):
            parsed = yaml.safe_load(block)
            assert isinstance(parsed, dict), f"{path.name} block {i} did not parse to a mapping"
            assert "triggers" in parsed and "actions" in parsed
