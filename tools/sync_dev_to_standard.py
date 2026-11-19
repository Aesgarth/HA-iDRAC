"""Sync code and assets from ha-idrac-controller-dev/ to ha-idrac-standard/."""
import argparse
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
IDENTITY_FILES = {"config.yaml", "CHANGELOG.md", "README.md", "Dockerfile"}
SKIP_DIRS = {"data", "__pycache__", ".pytest_cache"}


def _rewrite_py_header(content_bytes, rel_posix):
    if not rel_posix.endswith(".py"):
        return content_bytes
    try:
        text = content_bytes.decode("utf-8")
    except UnicodeDecodeError:
        return content_bytes
    lines = text.splitlines(keepends=True)
    if lines and lines[0].startswith("# HA-iDRAC/ha-idrac-controller-dev/"):
        lines[0] = lines[0].replace(
            "# HA-iDRAC/ha-idrac-controller-dev/",
            "# HA-iDRAC/ha-idrac-standard/",
            1,
        )
        return "".join(lines).encode("utf-8")
    return content_bytes


def _normalize_dockerfile(path):
    if not path.exists():
        return []
    out = []
    for line in path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if stripped.startswith("# HA-iDRAC/") and stripped.endswith("Dockerfile"):
            continue
        if stripped.startswith("ENV ADDON_ID="):
            continue
        out.append(line)
    return out


def sync(dev_dir, std_dir, check=False, delete=False):
    dev_dir = Path(dev_dir)
    std_dir = Path(std_dir)
    changed = []
    dev_rel_files = set()

    for root, dirs, files in os.walk(dev_dir):
        dirs[:] = [d for d in sorted(dirs) if d not in SKIP_DIRS]
        rel_root = Path(root).relative_to(dev_dir)
        for fname in sorted(files):
            if fname.endswith(".pyc"):
                continue
            rel_path = rel_root / fname
            rel_posix = rel_path.as_posix()
            if rel_root == Path(".") and fname in IDENTITY_FILES:
                continue
            dev_rel_files.add(rel_posix)
            src_path = dev_dir / rel_path
            dst_path = std_dir / rel_path
            expected_bytes = _rewrite_py_header(src_path.read_bytes(), rel_posix)
            current_bytes = dst_path.read_bytes() if dst_path.exists() else None
            if current_bytes != expected_bytes:
                changed.append(rel_posix)
                if not check:
                    dst_path.parent.mkdir(parents=True, exist_ok=True)
                    dst_path.write_bytes(expected_bytes)

    extra_in_std_app = []
    std_app = std_dir / "app"
    if std_app.exists():
        for root, dirs, files in os.walk(std_app):
            dirs[:] = [d for d in sorted(dirs) if d not in SKIP_DIRS]
            rel_root = Path(root).relative_to(std_dir)
            for fname in sorted(files):
                if fname.endswith(".pyc"):
                    continue
                rel_posix = (rel_root / fname).as_posix()
                if rel_posix not in dev_rel_files:
                    extra_in_std_app.append(rel_posix)
                    print(f"Extra file in standard app/: {rel_posix}")
                    if delete and not check:
                        (std_dir / rel_root / fname).unlink()

    dev_df = _normalize_dockerfile(dev_dir / "Dockerfile")
    std_df = _normalize_dockerfile(std_dir / "Dockerfile")
    if dev_df != std_df:
        print("Dockerfile differs between dev and standard (beyond header and ENV ADDON_ID=):")
        for line in dev_df:
            if line not in std_df:
                print(f"  dev only: {line}")
        for line in std_df:
            if line not in dev_df:
                print(f"  standard only: {line}")

    if check:
        for rel_posix in changed:
            print(f"Differs: {rel_posix}")
    else:
        for rel_posix in changed:
            print(f"Synced: {rel_posix}")

    return changed, extra_in_std_app


def main(argv=None):
    parser = argparse.ArgumentParser(description="Sync ha-idrac-controller-dev to ha-idrac-standard")
    parser.add_argument("--check", action="store_true", help="Report differences without modifying files")
    parser.add_argument("--delete", action="store_true", help="Delete files in standard/app/ removed from dev/app/")
    parser.add_argument("--dev-dir", default=str(ROOT / "ha-idrac-controller-dev"))
    parser.add_argument("--standard-dir", default=str(ROOT / "ha-idrac-standard"))
    args = parser.parse_args(argv)
    sync(args.dev_dir, args.standard_dir, check=args.check, delete=args.delete)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
