#!/usr/bin/env python3
from __future__ import annotations
import argparse
from pathlib import Path
import shutil
import subprocess
import sys

def copy_with_backup(source: Path, target: Path) -> None:
    if target.is_file():
        backup = target.with_suffix(target.suffix + ".bak_safety_tf_fix")
        if not backup.exists():
            shutil.copy2(target, backup)
            print(f"[BACKUP] {backup}")
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, target)
    print(f"[COPY] {target}")

def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", default="~/b3_cobot3_ws")
    args = parser.parse_args()
    workspace = Path(args.workspace).expanduser().resolve()
    root = Path(__file__).resolve().parents[1]
    relatives = [
        Path("src/forest_rescue_system/forest_rescue_system/drone/drone_controller_node.py"),
        Path("src/forest_rescue_system/forest_rescue_system/drone/obstacle_monitor_node.py"),
        Path("src/forest_rescue_system/config/forest_rescue.yaml"),
    ]
    for rel in relatives:
        copy_with_backup(root / rel, workspace / rel)
    subprocess.run([
        sys.executable,
        str(root / "tools/apply_coverage_tf_fallback.py"),
        "--workspace", str(workspace),
    ], check=True)
    print("[DONE] all fixes applied")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
