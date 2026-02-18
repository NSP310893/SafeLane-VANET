from __future__ import annotations
from pathlib import Path
import yaml
import datetime

def load_yaml(path: str | Path) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)

def make_run_dir(out_root: str | Path, exp_name: str) -> Path:
    out_root = Path(out_root)
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = out_root / f"{exp_name}" / ts
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir

def save_resolved_config(cfg: dict, run_dir: Path):
    p = run_dir / "config_resolved.yaml"
    with open(p, "w", encoding="utf-8") as f:
        yaml.safe_dump(cfg, f, sort_keys=False)
