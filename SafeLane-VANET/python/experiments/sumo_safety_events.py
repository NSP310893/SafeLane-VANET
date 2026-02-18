from dataclasses import dataclass
from pathlib import Path
import pandas as pd
import numpy as np

@dataclass
class SafetyEvents:
    collisions: int
    near_miss_ttc: int
    near_miss_gap: int
    min_ttc_global: float
    min_gap_global: float

def extract_sumo_collisions(run_dir: Path) -> int:
    c_csv = run_dir / "collisions.csv"
    if c_csv.exists():
        df = pd.read_csv(c_csv)
        return int(len(df))
    return 0

def compute_near_misses(actions_csv: Path, ttc_thr=1.5, gap_thr=2.0) -> SafetyEvents:
    a = pd.read_csv(actions_csv)
    a["min_ttc"] = pd.to_numeric(a.get("min_ttc"), errors="coerce")
    a["gap_min"] = pd.to_numeric(a.get("gap_min"), errors="coerce")

    ttc = a["min_ttc"].replace([np.inf, -np.inf], np.nan).dropna().to_numpy()
    gap = a["gap_min"].replace([np.inf, -np.inf], np.nan).dropna().to_numpy()

    min_ttc_global = float(np.min(ttc)) if ttc.size else 0.0
    min_gap_global = float(np.min(gap)) if gap.size else 0.0

    near_miss_ttc = int(np.sum(ttc < float(ttc_thr))) if ttc.size else 0
    near_miss_gap = int(np.sum(gap < float(gap_thr))) if gap.size else 0

    return SafetyEvents(
        collisions=0,
        near_miss_ttc=near_miss_ttc,
        near_miss_gap=near_miss_gap,
        min_ttc_global=min_ttc_global,
        min_gap_global=min_gap_global,
    )

def compute_safety_events(run_dir: str, ttc_thr=1.5, gap_thr=2.0) -> dict:
    run_dir = Path(run_dir)
    ev = compute_near_misses(run_dir / "actions.csv", ttc_thr=ttc_thr, gap_thr=gap_thr)
    col = extract_sumo_collisions(run_dir)
    ev.collisions = col
    return {
        "collisions": ev.collisions,
        "near_miss_ttc": ev.near_miss_ttc,
        "near_miss_gap": ev.near_miss_gap,
        "min_ttc_global": ev.min_ttc_global,
        "min_gap_global": ev.min_gap_global,
    }
