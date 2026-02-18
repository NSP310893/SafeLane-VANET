from pathlib import Path
import argparse
import pandas as pd
import numpy as np

from python.experiments.sumo_safety_events import compute_safety_events
from python.experiments.prediction_metrics import compute_pred_metrics
from python.experiments.comfort_metrics import compute_comfort

def mean_ci95(x):
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    if len(x) == 0:
        return 0.0, 0.0
    m = float(x.mean())
    if len(x) == 1:
        return m, 0.0
    se = float(x.std(ddof=1) / np.sqrt(len(x)))
    return m, 1.96 * se

def compute_run(run_dir: Path, dt=0.1):
    a = pd.read_csv(run_dir / "actions.csv")
    min_ttc = pd.to_numeric(a["min_ttc"], errors="coerce").replace([np.inf,-np.inf], np.nan).dropna().to_numpy()
    ttc_p5 = float(np.percentile(min_ttc, 5)) if min_ttc.size else 0.0

    cancel_rate = float((a["action"] == "CANCEL").mean())
    defer_rate  = float((a["action"] == "DEFER").mean())
    exec_rate   = float((a["action"] == "EXECUTE").mean())
    lane_changes = int((a["action"] == "EXECUTE").sum())

    pdr_mean = float(pd.to_numeric(a.get("pdr"), errors="coerce").mean()) if "pdr" in a.columns else 0.0
    lat_p95_mean = float(pd.to_numeric(a.get("lat_p95"), errors="coerce").mean()) if "lat_p95" in a.columns else 0.0

    saf = compute_safety_events(str(run_dir))
    pred = {"ade": 0.0, "fde": 0.0, "n_samples": 0}
    if (run_dir / "pred_rollouts.csv").exists():
        pred = compute_pred_metrics(str(run_dir), dt=dt)
    comf = compute_comfort(str(run_dir), dt=dt)

    return dict(
        ttc_p5=ttc_p5,
        cancel_rate=cancel_rate,
        defer_rate=defer_rate,
        exec_rate=exec_rate,
        lane_changes=lane_changes,
        pdr_mean=pdr_mean,
        lat_p95_mean=lat_p95_mean,
        **saf,
        **pred,
        **comf
    )

def infer_variant(path_str: str) -> str:
    s = path_str.lower()
    for v in ["full","no_pred","no_intent","non_adapt","mobil_only"]:
        if v in s:
            return v
    return "unknown"

def main(out_root="out", dt=0.1):
    out_root = Path(out_root)
    runs = []
    for p in out_root.rglob("*_step7/*"):
        if (p / "actions.csv").exists():
            runs.append(p)

    rows = []
    for r in runs:
        try:
            k = compute_run(r, dt=dt)
            k["run_dir"] = str(r)
            k["variant"] = infer_variant(str(r))
            rows.append(k)
        except Exception:
            pass

    df = pd.DataFrame(rows)
    if df.empty:
        print("No runs found.")
        return

    df.to_csv(out_root / "runs_kpis_full.csv", index=False)

    summary = []
    cols = [
        "ttc_p5","cancel_rate","defer_rate","exec_rate","lane_changes",
        "pdr_mean","lat_p95_mean",
        "collisions","near_miss_ttc","near_miss_gap","min_ttc_global","min_gap_global",
        "ade","fde","a_rms","j_rms"
    ]

    for v, g in df.groupby("variant"):
        row = {"variant": v, "n_runs": int(len(g))}
        for c in cols:
            if c not in g.columns:
                row[f"{c}_mean"] = 0.0
                row[f"{c}_ci95"] = 0.0
                continue
            m, ci = mean_ci95(g[c].to_numpy())
            row[f"{c}_mean"] = m
            row[f"{c}_ci95"] = ci
        summary.append(row)

    summ = pd.DataFrame(summary).sort_values("variant")
    summ.to_csv(out_root / "summary_kpis_full_ci95.csv", index=False)
    print("[OK] wrote:", out_root / "runs_kpis_full.csv")
    print("[OK] wrote:", out_root / "summary_kpis_full_ci95.csv")

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--out_root", default="out")
    ap.add_argument("--dt", type=float, default=0.1)
    args = ap.parse_args()
    main(out_root=args.out_root, dt=args.dt)
