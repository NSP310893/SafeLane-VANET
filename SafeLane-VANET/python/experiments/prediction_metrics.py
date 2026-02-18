from pathlib import Path
import pandas as pd
import numpy as np

def compute_pred_metrics(run_dir: str, dt=0.1):
    run_dir = Path(run_dir)
    pred = pd.read_csv(run_dir / "pred_rollouts.csv")
    mob  = pd.read_csv(run_dir / "mobility.csv")

    for c in ["t","h","px","py"]:
        pred[c] = pd.to_numeric(pred[c], errors="coerce")
    mob["t"] = pd.to_numeric(mob["t"], errors="coerce")
    mob["x"] = pd.to_numeric(mob["x"], errors="coerce")
    mob["y"] = pd.to_numeric(mob["y"], errors="coerce")
    if "node_id" in mob.columns:
        mob["node_id"] = pd.to_numeric(mob["node_id"], errors="coerce")
    else:
        mob["node_id"] = mob["veh_id"].astype(str).str.extract(r"(\d+)").astype(int)

    pred["track_node"] = pred["track_id"].astype(str).str.extract(r"(\d+)").astype(int)
    pred["t_gt"] = pred["t"] + pred["h"] * float(dt)

    pred["t_gt_r"] = pred["t_gt"].round(3)
    mob["t_r"] = mob["t"].round(3)

    gt = mob[["t_r","node_id","x","y"]].dropna().rename(columns={"t_r":"t_gt_r","x":"gx","y":"gy"})

    m = pred.merge(gt, left_on=["t_gt_r","track_node"], right_on=["t_gt_r","node_id"], how="inner")
    if m.empty:
        return {"ade": 0.0, "fde": 0.0, "n_samples": 0}

    m["err"] = np.sqrt((m["px"] - m["gx"])**2 + (m["py"] - m["gy"])**2)
    ade = float(m["err"].mean())

    idx = m.groupby(["t","track_node"])["h"].idxmax()
    fde = float(m.loc[idx, "err"].mean()) if len(idx) else 0.0

    return {"ade": ade, "fde": fde, "n_samples": int(len(m))}
