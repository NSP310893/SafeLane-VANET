from pathlib import Path
import pandas as pd
import numpy as np

def compute_comfort(run_dir: str, dt=0.1):
    run_dir = Path(run_dir)
    mob = pd.read_csv(run_dir / "mobility.csv")
    mob["t"] = pd.to_numeric(mob["t"], errors="coerce")
    mob["v"] = pd.to_numeric(mob["v"], errors="coerce")

    if "is_av" in mob.columns:
        mob = mob[mob["is_av"] == 1]

    mob = mob.dropna(subset=["t","veh_id","v"]).sort_values(["veh_id","t"])
    if mob.empty:
        return {"a_rms": 0.0, "j_rms": 0.0}

    mob["dv"] = mob.groupby("veh_id")["v"].diff()
    mob["a"] = mob["dv"] / float(dt)
    mob["da"] = mob.groupby("veh_id")["a"].diff()
    mob["j"] = mob["da"] / float(dt)

    a = mob["a"].replace([np.inf,-np.inf], np.nan).dropna().to_numpy()
    j = mob["j"].replace([np.inf,-np.inf], np.nan).dropna().to_numpy()

    a_rms = float(np.sqrt(np.mean(a*a))) if a.size else 0.0
    j_rms = float(np.sqrt(np.mean(j*j))) if j.size else 0.0
    return {"a_rms": a_rms, "j_rms": j_rms}
