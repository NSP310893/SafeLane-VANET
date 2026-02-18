from pathlib import Path
import argparse
import pandas as pd

def fmt(m, ci):
    return f"{m:.3f} ± {ci:.3f}"

def main(out_root="out"):
    out_root = Path(out_root)
    summ = pd.read_csv(out_root / "summary_kpis_full_ci95.csv")

    def pick(cols):
        out = pd.DataFrame()
        out["Variant"] = summ["variant"]
        for name, (mcol, cicol) in cols.items():
            out[name] = [fmt(m, ci) for m, ci in zip(summ[mcol], summ[cicol])]
        out["Runs"] = summ["n_runs"]
        return out

    safety = pick({
        "TTC p5 (s)": ("ttc_p5_mean", "ttc_p5_ci95"),
        "Collisions": ("collisions_mean", "collisions_ci95"),
        "Near-miss (TTC<τ)": ("near_miss_ttc_mean", "near_miss_ttc_ci95"),
        "Near-miss (gap<g)": ("near_miss_gap_mean", "near_miss_gap_ci95"),
        "Min TTC (s)": ("min_ttc_global_mean", "min_ttc_global_ci95"),
        "Min Gap (m)": ("min_gap_global_mean", "min_gap_global_ci95"),
    })
    safety.to_csv(out_root / "Table_Safety.csv", index=False)

    eff = pick({
        "Lane Changes": ("lane_changes_mean","lane_changes_ci95"),
        "Execute Rate": ("exec_rate_mean","exec_rate_ci95"),
        "Defer Rate": ("defer_rate_mean","defer_rate_ci95"),
        "Cancel Rate": ("cancel_rate_mean","cancel_rate_ci95"),
    })
    eff.to_csv(out_root / "Table_Efficiency.csv", index=False)

    comm = pick({
        "PDR": ("pdr_mean_mean","pdr_mean_ci95"),
        "Latency p95 (s)": ("lat_p95_mean_mean","lat_p95_mean_ci95"),
    })
    comm.to_csv(out_root / "Table_Communication.csv", index=False)

    pred = pick({
        "ADE (m)": ("ade_mean","ade_ci95"),
        "FDE (m)": ("fde_mean","fde_ci95"),
    })
    pred.to_csv(out_root / "Table_Prediction.csv", index=False)

    comfort = pick({
        "Accel RMS": ("a_rms_mean","a_rms_ci95"),
        "Jerk RMS": ("j_rms_mean","j_rms_ci95"),
    })
    comfort.to_csv(out_root / "Table_Comfort.csv", index=False)

    print("[OK] Wrote paper tables to:", out_root)

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--out_root", default="out")
    args = ap.parse_args()
    main(out_root=args.out_root)
