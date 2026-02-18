from __future__ import annotations
import argparse
from pathlib import Path
import pandas as pd

def lane_idx(lane_id: str) -> int:
    if isinstance(lane_id, str) and "_" in lane_id:
        try: return int(lane_id.rsplit("_", 1)[1])
        except: return -1
    return -1

def export_intents(actions_csv: str, out_csv: str):
    df = pd.read_csv(actions_csv)
    if "target_lane" not in df.columns:
        raise ValueError("actions.csv must contain target_lane column")

    df = df[df["target_lane"].notna()].copy()
    df["target_lane_idx"] = df["target_lane"].apply(lane_idx)
    df = df[df["target_lane_idx"] >= 0]

    # Intent trigger: DEFER (coordination_conflict) and EXECUTE
    key = df[df["action"].isin(["DEFER","EXECUTE"])].copy()
    if "ego_node" in key.columns:
        key.rename(columns={"ego_node":"sender_id"}, inplace=True)
    elif "ego_id" in key.columns:
        # fallback parse digits from ego_id
        key["sender_id"] = key["ego_id"].astype(str).str.extract(r"(\d+)").astype(int)
    else:
        raise ValueError("actions.csv must include ego_node or ego_id")

    key["t_tx"] = pd.to_numeric(key["t"], errors="coerce")
    out = key[["t_tx","sender_id","target_lane_idx"]].dropna().sort_values(["t_tx","sender_id"])
    Path(out_csv).parent.mkdir(parents=True, exist_ok=True)
    out.to_csv(out_csv, index=False)
    print(f"[OK] wrote intent schedule: {out_csv} rows={len(out)}")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--actions_csv", required=True, help="Path to actions.csv from a run")
    ap.add_argument("--out_csv", default="out/ns3/intent.csv")
    args = ap.parse_args()
    export_intents(args.actions_csv, args.out_csv)

if __name__ == "__main__":
    main()
