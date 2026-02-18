from __future__ import annotations
import argparse
from pathlib import Path
import pandas as pd

def lane_index_from_lane_id(lane_id: str) -> int:
    if isinstance(lane_id, str) and "_" in lane_id:
        try:
            return int(lane_id.rsplit("_", 1)[1])
        except Exception:
            return -1
    return -1

def export_from_sumo(sumo_cfg: str, sumo_bin: str, dt: float, sim_time: float, out_dir: str, n_nodes: int = 0):
    '''
    Runs SUMO and exports per-step mobility trace:
      mobility_ns3.csv: t,node_id,x,y,v,psi,lane_idx
    Also emits veh_to_node.csv mapping.

    This expects vehicle IDs include an integer suffix (e.g., AV0, car12). If not,
    you should provide your own mapping in this script.
    '''
    try:
        import traci
    except Exception as e:
        raise RuntimeError("TraCI not available. Install SUMO and ensure python can import traci.") from e

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    cmd = [sumo_bin, "-c", sumo_cfg, "--step-length", str(dt), "--no-warnings", "true"]
    traci.start(cmd)

    rows = []
    seen = {}

    def veh_to_node(vid: str) -> int:
        # default: parse trailing digits
        num = ""
        for ch in reversed(vid):
            if ch.isdigit(): num = ch + num
            else: break
        if num:
            return int(num)
        # fallback allocate new
        if vid not in seen:
            seen[vid] = len(seen)
        return seen[vid]

    steps = int(sim_time / dt)
    for k in range(steps):
        t = k * dt
        traci.simulationStep()
        vids = traci.vehicle.getIDList()
        for vid in vids:
            x, y = traci.vehicle.getPosition(vid)
            v = float(traci.vehicle.getSpeed(vid))
            psi = float(traci.vehicle.getAngle(vid)) * 3.141592653589793 / 180.0
            lane_id = str(traci.vehicle.getLaneID(vid))
            lane_idx = lane_index_from_lane_id(lane_id)
            node_id = veh_to_node(vid)
            rows.append([t, node_id, x, y, v, psi, lane_idx, vid, lane_id])

    traci.close()

    df = pd.DataFrame(rows, columns=["t","node_id","x","y","v","psi","lane_idx","veh_id","lane_id"])
    df.sort_values(["t","node_id"], inplace=True)

    df_out = df[["t","node_id","x","y","v","psi","lane_idx"]]
    df_out.to_csv(out_dir / "mobility_ns3.csv", index=False)

    m = df[["veh_id","node_id"]].drop_duplicates().sort_values("node_id")
    m.to_csv(out_dir / "veh_to_node.csv", index=False)

    # Optional: store full mobility for reference
    df.to_csv(out_dir / "mobility_full.csv", index=False)

    print("[OK] wrote:", out_dir / "mobility_ns3.csv")
    print("[OK] wrote:", out_dir / "veh_to_node.csv")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sumo_cfg", required=True, help="SUMO .sumocfg path")
    ap.add_argument("--sumo_bin", default="sumo", help="SUMO binary (sumo or sumo-gui)")
    ap.add_argument("--dt", type=float, default=0.1)
    ap.add_argument("--sim_time", type=float, default=120.0)
    ap.add_argument("--out_dir", default="out/ns3")
    args = ap.parse_args()
    export_from_sumo(args.sumo_cfg, args.sumo_bin, args.dt, args.sim_time, args.out_dir)

if __name__ == "__main__":
    main()
