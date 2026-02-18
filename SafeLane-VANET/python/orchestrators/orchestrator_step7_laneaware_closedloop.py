from __future__ import annotations
import os
import math
from pathlib import Path
import pandas as pd

from python.utils.config import load_yaml, make_run_dir, save_resolved_config
from python.utils.logger import CsvLogger
from python.utils.idmap import VehNodeMap

from python.sumo.traci_adapter import SumoAdapter
from python.sumo.lane_topology import lane_to_edge, build_legal_adj_same_edge
from python.sumo.neighborhood import build_lane_contexts

from python.core.lanemark_detect import LaneMarkDetect, EgoState
from python.core.neighbor_table import NeighborTable
from python.core.trajguard_ekf import TrajGuardEKF
from python.core.safemobil_comm import SafeMOBILComm, CommKpis, PredRisk
from python.comm.rx_intents import RxIntentRegistry
from python.comm.true_kpis import TrueKpiComputer

# Ablations (env)
ABL_NO_PRED   = os.getenv("SAFE_NO_PRED", "0") == "1"
ABL_NO_INTENT = os.getenv("SAFE_NO_INTENT", "0") == "1"
ABL_NON_ADAPT = os.getenv("SAFE_NON_ADAPT", "0") == "1"
ABL_MOBIL_ONLY= os.getenv("SAFE_MOBIL_ONLY", "0") == "1"
RUN_TAG       = os.getenv("SAFE_TAG", "step7")

def lane_idx_from_lane_id(lane_id: str) -> int:
    if isinstance(lane_id, str) and "_" in lane_id:
        try: return int(lane_id.rsplit("_", 1)[1])
        except: return -1
    return -1

def projected_gap(ex, ey, epsi, nx, ny):
    ux, uy = math.cos(epsi), math.sin(epsi)
    return ux * (nx - ex) + uy * (ny - ey)

def run(cfg_path: str):
    cfg = load_yaml(cfg_path)
    run_dir = make_run_dir(cfg["paths"]["out_root"], cfg["sim"]["exp_name"] + f"_{RUN_TAG}")
    save_resolved_config(cfg, run_dir)

    dt = float(cfg["sim"]["dt"])
    T  = float(cfg["sim"]["duration"])

    packets_csv = cfg["paths"].get("packets_csv", "out/ns3/packets.csv")
    tx_csv      = cfg["paths"].get("tx_csv", "out/ns3/tx.csv")

    # vehicle-node mapping (recommended)
    map_path = cfg["paths"].get("veh_to_node_csv", "out/ns3/veh_to_node.csv")
    vehmap = VehNodeMap(map_path) if Path(map_path).exists() else None

    # Load ns-3 RX stream
    pk = pd.read_csv(packets_csv)
    for c in ["t_tx","t_rx","sender_id","receiver_id","msg_type","dropped","x","y","v","psi","lane_idx","target_lane_idx"]:
        if c in pk.columns:
            pk[c] = pd.to_numeric(pk[c], errors="coerce")
    pk = pk.dropna(subset=["t_rx","sender_id","receiver_id","msg_type","x","y","v","psi","lane_idx","target_lane_idx"])
    if "dropped" in pk.columns:
        pk = pk[pk["dropped"] == 0]
    pk = pk.sort_values("t_rx").reset_index(drop=True)
    pk_idx = 0

    # Modules
    laner = LaneMarkDetect(**cfg["algo"]["lanemark"])
    ekf   = TrajGuardEKF(**cfg["algo"]["ekf"])
    ctrl  = SafeMOBILComm(**cfg["algo"]["controller"])
    intents = RxIntentRegistry(ttl_s=cfg["algo"]["controller"].get("coord_window", 0.4))
    kpi = TrueKpiComputer(packets_csv=packets_csv, tx_csv=tx_csv, window_s=cfg["algo"]["comm"]["window_s"])

    # State
    nb_tables = {}      # receiver_node -> NeighborTable
    last_exec = {}      # ego_id -> time
    lane_changes_count = 0

    # Logs
    actionlog = CsvLogger(run_dir / "actions.csv",
        ["t","ego_id","ego_node","curr_lane","target_lane","target_lane_idx",
         "action","reason","pdr","lat_p95","min_ttc","min_th","gap_min","coord_ok"])

    moblog = CsvLogger(run_dir / "mobility.csv",
        ["t","veh_id","x","y","v","psi","lane_id","road_id","is_av","node_id"])

    # Optional prediction rollout log
    predlog = CsvLogger(run_dir / "pred_rollouts.csv", ["t","ego_node","track_id","h","px","py"])

    sumo = SumoAdapter(cfg["paths"]["sumo_bin"], cfg["paths"]["sumo_cfg"], dt)
    sumo.start()

    t = 0.0
    try:
        while t < T:
            sumo.step()
            veh_ids = sumo.get_vehicle_ids()
            av_ids  = [vid for vid in veh_ids if vid.lower().startswith("av")]

            # Stream RX packets up to now
            while pk_idx < len(pk) and float(pk.loc[pk_idx, "t_rx"]) <= t:
                row = pk.loc[pk_idx]
                tx  = int(row["sender_id"])
                rx  = int(row["receiver_id"])
                msg_type = int(row["msg_type"])
                lane_idx = int(row["lane_idx"])
                tgt_lane = int(row["target_lane_idx"])

                nb = nb_tables.get(rx)
                if nb is None:
                    nb = NeighborTable()
                    nb_tables[rx] = nb

                nb.update(
                    sender_node=tx,
                    rx_t=float(row["t_rx"]),
                    x=float(row["x"]), y=float(row["y"]),
                    v=float(row["v"]), psi=float(row["psi"]),
                    lane_idx=lane_idx,
                    msg_type=msg_type,
                    target_lane_idx=tgt_lane
                )

                if msg_type == 2 and tgt_lane >= 0:
                    intents.update(sender=tx, t_rx=float(row["t_rx"]), target_lane_idx=tgt_lane)

                pk_idx += 1

            # Log mobility
            for vid in veh_ids:
                s = sumo.get_state(vid)
                node_id = (vehmap.node(vid) if vehmap else -1)
                moblog.write({"t": round(t,3), "veh_id": vid, "x": s.x, "y": s.y, "v": s.v, "psi": s.psi,
                              "lane_id": s.lane_id, "road_id": s.road_id,
                              "is_av": int(vid in av_ids), "node_id": node_id})

            # Decisions for each AV
            for ego_id in av_ids:
                s = sumo.get_state(ego_id)
                ego_node = vehmap.node(ego_id) if vehmap else 0

                # Lane scoring
                edge = lane_to_edge(s.lane_id)
                legal_adj = build_legal_adj_same_edge(edge)
                cand_lanes = legal_adj.get(s.lane_id, [s.lane_id])

                lane_ctxs = build_lane_contexts(ego_id, s.x, s.y, s.v, s.psi, cand_lanes)
                penalty_by_lane = {ln: 0.0 for ln in cand_lanes}

                ego = EgoState(veh_id=ego_id, x=s.x, y=s.y, v=s.v, psi=s.psi, lane_id=s.lane_id)
                ranked, target_lane = laner.run(ego, legal_adj, lane_ctxs, penalty_by_lane)
                target_lane_idx = lane_idx_from_lane_id(target_lane)

                # True comm KPIs (receiver-centric)
                link = kpi.get(receiver_id=ego_node, t_end=t)
                comm = CommKpis(pdr=link.pdr, lat_p95=link.lat_p95)

                if ABL_NON_ADAPT or ABL_MOBIL_ONLY:
                    comm = CommKpis(pdr=1.0, lat_p95=0.0)

                # Neighbor table update + tracking
                nb = nb_tables.get(ego_node)
                neighbor_nodes = set()
                if nb is not None:
                    nb.refresh_ages(t)
                    for tx_node, st in nb.items():
                        neighbor_nodes.add(tx_node)
                        if not (ABL_NO_PRED or ABL_MOBIL_ONLY):
                            ekf.step_track(
                                veh_id=str(tx_node),
                                now=t,
                                z_xyvpsi=(st.x, st.y, st.v, st.psi),
                                age=st.age,
                                pdr=comm.pdr,
                                dt=dt
                            )

                # Coordination from received LCI only
                if ABL_NO_INTENT or ABL_MOBIL_ONLY or target_lane == s.lane_id:
                    coord_ok = True
                else:
                    conflict = intents.has_conflict(t, neighbor_nodes, target_lane_idx, ego_node)
                    coord_ok = not conflict

                # Leader/follower on target lane using lane_idx match
                lead_track = None
                best_ahead = float("inf")

                if nb is not None and target_lane_idx >= 0:
                    for tx_node, st in nb.items():
                        if st.lane_idx != target_lane_idx:
                            continue
                        g = projected_gap(s.x, s.y, s.psi, st.x, st.y)
                        if g > 0 and g < best_ahead:
                            best_ahead = g
                            lead_track = str(tx_node)

                # Risk
                if ABL_NO_PRED or ABL_MOBIL_ONLY:
                    if best_ahead == float("inf"):
                        min_ttc, min_th, gap_min = 0.0, 0.0, 0.0
                    else:
                        gap_min = float(best_ahead)
                        closing = max(0.1, s.v)
                        min_ttc = gap_min / closing
                        min_th  = gap_min / max(0.1, s.v)
                else:
                    if lead_track and lead_track in ekf.tracks:
                        traj = ekf.rollout(lead_track)

                        # Log a short rollout for prediction metrics
                        H = min(len(traj), 10)
                        for h in range(H):
                            predlog.write({"t": round(t,3), "ego_node": ego_node, "track_id": lead_track,
                                           "h": h, "px": traj[h].x, "py": traj[h].y})

                        riskL = ekf.risk_vs_ego((s.x, s.y, s.v, s.psi), traj)
                        min_ttc = float(riskL.min_ttc)
                        min_th  = float(riskL.min_th)
                        gap_min = float(min(riskL.gap_profile)) if riskL.gap_profile else float("inf")
                    else:
                        min_ttc, min_th, gap_min = 0.0, 0.0, 0.0

                if ABL_MOBIL_ONLY:
                    coord_ok = True

                last_t = last_exec.get(ego_id, -1e9)
                decision = ctrl.decide(
                    now=t,
                    last_exec_t=last_t,
                    risk=PredRisk(min_ttc=min_ttc, min_th=min_th, gap_min=gap_min),
                    comm=comm,
                    coord_ok=coord_ok
                )

                if decision.action == "EXECUTE" and target_lane != s.lane_id:
                    sumo.change_lane(ego_id, target_lane, duration=1.0)
                    last_exec[ego_id] = t
                    lane_changes_count += 1

                actionlog.write({
                    "t": round(t,3),
                    "ego_id": ego_id,
                    "ego_node": ego_node,
                    "curr_lane": s.lane_id,
                    "target_lane": target_lane,
                    "target_lane_idx": target_lane_idx,
                    "action": decision.action,
                    "reason": decision.reason,
                    "pdr": comm.pdr,
                    "lat_p95": comm.lat_p95,
                    "min_ttc": min_ttc,
                    "min_th": min_th,
                    "gap_min": gap_min,
                    "coord_ok": int(coord_ok)
                })

            t += dt

    finally:
        sumo.close()
        actionlog.close()
        moblog.close()
        predlog.close()

    print(f"[OK] Step 7 lane-aware run written to: {run_dir}")

if __name__ == "__main__":
    run("scenarios/configs/base.yaml")
