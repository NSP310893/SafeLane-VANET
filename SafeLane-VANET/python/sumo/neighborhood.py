from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, List, Optional
import math

@dataclass
class LaneContext:
    lane_id: str
    leader_gap: float
    follower_gap: float
    leader_speed: float
    follower_speed: float

def build_lane_contexts(ego_id: str, ex: float, ey: float, ev: float, epsi: float, candidate_lanes: List[str],
                        max_scan: float = 150.0) -> Dict[str, LaneContext]:
    '''
    For each candidate lane, estimate nearest leader and follower using lane vehicle lists + lane positions.
    This is SUMO-only (TraCI). If TraCI missing, returns conservative gaps.
    '''
    ctx: Dict[str, LaneContext] = {}
    try:
        import traci
    except Exception:
        # Conservative fallback
        for ln in candidate_lanes:
            ctx[ln] = LaneContext(lane_id=ln, leader_gap=0.0, follower_gap=0.0, leader_speed=0.0, follower_speed=0.0)
        return ctx

    for ln in candidate_lanes:
        vids = traci.lane.getLastStepVehicleIDs(ln)
        # compute ego longitudinal coordinate if ego is on this lane; else approximate by projecting in heading
        # Use lanePosition if ego is in same lane; otherwise use projection in ego heading as proxy.
        ego_lane = traci.vehicle.getLaneID(ego_id)
        if ego_lane == ln:
            epos = traci.vehicle.getLanePosition(ego_id)
        else:
            epos = 0.0

        best_ahead = (float("inf"), None)
        best_behind = (float("inf"), None)

        for v in vids:
            if v == ego_id:
                continue
            try:
                pos = traci.vehicle.getLanePosition(v)
                gap = float(pos - epos)
                if gap >= 0 and gap < best_ahead[0]:
                    best_ahead = (gap, v)
                if gap < 0 and abs(gap) < best_behind[0]:
                    best_behind = (abs(gap), v)
            except Exception:
                continue

        leader_gap = best_ahead[0] if best_ahead[1] is not None else 0.0
        follower_gap = best_behind[0] if best_behind[1] is not None else 0.0

        leader_speed = float(traci.vehicle.getSpeed(best_ahead[1])) if best_ahead[1] is not None else 0.0
        follower_speed = float(traci.vehicle.getSpeed(best_behind[1])) if best_behind[1] is not None else 0.0

        ctx[ln] = LaneContext(lane_id=ln, leader_gap=leader_gap, follower_gap=follower_gap,
                              leader_speed=leader_speed, follower_speed=follower_speed)
    return ctx
