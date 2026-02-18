from __future__ import annotations
from typing import Dict, List

def lane_to_edge(lane_id: str) -> str:
    # SUMO lane ids often: edgeId_0, edgeId_1, ...
    if "_" in lane_id:
        return lane_id.rsplit("_", 1)[0]
    return lane_id

def build_legal_adj_same_edge(edge_id: str, n_lanes: int | None = None) -> Dict[str, List[str]]:
    '''
    Builds a simple adjacency graph for lanes on the SAME edge:
    lane i can stay, and can move to i-1 or i+1 if exists.

    If n_lanes is None, we try to query SUMO via traci (if connected).
    '''
    if n_lanes is None:
        try:
            import traci
            lanes = traci.edge.getLaneNumber(edge_id)
            n_lanes = int(lanes)
        except Exception:
            n_lanes = 1

    adj: Dict[str, List[str]] = {}
    for i in range(n_lanes):
        ln = f"{edge_id}_{i}"
        cand = [ln]
        if i - 1 >= 0:
            cand.append(f"{edge_id}_{i-1}")
        if i + 1 < n_lanes:
            cand.append(f"{edge_id}_{i+1}")
        adj[ln] = cand
    return adj
