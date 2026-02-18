from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, List, Tuple
from python.sumo.neighborhood import LaneContext

@dataclass
class EgoState:
    veh_id: str
    x: float
    y: float
    v: float
    psi: float
    lane_id: str

class LaneMarkDetect:
    '''
    Simple, robust lane scoring:
    Score(lane) = w_leader * clip(leader_gap) + w_follower * clip(follower_gap) - marking_penalty
    '''
    def __init__(self, w_leader: float = 1.0, w_follower: float = 0.7, gap_clip: float = 80.0):
        self.w_leader = float(w_leader)
        self.w_follower = float(w_follower)
        self.gap_clip = float(gap_clip)

    def run(self, ego: EgoState, legal_adj: Dict[str, List[str]],
            lane_ctxs: Dict[str, LaneContext], penalty_by_lane: Dict[str, float]) -> Tuple[List[Tuple[str, float]], str]:
        cand = legal_adj.get(ego.lane_id, [ego.lane_id])
        ranked = []
        for ln in cand:
            ctx = lane_ctxs.get(ln)
            if ctx is None:
                leader_gap = 0.0
                follower_gap = 0.0
            else:
                leader_gap = min(self.gap_clip, max(0.0, float(ctx.leader_gap)))
                follower_gap = min(self.gap_clip, max(0.0, float(ctx.follower_gap)))
            pen = float(penalty_by_lane.get(ln, 0.0))
            score = self.w_leader * leader_gap + self.w_follower * follower_gap - pen
            ranked.append((ln, score))
        ranked.sort(key=lambda x: x[1], reverse=True)
        target = ranked[0][0] if ranked else ego.lane_id
        return ranked, target
