from dataclasses import dataclass
from typing import Dict, Set

@dataclass
class IntentMsg:
    t_rx: float
    target_lane_idx: int

class RxIntentRegistry:
    def __init__(self, ttl_s: float = 0.5):
        self.ttl = float(ttl_s)
        self.last: Dict[int, IntentMsg] = {}

    def update(self, sender: int, t_rx: float, target_lane_idx: int):
        self.last[int(sender)] = IntentMsg(float(t_rx), int(target_lane_idx))

    def prune(self, now: float):
        dead = [s for s, it in self.last.items() if (now - it.t_rx) > self.ttl]
        for s in dead:
            del self.last[s]

    def has_conflict(self, now: float, neighbor_nodes: Set[int], target_lane_idx: int, ego_node: int) -> bool:
        self.prune(now)
        for s, it in self.last.items():
            if s == ego_node:
                continue
            if s in neighbor_nodes and it.target_lane_idx == int(target_lane_idx):
                return True
        return False
