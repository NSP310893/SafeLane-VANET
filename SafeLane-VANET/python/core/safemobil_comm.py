from __future__ import annotations
from dataclasses import dataclass
from typing import Optional

@dataclass
class CommKpis:
    pdr: float
    lat_p95: float

@dataclass
class PredRisk:
    min_ttc: float
    min_th: float
    gap_min: float

@dataclass
class Decision:
    action: str   # EXECUTE / DEFER / CANCEL
    reason: str

class SafeMOBILComm:
    '''
    Communication-aware guarded lane-change controller.
    - Feasibility is assumed handled upstream by lane scoring.
    - Safety gate uses min TTC / TH from prediction or instantaneous proxy.
    - Coordination gate uses LCI conflicts within a listen window.
    - Adaptation: if PDR low / latency high -> require stricter TTC/TH.
    '''
    def __init__(self,
                 ttc_min: float = 2.0,
                 th_min: float = 1.2,
                 gap_min: float = 5.0,
                 cooldown_s: float = 2.0,
                 pdr_min: float = 0.85,
                 lat_max: float = 0.15,
                 strict_factor: float = 1.35):
        self.ttc_min = float(ttc_min)
        self.th_min = float(th_min)
        self.gap_min = float(gap_min)
        self.cooldown_s = float(cooldown_s)
        self.pdr_min = float(pdr_min)
        self.lat_max = float(lat_max)
        self.strict_factor = float(strict_factor)

    def decide(self, now: float, last_exec_t: float, risk: PredRisk, comm: CommKpis, coord_ok: bool) -> Decision:
        # cooldown
        if (now - last_exec_t) < self.cooldown_s:
            return Decision("DEFER", "cooldown")

        # adapt thresholds
        ttc_thr = self.ttc_min
        th_thr  = self.th_min
        gap_thr = self.gap_min

        degraded = (comm.pdr < self.pdr_min) or (comm.lat_p95 > self.lat_max)
        if degraded:
            ttc_thr *= self.strict_factor
            th_thr  *= self.strict_factor
            gap_thr *= self.strict_factor

        # safety gate
        if risk.gap_min < gap_thr or risk.min_ttc < ttc_thr or risk.min_th < th_thr:
            return Decision("CANCEL", "safety_gate")

        # coordination gate
        if not coord_ok:
            return Decision("DEFER", "coordination_conflict")

        return Decision("EXECUTE", "ok")
