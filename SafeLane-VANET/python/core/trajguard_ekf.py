from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, List, Tuple
import math

@dataclass
class Track:
    x: float
    y: float
    v: float
    psi: float
    t: float

@dataclass
class TrajPoint:
    x: float
    y: float
    v: float
    psi: float
    t: float

@dataclass
class RiskOut:
    min_ttc: float
    min_th: float
    gap_profile: List[float]

class TrajGuardEKF:
    '''
    Lightweight EKF-style tracker under noisy beacon updates.
    For reproducibility and simplicity, this uses a stable measurement blending scheme:
      x <- alpha*x_meas + (1-alpha)*x_pred
    alpha increases with PDR and decreases with age.
    '''
    def __init__(self, horizon_s: float = 2.0, dt_pred: float = 0.1):
        self.horizon_s = float(horizon_s)
        self.dt_pred = float(dt_pred)
        self.tracks: Dict[str, Track] = {}

    def _predict(self, tr: Track, now: float) -> Track:
        dt = max(0.0, now - tr.t)
        x = tr.x + tr.v * math.cos(tr.psi) * dt
        y = tr.y + tr.v * math.sin(tr.psi) * dt
        return Track(x=x, y=y, v=tr.v, psi=tr.psi, t=now)

    def step_track(self, veh_id: str, now: float, z_xyvpsi: Tuple[float,float,float,float], age: float, pdr: float, dt: float):
        zx, zy, zv, zpsi = map(float, z_xyvpsi)
        if veh_id not in self.tracks:
            self.tracks[veh_id] = Track(x=zx, y=zy, v=zv, psi=zpsi, t=now)
            return

        pred = self._predict(self.tracks[veh_id], now)

        # alpha: trust measurement more when PDR high and beacon is fresh
        freshness = max(0.0, min(1.0, 1.0 - age / max(1e-6, 1.0)))
        alpha = max(0.1, min(0.95, 0.25 + 0.6 * pdr * freshness))

        x = alpha * zx + (1 - alpha) * pred.x
        y = alpha * zy + (1 - alpha) * pred.y
        v = alpha * zv + (1 - alpha) * pred.v
        psi = alpha * zpsi + (1 - alpha) * pred.psi

        self.tracks[veh_id] = Track(x=x, y=y, v=v, psi=psi, t=now)

    def rollout(self, veh_id: str) -> List[TrajPoint]:
        if veh_id not in self.tracks:
            return []
        tr = self.tracks[veh_id]
        pts: List[TrajPoint] = []
        steps = int(self.horizon_s / self.dt_pred)
        for k in range(1, steps + 1):
            tt = tr.t + k * self.dt_pred
            x = tr.x + tr.v * math.cos(tr.psi) * k * self.dt_pred
            y = tr.y + tr.v * math.sin(tr.psi) * k * self.dt_pred
            pts.append(TrajPoint(x=x, y=y, v=tr.v, psi=tr.psi, t=tt))
        return pts

    def risk_vs_ego(self, ego_xyvpsi: Tuple[float,float,float,float], traj: List[TrajPoint]) -> RiskOut:
        ex, ey, ev, epsi = map(float, ego_xyvpsi)
        ux, uy = math.cos(epsi), math.sin(epsi)

        gap_profile: List[float] = []
        ttc_best = float("inf")
        th_best = float("inf")

        for p in traj:
            # longitudinal gap along ego heading
            gap = ux * (p.x - ex) + uy * (p.y - ey)
            gap_profile.append(float(gap))

            # closing rate approx: ego speed along heading - neighbor speed along heading
            nv_proj = p.v * math.cos(p.psi - epsi)
            closing = ev - nv_proj

            if gap > 0 and closing > 1e-3:
                ttc = gap / closing
                ttc_best = min(ttc_best, ttc)
            if gap > 0:
                th = gap / max(1e-3, ev)
                th_best = min(th_best, th)

        if ttc_best == float("inf"):
            ttc_best = 0.0
        if th_best == float("inf"):
            th_best = 0.0

        return RiskOut(min_ttc=float(ttc_best), min_th=float(th_best), gap_profile=gap_profile)
