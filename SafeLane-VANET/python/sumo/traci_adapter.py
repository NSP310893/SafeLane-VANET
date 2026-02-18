from __future__ import annotations
from dataclasses import dataclass
from typing import List
import os

@dataclass
class VehicleState:
    veh_id: str
    x: float
    y: float
    v: float
    psi: float
    lane_id: str
    road_id: str

class SumoAdapter:
    def __init__(self, sumo_bin: str, sumo_cfg: str, step_length: float):
        self.sumo_bin = sumo_bin
        self.sumo_cfg = sumo_cfg
        self.step_length = float(step_length)
        self._traci = None

    def start(self):
        try:
            import traci
        except Exception as e:
            raise RuntimeError("TraCI not available. Install SUMO and ensure python can import traci.") from e
        self._traci = traci
        cmd = [self.sumo_bin, "-c", self.sumo_cfg, "--step-length", str(self.step_length)]
        # quiet by default
        cmd += ["--no-warnings", "true"]
        self._traci.start(cmd)

    def step(self):
        self._traci.simulationStep()

    def get_vehicle_ids(self) -> List[str]:
        return list(self._traci.vehicle.getIDList())

    def get_state(self, veh_id: str) -> VehicleState:
        x, y = self._traci.vehicle.getPosition(veh_id)
        v = float(self._traci.vehicle.getSpeed(veh_id))
        psi = float(self._traci.vehicle.getAngle(veh_id)) * 3.141592653589793 / 180.0
        lane_id = str(self._traci.vehicle.getLaneID(veh_id))
        road_id = str(self._traci.vehicle.getRoadID(veh_id))
        return VehicleState(veh_id=veh_id, x=float(x), y=float(y), v=v, psi=psi, lane_id=lane_id, road_id=road_id)

    def change_lane(self, veh_id: str, lane_id: str, duration: float = 1.0):
        # SUMO uses lane index for changeLane; we use changeLaneRelative if possible.
        # We use a safe fallback: setLaneChangeMode and changeLane.
        try:
            lane_index = int(lane_id.rsplit("_", 1)[1])
            self._traci.vehicle.changeLane(veh_id, lane_index, duration)
        except Exception:
            # if lane_id not in expected format, attempt direct changeLane with derived index 0
            try:
                self._traci.vehicle.changeLane(veh_id, 0, duration)
            except Exception:
                pass

    def close(self):
        try:
            self._traci.close()
        except Exception:
            pass
