import pandas as pd

class VehNodeMap:
    def __init__(self, map_csv: str):
        df = pd.read_csv(map_csv)
        self.veh_to_node = {str(r["veh_id"]): int(r["node_id"]) for _, r in df.iterrows()}

    def node(self, veh_id: str) -> int:
        if veh_id not in self.veh_to_node:
            raise KeyError(f"veh_id not in map: {veh_id}")
        return self.veh_to_node[veh_id]
