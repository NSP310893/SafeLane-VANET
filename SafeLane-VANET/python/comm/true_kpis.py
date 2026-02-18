import pandas as pd
import numpy as np
from dataclasses import dataclass

@dataclass
class RxKpis:
    pdr: float
    lat_p95: float
    cbr: float

class TrueKpiComputer:
    '''
    True PDR = delivered / attempted, using ns-3 logs:
      - tx.csv: attempted
      - packets.csv: delivered (per receiver)
    '''
    def __init__(self, packets_csv: str, tx_csv: str, window_s: float = 1.0, msg_type_filter=(1,2)):
        self.rx = pd.read_csv(packets_csv)
        self.tx = pd.read_csv(tx_csv)
        self.window_s = float(window_s)
        self.msg_types = set(msg_type_filter)

        self.rx = self.rx[self.rx["msg_type"].isin(self.msg_types)].copy()
        self.tx = self.tx[self.tx["msg_type"].isin(self.msg_types)].copy()

        self.rx["t_rx"] = pd.to_numeric(self.rx["t_rx"], errors="coerce")
        self.rx["t_tx"] = pd.to_numeric(self.rx["t_tx"], errors="coerce")
        self.tx["t_tx"] = pd.to_numeric(self.tx["t_tx"], errors="coerce")

        self.rx["lat"] = self.rx["t_rx"] - self.rx["t_tx"]

        self.rx.sort_values("t_rx", inplace=True)
        self.tx.sort_values("t_tx", inplace=True)

        self._rx_by_rcv = {int(r): g for r, g in self.rx.groupby("receiver_id")}
        self._tx_by_snd = {int(s): g for s, g in self.tx.groupby("sender_id")}

    def get(self, receiver_id: int, t_end: float) -> RxKpis:
        r = int(receiver_id)
        t0 = float(t_end) - self.window_s

        if r not in self._rx_by_rcv:
            return RxKpis(pdr=0.0, lat_p95=0.0, cbr=0.0)

        rxw = self._rx_by_rcv[r]
        rxw = rxw[(rxw["t_rx"] > t0) & (rxw["t_rx"] <= t_end)]
        delivered = len(rxw)

        # conservative approximation: attempted from senders seen delivering in window
        senders = rxw["sender_id"].dropna().unique().astype(int).tolist()
        attempted = 0
        for s in senders:
            if s in self._tx_by_snd:
                txs = self._tx_by_snd[s]
                txw = txs[(txs["t_tx"] > t0) & (txs["t_tx"] <= t_end)]
                attempted += len(txw)

        pdr = (delivered / attempted) if attempted > 0 else 0.0

        lat = rxw["lat"].replace([np.inf, -np.inf], np.nan).dropna().to_numpy()
        lat_p95 = float(np.percentile(lat, 95)) if lat.size else 0.0

        return RxKpis(pdr=float(max(0.0, min(1.0, pdr))), lat_p95=lat_p95, cbr=0.0)
