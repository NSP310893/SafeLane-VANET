# SafeLane-VANET

**SafeLane-VANET: Lane Determination, V2X-Aware Collision Avoidance, and Non-Autonomous Vehicle Motion Prediction for Safe Lane Changes**

This repository provides a **simulation-only, reproducible research codebase** for mixed traffic lane-change safety in VANETs using:

- **SUMO** for microscopic vehicle mobility and lane topology,
- **ns-3 (v3.46.1)** for IEEE **802.11p / WAVE** V2V communications (trace-driven),
- **Python** for orchestration, lane scoring, mixed-fleet coordination, motion prediction, and KPI reporting.

The implementation is aligned with the paper’s methodology:
- **LaneMark-Detect**: lane availability + target lane scoring using lane markings/topology and local gaps.
- **TrajGuard-EKF**: tracking + short-horizon prediction of non-autonomous (non-AV) vehicles from periodic beacons.
- **SafeMOBIL-Comm**: V2X-aware lane-change controller with intent messaging and network-quality adaptive safety gating.

> The focus is **inter-vehicular communication between autonomous and non-autonomous vehicles**: non-AV beacons improve AV tracking/prediction, and AV intent messages support coordination prior to lane change.

---

## Repository structure

```
SafeLane-VANET/
├─ ns3/
│  └─ scratch/
│     └─ safelane_trace_wave.cc          # ns-3.46.1 trace-driven 802.11p CAM + intent messaging
├─ python/
│  ├─ orchestrators/
│  │  └─ orchestrator_step7_laneaware_closedloop.py
│  ├─ core/
│  │  ├─ lanemark_detect.py              # LaneMark-Detect
│  │  ├─ trajguard_ekf.py                # TrajGuard-EKF (lightweight)
│  │  └─ safemobil_comm.py               # SafeMOBIL-Comm controller
│  ├─ comm/
│  │  ├─ mobility_export.py              # SUMO -> ns-3 mobility trace
│  │  ├─ intent_export.py                # actions.csv -> ns-3 intent.csv
│  │  ├─ rx_intents.py                   # received intent registry for coordination
│  │  └─ true_kpis.py                    # true PDR/latency from ns-3 tx/rx logs
│  ├─ sumo/
│  │  ├─ traci_adapter.py                # SUMO control interface
│  │  ├─ lane_topology.py                # simple legality graph on same edge
│  │  └─ neighborhood.py                 # leader/follower gaps on candidate lanes
│  ├─ experiments/
│  │  ├─ run_step7_variants.py           # ablations
│  │  ├─ compute_kpis_full.py            # KPI aggregation (CI95)
│  │  ├─ make_paper_tables.py            # CSV tables for MS-Word
│  │  ├─ sumo_safety_events.py           # collisions/near-miss extraction
│  │  ├─ prediction_metrics.py           # ADE/FDE from rollouts
│  │  └─ comfort_metrics.py              # accel/jerk RMS
│  └─ utils/
│     ├─ config.py
│     └─ logger.py
├─ scenarios/
│  └─ configs/
│     └─ base.yaml                       # experiment config (paths + parameters)
├─ scripts/
│  └─ run_all.sh                         # convenience run script (edit paths)
├─ requirements.txt
└─ README.md
```

---

## System requirements

### Required software
1. **SUMO** (recommended: 1.16+)
   - Must include Python **TraCI** bindings (installed with SUMO).
2. **ns-3 v3.46.1**
   - Required to run the trace-driven 802.11p/WAVE program.
3. **Python 3.10+**
   - Used for orchestration and KPI generation.

### OS
- Tested-oriented for Linux (Ubuntu recommended). Windows is possible with WSL2.

---

## Installation

### 1) Python environment
From the repository root:

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

> If `import traci` fails, ensure SUMO is installed correctly and that SUMO’s Python tools directory is in `PYTHONPATH`.

---

### 2) SUMO setup
Install SUMO and verify:

```bash
sumo --version
python -c "import traci; print('TraCI OK')"
```

Place (or generate) your SUMO scenario files under `scenarios/sumo/`, and point to your `.sumocfg` in:

- `scenarios/configs/base.yaml` → `paths.sumo_cfg`

---

### 3) ns-3.46.1 setup
Copy the scratch program into your ns-3 tree:

```bash
cp ns3/scratch/safelane_trace_wave.cc <path-to-ns-3.46.1>/scratch/
```

Build ns-3:

```bash
cd <path-to-ns-3.46.1>
./waf configure
./waf build
```

---

## Data flow (what runs first)

1) **SUMO** generates lane-level mobility (via TraCI) → `out/ns3/mobility_ns3.csv`  
2) **Python closed-loop** produces lane-change decisions → `out/.../actions.csv`  
3) Extract **intent schedule** from `actions.csv` → `out/ns3/intent.csv`  
4) **ns-3** replays mobility + intent and logs packet deliveries → `out/ns3/packets.csv`, `out/ns3/tx.csv`  
5) **Python closed-loop** runs again and uses ns-3 logs to compute **true communication KPIs** and safety gating  
6) KPI scripts produce **SCI tables** (CSV) for the paper

> This matches a **trace-driven ns-3-first** communication workflow while maintaining a closed-loop controller in Python.

---

## Configuration

Edit:

### `scenarios/configs/base.yaml`
Key fields:

- `paths.sumo_cfg`: path to your SUMO `.sumocfg`
- `sim.dt`, `sim.duration`: simulation time step and duration
- `algo.controller`: TTC/TH thresholds, adaptation parameters
- `paths.packets_csv`, `paths.tx_csv`: ns-3 logs produced in Step (4)

---

## Running the full pipeline (recommended)

### Step A — Export mobility from SUMO (for ns-3)
```bash
python -m python.comm.mobility_export   --sumo_cfg scenarios/sumo/your.sumocfg   --out_dir out/ns3   --dt 0.1   --sim_time 120
```

Outputs:
- `out/ns3/mobility_ns3.csv`
- `out/ns3/veh_to_node.csv`
- `out/ns3/mobility_full.csv`

---

### Step B — Run one controller pass to generate actions (for intent schedule)
```bash
python -m python.orchestrators.orchestrator_step7_laneaware_closedloop
```

This creates:
- `out/SafeLaneVANET_<tag>/<timestamp>/actions.csv`

---

### Step C — Convert actions to intent schedule for ns-3
Point to the `actions.csv` from Step B:
```bash
python -m python.comm.intent_export   --actions_csv out/SafeLaneVANET_full/<timestamp>/actions.csv   --out_csv out/ns3/intent.csv
```

---

### Step D — Run ns-3 trace-driven 802.11p messaging
Run from your ns-3.46.1 directory:

```bash
./waf --run "safelane_trace_wave   --mobPath=out/ns3/mobility_ns3.csv   --intentPath=out/ns3/intent.csv   --rxLogPath=out/ns3/packets.csv   --txLogPath=out/ns3/tx.csv   --nNodes=20   --simTime=120   --hz=10"
```

Outputs:
- `out/ns3/packets.csv` (per-RX packet deliveries with latency)
- `out/ns3/tx.csv` (TX attempts)

---

### Step E — Run ablations (full, no_pred, no_intent, non_adapt, mobil_only)
```bash
python -m python.experiments.run_step7_variants --cfg scenarios/configs/base.yaml
```

Each run produces:
- `actions.csv`, `mobility.csv`, `pred_rollouts.csv` (optional), `config_resolved.yaml`

---

### Step F — Compute SCI KPIs and tables
```bash
python -m python.experiments.compute_kpis_full --out_root out --dt 0.1
python -m python.experiments.make_paper_tables --out_root out
```

Outputs (paper-ready CSV tables):
- `out/Table_Safety.csv`
- `out/Table_Efficiency.csv`
- `out/Table_Communication.csv`
- `out/Table_Prediction.csv`
- `out/Table_Comfort.csv`

---

## Key outputs and meaning

### `actions.csv`
Per-tick decisions for AVs:
- `curr_lane`, `target_lane`, `action` (EXECUTE/DEFER/CANCEL)
- network KPIs: `pdr`, `lat_p95`
- predictive risk: `min_ttc`, `min_th`, `gap_min`
- coordination signal: `coord_ok`

### `packets.csv` (ns-3)
Per received packet:
- `t_tx`, `t_rx`, `sender_id`, `receiver_id`, `msg_type` (1=CAM/BSM, 2=LCI-CAM)
- kinematic payload: `x,y,v,psi,lane_idx,target_lane_idx`

### Paper tables
Generated as **mean ± 95% CI** across runs.

---

## Reproducibility checklist (for SCI submission)
- All run configurations are stored as: `config_resolved.yaml`
- Use fixed random seeds by:
  - controlling SUMO route generation seeds (in your SUMO scenario)
  - repeating runs with distinct seeds and reporting CI95
- Keep logs consistent:
  - `mobility.csv` + `actions.csv` + ns-3 `packets.csv/tx.csv`

---

## Troubleshooting

### 1) `import traci` fails
- Verify SUMO installation
- Ensure SUMO’s Python tools are discoverable:
  - Example (Linux):
    ```bash
    export PYTHONPATH=$PYTHONPATH:/usr/share/sumo/tools
    ```

### 2) Lane IDs don’t follow `edge_0`, `edge_1` format
Update the parsing helper:
- `lane_idx_from_lane_id()` in `python/orchestrators/orchestrator_step7_laneaware_closedloop.py`

### 3) Node count mismatch between SUMO and ns-3
- `mobility_export.py` maps vehicles to nodes using trailing digits (e.g., `AV0`, `car12`)
- Ensure `--nNodes` in ns-3 is at least the maximum assigned node id + 1.

### 4) `out/ns3/intent.csv` empty
- Ensure `actions.csv` contains `target_lane`
- Ensure some actions are EXECUTE or DEFER (intent triggers in `intent_export.py`)

---

## Citing and paper artifacts
This repo produces:
- Communication metrics: PDR and latency from ns-3
- Safety metrics: near-miss via TTC/gap; collisions if enabled in SUMO
- Prediction metrics: ADE/FDE via logged rollouts
- Comfort metrics: accel/jerk RMS

---

