#!/usr/bin/env bash
set -euo pipefail

CFG=${1:-scenarios/configs/base.yaml}

echo "[1] Export mobility from SUMO -> out/ns3"
python -m python.comm.mobility_export --sumo_cfg $(python -c "import yaml;print(yaml.safe_load(open('$CFG'))['paths']['sumo_cfg'])") --out_dir out/ns3

echo "[2] Run a single Step-7 closed-loop to produce actions.csv (for intent generation)"
python -m python.orchestrators.orchestrator_step7_laneaware_closedloop

echo "[3] Generate intent.csv from latest run (update path if needed)"
echo "NOTE: Please point --actions_csv to the actions.csv you want. This is a placeholder."
# python -m python.comm.intent_export --actions_csv out/SafeLaneVANET_full/XXXX/actions.csv --out_csv out/ns3/intent.csv

echo "[4] Run ns-3 trace wave (do this in ns-3 directory after copying scratch file)"
echo "./waf --run \"safelane_trace_wave --mobPath=out/ns3/mobility_ns3.csv --intentPath=out/ns3/intent.csv --rxLogPath=out/ns3/packets.csv --txLogPath=out/ns3/tx.csv --nNodes=20 --simTime=120 --hz=10\""

echo "[5] Run variants + compute KPIs"
python -m python.experiments.run_step7_variants --cfg "$CFG"

echo "[6] Make paper tables"
python -m python.experiments.make_paper_tables --out_root out
