import os
import argparse
import subprocess

def run(cmd, env=None):
    print(" ".join(cmd))
    subprocess.check_call(cmd, env=env)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cfg", default="scenarios/configs/base.yaml")
    args = ap.parse_args()

    variants = [
        ("full",        dict(SAFE_NO_PRED="0", SAFE_NO_INTENT="0", SAFE_NON_ADAPT="0", SAFE_MOBIL_ONLY="0")),
        ("no_pred",     dict(SAFE_NO_PRED="1", SAFE_NO_INTENT="0", SAFE_NON_ADAPT="0", SAFE_MOBIL_ONLY="0")),
        ("no_intent",   dict(SAFE_NO_PRED="0", SAFE_NO_INTENT="1", SAFE_NON_ADAPT="0", SAFE_MOBIL_ONLY="0")),
        ("non_adapt",   dict(SAFE_NO_PRED="0", SAFE_NO_INTENT="0", SAFE_NON_ADAPT="1", SAFE_MOBIL_ONLY="0")),
        ("mobil_only",  dict(SAFE_NO_PRED="1", SAFE_NO_INTENT="1", SAFE_NON_ADAPT="1", SAFE_MOBIL_ONLY="1")),
    ]

    for tag, flags in variants:
        env = os.environ.copy()
        env.update(flags)
        env["SAFE_TAG"] = tag
        run(["python", "-m", "python.orchestrators.orchestrator_step7_laneaware_closedloop"], env=env)

    run(["python", "-m", "python.experiments.compute_kpis_full", "--out_root", "out", "--dt", "0.1"])

if __name__ == "__main__":
    main()
