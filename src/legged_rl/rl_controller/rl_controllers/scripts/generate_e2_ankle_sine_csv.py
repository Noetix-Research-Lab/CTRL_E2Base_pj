#!/usr/bin/env python3
"""Generate an E2 ankle pitch/roll sine CSV around the stand pose.

Only the four ankle joints are written so hip/knee/waist/arm hold the pose
captured at WALK entry. First and last samples match stand (sin(0)=0).
The output filename is derived from DURATION and DT
(e.g. ankle_sine_4s_500Hz.csv).

Re-run:

  python3 scripts/generate_e2_ankle_sine_csv.py
"""

from __future__ import annotations

import csv
import math
from pathlib import Path

DT = 0.002
DURATION = 10.0
# E2 stand pose (see RLControllerBase::standjointState_).
STAND = {
    "l_leg_ankle_pitch_joint": -0.1720,
    "l_leg_ankle_roll_joint": 0.0,
    "r_leg_ankle_pitch_joint": -0.1720,
    "r_leg_ankle_roll_joint": 0.0,
}
COLUMNS = ["time"] + list(STAND.keys())

# Conservative amplitudes vs URDF limits:
#   pitch [-1.0472, 0.4363], roll [-0.3491, 0.3491]
PITCH_AMP = 0.15   # rad, ~8.6 deg; range about [-0.322, -0.022]
ROLL_AMP = 0.12    # rad, ~6.9 deg; range about [-0.12, 0.12]
PITCH_PERIOD = 5.0  # two cycles in 10 s
ROLL_PERIOD = 10.0   # one cycle in 10 s


def sample(t: float) -> dict[str, float]:
    pitch = STAND["l_leg_ankle_pitch_joint"] + PITCH_AMP * math.sin(
        2.0 * math.pi * t / PITCH_PERIOD
    )
    roll = ROLL_AMP * math.sin(2.0 * math.pi * t / ROLL_PERIOD)
    return {
        "l_leg_ankle_pitch_joint": pitch,
        "r_leg_ankle_pitch_joint": pitch,
        "l_leg_ankle_roll_joint": roll,
        "r_leg_ankle_roll_joint": roll,
    }


def main() -> None:
    hz = int(round(1.0 / DT))
    filename = f"ankle_sine_{DURATION:g}s_{hz}Hz.csv"
    out = (
        Path(__file__).resolve().parents[1]
        / "trajectories"
        / "sim"
        / "ankle"
        / filename
    )
    out.parent.mkdir(parents=True, exist_ok=True)
    n = int(round(DURATION / DT)) + 1
    with out.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=COLUMNS)
        writer.writeheader()
        for i in range(n):
            t = i * DT
            row = sample(t)
            row["time"] = f"{t:.6f}"
            for name in STAND:
                row[name] = f"{row[name]:.6f}"
            writer.writerow(row)
    print(f"wrote {n} samples to {out}")


if __name__ == "__main__":
    main()
