#!/usr/bin/env python3
"""Generate an E2 waist roll/pitch sine CSV around the stand pose.

Only waist_roll and waist_pitch are written so legs, yaw, and arms hold the pose
captured at WALK entry. First and last samples match stand (sin(0)=0).

The output filename is derived from DURATION and DT (e.g. waist_sine_10s_500Hz.csv).

Re-run:

  python3 scripts/generate_e2_waist_sine_csv.py
"""

from __future__ import annotations

import csv
import math
from pathlib import Path

DT = 0.002
DURATION = 10.0
# E2 stand pose (see RLControllerBase::standjointState_).
STAND = {
    "waist_roll_joint": 0.0,
    "waist_pitch_joint": 0.0,
}
COLUMNS = ["time"] + list(STAND.keys())

AMP_DEG = 10.0
AMP = math.radians(AMP_DEG)
PITCH_PERIOD = 5.0  # two cycles in 10 s
ROLL_PERIOD = 10.0   # one cycle in 10 s


def sample(t: float) -> dict[str, float]:
    return {
        "waist_pitch_joint": AMP * math.sin(2.0 * math.pi * t / PITCH_PERIOD),
        "waist_roll_joint": AMP * math.sin(2.0 * math.pi * t / ROLL_PERIOD),
    }


def main() -> None:
    hz = int(round(1.0 / DT))
    filename = f"waist_sine_{DURATION:g}s_{hz}Hz.csv"
    out = (
        Path(__file__).resolve().parents[1]
        / "trajectories"
        / "sim"
        / "waist"
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
    print(f"wrote {n} samples to {out} (amp={AMP_DEG:.0f} deg = {AMP:.6f} rad)")


if __name__ == "__main__":
    main()
