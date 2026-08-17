#!/usr/bin/env python3
"""Generate an E2 arm-wave CSV around the stand pose.

Legs and waist are omitted so the controller holds the pose captured at WALK
entry. The output filename is derived from DURATION and DT
(e.g. arm_wave_4s_500Hz.csv).

Re-run:

  python3 scripts/generate_e2_arm_wave_csv.py
"""

from __future__ import annotations

import csv
import math
from pathlib import Path

DT = 0.002
DURATION = 4.0
# E2 stand pose (see RLControllerBase::standjointState_).
STAND = {
    "l_arm_shoulder_pitch_joint": 0.0,
    "l_arm_shoulder_roll_joint": 0.2618,
    "l_arm_shoulder_yaw_joint": 0.0,
    "l_arm_elbow_joint": 0.0,
    "r_arm_shoulder_pitch_joint": 0.0,
    "r_arm_shoulder_roll_joint": -0.2618,
    "r_arm_shoulder_yaw_joint": 0.0,
    "r_arm_elbow_joint": 0.0,
}
COLUMNS = ["time"] + list(STAND.keys())


def sample(t: float) -> dict[str, float]:
    row = dict(STAND)
    # Two pitch cycles on the left arm; right arm stays at stand.
    row["l_arm_shoulder_pitch_joint"] = 0.35 * math.sin(2.0 * math.pi * t / 2.0)
    row["l_arm_shoulder_roll_joint"] = 0.2618 + 0.08 * math.sin(2.0 * math.pi * t / 4.0)
    row["l_arm_elbow_joint"] = 0.40 * 0.5 * (1.0 - math.cos(2.0 * math.pi * t / 4.0))
    return row


def main() -> None:
    hz = int(round(1.0 / DT))
    filename = f"arm_wave_{DURATION:g}s_{hz}Hz.csv"
    out = Path(__file__).resolve().parents[1] / "trajectories" / "sim" / "arm" / filename
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
