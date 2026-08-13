#!/usr/bin/env python3
"""Record and compare synchronous/asynchronous controller timing metrics."""

import argparse
import csv
import json
import math
import statistics
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Sequence


TOPICS = {
    "control": "/data_analysis/control_loop_timing",
    "inference": "/data_analysis/rl_inference_timing",
    "action": "/data_analysis/rl_action_timing",
}

EXPECTED_COLUMNS = {
    "control": [
        "sequence", "start_ns", "period_us", "work_us", "sleep_budget_us",
        "period_error_us", "deadline_missed", "threshold_exceeded", "dropped_total",
    ],
    "inference": [
        "sequence", "policy", "epoch", "observation_ns", "start_ns", "completion_ns",
        "inference_us", "queue_us", "end_to_end_us", "success", "async_mode",
        "dropped_total",
    ],
    "action": [
        "sequence", "policy", "epoch", "observation_ns", "completion_ns", "consumed_ns",
        "completion_to_consume_us", "observation_to_consume_us", "async_mode", "dropped_total",
    ],
}


def percentile(sorted_values: Sequence[float], fraction: float) -> Optional[float]:
    if not sorted_values:
        return None
    position = (len(sorted_values) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return float(sorted_values[lower])
    weight = position - lower
    return float(sorted_values[lower] * (1.0 - weight) + sorted_values[upper] * weight)


def describe(values: Sequence[float]) -> Dict[str, Optional[float]]:
    finite = sorted(float(value) for value in values if math.isfinite(float(value)))
    if not finite:
        return {key: None for key in ("mean", "stdev", "min", "p50", "p90", "p99", "p999", "max")}
    return {
        "mean": statistics.fmean(finite),
        "stdev": statistics.pstdev(finite) if len(finite) > 1 else 0.0,
        "min": finite[0],
        "p50": percentile(finite, 0.50),
        "p90": percentile(finite, 0.90),
        "p99": percentile(finite, 0.99),
        "p999": percentile(finite, 0.999),
        "max": finite[-1],
    }


def maximum(rows: Sequence[Dict[str, float]], field: str) -> float:
    return max((float(row[field]) for row in rows), default=0.0)


def summarize(label: str, rows: Dict[str, List[Dict[str, float]]], metadata: Dict[str, object]) -> Dict[str, object]:
    control = rows["control"]
    inference = rows["inference"]
    action = rows["action"]

    control_summary = {
        "count": len(control),
        "period_us": describe([row["period_us"] for row in control]),
        "work_us": describe([row["work_us"] for row in control]),
        "sleep_budget_us": describe([row["sleep_budget_us"] for row in control]),
        "period_error_us": describe([row["period_error_us"] for row in control]),
        "deadline_miss_count": int(sum(row["deadline_missed"] >= 0.5 for row in control)),
        "deadline_miss_rate": (
            sum(row["deadline_missed"] >= 0.5 for row in control) / len(control) if control else None
        ),
        "threshold_exceeded_count": int(sum(row["threshold_exceeded"] >= 0.5 for row in control)),
        "threshold_exceeded_rate": (
            sum(row["threshold_exceeded"] >= 0.5 for row in control) / len(control) if control else None
        ),
        "dropped_total": maximum(control, "dropped_total"),
    }
    inference_summary = {
        "count": len(inference),
        "success_count": int(sum(row["success"] >= 0.5 for row in inference)),
        "async_mode_values": sorted(set(int(row["async_mode"] >= 0.5) for row in inference)),
        "inference_us": describe([row["inference_us"] for row in inference]),
        "queue_us": describe([row["queue_us"] for row in inference]),
        "end_to_end_us": describe([row["end_to_end_us"] for row in inference]),
        "dropped_total": maximum(inference, "dropped_total"),
    }
    action_summary = {
        "count": len(action),
        "async_mode_values": sorted(set(int(row["async_mode"] >= 0.5) for row in action)),
        "completion_to_consume_us": describe([row["completion_to_consume_us"] for row in action]),
        "observation_to_consume_us": describe([row["observation_to_consume_us"] for row in action]),
        "dropped_total": maximum(action, "dropped_total"),
    }
    return {
        "schema_version": 1,
        "label": label,
        "metadata": metadata,
        "control": control_summary,
        "inference": inference_summary,
        "action": action_summary,
    }


class TimingRecorder:
    def __init__(self, warmup_seconds: float):
        self.warmup_seconds = warmup_seconds
        self.recording_start = 0.0
        self.lock = threading.Lock()
        self.rows: Dict[str, List[Dict[str, float]]] = {name: [] for name in TOPICS}
        self.schema_errors: List[str] = []

    def begin(self) -> None:
        self.recording_start = time.monotonic() + self.warmup_seconds

    def callback(self, message, stream: str) -> None:
        if time.monotonic() < self.recording_start:
            return
        expected = EXPECTED_COLUMNS[stream]
        columns = message.layout.dim[1].size if len(message.layout.dim) > 1 else len(expected)
        label = message.layout.dim[1].label if len(message.layout.dim) > 1 else ""
        names = label.split(",") if label else expected
        if columns != len(expected) or names != expected or len(message.data) % columns != 0:
            with self.lock:
                self.schema_errors.append(
                    f"{stream}: expected {expected}, received columns={columns}, names={names}, values={len(message.data)}"
                )
            return

        receipt_wall_ns = time.time_ns()
        decoded: List[Dict[str, float]] = []
        for offset in range(0, len(message.data), columns):
            row = {name: float(message.data[offset + index]) for index, name in enumerate(names)}
            row["receipt_wall_ns"] = float(receipt_wall_ns)
            decoded.append(row)
        with self.lock:
            self.rows[stream].extend(decoded)


def write_csv(path: Path, stream: str, rows: Sequence[Dict[str, float]]) -> None:
    columns = ["receipt_wall_ns"] + EXPECTED_COLUMNS[stream]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)


def record_command(args: argparse.Namespace) -> int:
    try:
        import rospy
        from std_msgs.msg import Float64MultiArray
    except ImportError as error:
        print(f"error: ROS Python packages are unavailable: {error}", file=sys.stderr)
        return 2

    rospy.init_node("rl_timing_recorder", anonymous=True, disable_signals=True)
    recorder = TimingRecorder(args.warmup)
    subscribers = [
        rospy.Subscriber(topic, Float64MultiArray, recorder.callback, callback_args=name, queue_size=100)
        for name, topic in TOPICS.items()
    ]

    deadline = time.monotonic() + args.wait_timeout
    while time.monotonic() < deadline and not rospy.is_shutdown():
        missing = [name for name, subscriber in zip(TOPICS, subscribers) if subscriber.get_num_connections() == 0]
        if not missing:
            break
        time.sleep(0.05)
    else:
        missing = [name for name, subscriber in zip(TOPICS, subscribers) if subscriber.get_num_connections() == 0]
        print(f"error: timing publishers not available: {', '.join(missing)}", file=sys.stderr)
        print("enable both enable_timing_metrics parameters before launch", file=sys.stderr)
        for subscriber in subscribers:
            subscriber.unregister()
        return 3

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    session_dir = args.output_dir / f"{args.label}_{timestamp}"
    session_dir.mkdir(parents=True, exist_ok=False)
    metadata = {
        "label": args.label,
        "duration_seconds": args.duration,
        "warmup_seconds": args.warmup,
        "start_wall_ns": time.time_ns(),
        "topics": TOPICS,
    }
    print(f"Recording {args.label!r}: warmup={args.warmup:.1f}s duration={args.duration:.1f}s")
    print(f"Output: {session_dir}")
    recorder.begin()

    end_time = time.monotonic() + args.warmup + args.duration
    try:
        while time.monotonic() < end_time and not rospy.is_shutdown():
            time.sleep(min(0.1, max(0.0, end_time - time.monotonic())))
    except KeyboardInterrupt:
        print("Interrupted; writing collected samples.")
    finally:
        for subscriber in subscribers:
            subscriber.unregister()

    with recorder.lock:
        rows = {name: list(values) for name, values in recorder.rows.items()}
        schema_errors = list(recorder.schema_errors)
    observed_modes = sorted(set(int(row["async_mode"] >= 0.5) for row in rows["inference"]))
    expected_mode = None
    if args.label.lower() in ("sync", "synchronous"):
        expected_mode = 0
    elif args.label.lower() in ("async", "asynchronous"):
        expected_mode = 1
    if len(observed_modes) > 1:
        schema_errors.append(f"recording contains mixed async_mode values: {observed_modes}")
    elif expected_mode is not None and observed_modes and observed_modes[0] != expected_mode:
        schema_errors.append(
            f"label {args.label!r} expects async_mode={expected_mode}, observed {observed_modes[0]}"
        )
    metadata["end_wall_ns"] = time.time_ns()
    metadata["schema_errors"] = schema_errors
    for stream, values in rows.items():
        write_csv(session_dir / f"{stream}.csv", stream, values)
    summary = summarize(args.label, rows, metadata)
    (session_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    print(f"Samples: control={len(rows['control'])}, inference={len(rows['inference'])}, action={len(rows['action'])}")
    if schema_errors:
        print(f"warning: {len(schema_errors)} message schema errors; inspect summary.json", file=sys.stderr)
    print(f"Summary: {session_dir / 'summary.json'}")
    return 0 if all(rows.values()) and not schema_errors else 4


def nested_value(payload: Dict[str, object], path: str) -> Optional[float]:
    value: object = payload
    for part in path.split("."):
        if not isinstance(value, dict) or part not in value:
            return None
        value = value[part]
    return float(value) if isinstance(value, (int, float)) else None


COMPARISON_METRICS = [
    ("Control period p99 (us)", "control.period_us.p99"),
    ("Control period max (us)", "control.period_us.max"),
    ("Control work p99 (us)", "control.work_us.p99"),
    ("Control work max (us)", "control.work_us.max"),
    ("Control deadline miss rate", "control.deadline_miss_rate"),
    ("Inference mean (us)", "inference.inference_us.mean"),
    ("Inference p99 (us)", "inference.inference_us.p99"),
    ("Inference max (us)", "inference.inference_us.max"),
    ("Inference queue p99 (us)", "inference.queue_us.p99"),
    ("Observation->complete p99 (us)", "inference.end_to_end_us.p99"),
    ("Complete->consume p99 (us)", "action.completion_to_consume_us.p99"),
    ("Observation->consume p99 (us)", "action.observation_to_consume_us.p99"),
]


def find_summary(path: Path) -> Path:
    return path / "summary.json" if path.is_dir() else path


def format_number(value: Optional[float]) -> str:
    if value is None:
        return "n/a"
    if abs(value) < 0.01 and value != 0.0:
        return f"{value:.3e}"
    return f"{value:.3f}"


def compare_command(args: argparse.Namespace) -> int:
    baseline_path = find_summary(args.baseline)
    candidate_path = find_summary(args.candidate)
    baseline = json.loads(baseline_path.read_text(encoding="utf-8"))
    candidate = json.loads(candidate_path.read_text(encoding="utf-8"))
    baseline_label = str(baseline.get("label", "baseline"))
    candidate_label = str(candidate.get("label", "candidate"))

    lines = [
        f"# Timing comparison: {baseline_label} vs {candidate_label}",
        "",
        f"| Metric | {baseline_label} | {candidate_label} | Delta | Change |",
        "| --- | ---: | ---: | ---: | ---: |",
    ]
    for title, path in COMPARISON_METRICS:
        base = nested_value(baseline, path)
        candidate_value = nested_value(candidate, path)
        delta = candidate_value - base if base is not None and candidate_value is not None else None
        change = delta / base * 100.0 if delta is not None and base not in (None, 0.0) else None
        change_text = f"{format_number(change)}%" if change is not None else "n/a"
        lines.append(
            f"| {title} | {format_number(base)} | {format_number(candidate_value)} | "
            f"{format_number(delta)} | {change_text} |"
        )
    report = "\n".join(lines) + "\n"
    print(report, end="")
    if args.output:
        args.output.write_text(report, encoding="utf-8")
        print(f"Report written to {args.output}", file=sys.stderr)
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    record = subparsers.add_parser("record", help="record timing topics into CSV and summary JSON")
    record.add_argument("--label", required=True, help="session label, normally sync or async")
    record.add_argument("--duration", type=float, default=60.0, help="measured duration in seconds")
    record.add_argument("--warmup", type=float, default=5.0, help="discarded warmup duration in seconds")
    record.add_argument("--wait-timeout", type=float, default=10.0, help="publisher wait timeout")
    record.add_argument("--output-dir", type=Path, default=Path("timing_results"))
    record.set_defaults(handler=record_command)

    compare = subparsers.add_parser("compare", help="compare two recording summary files/directories")
    compare.add_argument("baseline", type=Path, help="baseline session directory or summary.json")
    compare.add_argument("candidate", type=Path, help="candidate session directory or summary.json")
    compare.add_argument("--output", type=Path, help="optional Markdown report path")
    compare.set_defaults(handler=compare_command)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if hasattr(args, "duration") and (args.duration <= 0 or args.warmup < 0 or args.wait_timeout <= 0):
        print("error: duration/wait-timeout must be positive and warmup non-negative", file=sys.stderr)
        return 2
    return args.handler(args)


if __name__ == "__main__":
    sys.exit(main())
