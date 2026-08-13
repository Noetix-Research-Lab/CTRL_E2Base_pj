#!/usr/bin/env python3
"""Check motor/joint position, velocity, and effort limits in a ROS bag."""

import argparse
import csv
import importlib
import math
import sys
import xml.etree.ElementTree as ET
from collections import Counter
from dataclasses import dataclass
from pathlib import Path

yaml = None
rosbag = None


HARDWARE_JOINT_ORDER = (
    [f"arm_l{i}_joint" for i in range(1, 6)]
    + [f"leg_l{i}_joint" for i in range(1, 7)]
    + [f"arm_r{i}_joint" for i in range(1, 6)]
    + [f"leg_r{i}_joint" for i in range(1, 7)]
    + ["waist_1_joint", "waist_2_joint"]
)

DEFAULT_HARDWARE_TOPICS = {
    "pos": "/legged_e2_hw/data_analysis/motor_pos",
    "vel": "/legged_e2_hw/data_analysis/motor_vel",
    "effort": "/legged_e2_hw/data_analysis/motor_torque",
}

DEFAULT_CONTROLLER_TOPICS = {
    "pos": "/data_analysis/real_joint_pos",
    "vel": "/data_analysis/real_joint_vel",
    "effort": "/data_analysis/real_torque",
}


@dataclass(frozen=True)
class TopicSpec:
    topic: str
    field: str
    order_name: str


@dataclass
class TopicStats:
    messages: int = 0
    checked_values: int = 0
    skipped_values: int = 0
    dims: Counter = None

    def __post_init__(self):
        if self.dims is None:
            self.dims = Counter()


@dataclass
class LimitStats:
    topic: str
    field: str
    joint: str
    index: int
    lower: float
    upper: float
    min_value: float = math.inf
    max_value: float = -math.inf
    low_count: int = 0
    high_count: int = 0
    nonfinite_count: int = 0
    first_violation_time: float = None
    first_violation_value: float = None
    max_margin: float = 0.0

    @property
    def violation_count(self):
        return self.low_count + self.high_count + self.nonfinite_count

    def add_value(self, value, stamp, tolerance):
        if not math.isfinite(value):
            self.nonfinite_count += 1
            self._record_violation(stamp, value, math.inf)
            return

        self.min_value = min(self.min_value, value)
        self.max_value = max(self.max_value, value)

        if value < self.lower - tolerance:
            self.low_count += 1
            self._record_violation(stamp, value, self.lower - value)
        elif value > self.upper + tolerance:
            self.high_count += 1
            self._record_violation(stamp, value, value - self.upper)

    def _record_violation(self, stamp, value, margin):
        if self.first_violation_time is None:
            self.first_violation_time = stamp
            self.first_violation_value = value
        self.max_margin = max(self.max_margin, margin)


def default_paths():
    package_dir = Path(__file__).resolve().parents[1]
    legged_robot_dir = package_dir.parents[1]
    legged_rl_dir = package_dir.parents[2]
    return {
        "ethercat_config": package_dir / "config" / "ethercat_config.yaml",
        "urdf": legged_robot_dir / "e2" / "legged_e2_description" / "urdf" / "e2_23dof.urdf",
        "joint_config": legged_rl_dir / "rl_controller" / "rl_controllers" / "config" / "e2_23dof_ac.yaml",
    }


def load_yaml(path):
    with Path(path).open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def parse_urdf_limits(path):
    tree = ET.parse(path)
    limits = {}
    for joint in tree.getroot().findall("joint"):
        name = joint.get("name")
        limit = joint.find("limit")
        if not name or limit is None:
            continue
        limits[name] = {
            "pos": (float(limit.get("lower", "-inf")), float(limit.get("upper", "inf"))),
            "vel": (-abs(float(limit.get("velocity", "inf"))), abs(float(limit.get("velocity", "inf")))),
            "effort": (-abs(float(limit.get("effort", "inf"))), abs(float(limit.get("effort", "inf")))),
        }
    return limits


def load_controller_joint_order(path):
    data = load_yaml(path)
    try:
        return data["LeggedRobotCfg"]["robot"]["control_joint_names"]
    except (TypeError, KeyError) as exc:
        raise SystemExit(f"Cannot find LeggedRobotCfg.robot.control_joint_names in {path}") from exc


def expand_motor_type_limits(config):
    table = {}

    def add(prefix, count, type_key):
        motor_types = config[type_key]
        for i in range(count):
            joint = f"{prefix}{i + 1}_joint"
            motor_type = motor_types[i]
            table[joint] = {
                "vel": (
                    float(config["motor_min_velocity"][motor_type]),
                    float(config["motor_max_velocity"][motor_type]),
                ),
                "effort": (
                    float(config["motor_min_torque"][motor_type]),
                    float(config["motor_max_torque"][motor_type]),
                ),
            }

    add("arm_l", 5, "left_arm_motor_type")
    add("arm_r", 5, "right_arm_motor_type")
    add("leg_l", 6, "left_leg_motor_type")
    add("leg_r", 6, "right_leg_motor_type")

    for i, motor_type in enumerate(config["waist_motor_type"]):
        table[f"waist_{i + 1}_joint"] = {
            "vel": (
                float(config["motor_min_velocity"][motor_type]),
                float(config["motor_max_velocity"][motor_type]),
            ),
            "effort": (
                float(config["motor_min_torque"][motor_type]),
                float(config["motor_max_torque"][motor_type]),
            ),
        }

    return table


def build_limits(urdf_limits, motor_limits, effort_source):
    limits = {}
    for joint in sorted(set(urdf_limits) | set(motor_limits)):
        limits[joint] = {}
        if joint in urdf_limits:
            limits[joint]["pos"] = urdf_limits[joint]["pos"]
            if effort_source == "urdf":
                limits[joint]["effort"] = urdf_limits[joint]["effort"]
        if joint in motor_limits:
            limits[joint]["vel"] = motor_limits[joint]["vel"]
            if effort_source == "ethercat":
                limits[joint]["effort"] = motor_limits[joint]["effort"]
    return limits


def extract_values(msg, field):
    if hasattr(msg, "data"):
        return list(msg.data)
    if field == "pos" and hasattr(msg, "position"):
        return list(msg.position)
    if field == "vel" and hasattr(msg, "velocity"):
        return list(msg.velocity)
    if field == "effort" and hasattr(msg, "effort"):
        return list(msg.effort)
    return None


def make_topic_specs(args):
    if args.pos_topic or args.vel_topic or args.effort_topic:
        specs = []
        for topic in args.pos_topic:
            specs.append(TopicSpec(topic, "pos", args.custom_order))
        for topic in args.vel_topic:
            specs.append(TopicSpec(topic, "vel", args.custom_order))
        for topic in args.effort_topic:
            specs.append(TopicSpec(topic, "effort", args.custom_order))
        return specs

    specs = []
    if args.topic_set in ("hardware", "both"):
        specs.extend(TopicSpec(topic, field, "hardware") for field, topic in DEFAULT_HARDWARE_TOPICS.items())
    if args.topic_set in ("controller", "both"):
        specs.extend(TopicSpec(topic, field, "controller") for field, topic in DEFAULT_CONTROLLER_TOPICS.items())
    return specs


def check_bag(args, limits, joint_orders, topic_specs):
    with rosbag.Bag(args.bag) as bag:
        topic_info = bag.get_type_and_topic_info()[1]
    available_topics = set(topic_info.keys())
    selected_specs = [spec for spec in topic_specs if spec.topic in available_topics]
    missing_topics = [spec.topic for spec in topic_specs if spec.topic not in available_topics]

    if not selected_specs:
        interesting = sorted(t for t in available_topics if "data_analysis" in t or "joint" in t)
        print("No requested/default topics were found in the bag.", file=sys.stderr)
        if interesting:
            print("Available related topics:", file=sys.stderr)
            for topic in interesting:
                print(f"  {topic}", file=sys.stderr)
        return {}, {}, missing_topics

    stats_by_key = {}
    topic_stats = {spec.topic: TopicStats() for spec in selected_specs}
    spec_by_topic = {spec.topic: spec for spec in selected_specs}
    start_time = None

    with rosbag.Bag(args.bag) as bag:
        try:
            start_time = bag.get_start_time()
        except rosbag.ROSBagException:
            start_time = None

        for topic, msg, stamp in bag.read_messages(topics=[spec.topic for spec in selected_specs]):
            spec = spec_by_topic[topic]
            values = extract_values(msg, spec.field)
            topic_stat = topic_stats[topic]
            topic_stat.messages += 1

            if values is None:
                topic_stat.skipped_values += 1
                continue

            topic_stat.dims[len(values)] += 1
            joint_order = joint_orders[spec.order_name]
            rel_stamp = stamp.to_sec() - start_time if start_time is not None else stamp.to_sec()

            for index, value in enumerate(values):
                if index >= len(joint_order):
                    topic_stat.skipped_values += 1
                    continue

                joint = joint_order[index]
                field_limits = limits.get(joint, {}).get(spec.field)
                if field_limits is None:
                    topic_stat.skipped_values += 1
                    continue

                key = (topic, spec.field, joint, index)
                if key not in stats_by_key:
                    stats_by_key[key] = LimitStats(
                        topic=topic,
                        field=spec.field,
                        joint=joint,
                        index=index,
                        lower=field_limits[0],
                        upper=field_limits[1],
                    )
                stats_by_key[key].add_value(float(value), rel_stamp, args.tolerance)
                topic_stat.checked_values += 1

    return stats_by_key, topic_stats, missing_topics


def write_csv(path, stats):
    rows = sorted(stats.values(), key=lambda item: (item.topic, item.index, item.field))
    with Path(path).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "topic",
                "field",
                "index",
                "joint",
                "lower",
                "upper",
                "min",
                "max",
                "low_count",
                "high_count",
                "nonfinite_count",
                "first_violation_time_s",
                "first_violation_value",
                "max_margin",
            ]
        )
        for item in rows:
            writer.writerow(
                [
                    item.topic,
                    item.field,
                    item.index,
                    item.joint,
                    item.lower,
                    item.upper,
                    finite_or_empty(item.min_value),
                    finite_or_empty(item.max_value),
                    item.low_count,
                    item.high_count,
                    item.nonfinite_count,
                    none_or_float(item.first_violation_time),
                    none_or_float(item.first_violation_value),
                    item.max_margin,
                ]
            )


def finite_or_empty(value):
    return "" if value in (math.inf, -math.inf) else f"{value:.9g}"


def none_or_float(value):
    return "" if value is None else f"{value:.9g}"


def print_report(stats, topic_stats, missing_topics, max_report):
    print("Checked topics:")
    for topic, item in sorted(topic_stats.items()):
        dim_text = ", ".join(f"{dim}x{count}" for dim, count in sorted(item.dims.items())) or "unknown"
        print(
            f"  {topic}: messages={item.messages}, dims={dim_text}, "
            f"checked_values={item.checked_values}, skipped_values={item.skipped_values}"
        )

    if missing_topics:
        unique_missing = sorted(set(missing_topics))
        print("\nTopics not present in bag:")
        for topic in unique_missing:
            print(f"  {topic}")

    violations = [item for item in stats.values() if item.violation_count > 0]
    if not violations:
        print("\nPASS: no limit violations found.")
        return False

    violations.sort(key=lambda item: (-item.max_margin, item.topic, item.index))
    print(f"\nFAIL: found {len(violations)} joint/field limit violations.")
    print("Top violations:")
    for item in violations[:max_report]:
        value_range = f"[{finite_or_empty(item.min_value)}, {finite_or_empty(item.max_value)}]"
        limit_range = f"[{item.lower:.9g}, {item.upper:.9g}]"
        print(
            f"  {item.topic} {item.field}[{item.index}] {item.joint}: "
            f"values={value_range}, limit={limit_range}, "
            f"low={item.low_count}, high={item.high_count}, nonfinite={item.nonfinite_count}, "
            f"first_t={none_or_float(item.first_violation_time)}s, "
            f"first_value={none_or_float(item.first_violation_value)}, "
            f"max_margin={item.max_margin:.9g}"
        )

    if len(violations) > max_report:
        print(f"  ... {len(violations) - max_report} more; use --max-report to show more.")

    return True


def parse_args():
    paths = default_paths()
    parser = argparse.ArgumentParser(
        description="Read a ROS bag and check motor/joint pos, velocity, and effort limits."
    )
    parser.add_argument("bag", help="Path to .bag file")
    parser.add_argument("--urdf", default=str(paths["urdf"]), help="URDF used for position limits")
    parser.add_argument(
        "--ethercat-config",
        default=str(paths["ethercat_config"]),
        help="ethercat_config.yaml used for motor velocity/effort limits",
    )
    parser.add_argument(
        "--joint-config",
        default=str(paths["joint_config"]),
        help="Controller YAML used for /data_analysis/* joint order",
    )
    parser.add_argument(
        "--topic-set",
        choices=("hardware", "controller", "both"),
        default="both",
        help="Default topic set to check when no explicit topics are given",
    )
    parser.add_argument(
        "--effort-source",
        choices=("ethercat", "urdf"),
        default="ethercat",
        help="Use motor torque limits from ethercat_config.yaml or effort limits from URDF",
    )
    parser.add_argument("--tolerance", type=float, default=1e-6, help="Allowed numerical tolerance")
    parser.add_argument("--max-report", type=int, default=50, help="Maximum violations to print")
    parser.add_argument("--csv", help="Optional CSV output path for all checked joint stats")
    parser.add_argument(
        "--no-fail-on-violation",
        action="store_true",
        help="Return exit code 0 even if violations are found",
    )

    parser.add_argument("--pos-topic", action="append", default=[], help="Custom position topic")
    parser.add_argument("--vel-topic", action="append", default=[], help="Custom velocity topic")
    parser.add_argument("--effort-topic", action="append", default=[], help="Custom effort/torque topic")
    parser.add_argument(
        "--custom-order",
        choices=("hardware", "controller"),
        default="hardware",
        help="Joint order for custom topics",
    )
    return parser.parse_args()


def import_runtime_dependencies():
    global yaml, rosbag
    try:
        yaml = importlib.import_module("yaml")
    except ImportError as exc:
        raise SystemExit("PyYAML is required: sudo apt install python3-yaml") from exc

    try:
        rosbag = importlib.import_module("rosbag")
    except ImportError as exc:
        raise SystemExit("rosbag is required. Run this script in a sourced ROS environment.") from exc


def main():
    args = parse_args()
    import_runtime_dependencies()
    urdf_limits = parse_urdf_limits(args.urdf)
    motor_limits = expand_motor_type_limits(load_yaml(args.ethercat_config))
    limits = build_limits(urdf_limits, motor_limits, args.effort_source)
    joint_orders = {
        "hardware": HARDWARE_JOINT_ORDER,
        "controller": load_controller_joint_order(args.joint_config),
    }

    topic_specs = make_topic_specs(args)
    stats, topic_stats, missing_topics = check_bag(args, limits, joint_orders, topic_specs)
    if args.csv:
        write_csv(args.csv, stats)
        print(f"Wrote CSV: {args.csv}")

    has_violations = print_report(stats, topic_stats, missing_topics, args.max_report)
    if has_violations and not args.no_fail_on_violation:
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
