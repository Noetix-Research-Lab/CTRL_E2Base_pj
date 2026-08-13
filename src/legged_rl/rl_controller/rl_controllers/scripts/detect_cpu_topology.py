#!/usr/bin/env python3
"""Detect Linux heterogeneous CPU topology and recommend controller affinities.

The script is intentionally read-only.  It prefers kernel scheduler capacity or
core-type information and falls back to grouping physical cores by maximum
frequency.  Logical SMT siblings are never treated as independent physical
cores when selecting the control and inference CPUs.
"""

import argparse
import json
import os
import statistics
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class CpuInfo:
    cpu: int
    package: int
    core: int
    siblings: Tuple[int, ...]
    capacity: Optional[int]
    max_freq_khz: Optional[int]
    core_type: Optional[int]


@dataclass
class CoreInfo:
    package: int
    core: int
    cpus: List[int]
    capacity: Optional[int]
    max_freq_khz: Optional[int]
    core_type: Optional[int]
    tier: str = "uniform"

    @property
    def key(self) -> Tuple[int, int]:
        return self.package, self.core


def read_text(path: Path) -> Optional[str]:
    try:
        return path.read_text(encoding="utf-8").strip()
    except (FileNotFoundError, PermissionError, OSError):
        return None


def read_int(path: Path) -> Optional[int]:
    value = read_text(path)
    if value is None:
        return None
    try:
        return int(value, 0)
    except ValueError:
        return None


def parse_cpu_list(value: str) -> List[int]:
    cpus: List[int] = []
    for item in value.split(","):
        item = item.strip()
        if not item:
            continue
        if "-" in item:
            first, last = item.split("-", 1)
            cpus.extend(range(int(first), int(last) + 1))
        else:
            cpus.append(int(item))
    return sorted(set(cpus))


def discover_online_cpus(sysfs_root: Path) -> List[int]:
    online = read_text(sysfs_root / "online")
    if online:
        return parse_cpu_list(online)

    cpus: List[int] = []
    for path in sysfs_root.glob("cpu[0-9]*"):
        suffix = path.name[3:]
        if suffix.isdigit() and read_text(path / "online") != "0":
            cpus.append(int(suffix))
    return sorted(cpus)


def first_int(paths: Iterable[Path]) -> Optional[int]:
    for path in paths:
        value = read_int(path)
        if value is not None:
            return value
    return None


def read_cpu_info(sysfs_root: Path, cpu: int, online: Sequence[int]) -> CpuInfo:
    base = sysfs_root / f"cpu{cpu}"
    topology = base / "topology"
    siblings_text = read_text(topology / "thread_siblings_list")
    online_set = set(online)
    siblings = tuple(
        item for item in (parse_cpu_list(siblings_text) if siblings_text else [cpu])
        if item in online_set
    )
    package_id = read_int(topology / "physical_package_id")
    core_id = read_int(topology / "core_id")
    return CpuInfo(
        cpu=cpu,
        package=package_id if package_id is not None else 0,
        core=core_id if core_id is not None else min(siblings),
        siblings=siblings,
        capacity=first_int((base / "cpu_capacity", topology / "cpu_capacity")),
        max_freq_khz=first_int((
            base / "cpufreq" / "cpuinfo_max_freq",
            base / "cpufreq" / "scaling_max_freq",
        )),
        core_type=read_int(topology / "core_type"),
    )


def build_cores(cpus: Sequence[CpuInfo]) -> List[CoreInfo]:
    grouped: Dict[Tuple[int, int], List[CpuInfo]] = {}
    for cpu in cpus:
        grouped.setdefault((cpu.package, cpu.core), []).append(cpu)

    cores: List[CoreInfo] = []
    for (package, core), members in sorted(grouped.items()):
        capacities = [item.capacity for item in members if item.capacity is not None]
        frequencies = [item.max_freq_khz for item in members if item.max_freq_khz is not None]
        core_types = [item.core_type for item in members if item.core_type is not None]
        cores.append(CoreInfo(
            package=package,
            core=core,
            cpus=sorted(item.cpu for item in members),
            capacity=max(capacities) if capacities else None,
            max_freq_khz=max(frequencies) if frequencies else None,
            core_type=max(core_types) if core_types else None,
        ))
    return cores


def split_on_largest_gap(values: Sequence[int], minimum_ratio: float) -> Optional[int]:
    unique = sorted(set(values))
    if len(unique) < 2 or unique[0] <= 0:
        return None
    gaps = [(unique[index + 1] / unique[index], index) for index in range(len(unique) - 1)]
    ratio, index = max(gaps)
    return unique[index] if ratio >= minimum_ratio else None


def classify_cores(cores: List[CoreInfo]) -> Tuple[str, str]:
    capacities = [core.capacity for core in cores if core.capacity is not None]
    if len(capacities) == len(cores):
        boundary = split_on_largest_gap(capacities, 1.10)
        if boundary is not None:
            for core in cores:
                core.tier = "efficiency" if core.capacity <= boundary else "performance"
            return "scheduler_capacity", "high"

    typed = [core for core in cores if core.core_type is not None]
    if len(typed) == len(cores) and len({core.core_type for core in cores}) > 1:
        type_scores: Dict[int, List[float]] = {}
        for core in cores:
            score = float(core.capacity or core.max_freq_khz or len(core.cpus))
            type_scores.setdefault(core.core_type, []).append(score)
        ranked_types = sorted(
            type_scores,
            key=lambda item: (statistics.median(type_scores[item]), item),
        )
        performance_type = ranked_types[-1]
        for core in cores:
            core.tier = "performance" if core.core_type == performance_type else "efficiency"
        return "kernel_core_type", "high"

    frequencies = [core.max_freq_khz for core in cores if core.max_freq_khz is not None]
    if len(frequencies) == len(cores):
        boundary = split_on_largest_gap(frequencies, 1.15)
        if boundary is not None:
            for core in cores:
                core.tier = "efficiency" if core.max_freq_khz <= boundary else "performance"
            return "maximum_frequency_gap", "medium"

    for core in cores:
        core.tier = "uniform"
    return "no_reliable_heterogeneous_signal", "low"


def core_rank(core: CoreInfo) -> Tuple[int, int, int, int]:
    return (
        core.capacity or 0,
        core.max_freq_khz or 0,
        len(core.cpus),
        -min(core.cpus),
    )


def choose_cpu(core: CoreInfo) -> int:
    return min(core.cpus)


def recommend(cores: Sequence[CoreInfo], inference_tier: str) -> Dict[str, object]:
    performance = [core for core in cores if core.tier in ("performance", "uniform")]
    efficiency = [core for core in cores if core.tier == "efficiency"]
    performance.sort(key=core_rank, reverse=True)
    efficiency.sort(key=core_rank, reverse=True)

    control_pool = performance or list(cores)
    nonzero_pool = [core for core in control_pool if 0 not in core.cpus]
    control_core = (nonzero_pool or control_pool)[0]

    remaining_performance = [core for core in performance if core.key != control_core.key]
    remaining_efficiency = [core for core in efficiency if core.key != control_core.key]
    if inference_tier == "efficiency" and remaining_efficiency:
        inference_core = remaining_efficiency[0]
    elif remaining_performance:
        inference_core = remaining_performance[0]
    elif remaining_efficiency:
        inference_core = remaining_efficiency[0]
    else:
        remaining = [core for core in cores if core.key != control_core.key]
        inference_core = remaining[0] if remaining else control_core

    control_cpu = choose_cpu(control_core)
    inference_cpu = choose_cpu(inference_core)
    same_physical_core = control_core.key == inference_core.key
    if same_physical_core:
        alternative = [cpu for cpu in inference_core.cpus if cpu != control_cpu]
        if alternative:
            inference_cpu = alternative[0]

    return {
        "control_cpu": control_cpu,
        "control_core": list(control_core.key),
        "control_siblings": control_core.cpus,
        "control_tier": control_core.tier,
        "inference_cpu": inference_cpu,
        "inference_core": list(inference_core.key),
        "inference_siblings": inference_core.cpus,
        "inference_tier": inference_core.tier,
        "same_physical_core": same_physical_core,
    }


def cpu_list_text(cpus: Sequence[int]) -> str:
    return ",".join(str(cpu) for cpu in cpus)


def print_human(cores: Sequence[CoreInfo], method: str, confidence: str,
                recommendation: Dict[str, object]) -> None:
    print(f"Detection method: {method} (confidence: {confidence})")
    print()
    print("TIER         PACKAGE CORE  LOGICAL_CPUS  SMT  CAPACITY  MAX_MHZ  CORE_TYPE")
    for core in cores:
        max_mhz = f"{core.max_freq_khz / 1000:.0f}" if core.max_freq_khz else "-"
        print(f"{core.tier:12} {core.package:7} {core.core:4}  "
              f"{cpu_list_text(core.cpus):12} {len(core.cpus):3}  "
              f"{str(core.capacity or '-'):8}  {max_mhz:7}  "
              f"{str(core.core_type if core.core_type is not None else '-'):9}")

    print()
    print("Recommended bindings:")
    print(f"  RT control thread : CPU {recommendation['control_cpu']} "
          f"(physical core {recommendation['control_core']}, "
          f"siblings {recommendation['control_siblings']})")
    print(f"  Inference thread  : CPU {recommendation['inference_cpu']} "
          f"(physical core {recommendation['inference_core']}, "
          f"siblings {recommendation['inference_siblings']})")
    if recommendation["same_physical_core"]:
        print("  WARNING: only one physical core is available; the threads cannot be isolated.")
    if len(recommendation["control_siblings"]) > 1:
        print("  Reserve every control sibling from unrelated workloads; SMT siblings share one core.")

    print()
    print("Set these project parameters:")
    print("  legged_e2_hw/config/e2.yaml:")
    print(f"    cpu_affinity: {recommendation['control_cpu']}")
    print("  rl_controllers/config/e1_<dof>_ac.yaml:")
    print(f"    inference_cpu_affinity: {recommendation['inference_cpu']}")
    print()
    print("After launch, verify placement with:")
    print("  ps -eLo pid,tid,psr,cls,rtprio,comm | grep -E 'legged|controller|ac_inference'")
    print("  taskset -pc <TID>")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--sysfs-root",
        type=Path,
        default=Path("/sys/devices/system/cpu"),
        help="CPU sysfs root (primarily useful for tests)",
    )
    parser.add_argument(
        "--inference-tier",
        choices=("performance", "efficiency"),
        default="performance",
        help="Prefer a separate performance core for latency, or an efficiency core for isolation",
    )
    parser.add_argument(
        "--format",
        choices=("human", "json", "yaml"),
        default="human",
        help="Output format",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    online = discover_online_cpus(args.sysfs_root)
    if args.sysfs_root == Path("/sys/devices/system/cpu") and hasattr(os, "sched_getaffinity"):
        allowed = os.sched_getaffinity(0)
        online = [cpu for cpu in online if cpu in allowed]
    if not online:
        print(f"error: no usable online CPUs found below {args.sysfs_root}", file=sys.stderr)
        return 2

    cpus = [read_cpu_info(args.sysfs_root, cpu, online) for cpu in online]
    cores = build_cores(cpus)
    method, confidence = classify_cores(cores)
    binding = recommend(cores, args.inference_tier)

    if args.format == "human":
        print_human(cores, method, confidence, binding)
    elif args.format == "yaml":
        print("legged_e2_hw:")
        print(f"  cpu_affinity: {binding['control_cpu']}")
        print("LeggedRobotCfg:")
        print("  control:")
        print(f"    inference_cpu_affinity: {binding['inference_cpu']}")
    else:
        payload = {
            "online_cpus": online,
            "detection_method": method,
            "confidence": confidence,
            "cores": [asdict(core) for core in cores],
            "recommendation": binding,
        }
        json.dump(payload, sys.stdout, indent=2)
        print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
