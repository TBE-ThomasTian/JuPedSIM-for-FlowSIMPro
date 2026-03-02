#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-3.0-or-later

from __future__ import annotations

import argparse
import copy
import json
import math
import os
import random
import re
import shutil
import struct
import subprocess
import tempfile
import xml.etree.ElementTree as ET
import zlib
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


CLI_SUMMARY_RE = re.compile(
    r"iterations=(?P<iterations>\d+)\s+elapsed_time=(?P<elapsed>[0-9eE+.\-]+)\s+remaining_agents=(?P<remaining>\d+)"
)


@dataclass
class FrameIndexEntry:
    iteration: int
    time_seconds: float
    agent_count: int
    data_offset: int
    compressed_size: int
    uncompressed_size: int


@dataclass
class RunResult:
    run_id: int
    seed: int
    speed_scale: float
    time_gap_scale: float
    completed: bool
    iterations: int
    elapsed_time: float
    remaining_agents: int
    return_code: int
    throughput_total: float
    throughput_by_exit: list[float]
    evacuated_by_exit: list[int]
    hotspot_samples: int
    stderr: str


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run Monte-Carlo evacuation uncertainty analysis via jupedsim XML CLI"
    )
    parser.add_argument(
        "--scenario",
        required=True,
        help="Input XML scenario for jupedsim XML CLI",
    )
    parser.add_argument(
        "--cli",
        default="build/bin/jupedsim",
        help="Path to jupedsim binary",
    )
    parser.add_argument(
        "--runs",
        type=int,
        default=20,
        help="Number of Monte-Carlo runs",
    )
    parser.add_argument(
        "--seed-base",
        type=int,
        default=1000,
        help="Base random seed for scenario perturbation",
    )
    parser.add_argument(
        "--seed-step",
        type=int,
        default=1,
        help="Increment between run seeds",
    )
    parser.add_argument(
        "--speed-scale-mean",
        type=float,
        default=1.0,
        help="Mean multiplier for desired_speed perturbation",
    )
    parser.add_argument(
        "--speed-scale-std",
        type=float,
        default=0.05,
        help="Stddev multiplier for desired_speed perturbation",
    )
    parser.add_argument(
        "--time-gap-scale-mean",
        type=float,
        default=1.0,
        help="Mean multiplier for time_gap perturbation",
    )
    parser.add_argument(
        "--time-gap-scale-std",
        type=float,
        default=0.03,
        help="Stddev multiplier for time_gap perturbation",
    )
    parser.add_argument(
        "--max-iterations",
        type=int,
        default=None,
        help="Optional override passed to jupedsim",
    )
    parser.add_argument(
        "--cell-size",
        type=float,
        default=1.0,
        help="Hotspot grid cell size in meters",
    )
    parser.add_argument(
        "--top-hotspots",
        type=int,
        default=10,
        help="Number of top hotspot cells to report",
    )
    parser.add_argument(
        "--output-dir",
        default="artifacts/evac_uncertainty",
        help="Directory for report artifacts",
    )
    parser.add_argument(
        "--keep-run-files",
        action="store_true",
        help="Keep per-run generated XML/JSP files",
    )
    return parser.parse_args()


def clamp_positive(value: float, floor: float = 0.2) -> float:
    return max(floor, value)


def percentile(values: list[float], p: float) -> float:
    if not values:
        return float("nan")
    if len(values) == 1:
        return values[0]
    sorted_values = sorted(values)
    pos = (len(sorted_values) - 1) * p
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return sorted_values[lo]
    frac = pos - lo
    return sorted_values[lo] * (1.0 - frac) + sorted_values[hi] * frac


def format_float(value: float, digits: int = 6) -> str:
    return f"{value:.{digits}g}"


def parse_polygons_from_node(node: ET.Element) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for vertex in node.findall("vertex"):
        x = float(vertex.attrib["x"])
        y = float(vertex.attrib["y"])
        points.append((x, y))
    if len(points) < 3:
        raise RuntimeError("Exit polygon must contain at least 3 vertices")
    return points


def parse_exit_polygons(root: ET.Element) -> list[list[tuple[float, float]]]:
    exits_node = root.find("exits")
    if exits_node is not None:
        polygons: list[list[tuple[float, float]]] = []
        for exit_node in exits_node.findall("exit"):
            polygons.append(parse_polygons_from_node(exit_node))
        if not polygons:
            raise RuntimeError("<exits> exists but contains no <exit>")
        return polygons
    exit_node = root.find("exit")
    if exit_node is None:
        raise RuntimeError("Scenario has neither <exit> nor <exits>")
    return [parse_polygons_from_node(exit_node)]


def set_seed_if_distribution_exists(root: ET.Element, seed: int) -> None:
    dist = root.find("agents/distribution")
    if dist is not None:
        dist.attrib["seed"] = str(seed)
    top_dist = root.find("agent_distribution")
    if top_dist is not None:
        top_dist.attrib["seed"] = str(seed)


def apply_agent_perturbations(
    root: ET.Element,
    speed_scale: float,
    time_gap_scale: float,
) -> None:
    for node in root.iter():
        tag = node.tag
        if tag not in {"agent", "profile"}:
            continue
        if "desired_speed" in node.attrib:
            node.attrib["desired_speed"] = format_float(
                float(node.attrib["desired_speed"]) * speed_scale
            )
        if "time_gap" in node.attrib:
            node.attrib["time_gap"] = format_float(
                float(node.attrib["time_gap"]) * time_gap_scale
            )


def point_in_polygon(x: float, y: float, polygon: list[tuple[float, float]]) -> bool:
    inside = False
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        if (y1 > y) == (y2 > y):
            continue
        denom = y2 - y1
        if abs(denom) < 1e-12:
            continue
        x_cross = x1 + (x2 - x1) * (y - y1) / denom
        if x_cross >= x:
            inside = not inside
    return inside


def parse_jsp_index(data: bytes) -> list[FrameIndexEntry]:
    if len(data) < 52:
        raise RuntimeError("JSP file too small")
    magic = data[0:4]
    if magic != b"JSP1":
        raise RuntimeError(f"Unsupported JSP magic: {magic!r}")
    frame_count = struct.unpack_from("<Q", data, 28)[0]
    index_offset = struct.unpack_from("<Q", data, 36)[0]
    entries: list[FrameIndexEntry] = []
    for idx in range(frame_count):
        off = index_offset + idx * 48
        if off + 48 > len(data):
            raise RuntimeError("Corrupt JSP frame index")
        iteration = struct.unpack_from("<Q", data, off)[0]
        time_seconds = struct.unpack_from("<d", data, off + 8)[0]
        agent_count = struct.unpack_from("<I", data, off + 16)[0]
        data_offset = struct.unpack_from("<Q", data, off + 24)[0]
        compressed_size = struct.unpack_from("<Q", data, off + 32)[0]
        uncompressed_size = struct.unpack_from("<Q", data, off + 40)[0]
        entries.append(
            FrameIndexEntry(
                iteration=iteration,
                time_seconds=time_seconds,
                agent_count=agent_count,
                data_offset=data_offset,
                compressed_size=compressed_size,
                uncompressed_size=uncompressed_size,
            )
        )
    return entries


def parse_frame_agents(data: bytes) -> Iterable[tuple[int, float, float]]:
    record_size = 24
    if len(data) % record_size != 0:
        raise RuntimeError("Corrupt JSP frame payload size")
    for off in range(0, len(data), record_size):
        agent_id = struct.unpack_from("<Q", data, off)[0]
        x = struct.unpack_from("<f", data, off + 8)[0]
        y = struct.unpack_from("<f", data, off + 12)[0]
        yield (agent_id, float(x), float(y))


def analyze_jsp(
    jsp_path: Path,
    exit_polygons: list[list[tuple[float, float]]],
    cell_size: float,
) -> tuple[list[int], Counter[tuple[int, int]], int]:
    content = jsp_path.read_bytes()
    index = parse_jsp_index(content)
    agent_exit_assignment: dict[int, int] = {}
    evacuated_by_exit = [0 for _ in exit_polygons]
    hotspot_counter: Counter[tuple[int, int]] = Counter()
    total_samples = 0

    for entry in index:
        start = entry.data_offset
        end = entry.data_offset + entry.compressed_size
        if end > len(content):
            raise RuntimeError("Corrupt JSP frame data range")
        compressed = content[start:end]
        payload = zlib.decompress(compressed, wbits=-zlib.MAX_WBITS)
        if len(payload) != entry.uncompressed_size:
            raise RuntimeError("Unexpected uncompressed frame size")

        for agent_id, x, y in parse_frame_agents(payload):
            if agent_id not in agent_exit_assignment:
                for exit_idx, polygon in enumerate(exit_polygons):
                    if point_in_polygon(x, y, polygon):
                        agent_exit_assignment[agent_id] = exit_idx
                        evacuated_by_exit[exit_idx] += 1
                        break
            hotspot_counter[(int(math.floor(x / cell_size)), int(math.floor(y / cell_size)))] += 1
            total_samples += 1

    return evacuated_by_exit, hotspot_counter, total_samples


def parse_cli_summary(stdout: str) -> tuple[int, float, int]:
    match = CLI_SUMMARY_RE.search(stdout)
    if not match:
        raise RuntimeError("Could not parse jupedsim summary line")
    iterations = int(match.group("iterations"))
    elapsed_time = float(match.group("elapsed"))
    remaining = int(match.group("remaining"))
    return iterations, elapsed_time, remaining


def write_markdown_report(
    output_path: Path,
    scenario: str,
    runs: list[RunResult],
    p50_time: float,
    p95_time: float,
    p50_throughput: float,
    p95_throughput: float,
    top_hotspots: list[tuple[tuple[int, int], int]],
    cell_size: float,
) -> None:
    completed = [r for r in runs if r.completed]
    failed = [r for r in runs if not r.completed]

    lines: list[str] = []
    lines.append("# Evacuation Uncertainty Report")
    lines.append("")
    lines.append(f"- Scenario: `{scenario}`")
    lines.append(f"- Runs: `{len(runs)}`")
    lines.append(f"- Completed runs: `{len(completed)}`")
    lines.append(f"- Incomplete runs: `{len(failed)}`")
    lines.append("")
    lines.append("## Core KPIs")
    lines.append("")
    lines.append(f"- Evacuation time P50: `{p50_time:.3f}` s")
    lines.append(f"- Evacuation time P95: `{p95_time:.3f}` s")
    lines.append(f"- Total throughput P50: `{p50_throughput:.4f}` agents/s")
    lines.append(f"- Total throughput P95: `{p95_throughput:.4f}` agents/s")
    lines.append("")
    lines.append("## Per-Run Summary")
    lines.append("")
    lines.append("| run | seed | completed | iterations | elapsed_s | remaining | throughput_total |")
    lines.append("|---:|---:|:---:|---:|---:|---:|---:|")
    for run in runs:
        lines.append(
            f"| {run.run_id} | {run.seed} | {'yes' if run.completed else 'no'} | "
            f"{run.iterations} | {run.elapsed_time:.3f} | {run.remaining_agents} | "
            f"{run.throughput_total:.4f} |"
        )
    lines.append("")
    lines.append("## Hotspots")
    lines.append("")
    lines.append(
        f"Top {len(top_hotspots)} hotspot cells (grid size `{cell_size}` m, in global coordinates):"
    )
    lines.append("")
    lines.append("| rank | cell_x | cell_y | samples |")
    lines.append("|---:|---:|---:|---:|")
    for rank, ((cx, cy), samples) in enumerate(top_hotspots, start=1):
        lines.append(f"| {rank} | {cx} | {cy} | {samples} |")
    lines.append("")
    output_path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    args = parse_args()
    scenario_path = Path(args.scenario).resolve()
    cli_path = Path(args.cli).resolve()
    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    if args.runs <= 0:
        raise RuntimeError("--runs must be > 0")
    if args.cell_size <= 0:
        raise RuntimeError("--cell-size must be > 0")
    if not cli_path.exists():
        raise RuntimeError(f"CLI binary not found: {cli_path}")
    if not scenario_path.exists():
        raise RuntimeError(f"Scenario file not found: {scenario_path}")

    base_tree = ET.parse(scenario_path)
    base_root = base_tree.getroot()
    exit_polygons = parse_exit_polygons(base_root)

    hotspot_sum: Counter[tuple[int, int]] = Counter()
    run_results: list[RunResult] = []
    work_dir_obj: tempfile.TemporaryDirectory[str] | None = None
    work_dir: Path
    if args.keep_run_files:
        work_dir = output_dir / "runs"
        work_dir.mkdir(parents=True, exist_ok=True)
    else:
        work_dir_obj = tempfile.TemporaryDirectory(prefix="jps_uncertainty_")
        work_dir = Path(work_dir_obj.name)

    for run_id in range(args.runs):
        seed = args.seed_base + run_id * args.seed_step
        rng = random.Random(seed ^ 0x9E3779B97F4A7C15)
        speed_scale = clamp_positive(
            rng.normalvariate(args.speed_scale_mean, args.speed_scale_std)
        )
        time_gap_scale = clamp_positive(
            rng.normalvariate(args.time_gap_scale_mean, args.time_gap_scale_std)
        )

        run_root = copy.deepcopy(base_root)
        set_seed_if_distribution_exists(run_root, seed)
        apply_agent_perturbations(run_root, speed_scale, time_gap_scale)

        run_scenario = work_dir / f"run_{run_id:03d}.xml"
        run_jsp = work_dir / f"run_{run_id:03d}.jsp"
        ET.ElementTree(run_root).write(run_scenario, encoding="utf-8", xml_declaration=True)

        cmd = [
            str(cli_path),
            str(run_scenario),
            "--out-jsp",
            str(run_jsp),
        ]
        if args.max_iterations is not None:
            cmd += ["--max-iterations", str(args.max_iterations)]

        proc = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=False,
        )
        iterations = 0
        elapsed_time = float("nan")
        remaining_agents = -1
        completed = False
        throughput_total = 0.0
        throughput_by_exit = [0.0 for _ in exit_polygons]
        evacuated_by_exit = [0 for _ in exit_polygons]
        hotspot_samples = 0

        try:
            iterations, elapsed_time, remaining_agents = parse_cli_summary(proc.stdout)
            completed = remaining_agents == 0
        except Exception:
            pass

        if run_jsp.exists():
            try:
                evacuated_by_exit, hotspots, hotspot_samples = analyze_jsp(
                    run_jsp,
                    exit_polygons,
                    args.cell_size,
                )
                hotspot_sum.update(hotspots)
                if elapsed_time > 0:
                    throughput_by_exit = [
                        float(count) / elapsed_time for count in evacuated_by_exit
                    ]
                    throughput_total = sum(throughput_by_exit)
            except Exception as exc:
                proc.stderr += f"\n[analysis-error] {exc}\n"

        run_results.append(
            RunResult(
                run_id=run_id,
                seed=seed,
                speed_scale=speed_scale,
                time_gap_scale=time_gap_scale,
                completed=completed,
                iterations=iterations,
                elapsed_time=elapsed_time,
                remaining_agents=remaining_agents,
                return_code=proc.returncode,
                throughput_total=throughput_total,
                throughput_by_exit=throughput_by_exit,
                evacuated_by_exit=evacuated_by_exit,
                hotspot_samples=hotspot_samples,
                stderr=proc.stderr.strip(),
            )
        )

        print(
            f"[run {run_id:03d}] seed={seed} completed={completed} "
            f"iter={iterations} elapsed={elapsed_time:.3f}s throughput={throughput_total:.4f}"
        )

    completed_runs = [r for r in run_results if r.completed and math.isfinite(r.elapsed_time)]
    evac_times = [r.elapsed_time for r in completed_runs]
    throughputs = [r.throughput_total for r in completed_runs if r.throughput_total > 0]

    p50_time = percentile(evac_times, 0.50)
    p95_time = percentile(evac_times, 0.95)
    p50_throughput = percentile(throughputs, 0.50)
    p95_throughput = percentile(throughputs, 0.95)

    top_hotspots = hotspot_sum.most_common(args.top_hotspots)

    summary = {
        "scenario": str(scenario_path),
        "cli": str(cli_path),
        "runs": args.runs,
        "completed_runs": len(completed_runs),
        "incomplete_runs": args.runs - len(completed_runs),
        "kpi": {
            "evac_time_p50_s": p50_time,
            "evac_time_p95_s": p95_time,
            "throughput_p50_agents_per_s": p50_throughput,
            "throughput_p95_agents_per_s": p95_throughput,
        },
        "top_hotspots": [
            {"cell_x": cx, "cell_y": cy, "samples": samples}
            for (cx, cy), samples in top_hotspots
        ],
        "runs_detail": [
            {
                "run_id": r.run_id,
                "seed": r.seed,
                "speed_scale": r.speed_scale,
                "time_gap_scale": r.time_gap_scale,
                "completed": r.completed,
                "iterations": r.iterations,
                "elapsed_time_s": r.elapsed_time,
                "remaining_agents": r.remaining_agents,
                "return_code": r.return_code,
                "throughput_total_agents_per_s": r.throughput_total,
                "throughput_by_exit_agents_per_s": r.throughput_by_exit,
                "evacuated_by_exit": r.evacuated_by_exit,
                "hotspot_samples": r.hotspot_samples,
                "stderr": r.stderr,
            }
            for r in run_results
        ],
    }

    json_path = output_dir / "summary.json"
    md_path = output_dir / "summary.md"
    json_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    write_markdown_report(
        output_path=md_path,
        scenario=str(scenario_path),
        runs=run_results,
        p50_time=p50_time,
        p95_time=p95_time,
        p50_throughput=p50_throughput,
        p95_throughput=p95_throughput,
        top_hotspots=top_hotspots,
        cell_size=args.cell_size,
    )

    if work_dir_obj is not None:
        work_dir_obj.cleanup()
    elif not args.keep_run_files:
        shutil.rmtree(work_dir, ignore_errors=True)

    print(f"\nReport written to:\n  {json_path}\n  {md_path}")
    if os.name == "posix":
        print("\nQuick peek:")
        print(f"  jq '.kpi' {json_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
