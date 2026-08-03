#!/usr/bin/env python3
"""Run the RiMEA validation scenarios and check each against its own criterion.

A scenario that merely runs proves nothing. RiMEA states a criterion for every
test, and this is what turns "the case exists" into "the case passes": each
check below is the sentence from the guideline, expressed as a number.

    python3 scripts/check_rimea.py [--solver build-final/bin/jupedsim]

Exits non-zero if any check fails, so it can be used as a regression gate.
Criteria are cited with their clause; where RiMEA leaves something open, the
assumption is named in the scenario file's own header rather than here.
"""

import argparse
import re
import struct
import subprocess
import sys
import tempfile
import zlib
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
EXAMPLES = REPO / "examples" / "xml"

SUMMARY_RE = re.compile(
    r"iterations=(?P<iterations>\d+)\s+"
    r"elapsed_time=(?P<elapsed>[0-9eE+.\-]+)\s+"
    r"remaining_agents=(?P<remaining>\d+)"
)
EXIT_RE = re.compile(r"^exit=(?P<index>\d+) .*?left=(?P<left>\d+)", re.MULTILINE)


def read_frames(path):
    """Yield (time, [(agent_id, x, y, floor), ...]) for every frame of a .jsp."""
    data = path.read_bytes()
    record_size, = struct.unpack_from("<I", data, 20)
    frame_count, index_offset = struct.unpack_from("<QQ", data, 28)
    for i in range(frame_count):
        entry = index_offset + i * 48
        _, seconds, agent_count, _, offset, packed, _ = struct.unpack_from(
            "<QdIIQQQ", data, entry
        )
        raw = zlib.decompressobj(-15).decompress(data[offset:offset + packed])
        agents = []
        for k in range(len(raw) // record_size):
            agent_id, x, y, _, _, floor = struct.unpack_from(
                "<QffffI", raw, k * record_size
            )
            agents.append((agent_id, x, y, floor))
        yield seconds, agents


def run(solver, scenario, jsp, extra=()):
    result = subprocess.run(
        [str(solver), str(scenario), "--out-jsp", str(jsp), *extra],
        capture_output=True,
        text=True,
        timeout=1800,
    )
    if result.returncode not in (0, 2):
        raise RuntimeError(f"{scenario.name}: solver failed\n{result.stdout}{result.stderr}")
    match = SUMMARY_RE.search(result.stdout)
    if not match:
        raise RuntimeError(f"{scenario.name}: no summary line in the output")
    return result.stdout, float(match.group("elapsed")), int(match.group("remaining"))


# ── the checks ──────────────────────────────────────────────────────────────


def check_test1(solver, jsp):
    """Test 1: 40 m at 1.33 m/s has to take 26 to 34 s (RiMEA 4.1.1, Test 1)."""
    _, elapsed, remaining = run(solver, EXAMPLES / "rimea_test1_corridor.xml", jsp)
    ok = remaining == 0 and 26.0 <= elapsed <= 34.0
    return ok, f"{elapsed:.2f} s (required 26 to 34 s)"


def check_test2(solver, jsp):
    """Test 2: 10 m of stair at 0.50 m/s upwards, so 20.0 s."""
    _, elapsed, remaining = run(solver, EXAMPLES / "rimea_test2_stair_ascent.xml", jsp)
    expected = 10.0 / 0.50
    ok = remaining == 0 and abs(elapsed - expected) <= 0.05 * expected
    return ok, f"{elapsed:.2f} s (expected {expected:.2f} s, 5 % allowed)"


def check_test3(solver, jsp):
    """Test 3: 10 m of stair at 0.65 m/s downwards, so 15.38 s."""
    _, elapsed, remaining = run(solver, EXAMPLES / "rimea_test3_stair_descent.xml", jsp)
    expected = 10.0 / 0.65
    ok = remaining == 0 and abs(elapsed - expected) <= 0.05 * expected
    return ok, f"{elapsed:.2f} s (expected {expected:.2f} s, 5 % allowed)"


def check_test5(solver, jsp):
    """Test 5: every person starts at its own premovement time, not before."""
    _, _, remaining = run(
        solver, EXAMPLES / "rimea_test5_premovement.xml", jsp, ("--every-nth-frame", "1")
    )
    expected = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100]
    start, moved = {}, {}
    for seconds, agents in read_frames(jsp):
        for agent_id, x, y, _ in agents:
            if agent_id not in start:
                start[agent_id] = (x, y)
            elif agent_id not in moved:
                sx, sy = start[agent_id]
                if abs(x - sx) + abs(y - sy) > 0.01:
                    moved[agent_id] = seconds
    if len(moved) != len(expected):
        return False, f"only {len(moved)} of {len(expected)} persons ever moved"
    worst = max(
        abs(moved[a] - w) for a, w in zip(sorted(moved), expected)
    )
    ok = remaining == 0 and worst <= 0.2
    return ok, f"worst departure off by {worst:.2f} s (0.2 s allowed)"


def check_test6(solver, jsp):
    """Test 6: round the corner "ohne Waende zu durchqueren" - a hard criterion."""
    _, _, remaining = run(
        solver, EXAMPLES / "rimea_test6_corner.xml", jsp, ("--every-nth-frame", "1")
    )
    # The block that makes the L: x < 10 and y > 2 is solid.
    inside = 0
    samples = 0
    for _, agents in read_frames(jsp):
        for _, x, y, _ in agents:
            samples += 1
            if x < 10.0 and y > 2.0:
                inside += 1
    ok = remaining == 0 and inside == 0
    return ok, f"{inside} of {samples} positions inside the wall (0 allowed)"


def check_test10(solver, jsp):
    """Test 10: everybody leaves through the exit it was assigned."""
    output, _, remaining = run(solver, EXAMPLES / "rimea_test10_corridor_rooms.xml", jsp)
    left = {int(m.group("index")): int(m.group("left")) for m in EXIT_RE.finditer(output)}
    # Rooms 1-4 and 7-10 hold 15 persons, rooms 5, 6, 11 and 12 hold 8.
    ok = remaining == 0 and left.get(1) == 15 and left.get(2) == 8
    return ok, f"main {left.get(1)}, secondary {left.get(2)} (expected 15 and 8)"


CHECKS = [
    ("Test 1  - walking speed in a corridor", check_test1),
    ("Test 2  - walking speed up a stair", check_test2),
    ("Test 3  - walking speed down a stair", check_test3),
    ("Test 5  - premovement time", check_test5),
    ("Test 6  - movement around a corner", check_test6),
    ("Test 10 - allocation of escape routes", check_test10),
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--solver",
        default=REPO / "build-final" / "bin" / "jupedsim",
        type=Path,
        help="path to the jupedsim binary",
    )
    args = parser.parse_args()
    if not args.solver.is_file():
        print(f"solver not found: {args.solver}", file=sys.stderr)
        return 1

    failures = 0
    with tempfile.TemporaryDirectory() as tmp:
        for title, check in CHECKS:
            jsp = Path(tmp) / "run.jsp"
            try:
                ok, detail = check(args.solver, jsp)
            except Exception as error:  # noqa: BLE001 - report, do not hide
                ok, detail = False, str(error)
            print(f"{'PASS' if ok else 'FAIL'}  {title}: {detail}")
            failures += 0 if ok else 1

    print()
    print(f"{len(CHECKS) - failures} of {len(CHECKS)} checks passed")
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
