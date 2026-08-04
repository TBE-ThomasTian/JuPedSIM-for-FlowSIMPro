#!/usr/bin/env python3
"""Run the RiMEA validation scenarios and check each against its own criterion.

A scenario that merely runs proves nothing. RiMEA states a criterion for every
test, and this is what turns "the case exists" into "the case passes": each
check below is the sentence from the guideline, expressed as a number.

    python3 scripts/check_rimea.py [--solver build-final/bin/jupedsim]
                                   [--quick]

Exits non-zero if any check fails, so it can be used as a regression gate.
Criteria are cited with their clause; where RiMEA leaves something open, the
assumption is named in the scenario file's own header rather than here.

Not every test states a threshold, and pretending otherwise would be the worse
error. Three kinds appear below, and each check says which it is:

  PASS/FAIL   the guideline names a number or an outcome (Tests 1, 2, 3, 5, 6,
              10, 11, 12b)
  DIRECTION   the guideline names which way the result must go, but explicitly
              not by how much (Tests 8, 9)
  MEASURE     the guideline asks for a quantity to be determined and written
              down, with nothing to fail against (Test 12c)

A DIRECTION or MEASURE check still fails when nobody gets out or the run does
not finish: those are not opinions.

--quick leaves out the four cases with a thousand people or a parameter series,
which are most of the runtime.
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


def check_test8(solver, jsp):
    """Test 8: DIRECTION. A slower population may not evacuate faster.

    RiMEA asks for a series and for graphs, not for a threshold: "wie sich die
    Gesamtraeumungsdauer veraendert, wenn einzelne Personenparameter variiert
    werden [...] Geschwindigkeit aller Personen: 0,5 m/s, 0,75 m/s, 1,0 m/s".
    The speeds below are the guideline's own example. What can be asserted is
    that the series runs the right way; a model where it does not is broken
    whatever the numbers are.
    """
    source = (EXAMPLES / "rimea_test8_parameter_study.xml").read_text()
    times = []
    for speed in (0.50, 0.75, 1.00):
        variant = jsp.with_name(f"test8_{speed:.2f}.xml")
        variant.write_text(
            re.sub(r'desired_speed="[0-9.]+"', f'desired_speed="{speed:.2f}"', source)
        )
        _, elapsed, remaining = run(solver, variant, jsp)
        if remaining != 0:
            return False, f"{remaining} persons left in the building at {speed:.2f} m/s"
        times.append((speed, elapsed))
    series = ", ".join(f"{v:.2f} m/s: {t:.1f} s" for v, t in times)
    ok = all(a[1] > b[1] for a, b in zip(times, times[1:]))
    return ok, f"{series} (has to fall)"


def check_test9(solver, jsp):
    """Test 9: DIRECTION. Locking two of four doors has to take longer.

    The guideline expects "eine ungefaehre Verdopplung", and its own footnote
    then withdraws that as a criterion: "sollte Test 9 nicht im Sinne eines
    Ausschlusskriteriums [verwendet werden], sondern nur das Modellverhalten
    dokumentieren", because the bigger crowd in front of the remaining doors
    can raise the density and lower the flow. So the factor is printed, not
    judged - only the direction is asserted.
    """
    _, open_time, open_left = run(
        solver, EXAMPLES / "rimea_test9_public_space.xml", jsp)
    _, locked_time, locked_left = run(
        solver, EXAMPLES / "rimea_test9_public_space_locked.xml", jsp)
    if open_left or locked_left:
        return False, f"{open_left} and {locked_left} persons never got out"
    ok = locked_time > open_time
    factor = locked_time / open_time if open_time > 0 else 0.0
    return ok, (f"{open_time:.1f} s with four doors, {locked_time:.1f} s with two, "
                f"factor {factor:.2f} (guideline says about 2, and says not to "
                f"fail a model on it)")


def check_test11(solver, jsp):
    """Test 11: most take the nearer exit, but not everybody.

    "Das erwartete Ergebnis ist, dass die Personen den naeheren Ausgang 1 zwar
    bevorzugen [...] jedoch einzelne Personen auch den alternativen Ausgang 2
    benutzen." Both halves are the criterion: sending everybody to exit 1 fails
    it, and so does splitting the crowd evenly the other way.
    """
    output, _, remaining = run(
        solver, EXAMPLES / "rimea_test11_escape_route_choice.xml", jsp)
    left = {int(m.group("index")): int(m.group("left")) for m in EXIT_RE.finditer(output)}
    near, far = left.get(1, 0), left.get(2, 0)
    ok = remaining == 0 and near > far and far > 0
    return ok, f"exit 1 took {near}, exit 2 took {far} (1 has to lead, 2 may not be empty)"


def check_test12b(solver, jsp):
    """Test 12b: room 1 has to take longer to empty through the long bottleneck.

    "Zeige auf, dass die Raeumungsdauer von Raum 1 fuer die Geometrie mit der
    langen Engstelle groesser ist." Room 1 is x < 10 in both files, and it is
    room 1 that is measured and not the whole run: the long geometry is 5 m
    longer overall, so a total would differ even if the bottleneck did nothing.
    """
    emptied = {}
    for name in ("short", "long"):
        scenario = EXAMPLES / f"rimea_test12b_{name}.xml"
        variant = jsp.with_name(f"12b_{name}.jsp")
        _, _, remaining = run(solver, scenario, variant, ("--every-nth-frame", "4"))
        if remaining != 0:
            return False, f"{remaining} persons never left the {name} geometry"
        last = 0.0
        for seconds, agents in read_frames(variant):
            if any(x < 10.0 for _, x, _, _ in agents):
                last = seconds
        emptied[name] = last
    ok = emptied["long"] > emptied["short"]
    return ok, (f"room 1 empties at {emptied['short']:.1f} s through the short "
                f"bottleneck and {emptied['long']:.1f} s through the long one")


def check_test12c(solver, jsp):
    """Test 12c: MEASURE. Determine the counts and the flows, and report them.

    "Bestimme den zeitlichen Verlauf der Agentenzahlen in Raum 1 und 2 sowie die
    Fluesse durch die Engstellen 1 und 2." There is nothing here to fail
    against, so the numbers are printed and only the run itself is asserted.
    """
    scenario = EXAMPLES / "rimea_test12c_congestion.xml"
    variant = jsp.with_name("12c.jsp")
    _, elapsed, remaining = run(solver, scenario, variant, ("--every-nth-frame", "4"))
    if remaining != 0:
        return False, f"{remaining} persons never got through"

    # A crossing is a person whose x steps over one of the walls between two
    # frames. Each bottleneck is timed over its OWN window - first crossing to
    # last - and not over the run: dividing both by the same total would make
    # the two flows come out equal by construction, which is precisely the
    # comparison this test exists for.
    side = {}
    crossed = [0, 0]
    first = [None, None]
    last = [0.0, 0.0]
    peak = [0, 0]
    for seconds, agents in read_frames(variant):
        room1 = room2 = 0
        for agent_id, x, _, _ in agents:
            room = 0 if x < 10.1 else (1 if x < 20.3 else 2)
            if room == 0:
                room1 += 1
            elif room == 1:
                room2 += 1
            was = side.get(agent_id)
            if was is not None and room > was:
                for k in range(was, room):
                    crossed[k] += 1
                    if first[k] is None:
                        first[k] = seconds
                    last[k] = seconds
            side[agent_id] = room
        peak[0] = max(peak[0], room1)
        peak[1] = max(peak[1], room2)

    flows = []
    for k in range(2):
        span = last[k] - (first[k] if first[k] is not None else 0.0)
        flows.append(crossed[k] / span if span > 1e-9 else 0.0)
    return True, (f"peak occupancy {peak[0]} in room 1, {peak[1]} in room 2; "
                  f"bottleneck 1 passed {crossed[0]} persons at {flows[0]:.2f}/s, "
                  f"bottleneck 2 passed {crossed[1]} at {flows[1]:.2f}/s")


CHECKS = [
    ("Test 1  - walking speed in a corridor", check_test1, False),
    ("Test 2  - walking speed up a stair", check_test2, False),
    ("Test 3  - walking speed down a stair", check_test3, False),
    ("Test 5  - premovement time", check_test5, False),
    ("Test 6  - movement around a corner", check_test6, False),
    ("Test 8  - parameter study over walking speed", check_test8, True),
    ("Test 9  - crowd leaving a public space", check_test9, True),
    ("Test 10 - allocation of escape routes", check_test10, False),
    ("Test 11 - choice of escape route", check_test11, True),
    ("Test 12b- length of a bottleneck", check_test12b, False),
    ("Test 12c- influence of congestion", check_test12c, False),
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--solver",
        default=REPO / "build-final" / "bin" / "jupedsim",
        type=Path,
        help="path to the jupedsim binary",
    )
    parser.add_argument(
        "--quick",
        action="store_true",
        help="leave out the cases with a thousand people or a parameter series",
    )
    args = parser.parse_args()
    if not args.solver.is_file():
        print(f"solver not found: {args.solver}", file=sys.stderr)
        return 1

    selected = [c for c in CHECKS if not (args.quick and c[2])]
    failures = 0
    with tempfile.TemporaryDirectory() as tmp:
        for title, check, _slow in selected:
            jsp = Path(tmp) / "run.jsp"
            try:
                ok, detail = check(args.solver, jsp)
            except Exception as error:  # noqa: BLE001 - report, do not hide
                ok, detail = False, str(error)
            print(f"{'PASS' if ok else 'FAIL'}  {title}: {detail}")
            failures += 0 if ok else 1

    print()
    print(f"{len(selected) - failures} of {len(selected)} checks passed")
    if args.quick:
        print(f"({len(CHECKS) - len(selected)} slow checks left out by --quick)")
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
