#!/usr/bin/env python3
"""Sweep start positions around a single wall tip and count who never gets out.

The measure behind the routing work: a lone agent, no crowd, one 0.20 x 3.00 m
wall between it and the exit. The agent used to walk into that wall and stay
there at full walking speed - net displacement zero, indistinguishable from
standing still.

    before the funnel fix:  177 of 209 start positions trapped
    after it:                0 of 209

The fix is in RoutingEngine::straightenPath: the funnel ran to portalCount
EXCLUSIVE, so the closing zero-length portal at the destination was never built
and the loop stopped one turn early. The corner the path had to round was still
inside the funnel when it stopped, and everything left in there was dropped -
what came out was the destination alone, a straight line through the wall.

Worth saying plainly, because the wrong cause was believed for a while: a
remembered waypoint in TacticalDecisionSystem also brought this to 0 of 209, and
looked like the cure. It was not. It hid a route that led through a wall, and
paid for it elsewhere - people stranded at door jambs in RiMEA Tests 12b and 12c.
With the funnel repaired the memory buys nothing and was removed again; this
probe reads 0 of 209 without it. The 177 above is measured against the funnel as
it was, not against the memory.

The 12 unusable ones are start positions inside or too close to the wall, which
the solver rejects outright; they are not failures.

Keep this: it is the only cheap check that the trap has not come back.
"""
import subprocess, re, sys, tempfile, os
SOLVER="/home/flowsimpro/Coding/JuPedSIM-for-FlowSIMPro/build-final/bin/jupedsim"
TPL='''<?xml version="1.0" encoding="UTF-8"?>
<scenario dt="0.01" max_iterations="12000">
  <geometry>
    <walkable><vertex x="0" y="0"/><vertex x="26" y="0"/><vertex x="26" y="14"/><vertex x="0" y="14"/></walkable>
    <obstacle><vertex x="19.90" y="8"/><vertex x="20.10" y="8"/><vertex x="20.10" y="11"/><vertex x="19.90" y="11"/></obstacle>
  </geometry>
  <exit><vertex x="22.2" y="3"/><vertex x="26" y="3"/><vertex x="26" y="11"/><vertex x="22.2" y="11"/></exit>
  <agents><agent x="{x:.3f}" y="{y:.3f}" radius="0.2" time_gap="1.0" desired_speed="1.2"/></agents>
</scenario>'''
trapped=escaped=err=0
tmp=tempfile.mkdtemp()
xs=[13.0+0.4*i for i in range(19)]      # 13.0 .. 20.2
ys=[9.0+0.4*i for i in range(11)]       # 9.0 .. 13.0
for x in xs:
    for y in ys:
        p=os.path.join(tmp,"p.xml")
        open(p,"w").write(TPL.format(x=x,y=y))
        try:
            r=subprocess.run([SOLVER,p,"--out-jsp",os.path.join(tmp,"p.jsp")],
                             capture_output=True,text=True,timeout=120)
        except Exception:
            err+=1; continue
        m=re.search(r"remaining_agents=(\d+)",r.stdout)
        if not m: err+=1
        elif m.group(1)=="0": escaped+=1
        else: trapped+=1
print(f"entkommen {escaped}, festgesetzt {trapped}, unbrauchbar {err}  (von {len(xs)*len(ys)})")
