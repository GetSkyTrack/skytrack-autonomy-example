"""Hello mission — take off, hover, land. The smallest complete mission.

Level 1 · Basics

What you learn
==============
* A mission is a plain generator function: each ``yield`` is one step.
* ``takeoff`` → ``brake`` → ``land`` is the safe start/finish pattern.
* ``requires_senses`` + ``main()`` + ``boot_drone()`` — the shape every
  mission file in this folder follows.

Run::

    python -m local_planner.examples.hello_mission
"""
from __future__ import annotations

from typing import Any, Iterator

from local_planner import boot_drone, brake, land, takeoff

ALT_M = 3.0          # takeoff altitude, metres above home (positive up)


# ── Mission — a generator function ─────────────────────────────────

def hello_mission(ctx: Any) -> Iterator[Any]:
    """Climb to 3 m, settle, land."""
    yield takeoff(alt_m=ALT_M)
    ctx.world.log_info("[HELLO] airborne")
    yield brake(name="hover")
    yield land()


# Senses this mission's steps need. Checked before the mission starts.
hello_mission.requires_senses = ["pose", "status"]


# ── Wire + run ─────────────────────────────────────────────────────

def main() -> None:
    with boot_drone() as drone:
        drone.fly(hello_mission)
        print(f"[hello_mission] modes:  {drone.list_modes()}")
        print(f"[hello_mission] senses: {drone.list_senses()}")
        drone.run()


if __name__ == "__main__":
    main()
