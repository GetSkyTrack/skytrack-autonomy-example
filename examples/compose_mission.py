"""Compose mission — build missions from reusable sub-missions.

Level 3 · Mission logic

What you learn
==============
* ``yield from sub_mission(ctx, ...)`` inlines another generator, so
  you can keep a small library of reusable pieces.
* Sub-missions take parameters like any function.
* The main mission reads like a checklist.

Run::

    python -m local_planner.examples.compose_mission
"""
from __future__ import annotations

from typing import Any, Iterator

from local_planner import boot_drone, brake, fly_to, land, orbit, takeoff

ALT_M = 3.0
TAG = "[COMPOSE]"

# Points of interest: (name, north, east).
TARGETS = [
    ("tower_A", 6.0, 0.0),
    ("tower_B", 6.0, 8.0),
]


# ── Reusable sub-missions ───────────────────────────────────────────

def climb_out(ctx: Any, *, alt_m: float) -> Iterator[Any]:
    """Take off and settle."""
    yield takeoff(alt_m=alt_m)
    yield brake(name="settle_after_takeoff")


def inspect_point(ctx: Any, *, name: str, north: float, east: float,
                  alt_m: float) -> Iterator[Any]:
    """Fly to a stand-off point south of the target, then circle it."""
    yield fly_to(north=north - 3.0, east=east, alt_m=alt_m, name=f"approach_{name}")
    yield orbit(center_north=north, center_east=east, alt_m=alt_m,
                radius_m=3.0, period_s=15.0, duration_s=15.0, name=f"orbit_{name}")
    yield brake(name=f"settle_{name}")
    ctx.world.log_info(f"{TAG} inspected {name}")


def return_and_land(ctx: Any, *, alt_m: float) -> Iterator[Any]:
    """Fly home, settle, land."""
    yield fly_to(north=0, east=0, alt_m=alt_m, name="return_home")
    yield brake(name="pre_land")
    yield land()


# ── The mission ─────────────────────────────────────────────────────

def compose_mission(ctx: Any) -> Iterator[Any]:
    """Take off, inspect every target, return, land."""
    yield from climb_out(ctx, alt_m=ALT_M)
    for name, north, east in TARGETS:
        yield from inspect_point(ctx, name=name, north=north, east=east, alt_m=ALT_M)
    yield from return_and_land(ctx, alt_m=ALT_M)


compose_mission.requires_senses = ["pose", "obstacle", "status"]


def main() -> None:
    with boot_drone() as drone:
        drone.fly(compose_mission)
        print(f"[compose_mission] modes:  {drone.list_modes()}")
        print(f"[compose_mission] senses: {drone.list_senses()}")
        drone.run()


if __name__ == "__main__":
    main()
