"""Waypoints mission — fly a list of waypoints with a loop.

Level 1 · Basics

What you learn
==============
* Missions are normal Python: keep the route as data, loop over it.
* ``target_speed`` sets the cruise speed for one leg only.
* Logging progress with ``ctx.world.log_info``.

Run::

    python -m local_planner.examples.waypoints_mission
"""
from __future__ import annotations

from typing import Any, Iterator

from local_planner import boot_drone, brake, fly_to, land, takeoff

ALT_M = 3.0
SPEED_M_S = 2.0
TAG = "[WAYPOINTS]"

# (north, east) in metres from home.
WAYPOINTS = [
    (6.0, 0.0),
    (6.0, 6.0),
    (0.0, 6.0),
    (-4.0, 2.0),
]


def waypoints_mission(ctx: Any) -> Iterator[Any]:
    """Visit every waypoint in order, come home, land."""
    yield takeoff(alt_m=ALT_M)

    for i, (north, east) in enumerate(WAYPOINTS, start=1):
        yield fly_to(north=north, east=east, alt_m=ALT_M,
                     target_speed=SPEED_M_S, name=f"wp_{i}")
        ctx.world.log_info(f"{TAG} reached waypoint {i}/{len(WAYPOINTS)}")

    yield fly_to(north=0, east=0, alt_m=ALT_M, name="return_home")
    yield brake(name="pre_land")
    yield land()


waypoints_mission.requires_senses = ["pose", "obstacle", "status"]


def main() -> None:
    with boot_drone() as drone:
        drone.fly(waypoints_mission)
        print(f"[waypoints_mission] modes:  {drone.list_modes()}")
        print(f"[waypoints_mission] senses: {drone.list_senses()}")
        drone.run()


if __name__ == "__main__":
    main()
