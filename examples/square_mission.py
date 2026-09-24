"""Square mission — fly the four corners of a 5 m square, then land.

Level 1 · Basics

What you learn
==============
* ``fly_to(north=, east=, alt_m=)`` — the friendly coordinate form:
  metres north / east of home, altitude positive up.
* ``fly_to`` plans around obstacles, so it needs the ``obstacle`` sense.
* Giving each step a ``name=`` so it reads clearly in logs and reports.

Run::

    python -m local_planner.examples.square_mission
"""
from __future__ import annotations

from typing import Any, Iterator

from local_planner import boot_drone, brake, fly_to, land, takeoff

ALT_M = 3.0
SIDE_M = 5.0


def square_mission(ctx: Any) -> Iterator[Any]:
    """Takeoff, visit the 4 corners of a square, return home, land."""
    yield takeoff(alt_m=ALT_M)

    yield fly_to(north=SIDE_M, east=0, alt_m=ALT_M, name="corner_NW")
    yield fly_to(north=SIDE_M, east=SIDE_M, alt_m=ALT_M, name="corner_NE")
    yield fly_to(north=0, east=SIDE_M, alt_m=ALT_M, name="corner_SE")
    yield fly_to(north=0, east=0, alt_m=ALT_M, name="home")

    yield brake(name="pre_land")
    yield land()


square_mission.requires_senses = ["pose", "obstacle", "status"]


def main() -> None:
    with boot_drone() as drone:
        drone.fly(square_mission)
        print(f"[square_mission] modes:  {drone.list_modes()}")
        print(f"[square_mission] senses: {drone.list_senses()}")
        drone.run()


if __name__ == "__main__":
    main()
