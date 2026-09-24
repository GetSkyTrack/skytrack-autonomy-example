"""Lawnmower mission — sweep a rectangle in parallel lines.

Level 2 · Flight patterns

What you learn
==============
* Generating a route with code instead of typing waypoints.
* ``mode="coverage"``: hug the straight line between points and only
  deviate where blocked — right for spraying, scanning, mapping.
  (The default ``"transit"`` takes the shortest path instead.)
* ``replan_mode="fast"``: replan while flying instead of stopping —
  smoother long lines in a mapped area.

Run::

    python -m local_planner.examples.lawnmower_mission
"""
from __future__ import annotations

from typing import Any, Iterator, List, Tuple

from local_planner import boot_drone, brake, fly_to, land, takeoff

ALT_M = 4.0
AREA_NORTH_M = 12.0          # length of each sweep line
AREA_EAST_M = 8.0            # total width covered
LINE_SPACING_M = 2.0         # distance between sweep lines
SWEEP_SPEED_M_S = 1.5
TAG = "[MOWER]"


def sweep_lines() -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
    """(start, end) of each line, alternating direction (boustrophedon)."""
    lines = []
    east = 0.0
    flip = False
    while east <= AREA_EAST_M + 1e-6:
        start, end = (0.0, east), (AREA_NORTH_M, east)
        lines.append((end, start) if flip else (start, end))
        east += LINE_SPACING_M
        flip = not flip
    return lines


def lawnmower_mission(ctx: Any) -> Iterator[Any]:
    """Sweep the area line by line, come home, land."""
    lines = sweep_lines()
    yield takeoff(alt_m=ALT_M)

    for i, (start, end) in enumerate(lines, start=1):
        # Get to the start of the line the quickest way.
        yield fly_to(north=start[0], east=start[1], alt_m=ALT_M,
                     name=f"line_{i}_start")
        # Then fly the line itself as straight as possible.
        yield fly_to(north=end[0], east=end[1], alt_m=ALT_M,
                     mode="coverage", replan_mode="fast",
                     target_speed=SWEEP_SPEED_M_S, name=f"line_{i}_sweep")
        ctx.world.log_info(f"{TAG} line {i}/{len(lines)} done")

    yield fly_to(north=0, east=0, alt_m=ALT_M, name="return_home")
    yield brake(name="pre_land")
    yield land()


lawnmower_mission.requires_senses = ["pose", "obstacle", "status"]


def main() -> None:
    with boot_drone() as drone:
        drone.fly(lawnmower_mission)
        print(f"[lawnmower_mission] modes:  {drone.list_modes()}")
        print(f"[lawnmower_mission] senses: {drone.list_senses()}")
        drone.run()


if __name__ == "__main__":
    main()
