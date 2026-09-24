"""Heading control mission — choose where the drone's nose points.

Level 2 · Flight patterns

What you learn
==============
* ``yaw_mode="course"`` (default): the nose follows the path and the
  drone stops to turn at sharp corners.
* ``yaw_mode="hold"``: the heading is never commanded — the drone
  "crabs" sideways, keeping whatever heading it had.
* ``yaw_rate_deg_s``: how fast a "course" leg turns.
* ``yaw_to``: rotate in place to face a point, without moving.

Run::

    python -m local_planner.examples.heading_control_mission
"""
from __future__ import annotations

from typing import Any, Iterator

from local_planner import boot_drone, brake, fly_to, land, takeoff, yaw_to

ALT_M = 3.0


def heading_control_mission(ctx: Any) -> Iterator[Any]:
    """Three legs with three heading policies, then face home."""
    yield takeoff(alt_m=ALT_M)

    # Nose follows the path, turning slowly (20°/s) onto each leg.
    yield fly_to(north=6, east=0, alt_m=ALT_M,
                 yaw_mode="course", yaw_rate_deg_s=20, name="leg_course_slow")

    # Same policy, faster turns.
    yield fly_to(north=6, east=6, alt_m=ALT_M,
                 yaw_mode="course", yaw_rate_deg_s=90, name="leg_course_fast")

    # Keep the current heading while sliding sideways.
    yield fly_to(north=0, east=6, alt_m=ALT_M,
                 yaw_mode="hold", name="leg_hold_heading")

    # Turn on the spot to look back at home.
    yield yaw_to(north=0, east=0, name="face_home")

    yield fly_to(north=0, east=0, alt_m=ALT_M, name="return_home")
    yield brake(name="pre_land")
    yield land()


heading_control_mission.requires_senses = ["pose", "obstacle", "status"]


def main() -> None:
    with boot_drone() as drone:
        drone.fly(heading_control_mission)
        print(f"[heading_control_mission] modes:  {drone.list_modes()}")
        print(f"[heading_control_mission] senses: {drone.list_senses()}")
        drone.run()


if __name__ == "__main__":
    main()
