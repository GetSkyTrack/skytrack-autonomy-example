"""Orbit + helix mission — circle a point, then spiral up around it.

Level 2 · Flight patterns

What you learn
==============
* ``orbit`` circles a point at a fixed radius and altitude, nose
  pointed at the centre — good for filming or inspecting a structure.
* ``helix`` does the same while climbing (or descending) from
  ``alt_m`` to ``alt_m_end``.
* ``brake`` after a circle kills the sideways speed before landing.

Run::

    python -m local_planner.examples.orbit_helix_mission
"""
from __future__ import annotations

from typing import Any, Iterator

from local_planner import boot_drone, brake, fly_to, helix, land, orbit, takeoff

ALT_M = 3.0
TOP_ALT_M = 6.0
CENTER = (5.0, 0.0)          # (north, east) of the point of interest
RADIUS_M = 3.0
LAP_S = 12.0                 # seconds per lap


def orbit_helix_mission(ctx: Any) -> Iterator[Any]:
    """Orbit the point once, helix up to 6 m, come home, land."""
    north, east = CENTER
    yield takeoff(alt_m=ALT_M)

    yield orbit(center_north=north, center_east=east, alt_m=ALT_M,
                radius_m=RADIUS_M, period_s=LAP_S, duration_s=LAP_S,
                name="orbit_1_lap")

    yield helix(center_north=north, center_east=east,
                alt_m=ALT_M, alt_m_end=TOP_ALT_M,
                radius_m=RADIUS_M, period_s=LAP_S, duration_s=2 * LAP_S,
                name="helix_up")

    yield brake(name="after_helix")
    yield fly_to(north=0, east=0, alt_m=ALT_M, name="return_home")
    yield brake(name="pre_land")
    yield land()


orbit_helix_mission.requires_senses = ["pose", "obstacle", "status"]


def main() -> None:
    with boot_drone() as drone:
        drone.fly(orbit_helix_mission)
        print(f"[orbit_helix_mission] modes:  {drone.list_modes()}")
        print(f"[orbit_helix_mission] senses: {drone.list_senses()}")
        drone.run()


if __name__ == "__main__":
    main()
