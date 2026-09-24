"""Capture photos mission — photograph each waypoint.

Level 4 · Camera & services

What you learn
==============
* Adding the ``CameraSense`` in ``main()`` **before** ``drone.fly()``.
* ``yaw_to`` to point the camera, ``brake`` to stop moving, then
  ``capture`` for a sharp still.
* Adding ``"camera"`` to ``requires_senses``.

Photos are saved to ``~/.ros/captures/survey_<n>.png``.

Requires
========
* A drone with a camera. Without one, each ``capture`` fails after
  5 s ("no camera frame within 5s") and the mission may stall.

Run::

    python -m local_planner.examples.capture_photos_mission
"""
from __future__ import annotations

from typing import Any, Iterator

from local_planner import (
    CameraSense,
    boot_drone,
    brake,
    capture,
    fly_to,
    land,
    takeoff,
    yaw_to,
)

ALT_M = 4.0
OUTPUT_DIR = "~/.ros/captures"

# (north, east) of each photo spot and the point to look at from there.
SHOTS = [
    ((6.0, 0.0), (10.0, 0.0)),
    ((6.0, 6.0), (10.0, 10.0)),
    ((0.0, 6.0), (0.0, 10.0)),
]


def capture_photos_mission(ctx: Any) -> Iterator[Any]:
    """Fly to each spot, face the subject, take a photo."""
    yield takeoff(alt_m=ALT_M)

    for i, ((north, east), (look_n, look_e)) in enumerate(SHOTS, start=1):
        yield fly_to(north=north, east=east, alt_m=ALT_M, name=f"to_spot_{i}")
        yield yaw_to(north=look_n, east=look_e, name=f"face_subject_{i}")
        yield brake(name=f"settle_{i}")                 # no motion blur
        yield capture(output_dir=OUTPUT_DIR, filename=f"survey_{i}.png",
                      name=f"photo_{i}")

    yield fly_to(north=0, east=0, alt_m=ALT_M, name="return_home")
    yield brake(name="pre_land")
    yield land()


capture_photos_mission.requires_senses = ["pose", "obstacle", "status", "camera"]


def main() -> None:
    with boot_drone() as drone:
        drone.add_sense(CameraSense())          # before drone.fly()
        drone.fly(capture_photos_mission)
        print(f"[capture_photos_mission] modes:  {drone.list_modes()}")
        print(f"[capture_photos_mission] senses: {drone.list_senses()}")
        drone.run()


if __name__ == "__main__":
    main()
