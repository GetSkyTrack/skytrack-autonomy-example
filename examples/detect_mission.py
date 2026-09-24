"""Detect mission — look for people at each viewpoint.

Level 4 · Camera & services

What you learn
==============
* The ``Detector`` service asks the onboard AI model what it sees.
* ``request(...)`` returns at once; the answer arrives later in
  ``last_result``. Hover with a ``SkillStep`` until it is ready.
* Reacting to the result: log each hit, count them, decide what next.

Requires
========
* A drone with a camera and the ``object_detection`` server running.
  Each detection takes about 1–1.5 s in simulation. In an empty Gazebo
  world the result is 0 detections, which is expected; add a person
  model to the world to see hits.

Run::

    python -m local_planner.examples.detect_mission
"""
from __future__ import annotations

from typing import Any, Iterator

from local_planner import SkillStep, boot_drone, brake, fly_to, land, takeoff, yaw_to
from skytrack_autonomy import Detector

ALT_M = 5.0
MODEL = "det-coco-v26n-b-quantized-fp16"
CLASSES = ["human"]
DETECT_TIMEOUT_S = 15.0
TAG = "[DETECT]"

# (north, east) viewpoint and the point to look at.
VIEWPOINTS = [
    ((5.0, 0.0), (15.0, 0.0)),
    ((5.0, 5.0), (15.0, 15.0)),
]


def wait_for_detection(ctx: Any, name: str) -> SkillStep:
    """Hover until the detector has finished (or the timeout)."""
    det = ctx.services.detector
    deadline = ctx.world.now() + DETECT_TIMEOUT_S
    return SkillStep(
        skill=brake(name=name).skill,
        is_done=lambda c: not det.is_busy or c.world.now() >= deadline,
        name=name,
    )


def detect_mission(ctx: Any) -> Iterator[Any]:
    """At each viewpoint: face the area, detect, log what was found."""
    det = ctx.services.detector
    total = 0

    yield takeoff(alt_m=ALT_M)
    for i, ((north, east), (look_n, look_e)) in enumerate(VIEWPOINTS, start=1):
        yield fly_to(north=north, east=east, alt_m=ALT_M, name=f"to_view_{i}")
        yield yaw_to(north=look_n, east=look_e, name=f"face_area_{i}")
        yield brake(name=f"settle_{i}")

        if not det.request(model_name=MODEL, classes=CLASSES,
                           confidence_threshold=0.5):
            ctx.world.log_warn(f"{TAG} detector refused request at view {i}")
            continue
        yield wait_for_detection(ctx, name=f"detect_{i}")

        result = det.last_result
        if result is None or not result.success:
            ctx.world.log_warn(f"{TAG} no result at view {i}")
            continue
        for d in result.detections:
            ctx.world.log_info(
                f"{TAG} view {i}: {d.class_name} {d.score:.0%} at pixel "
                f"({d.bbox.center_x:.0f}, {d.bbox.center_y:.0f})")
        total += result.num_detections

    ctx.world.log_info(f"{TAG} total detections: {total}")
    yield fly_to(north=0, east=0, alt_m=ALT_M, name="return_home")
    yield brake(name="pre_land")
    yield land()


detect_mission.requires_senses = ["pose", "obstacle", "status"]


def main() -> None:
    with boot_drone() as drone:
        drone.add_service(Detector(model_name=MODEL, classes=CLASSES))
        drone.fly(detect_mission)
        print(f"[detect_mission] modes:    {drone.list_modes()}")
        print(f"[detect_mission] senses:   {drone.list_senses()}")
        print(f"[detect_mission] services: {drone.list_services()}")
        drone.run()


if __name__ == "__main__":
    main()
