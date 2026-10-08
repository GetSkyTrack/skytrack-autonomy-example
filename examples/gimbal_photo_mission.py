"""Gimbal photo mission — fly out, look down, take one photo, fly back.

Level 4 · Camera & services

What you learn
==============
* A full out-and-back flight: ``takeoff`` → ``fly_to`` → ``fly_to`` →
  ``brake`` → ``land``.
* Pointing the camera straight down with the ``Gimbal`` service and
  hovering until the ``gimbal`` sense reports it got there.
* Taking a sharp photo of the new view: wait for fresh camera frames,
  then ``capture``.

Route (north, east, altitude in metres)::

    take off to 2 m → fly to (0, 10, 2) → gimbal down → photo
    → fly back to (0, 0, 2) → land

The photo is saved to ``~/.ros/captures/gimbal_photo_down.png``.

Requires
========
* A drone with a camera gimbal. In the simulation the gimbal controller
  serves camera id ``cam_0``; ``main()`` sets the ``gimbal_camera_id``
  ROS parameter to it.
* A camera (``/camera`` is the camera on the gimbal).

Run::

    python -m local_planner.examples.gimbal_photo_mission
"""
from __future__ import annotations

import sys
from typing import Any, Iterator

from local_planner import (
    CameraSense,
    SkillStep,
    boot_drone,
    brake,
    capture,
    fly_to,
    land,
    takeoff,
)
from skytrack_autonomy import Gimbal

GIMBAL_CAMERA_ID = "cam_0"   # camera whose gimbal_control service is used
ALT_M = 2.0
PHOTO_POINT = (0.0, 10.0)    # (north, east)
HOME = (0.0, 0.0)            # (north, east)
PITCH_DOWN_DEG = -90.0
AIM_WAIT_S = 5.0
AIM_TOLERANCE_DEG = 3.0
FRESH_FRAMES = 2             # new frames to wait for before the photo
FRAME_WAIT_S = 5.0
OUTPUT_DIR = "~/.ros/captures"
TAG = "[GIMBAL_PHOTO]"


def hover_until(condition, *, timeout_s: float, ctx: Any, name: str) -> SkillStep:
    """Hover in place until ``condition(ctx)`` is true or time runs out."""
    deadline = ctx.world.now() + timeout_s
    return SkillStep(
        skill=brake(name=name).skill,
        is_done=lambda c: condition(c) or c.world.now() >= deadline,
        name=name,
    )


def gimbal_photo_mission(ctx: Any) -> Iterator[Any]:
    """Take off, fly to the photo point, look down, photo, fly back, land."""
    gimbal = ctx.services.gimbal

    yield takeoff(alt_m=ALT_M)
    yield fly_to(north=PHOTO_POINT[0], east=PHOTO_POINT[1], alt_m=ALT_M,
                 name="to_photo_point")
    yield brake(name="settle_at_photo_point")

    # Point the camera straight down and wait until the gimbal is there.
    gimbal.point(pitch_deg=PITCH_DOWN_DEG, yaw_deg=0.0)
    yield hover_until(
        lambda c: c.senses.gimbal.is_near(yaw_deg=0.0, pitch_deg=PITCH_DOWN_DEG,
                                          tolerance_deg=AIM_TOLERANCE_DEG),
        timeout_s=AIM_WAIT_S, ctx=ctx, name="gimbal_down")

    # capture saves the latest frame it has: wait for frames taken after
    # the gimbal moved (~1 frame/s in flight).
    seq0 = ctx.senses.camera.seq
    yield hover_until(lambda c: c.senses.camera.seq >= seq0 + FRESH_FRAMES,
                      timeout_s=FRAME_WAIT_S, ctx=ctx, name="fresh_frame")
    yield capture(output_dir=OUTPUT_DIR, filename="gimbal_photo_down.png",
                  name="photo_down")
    ctx.world.log_info(f"{TAG} photo taken at N{PHOTO_POINT[0]:.0f} "
                       f"E{PHOTO_POINT[1]:.0f}, gimbal target={gimbal.target}")

    # Gimbal back to center before flying home.
    gimbal.center()
    yield fly_to(north=HOME[0], east=HOME[1], alt_m=ALT_M, name="return_home")
    yield brake(name="pre_land")
    yield land()


gimbal_photo_mission.requires_senses = ["pose", "obstacle", "status", "camera"]


def main() -> None:
    sys.argv += ["--ros-args", "-p", f"gimbal_camera_id:={GIMBAL_CAMERA_ID}"]
    with boot_drone() as drone:
        drone.add_sense(CameraSense())          # before drone.fly()
        drone.add_service(Gimbal())             # also registers the gimbal sense
        drone.fly(gimbal_photo_mission)
        drone.run()


if __name__ == "__main__":
    main()
