"""Gimbal mission — aim the camera gimbal during a flight.

Level 4 · Camera & services

What you learn
==============
* The ``Gimbal`` service: ``point(pitch_deg=, yaw_deg=)``, ``rate(yaw=,
  pitch=)``, ``stop()``, ``center()``. Each call returns at once with a
  request id (``None`` if refused); ``result(id)`` gives
  ``(success, message)`` once the gimbal controller answered.
* Reading the measured attitude from the ``gimbal`` sense, which the
  service registers for you: ``ctx.senses.gimbal.is_near(...)``.
* Hovering until the gimbal has arrived: wrap ``brake`` in a
  ``SkillStep`` with your own ``is_done``.
* Taking a photo once the gimbal is aimed: ``capture`` (needs the
  ``CameraSense`` added in ``main()`` and ``"camera"`` in
  ``requires_senses``). ``capture`` saves the latest frame it already
  has, so first wait for fresh frames taken after the gimbal arrived.
* Picking the gimbal: the ``gimbal_camera_id`` ROS parameter. In the
  simulation the gimbal controller serves ``cam_0``.

Photos are saved to ``~/.ros/captures/gimbal_<view>.png``.

Angles are body-frame degrees, pitch negative down. The gimbal never
touches flight; a rate is stopped automatically on pause, abort and
mission end.

Requires
========
* A drone with a camera gimbal (``set_gimbal`` on the world adapter).
  Without one, every command is refused ("world has no gimbal wired")
  and each wait below ends on its timeout.
* A camera. Without one, each ``capture`` fails after 5 s
  ("no camera frame within 5s"). In the simulation ``/camera`` is the
  image of the camera on the gimbal.

Run::

    python -m local_planner.examples.gimbal_mission
"""
from __future__ import annotations

import sys
from typing import Any, Iterator, Optional

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
ALT_M = 5.0
LINE_END = (12.0, 0.0)       # (north, east)
AIM_WAIT_S = 5.0
AIM_TOLERANCE_DEG = 3.0
PAN_RATE = 30.0              # normalised -100..100
PAN_S = 4.0
FRESH_FRAMES = 2             # new frames to wait for before a photo
FRAME_WAIT_S = 5.0
OUTPUT_DIR = "~/.ros/captures"
TAG = "[GIMBAL]"


def hover_until(condition, *, timeout_s: float, ctx: Any, name: str) -> SkillStep:
    """Hover in place until ``condition(ctx)`` is true or time runs out."""
    deadline = ctx.world.now() + timeout_s
    return SkillStep(
        skill=brake(name=name).skill,
        is_done=lambda c: condition(c) or c.world.now() >= deadline,
        name=name,
    )


def aim(ctx: Any, *, pitch_deg: float, yaw_deg: float, name: str) -> SkillStep:
    """Point the gimbal and hover until it reports the new attitude."""
    ctx.services.gimbal.point(pitch_deg=pitch_deg, yaw_deg=yaw_deg)
    return hover_until(
        lambda c: c.senses.gimbal.is_near(
            yaw_deg=yaw_deg, pitch_deg=pitch_deg,
            tolerance_deg=AIM_TOLERANCE_DEG),
        timeout_s=AIM_WAIT_S, ctx=ctx, name=name)


def photo(ctx: Any, view: str) -> Iterator[Any]:
    """Wait for fresh camera frames, then take one photo named after the
    view. Use with ``yield from``.

    The camera gives about 1 frame/s and ``capture`` saves the latest
    frame it has, which may predate the gimbal move.
    """
    seq0 = ctx.senses.camera.seq
    yield hover_until(lambda c: c.senses.camera.seq >= seq0 + FRESH_FRAMES,
                      timeout_s=FRAME_WAIT_S, ctx=ctx, name=f"fresh_frame_{view}")
    yield capture(output_dir=OUTPUT_DIR, filename=f"gimbal_{view}.png",
                  name=f"photo_{view}")


def log_result(ctx: Any, request_id: Optional[int], what: str) -> None:
    """Log the controller's answer to a request (``None`` = refused)."""
    answer = (None if request_id is None
              else ctx.services.gimbal.result(request_id))
    ctx.world.log_info(f"{TAG} {what}: {answer or 'no answer yet'}")


def gimbal_mission(ctx: Any) -> Iterator[Any]:
    """Look down along a line, look ahead, pan, taking a photo of each
    view, then center and land."""
    gimbal = ctx.services.gimbal

    yield takeoff(alt_m=ALT_M)

    # 1. Look straight down and fly a line (mapping / inspection view).
    yield aim(ctx, pitch_deg=-90.0, yaw_deg=0.0, name="aim_down")
    yield from photo(ctx, "down")
    yield fly_to(north=LINE_END[0], east=LINE_END[1], alt_m=ALT_M,
                 mode="coverage", name="line_looking_down")

    # 2. Look ahead and slightly down.
    yield aim(ctx, pitch_deg=-30.0, yaw_deg=0.0, name="aim_ahead")
    ctx.world.log_info(f"{TAG} target={gimbal.target}")
    yield from photo(ctx, "ahead")

    # 3. Pan right at a constant rate, then stop; the gimbal holds there.
    pan = gimbal.rate(yaw=PAN_RATE)
    yield hover_until(lambda c: False, timeout_s=PAN_S, ctx=ctx, name="pan")
    gimbal.stop()
    log_result(ctx, pan, "pan")
    yield from photo(ctx, "panned")

    # 4. Back to center before flying home.
    center = gimbal.center()
    yield hover_until(lambda c: gimbal.is_settled, timeout_s=AIM_WAIT_S,
                      ctx=ctx, name="wait_center")
    log_result(ctx, center, "center")

    yield fly_to(north=0, east=0, alt_m=ALT_M, name="return_home")
    yield brake(name="pre_land")
    yield land()


gimbal_mission.requires_senses = ["pose", "obstacle", "status", "camera"]


def main() -> None:
    sys.argv += ["--ros-args", "-p", f"gimbal_camera_id:={GIMBAL_CAMERA_ID}"]
    with boot_drone() as drone:
        drone.add_sense(CameraSense())          # before drone.fly()
        drone.add_service(Gimbal())             # also registers the gimbal sense
        drone.fly(gimbal_mission)
        print(f"[gimbal_mission] modes:    {drone.list_modes()}")
        print(f"[gimbal_mission] senses:   {drone.list_senses()}")
        print(f"[gimbal_mission] services: {drone.list_services()}")
        drone.run()


if __name__ == "__main__":
    main()
