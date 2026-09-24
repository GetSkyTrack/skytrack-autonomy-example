"""Record video mission — film one leg, grab stills while flying.

Level 4 · Camera & services

What you learn
==============
* A **service** runs next to the mission and never moves the drone.
  Add it with ``drone.add_service(...)``, use it via ``ctx.services``.
* ``VideoRecorder``: ``start(clip=...)`` / ``stop()`` around a leg.
* ``Snapshot``: take a still *without stopping* (unlike ``capture``).
* The default sink writes numbered PNG frames + ``meta.jsonl`` (pose
  per frame) and needs nothing extra. ``GStreamerSink`` writes one
  ``.mp4`` but needs the GStreamer plugins (``appsrc``, ``x264enc``)
  installed on the mission computer.

Requires
========
* A drone with a camera. Without one the recorder saves 0 frames.

Run::

    python -m local_planner.examples.record_video_mission
"""
from __future__ import annotations

from typing import Any, Iterator

from local_planner import (
    CameraSense,
    Snapshot,
    VideoRecorder,
    boot_drone,
    brake,
    fly_to,
    land,
    orbit,
    takeoff,
)

ALT_M = 4.0
TAG = "[VIDEO]"


def record_video_mission(ctx: Any) -> Iterator[Any]:
    """Record an orbit as one clip, with a still at the start and end."""
    rec = ctx.services.recorder
    snap = ctx.services.snapshot

    yield takeoff(alt_m=ALT_M)
    yield fly_to(north=2, east=0, alt_m=ALT_M, name="to_orbit_entry")

    rec.start(clip="orbit")                     # recording on
    snap.snap("orbit_start.png")
    yield orbit(center_north=5, center_east=0, alt_m=ALT_M,
                radius_m=3.0, period_s=20.0, duration_s=20.0, name="filmed_orbit")
    snap.snap("orbit_end.png")
    rec.stop()                                  # recording off
    ctx.world.log_info(f"{TAG} recorded {rec.frame_count} frames")

    yield brake(name="after_orbit")
    yield fly_to(north=0, east=0, alt_m=ALT_M, name="return_home")
    yield brake(name="pre_land")
    yield land()


record_video_mission.requires_senses = ["pose", "obstacle", "status", "camera"]


def main() -> None:
    with boot_drone() as drone:
        drone.add_sense(CameraSense())          # services read the camera
        # PNG frames under ~/.ros/recordings/<clip>/. For one .mp4 file on
        # a machine with GStreamer, pass sink=GStreamerSink(output_dir=...).
        drone.add_service(VideoRecorder(output_dir="~/.ros/recordings", fps=10.0))
        drone.add_service(Snapshot(output_dir="~/.ros/captures"))
        drone.fly(record_video_mission)
        print(f"[record_video_mission] modes:    {drone.list_modes()}")
        print(f"[record_video_mission] senses:   {drone.list_senses()}")
        print(f"[record_video_mission] services: {drone.list_services()}")
        drone.run()


if __name__ == "__main__":
    main()
