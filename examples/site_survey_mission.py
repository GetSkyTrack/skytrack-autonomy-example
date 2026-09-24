"""Site survey mission — a complete job using everything so far.

Level 6 · Putting it together

The job: survey a site in lawnmower lines while recording video,
photograph three inspection points, stay inside a geofence, log
telemetry, and go home early if the battery runs low.

What it combines
================
* Route generated in code + ``mode="coverage"``    (lawnmower_mission)
* Sub-missions with ``yield from``                  (compose_mission)
* Battery check + early return                      (battery_check_mission)
* ``CameraSense``, ``capture``, ``VideoRecorder``   (capture_photos / record_video)
* Custom sense ``GeofenceSense``                    (custom_sense_mission)
* Custom skill ``HoverForSeconds``                  (custom_skill_mission)
* Custom service ``TelemetryLogger``                (custom_service_mission)

Requires
========
* A drone with a camera.
* The three Level 5 files (``custom_sense_mission.py``,
  ``custom_skill_mission.py``, ``custom_service_mission.py``) in the
  same folder as this one.

Run::

    python -m local_planner.examples.site_survey_mission

or as a script, e.g. ``python3 /path/to/site_survey_mission.py``.
"""
from __future__ import annotations

import os
import sys
from typing import Any, Iterator, List, Tuple

from local_planner import (
    CameraSense,
    VideoRecorder,
    boot_drone,
    brake,
    capture,
    fly_to,
    land,
    takeoff,
    yaw_to,
)

# Parts built in the Level 5 examples. The relative import works with
# ``python -m``; run as a plain script there is no parent package, so
# fall back to importing the sibling files from this file's folder.
try:
    from .custom_service_mission import TelemetryLogger
    from .custom_sense_mission import GeofenceSense
    from .custom_skill_mission import HoverForSeconds
except ImportError:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from custom_service_mission import TelemetryLogger
    from custom_sense_mission import GeofenceSense
    from custom_skill_mission import HoverForSeconds

TAG = "[SURVEY]"

# ── Job settings ───────────────────────────────────────────────────
ALT_M = 5.0
AREA_NORTH_M = 16.0
AREA_EAST_M = 10.0
LINE_SPACING_M = 2.5
SWEEP_SPEED_M_S = 2.0
MIN_BATTERY_PERCENT = 35.0
FENCE_RADIUS_M = 25.0
FENCE_MAX_ALT_M = 15.0

# (name, north, east) of each inspection point.
INSPECTION_POINTS = [
    ("gate", 0.0, 10.0),
    ("tank", 16.0, 10.0),
    ("shed", 16.0, 0.0),
]


# ── Helpers ────────────────────────────────────────────────────────

def sweep_lines() -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
    lines, east, flip = [], 0.0, False
    while east <= AREA_EAST_M + 1e-6:
        start, end = (0.0, east), (AREA_NORTH_M, east)
        lines.append((end, start) if flip else (start, end))
        east, flip = east + LINE_SPACING_M, not flip
    return lines


def battery_low(ctx: Any) -> bool:
    percent = ctx.senses.battery.percent
    return percent is not None and percent < MIN_BATTERY_PERCENT


# ── Sub-missions ───────────────────────────────────────────────────

def survey_area(ctx: Any) -> Iterator[Any]:
    """Sweep the area while recording. Stops early on low battery."""
    rec, tlm, fence = ctx.services.recorder, ctx.services.telemetry, ctx.senses.geofence
    rec.start(clip="area_sweep")
    tlm.mark("sweep start")
    for i, (start, end) in enumerate(sweep_lines(), start=1):
        if battery_low(ctx):
            ctx.world.log_warn(f"{TAG} battery low, stopping sweep at line {i}")
            break
        if not (fence.allows(*start, ALT_M) and fence.allows(*end, ALT_M)):
            ctx.world.log_warn(f"{TAG} line {i} leaves the fence, skipping")
            continue
        yield fly_to(north=start[0], east=start[1], alt_m=ALT_M, name=f"line_{i}_start")
        yield fly_to(north=end[0], east=end[1], alt_m=ALT_M, mode="coverage",
                     replan_mode="fast", target_speed=SWEEP_SPEED_M_S,
                     name=f"line_{i}_sweep")
    rec.stop()
    tlm.mark("sweep end")


def inspect_points(ctx: Any) -> Iterator[Any]:
    """Photograph each point from 3 m to the south-west."""
    tlm = ctx.services.telemetry
    for name, north, east in INSPECTION_POINTS:
        if battery_low(ctx):
            ctx.world.log_warn(f"{TAG} battery low, skipping remaining points")
            return
        yield fly_to(north=north - 3.0, east=east - 3.0, alt_m=ALT_M, name=f"to_{name}")
        yield yaw_to(north=north, east=east, name=f"face_{name}")
        yield HoverForSeconds(2)                    # let the gimbal settle
        yield capture(filename=f"inspect_{name}.png", name=f"photo_{name}")
        tlm.mark(f"photographed {name}")


def go_home(ctx: Any) -> Iterator[Any]:
    yield fly_to(north=0, east=0, alt_m=ALT_M, name="return_home")
    yield brake(name="pre_land")
    yield land()


# ── The mission ────────────────────────────────────────────────────

def site_survey_mission(ctx: Any) -> Iterator[Any]:
    """Sweep, inspect, return — safely."""
    yield takeoff(alt_m=ALT_M)
    ctx.services.telemetry.mark("airborne")

    yield from survey_area(ctx)
    if not battery_low(ctx):
        yield from inspect_points(ctx)
    yield from go_home(ctx)

    ctx.world.log_info(
        f"{TAG} done; geofence breaches: {ctx.senses.geofence.breaches}")


site_survey_mission.requires_senses = [
    "pose", "obstacle", "status", "battery", "camera", "geofence",
]


# ── Wire + run ─────────────────────────────────────────────────────

def main() -> None:
    with boot_drone() as drone:
        drone.add_sense(CameraSense())
        drone.add_sense(GeofenceSense(radius_m=FENCE_RADIUS_M,
                                      max_alt_m=FENCE_MAX_ALT_M))
        drone.add_service(VideoRecorder(output_dir="~/.ros/recordings", fps=10.0))
        drone.add_service(TelemetryLogger(hz=5.0))
        drone.fly(site_survey_mission)
        print(f"[site_survey_mission] modes:    {drone.list_modes()}")
        print(f"[site_survey_mission] senses:   {drone.list_senses()}")
        print(f"[site_survey_mission] services: {drone.list_services()}")
        drone.run()


if __name__ == "__main__":
    main()
