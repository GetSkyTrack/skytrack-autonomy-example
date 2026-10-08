"""Hackathon example v1: survey the field, find stressed crop, spray it, land.

1. Fly the survey route, taking a picture every second.
2. Map yellow pixels onto a ground grid; save stressed areas for the report.
3. Spray each area with parallel lanes clipped to its outline.
4. Return home and land.

Coordinates are world ENU (x east, y north, z up). ``fly_to`` and the pose
are NED: ``fly_to(north=y, east=x)`` and ``east, north = pose.y, pose.x``.

Run::

    python -m local_planner.examples.hackathon_example_v1
"""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Iterator

import cv2
import numpy as np

from local_planner import boot_drone, brake, fly_to, land, takeoff
from skytrack_autonomy import Sprayer
from skytrack_autonomy.core.lib.scheduling import ScheduleGroup

# ── Survey ────────────────────────────────────────────────────────────
SURVEY_WAYPOINTS_ENU = [            # (x, y, z), flown in order
    (-4.63, -11.2, 5.0),
    (-3.22, -50.63, 5.0),
]
SURVEY_SPEED_M_S = 5.0
MAP_HZ = 1.0                        # pictures per second while surveying

# ── Stress detection ─────────────────────────────────────────────────
STRESS_AREA_PATH = Path("/root/.ros/captures/stress_area.json")   # attached to the report by the app
YELLOW_HSV_LOW = (15, 40, 40)       # OpenCV HSV: H 0-180, S/V 0-255
YELLOW_HSV_HIGH = (28, 255, 255)    # healthy crop is H 34-37
CELL_M = 0.2                        # ground map cell size
MIN_SAMPLES_PER_CELL = 2            # samples a cell needs to count
MIN_AREA_M2 = 2.0                   # smaller areas are dropped as noise
CAMERA_HFOV_RAD = 1.74
CAMERA_HEIGHT_AT_HOME_M = 0.2       # camera height above ground before takeoff

# ── Spray ─────────────────────────────────────────────────────────────
SPRAY_ALT_M = 3.0
SPRAY_SPEED_M_S = 5.0
SPRAY_CONE_DEG = 30.0               # full cone angle
SPRAY_SWATH_M = 2.0 * SPRAY_ALT_M * math.tan(math.radians(SPRAY_CONE_DEG / 2.0))
SPRAY_LANE_SPACING_M = SPRAY_SWATH_M * 0.85     # 15% overlap between lanes
MIN_SPRAY_RUN_M = 0.5               # shorter stretches are skipped


def hackathon_mission(ctx: Any) -> Iterator[Any]:
    """Survey, detect, spray, land."""
    log = ctx.world.log_info
    home = ctx.senses.pose.current_position
    home_north, home_east = home.x, home.y
    stress_map = StressMap(SURVEY_WAYPOINTS_ENU)

    # 1. Take off and fly to the first waypoint (not mapped).
    yield takeoff(alt_m=SURVEY_WAYPOINTS_ENU[0][2])
    start_x, start_y, start_z = SURVEY_WAYPOINTS_ENU[0]
    yield fly_to(north=start_y, east=start_x, alt_m=start_z,
                 target_speed=SURVEY_SPEED_M_S, name="fly_to_survey_start")

    # 2. Survey the remaining waypoints, mapping on a timer.
    legs = SURVEY_WAYPOINTS_ENU[1:]
    mapping = ctx.scheduler.schedule(lambda: stress_map.snap(ctx), hz=MAP_HZ,
                                     group=ScheduleGroup.MEDIA, name="mapper",
                                     now=ctx.world.now())
    try:
        for i, (x, y, z) in enumerate(legs, start=1):
            yield fly_to(north=y, east=x, alt_m=z, target_speed=SURVEY_SPEED_M_S,
                         mode="coverage", name=f"survey_leg_{i:02d}_of_{len(legs)}")
    finally:
        ctx.scheduler.unschedule(mapping)

    # 3. Save the stress areas.
    areas = stress_map.stress_areas()
    save_stress_areas(areas)
    log(f"[EXAMPLE] {stress_map.pictures} picture(s) -> {len(areas)} stress area(s), "
        f"saved to {STRESS_AREA_PATH}")

    # 4. Spray each lane: fly to its start, open the valve, fly to its end, close.
    sprayer = ctx.services.sprayer
    for a, area in enumerate(areas, start=1):
        for k, (start, end) in enumerate(spray_lanes(area["polygon"]), start=1):
            yield fly_to(north=start[1], east=start[0], alt_m=SPRAY_ALT_M,
                         target_speed=SPRAY_SPEED_M_S, name=f"area_{a:02d}_lane_{k:02d}_start")
            sprayer.on()
            yield fly_to(north=end[1], east=end[0], alt_m=SPRAY_ALT_M,
                         target_speed=SPRAY_SPEED_M_S, mode="coverage",
                         name=f"area_{a:02d}_lane_{k:02d}_spray")
            sprayer.off()
    log("[EXAMPLE] spraying done")

    # 5. Return home and land.
    yield fly_to(north=home_north, east=home_east, alt_m=SURVEY_WAYPOINTS_ENU[0][2],
                 target_speed=SURVEY_SPEED_M_S, name="return_home")
    yield brake()
    yield land()


hackathon_mission.requires_senses = ["pose", "obstacle", "status", "camera"]


def save_stress_areas(areas: list[dict[str, Any]]) -> None:
    """Write ``{"stress_area": areas}`` whole, so a copy never catches half a file.

    The temp file must not be named ``stress_area.json``: the app picks up
    that name anywhere under captures.
    """
    STRESS_AREA_PATH.parent.mkdir(parents=True, exist_ok=True)
    tmp = STRESS_AREA_PATH.with_name(".stress_area.json.tmp")
    tmp.write_text(json.dumps({"stress_area": areas}, separators=(",", ":")))
    tmp.replace(STRESS_AREA_PATH)


class StressMap:
    """Ground grid counting, per cell, how many pixels were seen and were yellow."""

    def __init__(self, waypoints_enu: list[tuple[float, float, float]]) -> None:
        margin = 25.0                   # room around the route for the picture footprint
        xs = [p[0] for p in waypoints_enu]
        ys = [p[1] for p in waypoints_enu]
        self.x0 = min(xs) - margin
        self.y0 = min(ys) - margin
        rows = int((max(ys) + margin - self.y0) / CELL_M) + 1
        cols = int((max(xs) + margin - self.x0) / CELL_M) + 1
        self.seen = np.zeros((rows, cols), dtype=np.int32)
        self.yellow = np.zeros((rows, cols), dtype=np.int32)
        self.pictures = 0
        self._last_seq = -1

    def snap(self, ctx: Any) -> None:
        """Take a picture with the Snapshot service and add it to the map."""
        camera = ctx.senses.camera
        if not camera.has_frame or camera.seq == self._last_seq:   # no new frame yet
            return
        pose = ctx.senses.pose.current_position
        if pose is None:
            return
        path = ctx.services.snapshot.snap()
        picture = cv2.imread(path) if path else None      # BGR
        if picture is None:
            return
        self.add_picture(cv2.cvtColor(picture, cv2.COLOR_BGR2RGB), pose)
        self._last_seq = camera.seq
        self.pictures += 1

    def add_picture(self, rgb: np.ndarray, pose: Any) -> None:
        """Project every 2nd pixel onto the ground and count it in its cell."""
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        is_yellow = cv2.inRange(hsv, YELLOW_HSV_LOW, YELLOW_HSV_HIGH) > 0

        # Nadir camera: picture top = drone front.
        h, w = is_yellow.shape
        v, u = np.mgrid[0:h:2, 0:w:2]
        focal = w / (2.0 * math.tan(CAMERA_HFOV_RAD / 2.0))
        height = -pose.z + CAMERA_HEIGHT_AT_HOME_M
        right = (u - w / 2.0) / focal * height          # metres right of the drone
        back = (v - h / 2.0) / focal * height           # metres behind the drone
        cos_h, sin_h = math.cos(pose.heading), math.sin(pose.heading)
        east = pose.y + right * cos_h - back * sin_h
        north = pose.x - right * sin_h - back * cos_h

        row = ((north - self.y0) / CELL_M).astype(int)
        col = ((east - self.x0) / CELL_M).astype(int)
        inside = (row >= 0) & (row < self.seen.shape[0]) & (col >= 0) & (col < self.seen.shape[1])
        row, col = row[inside], col[inside]
        np.add.at(self.seen, (row, col), 1)
        np.add.at(self.yellow, (row, col), is_yellow[v, u][inside].astype(np.int32))

    def stress_areas(self) -> list[dict[str, Any]]:
        """Outlines of cells yellow in at least half their samples, as ENU polygons."""
        stressed = (self.yellow * 2 >= self.seen) & (self.seen >= MIN_SAMPLES_PER_CELL)
        contours, _ = cv2.findContours(stressed.astype(np.uint8),
                                       cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        areas = []
        for contour in contours:
            if len(contour) < 3 or cv2.contourArea(contour) * CELL_M ** 2 < MIN_AREA_M2:
                continue
            polygon = [[float(self.x0 + (col + 0.5) * CELL_M), float(self.y0 + (row + 0.5) * CELL_M)]
                       for col, row in contour[:, 0]]
            areas.append({"class": "stressed", "polygon": polygon})
        return areas


def spray_lanes(polygon: list[list[float]]) -> list[tuple[tuple[float, float], tuple[float, float]]]:
    """Parallel spray lanes over the polygon, clipped to it, as (start, end) ENU pairs.

    Lanes run along the polygon's longer side and alternate direction.
    """
    along_north = (max(p[1] for p in polygon) - min(p[1] for p in polygon)) > \
                  (max(p[0] for p in polygon) - min(p[0] for p in polygon))
    # Work in (along, across) coordinates, then convert back to ENU.
    points = [(p[1], p[0]) if along_north else (p[0], p[1]) for p in polygon]
    out = (lambda along, across: (across, along)) if along_north else \
          (lambda along, across: (along, across))

    low = min(p[1] for p in points)
    high = max(p[1] for p in points)
    count = max(1, math.ceil((high - low) / SPRAY_LANE_SPACING_M))
    step = (high - low) / count         # even spacing across the polygon

    lanes = []
    for i in range(count):
        across = low + (i + 0.5) * step
        runs = [(a, b) for a, b in _inside_runs(points, across) if b - a >= MIN_SPRAY_RUN_M]
        if i % 2:                       # reverse every other lane
            runs = [(b, a) for a, b in reversed(runs)]
        lanes.extend((out(a, across), out(b, across)) for a, b in runs)
    return lanes


def _inside_runs(points: list[tuple[float, float]], across: float) -> list[tuple[float, float]]:
    """(start, end) stretches where the line at ``across`` is inside the polygon."""
    hits = []
    for (a_along, a_across), (b_along, b_across) in zip(points, points[1:] + points[:1]):
        if (a_across > across) == (b_across > across):
            continue                    # edge does not cross the line
        t = (across - a_across) / (b_across - a_across)
        hits.append(a_along + t * (b_along - a_along))
    hits.sort()
    return list(zip(hits[::2], hits[1::2]))


def main() -> None:
    with boot_drone() as drone:         # camera and Snapshot come with the node
        drone.add_service(Sprayer())
        drone.fly(hackathon_mission)
        drone.run()


if __name__ == "__main__":
    main()
