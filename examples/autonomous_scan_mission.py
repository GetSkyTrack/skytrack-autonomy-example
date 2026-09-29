"""Autonomous scan -> detect -> spray mission (X500 nozzle + camera).

Level 6
What you learn: Complex multi-phase mission, geometric planning, dynamic battery estimation, background services (mapping and spraying).
Requires: Camera, Sprayer
Run::
    python -m local_planner.examples.autonomous_scan_mission

Strategy
========
1. SCAN   High and fast (30 m, 7 m/s) in long lanes aligned strictly parallel to both
          the left and right major polygon borders, mapping yellow pixels.
          Uses 40% footprint overlap (+10% increased accuracy).
2. SPRAY  Low and slow (3 m, ~5 m/s). Uses 25% swath overlap (+10% increased accuracy).
          The valve is opened by a position + speed gate, so it is only open while
          the drone is inside the target run AND at cruise speed.
3. POWER  Battery (15 min) and mission clock (90 min) are estimated before every
          lane. If a lane plus the flight back to the nearest pad does not fit,
          the drone goes to a pad, lands, charges, resumes.
4. SAFETY No-fly rectangles (residential + 5 m buffer + margin) are cut out of every
          lane and every transit route is planned around them.

Coordinates are world ENU (x east, y north). ``fly_to(north=y, east=x)`` and the
pose is NED: ``east, north = pose.y, pose.x``.
"""
from __future__ import annotations

import heapq
import json
import math
from pathlib import Path
from typing import Any, Iterator

import cv2
import numpy as np

from local_planner import boot_drone, brake, brake_and_settle, fly_to, land, takeoff
from skytrack_autonomy import Sprayer
from skytrack_autonomy.core.lib.scheduling import ScheduleGroup

Pt = tuple[float, float]          # (x east, y north)

# ══ World Layout ═════════════════════════════════════════════════════════════
PADS = {
    "CS1": (0.00, 0.00),
    "CS2": (329.32, -234.73),
    "CS3": (356.43, -654.32),
    "CS4": (41.72, -582.67),
}

# AOI boundary outline (ENU metres)
AOI_POLYGONS: list[list[Pt]] = [
    [(-78.67, 50.77), (378.32, 104.18), (448.42, -641.53), (424.78, -660.47),
     (227.35, -712.59), (158.80, -609.15), (35.82, -641.37)],
]

# Exact coordinates for the 3 no-fly zones
ZONE_1_PTS = [
    (322.520, -223.330), (313.757, -222.479), (305.961, -231.013),
    (307.462, -243.437), (322.615, -248.834), (326.240, -244.500),
    (325.120, -239.924), (324.990, -234.255), (325.411, -228.423)
]
ZONE_2_PTS = [
    (36.033, -596.647), (170.996, -572.019), (228.033, -653.620),
    (324.290, -643.696), (342.996, -658.942), (351.907, -678.162),
    (380.876, -665.190), (372.742, -641.293), (444.092, -601.030),
    (450.635, -626.219), (424.436, -660.437), (229.897, -714.460),
    (162.650, -612.493), (41.494, -632.769)
]
ZONE_3_PTS = [
    (377.969, 101.711), (385.962, 50.478), (372.808, -1.237),
    (366.570, -6.173), (348.243, -7.688), (341.196, 3.197),
    (334.850, 20.772), (312.815, 21.571), (248.884, 2.240),
    (235.677, 17.248), (205.699, 18.592), (198.071, -2.722),
    (181.145, -8.342), (148.987, -12.423), (135.177, -2.891),
    (125.547, -10.772), (110.886, -8.477), (107.682, -3.273),
    (98.105, -3.938), (94.562, -8.081), (87.159, -9.309),
    (68.483, -3.714), (62.533, 3.773), (60.982, 6.839),
    (50.878, 5.446), (43.325, 1.545), (40.471, -5.048),
    (32.504, -10.741), (14.419, -17.552), (9.170, -4.406),
    (6.992, 0.137), (0.277, 3.197), (-5.536, 0.619),
    (-6.088, -3.366), (-6.029, -9.157), (-10.118, -15.587),
    (-14.951, -13.513), (-23.984, -9.263), (-25.785, -3.495),
    (-69.843, -8.048), (-79.167, 47.698), (370.576, 102.435)
]

def get_bbox(pts: list[Pt]) -> tuple[float, float, float, float]:
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return (min(xs), min(ys), max(xs), max(ys))

NO_FLY_RECTS: list[tuple[float, float, float, float]] = [
    get_bbox(ZONE_1_PTS),
    get_bbox(ZONE_2_PTS),
    get_bbox(ZONE_3_PTS),
]

NO_FLY_MARGIN_M = 8.0             # 5 m rule + 3 m safety buffer
SPRAY_EXTRA_MARGIN_M = 1.5

# ══ Mission Limits ═══════════════════════════════════════════════════════════
BATTERY_S = 900.0                 # 15 min battery life
BATTERY_USABLE = 0.85             # plan with 85% usable battery
RESERVE_S = 45.0                  # spare time for landing
MISSION_S = 90 * 60.0
END_RESERVE_S = 240.0
CHARGE_EST_S = 90.0
LAND_S = 10.0

# ══ Flight Parameters ════════════════════════════════════════════════════════
TRANSIT_SPEED_M_S = 8.0
SHORT_HOP_M = 60.0
CLIMB_M = 25.0
DESCEND_M = 30.0

# ══ Scan Parameters ══════════════════════════════════════════════════════════
SCAN_ALT_M = 30.0
SCAN_SPEED_M_S = 7.0
CAM_FX = 269.968
CAM_CX, CAM_CY = 320.0, 240.0
CAM_AHEAD_M = 0.125
CAM_HEIGHT_AT_HOME_M = 0.217
EFFECTIVE_SCAN_ALT_M = SCAN_ALT_M + CAM_HEIGHT_AT_HOME_M

# Exact Field of View calculation based on camera intrinsics
SCAN_WIDTH_M = 2.0 * (CAM_CX / CAM_FX) * EFFECTIVE_SCAN_ALT_M
# Increased overlap by +10% (30% -> 40% overlap for enhanced mapping precision)
SCAN_SPACING_M = 0.60 * SCAN_WIDTH_M

# Boundary margins pull drone center inside field while camera covers the edges
SCAN_MARGIN_ACROSS_M = (CAM_CX / CAM_FX) * EFFECTIVE_SCAN_ALT_M * 0.85
SCAN_MARGIN_ALONG_M = (CAM_CY / CAM_FX) * EFFECTIVE_SCAN_ALT_M * 0.85

MAP_HZ = 1.0

# ══ Stress Detection Parameters ══════════════════════════════════════════════
YELLOW_HSV_LOW = (15, 40, 40)
YELLOW_HSV_HIGH = (28, 255, 255)
CELL_M = 0.5
MIN_SAMPLES_PER_CELL = 2
YELLOW_FRACTION = 0.5
MIN_AREA_M2 = 2.0
STRESS_AREA_PATH = Path.home() / ".ros/captures/stress_area.json"

# ══ Spray Parameters ═════════════════════════════════════════════════════════
SPRAY_ALT_M = 3.0
SPRAY_SPEED_MAX_M_S = 5.0
TARGET_MEAN_DOSE = 1.5
FLOW_ML_S = 1000.0 / 60.0
EFFICIENCY = 0.7
NOZZLE_HEIGHT_M = SPRAY_ALT_M + 0.117
SPRAY_SWATH_M = 0.536 * NOZZLE_HEIGHT_M
SPRAY_SPEED_M_S = min(SPRAY_SPEED_MAX_M_S,
                      FLOW_ML_S * EFFICIENCY / (SPRAY_SWATH_M * TARGET_MEAN_DOSE))
PREDICTED_DOSE = FLOW_ML_S * EFFICIENCY / (SPRAY_SWATH_M * SPRAY_SPEED_M_S)

# Increased overlap by +10% (15% -> 25% swath overlap for target accuracy)
SPRAY_LANE_SPACING_M = 0.75 * SPRAY_SWATH_M

MIN_SPRAY_RUN_M = 0.5
RUN_IN_M = 12.0
GATE_HZ = 10.0
GATE_MIN_SPEED_FRAC = 0.85
GATE_MAX_OFFSET_M = 3.0

TAG = "[FARM]"


# ══ Geometry Helper Functions ════════════════════════════════════════════════
def dist(p: Pt, q: Pt) -> float:
    return math.hypot(q[0] - p[0], q[1] - p[1])


def lerp(p: Pt, q: Pt, t: float) -> Pt:
    return (p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t)


def inflate(r: tuple[float, float, float, float], m: float) -> tuple[float, float, float, float]:
    return (r[0] - m, r[1] - m, r[2] + m, r[3] + m)


def _seg_rect_interval(p: Pt, q: Pt, r: tuple[float, float, float, float]) -> tuple[float, float] | None:
    dx, dy = q[0] - p[0], q[1] - p[1]
    t0, t1 = 0.0, 1.0
    for d, lo, hi, o in ((dx, r[0], r[2], p[0]), (dy, r[1], r[3], p[1])):
        if abs(d) < 1e-9:
            if o <= lo or o >= hi:
                return None
        else:
            a, b = (lo - o) / d, (hi - o) / d
            if a > b:
                a, b = b, a
            t0, t1 = max(t0, a), min(t1, b)
            if t0 >= t1:
                return None
    return t0, t1


def seg_blocked(p: Pt, q: Pt, rects: list[tuple[float, float, float, float]]) -> bool:
    return any(_seg_rect_interval(p, q, r) for r in rects)


def clip_outside(p: Pt, q: Pt, rects: list[tuple[float, float, float, float]]) -> list[tuple[Pt, Pt]]:
    cuts = sorted(iv for r in rects if (iv := _seg_rect_interval(p, q, r)))
    merged: list[list[float]] = []
    for a, b in cuts:
        if merged and a <= merged[-1][1]:
            merged[-1][1] = max(merged[-1][1], b)
        else:
            merged.append([a, b])
    pieces, t = 0.0, 0.0
    pieces_list = []
    for a, b in merged + [[1.0, 1.0]]:
        if a > t:
            pieces_list.append((lerp(p, q, t), lerp(p, q, a)))
        t = max(t, b)
    return [(a, b) for a, b in pieces_list if dist(a, b) > 0.3]


def route(p: Pt, q: Pt, rects: list[tuple[float, float, float, float]]) -> list[Pt]:
    if not seg_blocked(p, q, rects):
        return [q]
    nodes = [p, q]
    for r in rects:
        x0, y0, x1, y1 = inflate(r, 0.5)
        for c in ((x0, y0), (x0, y1), (x1, y0), (x1, y1)):
            if not any(_seg_rect_interval(c, (c[0] + 1e-3, c[1]), rr) for rr in rects):
                nodes.append(c)
    best = {0: 0.0}
    prev: dict[int, int] = {}
    heap = [(0.0, 0)]
    while heap:
        d, i = heapq.heappop(heap)
        if i == 1:
            break
        if d > best.get(i, 1e18):
            continue
        for j in range(len(nodes)):
            if j == i or seg_blocked(nodes[i], nodes[j], rects):
                continue
            nd = d + dist(nodes[i], nodes[j])
            if nd < best.get(j, 1e18):
                best[j], prev[j] = nd, i
                heapq.heappush(heap, (nd, j))
    if 1 not in prev:
        return [q]
    path, k = [], 1
    while k != 0:
        path.append(nodes[k])
        k = prev[k]
    return path[::-1]


def get_primary_border_angle(polygon: list[Pt]) -> float:
    """Calculates the average orientation angle of the left and right outer boundaries."""
    n_pts = len(polygon)
    if n_pts < 3:
        return 0.0

    edges = []
    for i in range(n_pts):
        p1 = polygon[i]
        p2 = polygon[(i + 1) % n_pts]
        dx, dy = p2[0] - p1[0], p2[1] - p1[1]
        length = math.hypot(dx, dy)
        angle = math.atan2(dy, dx)
        edges.append((length, angle))

    # Get the two primary long edges forming the left and right boundaries
    edges.sort(key=lambda e: e[0], reverse=True)

    def norm_angle(a: float) -> float:
        while a > math.pi / 2:
            a -= math.pi
        while a < -math.pi / 2:
            a += math.pi
        return a

    a1 = norm_angle(edges[0][1])
    a2 = norm_angle(edges[1][1]) if len(edges) > 1 else a1

    diff = norm_angle(a2 - a1)
    return norm_angle(a1 + diff / 2.0)


def lanes_in_polygon(
    polygon: list[Pt],
    spacing: float,
    min_run: float = 15.0,
    margin_across: float = 0.0,
    margin_along: float = 0.0
) -> list[tuple[Pt, Pt]]:
    """Generates flight lanes aligned PARALLEL to the left/right field border bisector.

    1. Computes primary border orientation angle.
    2. Rotates polygon frame so lanes run parallel down the long axis of the field.
    3. Insets scan range with margin_across and margin_along.
    4. Converts generated flight tracks back to ENU coordinates.
    """
    n_pts = len(polygon)
    if n_pts < 3:
        return []

    angle = get_primary_border_angle(polygon)
    cos_a, sin_a = math.cos(-angle), math.sin(-angle)
    rot_poly = [(p[0] * cos_a - p[1] * sin_a, p[0] * sin_a + p[1] * cos_a) for p in polygon]

    ys = [p[1] for p in rot_poly]
    low = min(ys) + margin_across
    high = max(ys) - margin_across

    if high <= low:
        return []

    count = max(1, math.ceil((high - low) / spacing))
    step = (high - low) / count
    lanes: list[tuple[Pt, Pt]] = []

    cos_back, sin_back = math.cos(angle), math.sin(angle)

    def to_world(rx: float, ry: float) -> Pt:
        return (rx * cos_back - ry * sin_back, rx * sin_back + ry * cos_back)

    for i in range(count):
        y_scan = low + (i + 0.5) * step
        hits = []
        for j in range(n_pts):
            a_pt = rot_poly[j]
            b_pt = rot_poly[(j + 1) % n_pts]
            if (a_pt[1] <= y_scan < b_pt[1]) or (b_pt[1] <= y_scan < a_pt[1]):
                t = (y_scan - a_pt[1]) / (b_pt[1] - a_pt[1])
                x_int = a_pt[0] + t * (b_pt[0] - a_pt[0])
                hits.append(x_int)

        hits.sort()
        runs = []
        for k in range(0, len(hits) - 1, 2):
            x_start_in = hits[k] + margin_along
            x_end_in = hits[k + 1] - margin_along
            if x_end_in - x_start_in >= min_run:
                runs.append((x_start_in, x_end_in))

        # Alternate direction for continuous lawnmower scan pattern
        if i % 2 == 1:
            runs = [(b, a) for a, b in reversed(runs)]

        for x1, x2 in runs:
            lanes.append((to_world(x1, y_scan), to_world(x2, y_scan)))

    return lanes


def nearest_pad(p: Pt) -> tuple[str, Pt]:
    return min(PADS.items(), key=lambda kv: dist(p, kv[1]))


# ══ Flight State and Energy Budgets ══════════════════════════════════════════
def pos(ctx: Any) -> Pt:
    p = ctx.senses.pose.current_position
    return (p.y, p.x)                   # NED -> (east, north)


class Flight:
    def __init__(self, ctx: Any) -> None:
        self.t0 = ctx.world.now()
        self.t_air = self.t0
        self.charges = 0

    def elapsed(self, ctx: Any) -> float:
        return ctx.world.now() - self.t0

    def battery_left(self, ctx: Any) -> float:
        return BATTERY_S * BATTERY_USABLE - (ctx.world.now() - self.t_air)

    @staticmethod
    def back_s(p: Pt) -> float:
        return dist(p, nearest_pad(p)[1]) / TRANSIT_SPEED_M_S + LAND_S

    def fits_battery(self, ctx: Any, work_s: float, end: Pt) -> bool:
        return work_s + self.back_s(end) + RESERVE_S <= self.battery_left(ctx)

    def fits_clock(self, ctx: Any, work_s: float, end: Pt, charge: bool = False) -> bool:
        left = MISSION_S - END_RESERVE_S - self.elapsed(ctx)
        return work_s + self.back_s(end) + (CHARGE_EST_S if charge else 0.0) <= left


def travel_s(a: Pt, b: Pt) -> float:
    return dist(a, b) / TRANSIT_SPEED_M_S + 4.0


def hop(ctx: Any, target: Pt, alt: float, speed: float, rects: list[tuple[float, float, float, float]], name: str) -> Iterator[Any]:
    cur = pos(ctx)
    pts = route(cur, target, rects)
    if dist(cur, target) <= SHORT_HOP_M:
        seq = [(w, alt) for w in pts]
    else:
        seq = [(lerp(cur, pts[0], min(1.0, CLIMB_M / max(dist(cur, pts[0]), 1e-6))), alt)]
        seq += [(w, alt) for w in pts[:-1]]
        last_prev = pts[-2] if len(pts) > 1 else cur
        k = min(1.0, DESCEND_M / max(dist(last_prev, pts[-1]), 1e-6))
        seq.append((lerp(pts[-1], last_prev, k), alt))
        seq.append((pts[-1], alt))
    for i, (w, a) in enumerate(seq):
        yield fly_to(north=w[1], east=w[0], alt_m=a, target_speed=speed, name=f"{name}_{i}")


def mission_break_and_resume(ctx: Any) -> Iterator[Any]:
    yield from ()


def refuel(ctx: Any, st: Flight, alt: float, final: bool = False) -> Iterator[Any]:
    name, pad = nearest_pad(pos(ctx))
    pad_zones = [inflate(r, NO_FLY_MARGIN_M) for r in NO_FLY_RECTS]
    pad_zones = [z for z in pad_zones if not (z[0] <= pad[0] <= z[2] and z[1] <= pad[1] <= z[3])]
    yield from hop(ctx, pad, alt, TRANSIT_SPEED_M_S, pad_zones, f"to_{name}")
    yield fly_to(north=pad[1], east=pad[0], alt_m=6.0, target_speed=2.0, name=f"over_{name}")
    yield brake_and_settle(name=f"pre_land_{name}")
    yield land()
    ctx.world.log_info(f"{TAG} landed at {name} t={st.elapsed(ctx):.0f}s")
    if final:
        return
    yield from mission_break_and_resume(ctx)
    st.charges += 1
    yield takeoff(alt_m=alt)
    st.t_air = ctx.world.now()


# ══ Stress Mapping Engine ════════════════════════════════════════════════════
class StressMap:
    def __init__(self, polygons: list[list[Pt]]) -> None:
        margin = 50.0
        xs = [p[0] for poly in polygons for p in poly]
        ys = [p[1] for poly in polygons for p in poly]
        self.x0 = min(xs) - margin
        self.y0 = min(ys) - margin
        rows = int((max(ys) + margin - self.y0) / CELL_M) + 1
        cols = int((max(xs) + margin - self.x0) / CELL_M) + 1
        self.seen = np.zeros((rows, cols), dtype=np.int32)
        self.yellow = np.zeros((rows, cols), dtype=np.int32)
        self.aoi = np.zeros((rows, cols), dtype=np.uint8)
        for poly in polygons:
            cell = np.array([[(x - self.x0) / CELL_M, (y - self.y0) / CELL_M] for x, y in poly],
                            dtype=np.int32)
            cv2.fillPoly(self.aoi, [cell], 1)
        self.pictures = 0
        self._last_seq = -1

    def snap(self, ctx: Any) -> None:
        camera = ctx.senses.camera
        if not camera.has_frame or camera.seq == self._last_seq:
            return
        pose = ctx.senses.pose.current_position
        if pose is None:
            return
        path = ctx.services.snapshot.snap()
        picture = cv2.imread(path) if path else None
        if picture is None:
            return
        self.add_picture(cv2.cvtColor(picture, cv2.COLOR_BGR2RGB), pose)
        self._last_seq = camera.seq
        self.pictures += 1

    def add_picture(self, rgb: np.ndarray, pose: Any) -> None:
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        is_yellow = cv2.inRange(hsv, YELLOW_HSV_LOW, YELLOW_HSV_HIGH) > 0
        h, w = is_yellow.shape
        v, u = np.mgrid[0:h:2, 0:w:2]
        height = -pose.z + CAM_HEIGHT_AT_HOME_M
        right = (u - CAM_CX) / CAM_FX * height
        back = (v - CAM_CY) / CAM_FX * height
        cos_h, sin_h = math.cos(pose.heading), math.sin(pose.heading)
        cam_east = pose.y + CAM_AHEAD_M * sin_h
        cam_north = pose.x + CAM_AHEAD_M * cos_h
        east = cam_east + right * cos_h - back * sin_h
        north = cam_north - right * sin_h - back * cos_h
        row = ((north - self.y0) / CELL_M).astype(int)
        col = ((east - self.x0) / CELL_M).astype(int)
        ok = (row >= 0) & (row < self.seen.shape[0]) & (col >= 0) & (col < self.seen.shape[1])
        row, col = row[ok], col[ok]
        np.add.at(self.seen, (row, col), 1)
        np.add.at(self.yellow, (row, col), is_yellow[v, u][ok].astype(np.int32))

    def stress_areas(self) -> list[dict[str, Any]]:
        frac = self.yellow / np.maximum(self.seen, 1)
        mask = ((frac >= YELLOW_FRACTION) & (self.seen >= MIN_SAMPLES_PER_CELL)
                & (self.aoi > 0)).astype(np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        areas = []
        for contour in contours:
            if len(contour) < 3 or cv2.contourArea(contour) * CELL_M ** 2 < MIN_AREA_M2:
                continue
            contour = cv2.approxPolyDP(contour, 1.0, True)
            if len(contour) < 3:
                continue
            polygon = [[float(self.x0 + (c + 0.5) * CELL_M), float(self.y0 + (r + 0.5) * CELL_M)]
                       for c, r in contour[:, 0]]
            areas.append({"class": "stressed", "polygon": polygon})
        return areas


def save_stress_areas(areas: list[dict[str, Any]]) -> None:
    STRESS_AREA_PATH.parent.mkdir(parents=True, exist_ok=True)
    tmp = STRESS_AREA_PATH.with_name(".stress_area.json.tmp")
    tmp.write_text(json.dumps({"stress_area": areas}, separators=(",", ":")))
    tmp.replace(STRESS_AREA_PATH)


# ══ Spray Valve Controller ═══════════════════════════════════════════════════
class ValveGate:
    def __init__(self, sprayer: Any, a: Pt, b: Pt) -> None:
        self.sprayer, self.a = sprayer, a
        self.length = dist(a, b)
        self.ux, self.uy = (b[0] - a[0]) / self.length, (b[1] - a[1]) / self.length
        self.is_open = False
        self._last: tuple[float, Pt] | None = None
        self.speed = 0.0

    def tick(self, ctx: Any) -> None:
        p, t = pos(ctx), ctx.world.now()
        if self._last is not None and t > self._last[0]:
            self.speed = 0.5 * self.speed + 0.5 * dist(p, self._last[1]) / (t - self._last[0])
        self._last = (t, p)
        dx, dy = p[0] - self.a[0], p[1] - self.a[1]
        along = dx * self.ux + dy * self.uy
        off = abs(-dx * self.uy + dy * self.ux)
        want = (0.0 <= along <= self.length and off <= GATE_MAX_OFFSET_M
                and self.speed >= GATE_MIN_SPEED_FRAC * SPRAY_SPEED_M_S)
        if want and not self.is_open:
            self.sprayer.on()
        elif not want and self.is_open:
            self.sprayer.off()
        self.is_open = want


def safe_extend(p: Pt, u: Pt, length: float, rects: list[tuple[float, float, float, float]]) -> Pt:
    for k in (1.0, 0.5, 0.25, 0.0):
        q = (p[0] + u[0] * length * k, p[1] + u[1] * length * k)
        if not seg_blocked(p, q, rects):
            return q
    return p


# ══ Mission Phases ═══════════════════════════════════════════════════════════
def scan_phase(ctx: Any, st: Flight, smap: StressMap) -> Iterator[Any]:
    zones = [inflate(r, NO_FLY_MARGIN_M) for r in NO_FLY_RECTS]
    todo: list[tuple[Pt, Pt]] = []

    for poly in AOI_POLYGONS:
        for a, b in lanes_in_polygon(poly, SCAN_SPACING_M, min_run=15.0,
                                     margin_across=SCAN_MARGIN_ACROSS_M,
                                     margin_along=SCAN_MARGIN_ALONG_M):
            todo.extend(clip_outside(a, b, zones))

    ctx.world.log_info(f"{TAG} scan: {len(todo)} parallel lane piece(s), spacing {SCAN_SPACING_M:.1f} m (40% overlap)")

    n = 0
    while todo:
        cur = pos(ctx)
        a, b = todo[0]
        if dist(cur, b) < dist(cur, a):
            a, b = b, a
        cost = travel_s(cur, a) + dist(a, b) / SCAN_SPEED_M_S + 8.0
        if not st.fits_battery(ctx, cost, b):
            if not st.fits_clock(ctx, cost, b, charge=True):
                break
            save_stress_areas(smap.stress_areas())
            yield from refuel(ctx, st, alt=SCAN_ALT_M)
        elif not st.fits_clock(ctx, cost, b):
            break
        todo.pop(0)
        n += 1
        yield from hop(ctx, a, SCAN_ALT_M, TRANSIT_SPEED_M_S, zones, f"scan_{n:03d}_to")
        mapping = ctx.scheduler.schedule(lambda: smap.snap(ctx), hz=MAP_HZ,
                                         group=ScheduleGroup.MEDIA, name="mapper",
                                         now=ctx.world.now())
        try:
            yield fly_to(north=b[1], east=b[0], alt_m=SCAN_ALT_M, target_speed=SCAN_SPEED_M_S,
                         mode="coverage", name=f"scan_{n:03d}")
        finally:
            ctx.scheduler.unschedule(mapping)
        save_stress_areas(smap.stress_areas())
    ctx.world.log_info(f"{TAG} scan done: {smap.pictures} pictures, {len(todo)} piece(s) skipped")


def spray_phase(ctx: Any, st: Flight, areas: list[dict[str, Any]]) -> Iterator[Any]:
    fly_zones = [inflate(r, NO_FLY_MARGIN_M) for r in NO_FLY_RECTS]
    spray_zones = [inflate(r, NO_FLY_MARGIN_M + SPRAY_EXTRA_MARGIN_M) for r in NO_FLY_RECTS]
    sprayer = ctx.services.sprayer

    segs: list[tuple[Pt, Pt]] = []
    for area in areas:
        polygon = [(p[0], p[1]) for p in area["polygon"]]
        for a, b in lanes_in_polygon(polygon, SPRAY_LANE_SPACING_M, min_run=MIN_SPRAY_RUN_M):
            segs.extend(clip_outside(a, b, spray_zones))
    segs = [s for s in segs if dist(*s) >= MIN_SPRAY_RUN_M]
    ctx.world.log_info(f"{TAG} spray: {len(segs)} lane(s), {SPRAY_ALT_M} m @ "
                       f"{SPRAY_SPEED_M_S:.2f} m/s (25% overlap), predicted mean dose {PREDICTED_DOSE:.2f} ml/m2")

    n = 0
    while segs:
        cur = pos(ctx)
        i = min(range(len(segs)), key=lambda k: min(dist(cur, segs[k][0]), dist(cur, segs[k][1])))
        a, b = segs[i]
        if dist(cur, b) < dist(cur, a):
            a, b = b, a
        u = ((b[0] - a[0]) / dist(a, b), (b[1] - a[1]) / dist(a, b))
        run_in = safe_extend(a, (-u[0], -u[1]), RUN_IN_M, spray_zones)
        run_out = safe_extend(b, u, RUN_IN_M, spray_zones)
        cost = travel_s(cur, run_in) + dist(run_in, run_out) / SPRAY_SPEED_M_S + 6.0
        if not st.fits_battery(ctx, cost, run_out):
            if not st.fits_clock(ctx, cost, run_out, charge=True):
                break
            yield from refuel(ctx, st, alt=SPRAY_ALT_M)
            continue
        if not st.fits_clock(ctx, cost, run_out):
            break
        segs.pop(i)
        n += 1

        yield from hop(ctx, run_in, SPRAY_ALT_M, TRANSIT_SPEED_M_S, fly_zones, f"spray_{n:03d}_in")
        gate = ValveGate(sprayer, a, b)
        gate_task = ctx.scheduler.schedule(lambda: gate.tick(ctx), hz=GATE_HZ,
                                           group=ScheduleGroup.MEDIA, name="valve_gate",
                                           now=ctx.world.now())
        try:
            yield fly_to(north=run_out[1], east=run_out[0], alt_m=SPRAY_ALT_M,
                         target_speed=SPRAY_SPEED_M_S, mode="coverage", name=f"spray_{n:03d}")
        finally:
            ctx.scheduler.unschedule(gate_task)
            sprayer.off()
    ctx.world.log_info(f"{TAG} spray done: {n} lane(s) flown, {len(segs)} left, "
                       f"t={st.elapsed(ctx):.0f}s, charges={st.charges}")


def autonomous_scan_mission(ctx: Any) -> Iterator[Any]:
    log = ctx.world.log_info
    st = Flight(ctx)
    smap = StressMap(AOI_POLYGONS)
    assert SCAN_ALT_M <= 38.0, "scan altitude too close to the 40 m AGL limit"

    yield takeoff(alt_m=SCAN_ALT_M)
    st.t_air = ctx.world.now()

    yield from scan_phase(ctx, st, smap)
    areas = smap.stress_areas()
    save_stress_areas(areas)
    log(f"{TAG} {len(areas)} stress area(s) saved to {STRESS_AREA_PATH}")

    yield from spray_phase(ctx, st, areas)
    yield from refuel(ctx, st, alt=SPRAY_ALT_M, final=True)


autonomous_scan_mission.requires_senses = ["pose", "obstacle", "status", "camera"]


def main() -> None:
    with boot_drone() as drone:
        drone.add_service(Sprayer())
        drone.fly(autonomous_scan_mission)
        drone.run()


if __name__ == "__main__":
    main()