"""Custom sense mission — write your own sense: a geofence.

Level 5 · Build your own

A **sense** watches data from the drone and exposes read-only answers.
It never moves the drone. To write one, give it:

* ``name``           — the key under ``ctx.senses`` (``ctx.senses.geofence``)
* ``attach(world)``  — subscribe to data; called by ``drone.add_sense``
* properties         — what missions read

Subscriptions available on ``world``: ``on_local_position``,
``on_vehicle_status``, ``on_battery_status``, ``on_global_position``,
``on_attitude``.

This mission only flies to waypoints the geofence allows.

Run::

    python -m local_planner.examples.custom_sense_mission
"""
from __future__ import annotations

import math
from typing import Any, Iterator, Optional

from local_planner import boot_drone, brake, fly_to, land, takeoff

ALT_M = 3.0
FENCE_RADIUS_M = 10.0
FENCE_MAX_ALT_M = 8.0
TAG = "[FENCE]"

WAYPOINTS = [(6.0, 0.0), (6.0, 12.0), (0.0, 7.0), (-15.0, 0.0)]


# ── 1. The custom sense ────────────────────────────────────────────

class GeofenceSense:
    """A cylinder around home: radius ``radius_m``, height ``max_alt_m``."""

    name = "geofence"

    def __init__(self, *, radius_m: float, max_alt_m: float) -> None:
        self.radius_m = float(radius_m)
        self.max_alt_m = float(max_alt_m)
        self._pose: Optional[Any] = None
        self.breaches = 0

    def attach(self, world: Any) -> None:
        # Pose arrives in NED metres: x north, y east, z down.
        world.on_local_position(self._on_pose)

    def _on_pose(self, msg: Any) -> None:
        # Keep callbacks cheap: store the message, count breaches.
        self._pose = msg
        if not self.is_inside:
            self.breaches += 1

    # Read-only answers for missions.

    def allows(self, north: float, east: float, alt_m: float) -> bool:
        """Would a point be inside the fence?"""
        return math.hypot(north, east) <= self.radius_m and alt_m <= self.max_alt_m

    @property
    def distance_from_home_m(self) -> Optional[float]:
        if self._pose is None:
            return None
        return math.hypot(self._pose.x, self._pose.y)

    @property
    def is_inside(self) -> bool:
        if self._pose is None:
            return True
        return self.allows(self._pose.x, self._pose.y, -self._pose.z)


# ── 2. A mission that uses it ──────────────────────────────────────

def custom_sense_mission(ctx: Any) -> Iterator[Any]:
    """Fly only to waypoints inside the fence."""
    fence = ctx.senses.geofence
    yield takeoff(alt_m=ALT_M)

    for i, (north, east) in enumerate(WAYPOINTS, start=1):
        if not fence.allows(north, east, ALT_M):
            ctx.world.log_warn(
                f"{TAG} wp {i} ({north}, {east}) is outside the "
                f"{fence.radius_m:.0f} m fence, skipping")
            continue
        yield fly_to(north=north, east=east, alt_m=ALT_M, name=f"wp_{i}")
        distance = fence.distance_from_home_m      # None until the first pose
        if distance is not None:
            ctx.world.log_info(
                f"{TAG} wp {i}: {distance:.1f} m from home, "
                f"inside={fence.is_inside}")

    yield fly_to(north=0, east=0, alt_m=ALT_M, name="return_home")
    yield brake(name="pre_land")
    yield land()
    ctx.world.log_info(f"{TAG} breaches during flight: {fence.breaches}")


custom_sense_mission.requires_senses = ["pose", "obstacle", "status", "geofence"]


# ── 3. Wire + run ──────────────────────────────────────────────────

def main() -> None:
    with boot_drone() as drone:
        drone.add_sense(GeofenceSense(radius_m=FENCE_RADIUS_M,
                                      max_alt_m=FENCE_MAX_ALT_M))
        drone.fly(custom_sense_mission)
        print(f"[custom_sense_mission] modes:  {drone.list_modes()}")
        print(f"[custom_sense_mission] senses: {drone.list_senses()}")
        drone.run()


if __name__ == "__main__":
    main()
