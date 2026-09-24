"""Custom skill mission — write your own motion primitive.

Level 5 · Build your own

A **skill** is what actually moves the drone. It sends setpoints while
it runs and says when it has finished. To write one, give it:

* ``name``                — label for logs and reports
* ``start(ctx, params)``  — begin: schedule a setpoint publisher
* ``cancel(ctx, reason)`` — stop: unschedule it (safe to call twice)
* ``is_done``             — property, polled every tick

Then ``yield`` an instance from a mission, like any built-in step.

This file defines ``HoverForSeconds`` and ``ClimbBy`` and uses both.

Run::

    python -m local_planner.examples.custom_skill_mission
"""
from __future__ import annotations

from typing import Any, Iterator, Optional, Tuple

from local_planner import boot_drone, brake, fly_to, land, takeoff
from skytrack_autonomy.core.lib.scheduling import ScheduleGroup, ScheduleHandle

ALT_M = 3.0


# ── 1. Custom skills ───────────────────────────────────────────────

class HoverForSeconds:
    """Hold the current position for ``duration_s`` seconds."""

    name = "hover_for_seconds"
    PUBLISH_HZ = 10.0               # setpoints must keep flowing

    def __init__(self, duration_s: float) -> None:
        self._duration_s = float(duration_s)
        self._ctx: Any = None
        self._handle: Optional[ScheduleHandle] = None
        self._t_start: Optional[float] = None
        self._hold: Optional[Tuple[float, float, float, float]] = None

    def start(self, ctx: Any, params: Any = None) -> None:
        self._ctx = ctx
        self._t_start = ctx.world.now()          # never time.time()
        p = ctx.senses.pose.current_position
        self._hold = (p.x, p.y, p.z, getattr(p, "heading", 0.0))
        ctx.world.publish_enable_to_fly(True)
        self._handle = ctx.scheduler.schedule(
            self._publish, hz=self.PUBLISH_HZ, group=ScheduleGroup.CONTROL,
            name=self.name, now=ctx.world.now())
        ctx.world.log_info(f"[HOVER] holding for {self._duration_s:.0f}s")

    def cancel(self, ctx: Any, reason: str) -> None:
        if self._handle is not None:
            ctx.scheduler.unschedule(self._handle)
            self._handle = None

    @property
    def is_done(self) -> bool:
        if self._t_start is None:
            return False
        return self._ctx.world.now() - self._t_start >= self._duration_s

    def _publish(self) -> None:
        x, y, z, yaw = self._hold
        self._ctx.world.publish_trajectory_setpoint(
            position=[x, y, z], velocity=[0.0, 0.0, 0.0],
            acceleration=[0.0, 0.0, 0.0], yaw=yaw, yaw_rate=0.0)


class ClimbBy:
    """Climb (or descend, if negative) ``delta_m`` straight up in place.

    Finishes when within ``tolerance_m`` of the target altitude.
    """

    name = "climb_by"
    PUBLISH_HZ = 10.0

    def __init__(self, delta_m: float, *, tolerance_m: float = 0.2) -> None:
        self._delta_m = float(delta_m)
        self._tolerance_m = float(tolerance_m)
        self._ctx: Any = None
        self._handle: Optional[ScheduleHandle] = None
        self._target: Optional[Tuple[float, float, float, float]] = None

    def start(self, ctx: Any, params: Any = None) -> None:
        self._ctx = ctx
        p = ctx.senses.pose.current_position
        # NED: up is negative z, so climbing subtracts.
        self._target = (p.x, p.y, p.z - self._delta_m, getattr(p, "heading", 0.0))
        ctx.world.publish_enable_to_fly(True)
        self._handle = ctx.scheduler.schedule(
            self._publish, hz=self.PUBLISH_HZ, group=ScheduleGroup.CONTROL,
            name=self.name, now=ctx.world.now())
        ctx.world.log_info(f"[CLIMB] to {-self._target[2]:.1f} m")

    def cancel(self, ctx: Any, reason: str) -> None:
        if self._handle is not None:
            ctx.scheduler.unschedule(self._handle)
            self._handle = None

    @property
    def is_done(self) -> bool:
        if self._target is None:
            return False
        p = self._ctx.senses.pose.current_position
        return p is not None and abs(p.z - self._target[2]) <= self._tolerance_m

    def _publish(self) -> None:
        x, y, z, yaw = self._target
        self._ctx.world.publish_trajectory_setpoint(
            position=[x, y, z], velocity=[0.0, 0.0, 0.0],
            acceleration=[0.0, 0.0, 0.0], yaw=yaw, yaw_rate=0.0)


# ── 2. A mission that uses them ────────────────────────────────────

def custom_skill_mission(ctx: Any) -> Iterator[Any]:
    """Fly out, pause, pop up 3 m for a look, pause, come back, land."""
    yield takeoff(alt_m=ALT_M)
    yield fly_to(north=5, east=0, alt_m=ALT_M, name="to_lookout")

    yield HoverForSeconds(3)                  # ← custom skill
    yield ClimbBy(3.0)                        # ← custom skill
    yield HoverForSeconds(5)
    yield ClimbBy(-3.0)

    yield fly_to(north=0, east=0, alt_m=ALT_M, name="return_home")
    yield brake(name="pre_land")
    yield land()


custom_skill_mission.requires_senses = ["pose", "obstacle", "status"]


# ── 3. Wire + run ──────────────────────────────────────────────────

def main() -> None:
    with boot_drone() as drone:
        drone.fly(custom_skill_mission)
        print(f"[custom_skill_mission] modes:  {drone.list_modes()}")
        print(f"[custom_skill_mission] senses: {drone.list_senses()}")
        drone.run()


if __name__ == "__main__":
    main()
