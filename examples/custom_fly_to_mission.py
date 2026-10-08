"""Custom fly_to skill — write your own skill that flies straight to a point.

Level 5 · Build your own

Minimal example of the Skill protocol (see ``docs/tutorial-add-skill.md``
in skytrack-autonomy). A skill only needs:

* ``name``                    — label used in logs
* ``start(ctx, params=None)`` — snapshot state, register callbacks
* ``cancel(ctx, reason)``     — remove callbacks, idempotent
* ``is_done`` (property)      — when True, the mission runs the next step

``SimpleFlyToSkill`` flies a straight segment from the current position to
``(north, east, alt_m)`` with a trapezoidal speed profile (accelerate →
cruise → decelerate), publishes setpoints at 10 Hz, and holds the heading
from the start. NO planner, NO obstacle avoidance — only use it when the
path is clear. For obstacle avoidance use the built-in ``fly_to``.

Run::

    python -m local_planner.examples.custom_fly_to_mission
"""
from __future__ import annotations

import math
from typing import Any, Iterator, Optional

from local_planner import boot_drone, brake, land, takeoff
from skytrack_autonomy.core.lib.scheduling import ScheduleGroup, ScheduleHandle

ALT_M = 3.0
TAG = "[SIMPLE_FLY]"


# ── Skill ──────────────────────────────────────────────────────────

class SimpleFlyToSkill:
    """Fly straight to ``(north, east, alt_m)``, hold heading, no planner."""

    name = "simple_fly_to"
    PUBLISH_HZ = 10.0            # PX4 needs >= 2 Hz to stay in OFFBOARD; 10 Hz by convention

    def __init__(
        self,
        *,
        north: float,
        east: float,
        alt_m: float,
        speed_m_s: float = 1.5,
        accel_m_s2: float = 1.0,
        tolerance_m: float = 0.3,
        settle_speed_m_s: float = 0.2,
        timeout_s: float = 60.0,
    ) -> None:
        # Mission-NED: negative z points up.
        self._target = (float(north), float(east), -float(alt_m))
        self._v = float(speed_m_s)
        self._a = float(accel_m_s2)
        self._tol = float(tolerance_m)
        self._settle_v = float(settle_speed_m_s)
        self._timeout_s = float(timeout_s)
        # Per-run state — assigned in ``start``.
        self._ctx: Any = None
        self._handle: Optional[ScheduleHandle] = None
        self._t0: Optional[float] = None
        self._start = None           # (x, y, z) at start
        self._dir = (0.0, 0.0, 0.0)  # unit vector start → target
        self._length = 0.0
        self._yaw = math.nan
        self._result: Optional[str] = None   # "reached" | "timeout"

    # ── Lifecycle ────────────────────────────────────────────────────

    def start(self, ctx: Any, params: Any = None) -> None:
        self._ctx = ctx
        self._t0 = ctx.world.now()
        self._result = None
        pose = ctx.senses.pose.current_position
        if pose is not None:
            self._plan_from(pose)
        # Hand control over to our own setpoints (PX4 OFFBOARD).
        ctx.world.publish_enable_to_fly(True)
        ctx.world.engage_external_control()
        # CONTROL group is for high-rate callbacks (setpoints);
        # DECISION group is for 5 Hz planning / monitoring.
        self._handle = ctx.scheduler.schedule(
            self._tick, hz=self.PUBLISH_HZ, group=ScheduleGroup.CONTROL,
            name=self.name, now=ctx.world.now())
        ctx.world.log_info(
            f"{TAG} start → N{self._target[0]:.1f} E{self._target[1]:.1f} "
            f"alt={-self._target[2]:.1f}m ({self._length:.1f} m, "
            f"v={self._v:.1f} m/s)")

    def cancel(self, ctx: Any, reason: str) -> None:
        # Must be idempotent and must not crash if start never ran. The runtime
        # calls cancel even when the skill finished normally (next step takes over).
        if self._handle is not None:
            ctx.scheduler.unschedule(self._handle)
            self._handle = None
        if self._result is None:
            ctx.world.log_info(f"{TAG} cancelled (reason={reason})")

    @property
    def is_done(self) -> bool:
        return self._result is not None

    @property
    def result(self) -> Optional[str]:
        """``"reached"`` / ``"timeout"`` once finished, ``None`` while flying."""
        return self._result

    # ── Path ─────────────────────────────────────────────────────────

    def _plan_from(self, pose: Any) -> None:
        self._start = (float(pose.x), float(pose.y), float(pose.z))
        d = [t - s for t, s in zip(self._target, self._start)]
        self._length = math.sqrt(sum(c * c for c in d))
        self._dir = (tuple(c / self._length for c in d)
                     if self._length > 1e-6 else (0.0, 0.0, 0.0))
        self._yaw = float(pose.heading)    # hold heading from start

    def _profile(self, t: float):
        """Distance ``s`` and speed ``v`` along the segment at time ``t`` —
        trapezoidal (or triangular if the segment is too short)."""
        a, L = self._a, self._length
        v_peak = min(self._v, math.sqrt(a * L))      # triangular if L is short
        t_acc = v_peak / a
        d_acc = 0.5 * a * t_acc * t_acc
        t_cruise = max(0.0, (L - 2.0 * d_acc) / v_peak) if v_peak > 0 else 0.0
        if t < t_acc:                                  # accelerate
            return 0.5 * a * t * t, a * t
        if t < t_acc + t_cruise:                       # cruise
            return d_acc + v_peak * (t - t_acc), v_peak
        td = t - t_acc - t_cruise                      # decelerate
        if td < t_acc:
            return L - 0.5 * a * (t_acc - td) ** 2, v_peak - a * td
        return L, 0.0                                  # reached target

    # ── Callback 10 Hz (CONTROL group) ───────────────────────────────

    def _tick(self) -> None:
        ctx = self._ctx
        pose = ctx.senses.pose.current_position
        if pose is None:
            return                                     # no pose yet
        if self._start is None:
            self._plan_from(pose)                      # pose arrived late

        elapsed = ctx.world.now() - self._t0
        s, v = self._profile(elapsed)
        sp = [p + s * u for p, u in zip(self._start, self._dir)]
        vel = [v * u for u in self._dir]
        ctx.world.publish_trajectory_setpoint(
            position=sp, velocity=vel, acceleration=[0.0, 0.0, 0.0],
            yaw=self._yaw, yaw_rate=0.0)

        if self._result is not None:
            return       # done: keep holding the target setpoint until replaced
        dist = math.dist((pose.x, pose.y, pose.z), self._target)
        speed = math.sqrt(pose.vx ** 2 + pose.vy ** 2 + pose.vz ** 2)
        if s >= self._length and dist < self._tol and speed < self._settle_v:
            self._finish("reached", dist, elapsed)
        elif elapsed > self._timeout_s:
            self._finish("timeout", dist, elapsed)

    def _finish(self, result: str, dist: float, elapsed: float) -> None:
        self._result = result
        log = (self._ctx.world.log_info if result == "reached"
               else self._ctx.world.log_warn)
        log(f"{TAG} {result}: {dist:.2f} m from target after {elapsed:.1f}s")
        # Wake the arbiter so the mission advances to the next step this tick.
        self._ctx.notify_state_change()


# ── Mission ────────────────────────────────────────────────────────

def custom_fly_to_mission(ctx: Any) -> Iterator[Any]:
    """Take off to 3 m, fly 4 m straight north with the custom skill, fly
    back, land."""
    yield takeoff(alt_m=ALT_M)
    yield brake(name="settle_after_takeoff")

    out = SimpleFlyToSkill(north=4.0, east=0.0, alt_m=ALT_M)
    yield out                          # bare skill — runtime wraps it in a SkillStep
    if out.result != "reached":
        ctx.world.log_warn(f"{TAG} outbound leg {out.result} — landing here")
        yield brake(name="pre_land")
        yield land()
        return

    yield SimpleFlyToSkill(north=0.0, east=0.0, alt_m=ALT_M)
    yield brake(name="pre_land")
    yield land()


# Framework senses that must be available before the mission runs.
custom_fly_to_mission.requires_senses = ["pose", "obstacle", "status"]


def main() -> None:
    with boot_drone() as drone:
        drone.fly(custom_fly_to_mission)
        print(f"[custom_fly_to_mission] modes:  {drone.list_modes()}")
        print(f"[custom_fly_to_mission] senses: {drone.list_senses()}")
        drone.run()


if __name__ == "__main__":
    main()
