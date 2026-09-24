"""Battery check mission — read senses and decide what to do.

Level 3 · Mission logic

What you learn
==============
* Reading built-in senses between steps:
  ``ctx.senses.pose``, ``ctx.senses.status``, ``ctx.senses.battery``.
* Branching with ``if`` and stopping early with ``break``.
* Handling "unknown" readings (``None``) safely.
* Returning home to where the drone actually took off.

Run::

    python -m local_planner.examples.battery_check_mission
"""
from __future__ import annotations

from typing import Any, Iterator

from local_planner import boot_drone, brake, fly_to, land, takeoff

ALT_M = 3.0
MIN_BATTERY_PERCENT = 40.0   # below this, abandon the patrol
TAG = "[BATTERY]"

PATROL = [(8.0, 0.0), (8.0, 8.0), (0.0, 8.0), (-6.0, 4.0)]


def battery_ok(ctx: Any) -> bool:
    """True unless the battery is known to be low."""
    percent = ctx.senses.battery.percent       # 0..100, or None if unknown
    if percent is None:
        ctx.world.log_warn(f"{TAG} battery level unknown, continuing")
        return True
    return percent >= MIN_BATTERY_PERCENT


def log_state(ctx: Any, label: str) -> None:
    pose = ctx.senses.pose.current_position    # NED metres, None until a fix
    status = ctx.senses.status
    if pose is None:
        ctx.world.log_info(f"{TAG} {label}: no position yet")
        return
    ctx.world.log_info(
        f"{TAG} {label}: N={pose.x:.1f} E={pose.y:.1f} alt={-pose.z:.1f} m, "
        f"armed={status.is_armed}, mode={status.mode_name}, "
        f"battery={ctx.senses.battery.percent}")


def battery_check_mission(ctx: Any) -> Iterator[Any]:
    """Patrol while the battery allows; otherwise go home early."""
    yield takeoff(alt_m=ALT_M)
    yield brake(name="settle_after_takeoff")
    home = ctx.senses.pose.current_position
    log_state(ctx, "after takeoff")

    for i, (north, east) in enumerate(PATROL, start=1):
        if not battery_ok(ctx):
            ctx.world.log_warn(f"{TAG} battery low before wp {i}, returning home")
            break
        yield fly_to(north=north, east=east, alt_m=ALT_M, name=f"patrol_{i}")
        log_state(ctx, f"at wp {i}")

    if home is not None:
        yield fly_to(north=float(home.x), east=float(home.y), alt_m=ALT_M,
                     name="return_home")
    yield brake(name="pre_land")
    yield land()


battery_check_mission.requires_senses = ["pose", "obstacle", "status", "battery"]


def main() -> None:
    with boot_drone() as drone:
        drone.fly(battery_check_mission)
        print(f"[battery_check_mission] modes:  {drone.list_modes()}")
        print(f"[battery_check_mission] senses: {drone.list_senses()}")
        drone.run()


if __name__ == "__main__":
    main()
