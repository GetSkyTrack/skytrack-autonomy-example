"""Survey mission with continuous battery monitoring.

Level 3 · Mission logic

Run::

    python -m local_planner.examples.survey_mission
"""
from __future__ import annotations

from typing import Any, Iterator

from skytrack_autonomy import SkillStep
from local_planner import boot_drone, brake, fly_to, land, orbit, takeoff

ALT_M = 40.0
RADIUS_M = 50.0
MIN_BATTERY_PERCENT = 50.0
SURVEY_DURATION_S = 60.0
TAG = "[SURVEY]"


def survey_with_battery_check(ctx: Any, duration_s: float, name: str) -> SkillStep:
    """Orbit the area but abort early if battery drops below MIN_BATTERY_PERCENT."""
    deadline = ctx.world.now() + duration_s
    orbit_skill = orbit(
        center_north=0.0,
        center_east=0.0,
        alt_m=ALT_M,
        radius_m=RADIUS_M,
        period_s=30.0,
        duration_s=duration_s,
        name=name,
    ).skill

    def is_done(c: Any) -> bool:
        if c.world.now() >= deadline:
            return True
        percent = c.senses.battery.percent
        if percent is not None and percent < MIN_BATTERY_PERCENT:
            c.world.log_warn(f"{TAG} Battery dropped to {percent}%, aborting survey.")
            return True
        return False

    return SkillStep(skill=orbit_skill, is_done=is_done, name=name)


def survey_mission(ctx: Any) -> Iterator[Any]:
    """Take off to 40m, survey 50m area, monitor battery, return and land."""
    ctx.world.log_info(f"{TAG} Taking off to {ALT_M}m")
    yield takeoff(alt_m=ALT_M)

    ctx.world.log_info(f"{TAG} Starting survey (radius {RADIUS_M}m) around spawn")
    yield survey_with_battery_check(ctx, duration_s=SURVEY_DURATION_S, name="survey_orbit")

    percent = ctx.senses.battery.percent
    if percent is not None and percent < MIN_BATTERY_PERCENT:
        ctx.world.log_warn(f"{TAG} Returning to spawn due to low battery")
    else:
        ctx.world.log_info(f"{TAG} Survey complete, returning to spawn")

    yield fly_to(north=0.0, east=0.0, alt_m=ALT_M, name="return_home")
    yield brake(name="pre_land")
    yield land()


survey_mission.requires_senses = ["pose", "obstacle", "status", "battery"]


def main() -> None:
    with boot_drone() as drone:
        drone.fly(survey_mission)
        print(f"[survey_mission] modes:  {drone.list_modes()}")
        print(f"[survey_mission] senses: {drone.list_senses()}")
        drone.run()


if __name__ == "__main__":
    main()
