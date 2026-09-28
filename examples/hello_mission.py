"""Hello mission — take off, hover for 5 seconds, log a message, and land.

Level 1 · Basics

Run::

    python -m local_planner.examples.hello_mission
"""
from __future__ import annotations

from typing import Any, Iterator

from skytrack_autonomy import SkillStep
from local_planner import boot_drone, brake, land, takeoff

ALT_M = 4.0          # Takeoff altitude between 3 and 5 meters


def hover_until_timeout(timeout_s: float, ctx: Any, name: str) -> SkillStep:
    """Hover in place until the specific timeout has passed."""
    deadline = ctx.world.now() + timeout_s
    return SkillStep(
        skill=brake(name=name).skill,
        is_done=lambda c: c.world.now() >= deadline,
        name=name,
    )


# ── Mission — a generator function ─────────────────────────────────

def hello_mission(ctx: Any) -> Iterator[Any]:
    """Climb to safe altitude, hover for 5 seconds, land."""
    # 3. Arm the vehicle and execute a vertical takeoff to a low, safe altitude
    yield takeoff(alt_m=ALT_M)

    # 4. Hover for 5 seconds while logging message
    ctx.world.log_info("Hello SkyTrack Autonomy: vehicle stable in loiter/hover")
    yield hover_until_timeout(5.0, ctx, name="hover_5s")

    # 5. Command a controlled Land or Return to Launch (RTL) sequence
    # 6. Disarm upon touchdown and cleanly terminate the mission loop
    # land() implicitly waits for touchdown and disarms in SkyTrack API
    yield land()


# Senses this mission's steps need. Checked before the mission starts.
hello_mission.requires_senses = ["pose", "status"]


# ── Wire + run ─────────────────────────────────────────────────────

def main() -> None:
    # 2. Connect to the vehicle/simulated drone and verify connection state
    with boot_drone() as drone:
        # 7. Register hello_mission so it can be executed
        drone.fly(hello_mission)
        print(f"[hello_mission] modes:  {drone.list_modes()}")
        print(f"[hello_mission] senses: {drone.list_senses()}")
        drone.run()


if __name__ == "__main__":
    main()
