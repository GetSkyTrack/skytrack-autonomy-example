"""Spray mission — spray along a line, with the valve confirmed open.

Level 4 · Camera & services

What you learn
==============
* The ``Sprayer`` service: ``on()`` / ``off()``, ``state``.
* Waiting for something outside the flight to happen: wrap ``brake``
  in a ``SkillStep`` with your own ``is_done`` so the drone hovers
  until the valve reports open (or a timeout passes).
* ``mode="coverage"`` so the spray line stays straight.

Watch the valve with ``ros2 topic echo /spray``.

Requires
========
* A drone with a spray valve (``set_spray`` on the world adapter).
* Not yet tested in simulation.

Run::

    python -m local_planner.examples.spray_mission
"""
from __future__ import annotations

from typing import Any, Iterator

from local_planner import SkillStep, boot_drone, brake, fly_to, land, takeoff
from skytrack_autonomy import Sprayer

ALT_M = 3.0
LINE_START = (4.0, 0.0)      # (north, east)
LINE_END = (4.0, 12.0)
SPRAY_SPEED_M_S = 1.0
VALVE_WAIT_S = 3.0
TAG = "[SPRAY]"


def hover_until(condition, *, timeout_s: float, ctx: Any, name: str) -> SkillStep:
    """Hover in place until ``condition(ctx)`` is true or time runs out."""
    deadline = ctx.world.now() + timeout_s
    return SkillStep(
        skill=brake(name=name).skill,
        is_done=lambda c: condition(c) or c.world.now() >= deadline,
        name=name,
    )


def spray_mission(ctx: Any) -> Iterator[Any]:
    """Fly to the line start, open the valve, spray to the end, close."""
    spray = ctx.services.sprayer

    yield takeoff(alt_m=ALT_M)
    yield fly_to(north=LINE_START[0], east=LINE_START[1], alt_m=ALT_M,
                 name="to_line_start")

    spray.on()
    yield hover_until(lambda c: spray.state, timeout_s=VALVE_WAIT_S,
                      ctx=ctx, name="wait_valve_open")
    ctx.world.log_info(f"{TAG} spraying, valve={spray.state}")

    yield fly_to(north=LINE_END[0], east=LINE_END[1], alt_m=ALT_M,
                 mode="coverage", target_speed=SPRAY_SPEED_M_S, name="spray_line")

    spray.off()
    ctx.world.log_info(f"{TAG} line done, valve off")

    yield brake(name="settle_at_end")
    yield fly_to(north=0, east=0, alt_m=ALT_M, name="return_home")
    yield brake(name="pre_land")
    yield land()


spray_mission.requires_senses = ["pose", "obstacle", "status"]


def main() -> None:
    with boot_drone() as drone:
        drone.add_service(Sprayer())
        drone.fly(spray_mission)
        print(f"[spray_mission] modes:    {drone.list_modes()}")
        print(f"[spray_mission] senses:   {drone.list_senses()}")
        print(f"[spray_mission] services: {drone.list_services()}")
        drone.run()


if __name__ == "__main__":
    main()
