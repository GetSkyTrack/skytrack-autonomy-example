"""Custom mode mission — a behaviour started by a command.

Level 5 · Build your own

``drone.fly(fn)`` runs one mission once. A **mode** is different: it
is registered up front and waits until a **command** opens its gate.
The Supervisor then runs it, unless a higher-priority mode (emergency
stop, pause, landing) wants control. Use this for behaviours an
operator, a button or the chat assistant can trigger at any time.

Three pieces:

* ``InspectParams``  — what the behaviour needs (a frozen dataclass)
* ``InspectRequest`` — a ``Command`` whose ``apply`` stores the params
  and raises the gate flag
* ``InspectMode``    — a ``ControlMode`` whose ``program`` yields steps

Run::

    python -m local_planner.examples.custom_mode_mission

The process keeps running after the drone lands (it waits for the next
command). Stop it with Ctrl-C.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterator

from local_planner import (
    Command,
    ControlMode,
    OnDone,
    SkillStep,
    boot_drone,
    brake,
    fly_to,
    land,
    orbit,
    takeoff,
)

TAG = "[INSPECT]"


# ── 1. Parameters ──────────────────────────────────────────────────

@dataclass(frozen=True)
class InspectParams:
    north: float = 6.0
    east: float = 0.0
    alt_m: float = 4.0
    radius_m: float = 3.0
    laps: int = 1


# ── 2. The command that triggers it ────────────────────────────────

@dataclass(frozen=True)
class InspectRequest(Command):
    """Inspect a point. ``active=False`` withdraws the request."""
    active: bool = True
    params: InspectParams = InspectParams()

    def apply(self, ctx: Any) -> None:
        if self.active:
            ctx.inspect_params = self.params
            ctx.flags.inspect_requested = True
            ctx.world.log_info(f"{TAG} requested: {self.params}")
        else:
            ctx.flags.inspect_requested = False
            ctx.inspect_params = None


# ── 3. The mode ────────────────────────────────────────────────────

class InspectMode(ControlMode):
    """Take off, circle the requested point, come home, land."""

    requires_senses = ["pose", "obstacle", "status"]

    @property
    def id(self) -> str:
        return "inspect"

    def on_setup(self, supervisor: Any) -> None:
        # Optional hook, runs once on entry. Note: gets the supervisor;
        # the context is self._ctx.
        self._ctx.world.log_info(f"{TAG} mode entered")

    def program(self, ctx: Any) -> Iterator[SkillStep]:
        p = ctx.inspect_params
        if p is None:
            ctx.world.log_warn(f"{TAG} no params, aborting")
            return
        lap_s = 15.0
        yield takeoff(alt_m=p.alt_m)
        yield fly_to(north=p.north - p.radius_m, east=p.east, alt_m=p.alt_m,
                     name="inspect_approach")
        yield orbit(center_north=p.north, center_east=p.east, alt_m=p.alt_m,
                    radius_m=p.radius_m, period_s=lap_s,
                    duration_s=lap_s * p.laps, name="inspect_orbit")
        yield brake(name="inspect_settle")
        yield fly_to(north=0, east=0, alt_m=p.alt_m, name="inspect_return")
        yield brake(name="inspect_pre_land")
        yield land(name="inspect_land")


# ── 4. Wire + run ──────────────────────────────────────────────────

def main() -> None:
    with boot_drone() as drone:
        drone.add_mode(
            InspectMode,
            gate="inspect_requested",       # ctx.flags.inspect_requested
            gate_params="inspect_params",   # cleared when the program ends
            on_done=OnDone.LAND_AND_HOLD,   # stay down afterwards
        )

        # Optional: let the chat assistant trigger it ("inspect the tower").
        # drone.expose_chat_tool(
        #     name="inspect",
        #     description="Circle a point and come back.",
        #     command=InspectRequest,
        # )

        print(f"[custom_mode_mission] modes:  {drone.list_modes()}")
        print(f"[custom_mode_mission] senses: {drone.list_senses()}")

        # Trigger it once now. Any code (a button, a timer, a sense
        # reading) can submit the same command later.
        drone.submit(InspectRequest(params=InspectParams(laps=2)))

        # No drone.fly() mission here, so keep running until Ctrl-C.
        drone.run(exit_on_complete=False)


if __name__ == "__main__":
    main()
