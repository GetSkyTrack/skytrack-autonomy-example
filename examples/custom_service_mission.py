"""Custom service mission — write your own background service.

Level 5 · Build your own

A **service** runs alongside the whole mission. It may write files or
drive a payload but must **never send a setpoint** (that's a skill's
job). To write one, give it:

* ``name``             — the key under ``ctx.services``
* ``attach(ctx)``      — wire up; called by ``drone.add_service``
* ``shutdown()``       — flush + release; called at the end, must be
                         safe to call twice
* ``mission_ended()``  — optional, called when the mission finishes
* your own methods     — what the mission calls (here: ``mark()``)

Schedule periodic work on ``ScheduleGroup.MEDIA`` so it can never
delay flight control.

This one logs position to ``~/.ros/telemetry/flight_<time>.csv``.

Run::

    python -m local_planner.examples.custom_service_mission
"""
from __future__ import annotations

import csv
import os
import time
from typing import Any, Iterator, Optional

from local_planner import boot_drone, brake, fly_to, land, orbit, takeoff
from skytrack_autonomy.core.lib.scheduling import ScheduleGroup

ALT_M = 3.0


# ── 1. The custom service ──────────────────────────────────────────

class TelemetryLogger:
    """Write pose + arm state to CSV at ``hz``; ``mark()`` adds a note."""

    name = "telemetry"

    def __init__(self, *, output_dir: str = "~/.ros/telemetry",
                 hz: float = 5.0) -> None:
        self._output_dir = os.path.expanduser(output_dir)
        self._hz = float(hz)
        self._ctx: Any = None
        self._file = None
        self._writer = None
        self._handle = None
        self.rows = 0
        self.path: Optional[str] = None

    def attach(self, ctx: Any) -> None:
        self._ctx = ctx
        os.makedirs(self._output_dir, exist_ok=True)
        self.path = os.path.join(
            self._output_dir, time.strftime("flight_%Y%m%d_%H%M%S.csv"))
        self._file = open(self.path, "w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(["t", "north", "east", "alt", "armed", "note"])
        # Runs for the whole mission, so start sampling now.
        self._handle = ctx.scheduler.schedule(
            self._sample, hz=self._hz, group=ScheduleGroup.MEDIA,
            name=self.name, now=ctx.world.now())

    def mark(self, note: str) -> None:
        """Mission-facing API: write a labelled row right now."""
        if self._ctx is None:
            raise RuntimeError("TelemetryLogger.mark() before attach()")
        self._sample(note)

    def mission_ended(self) -> None:
        self.mark("mission ended")

    def shutdown(self) -> None:
        if self._handle is not None:
            self._ctx.scheduler.unschedule(self._handle)
            self._handle = None
        if self._file is not None:
            self._file.close()
            self._file = None
            self._ctx.world.log_info(f"[TLM] {self.rows} rows → {self.path}")

    def _sample(self, note: str = "") -> None:
        p = self._ctx.senses.pose.current_position
        if p is None or self._writer is None:
            return
        self._writer.writerow([
            f"{self._ctx.world.now():.2f}", f"{p.x:.2f}", f"{p.y:.2f}",
            f"{-p.z:.2f}", self._ctx.senses.status.is_armed, note])
        self.rows += 1


# ── 2. A mission that uses it ──────────────────────────────────────

def custom_service_mission(ctx: Any) -> Iterator[Any]:
    """Short flight with labelled events in the telemetry log."""
    tlm = ctx.services.telemetry

    yield takeoff(alt_m=ALT_M)
    tlm.mark("airborne")
    yield fly_to(north=5, east=0, alt_m=ALT_M, name="to_orbit")
    tlm.mark("orbit start")
    yield orbit(center_north=5, center_east=3, alt_m=ALT_M,
                radius_m=3.0, period_s=15.0, duration_s=15.0, name="orbit")
    tlm.mark("orbit end")
    yield fly_to(north=0, east=0, alt_m=ALT_M, name="return_home")
    yield brake(name="pre_land")
    yield land()


custom_service_mission.requires_senses = ["pose", "obstacle", "status"]


# ── 3. Wire + run ──────────────────────────────────────────────────

def main() -> None:
    with boot_drone() as drone:
        drone.add_service(TelemetryLogger(hz=5.0))
        drone.fly(custom_service_mission)
        print(f"[custom_service_mission] modes:    {drone.list_modes()}")
        print(f"[custom_service_mission] senses:   {drone.list_senses()}")
        print(f"[custom_service_mission] services: {drone.list_services()}")
        drone.run()


if __name__ == "__main__":
    main()
