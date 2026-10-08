"""No-fly zone mission — fly a straight line that crosses a no-fly zone.

Level 3 · Mission logic

What you learn
==============
* How the app handles no-fly zones (NFZ): the node watches a zone file
  (ROS parameter ``nfz_file``, default ``/app/nfz/no_fly_zones.json``)
  and feeds every zone to the planner **as an obstacle**. ``fly_to``
  goes around a zone exactly like it goes around a building.
* The zone-file format (``{"zones": [...]}``, ``frame`` and ``units``
  per zone, ``polygon`` or ``circle``, ``z_range``).
* Splitting a route at zone crossings with
  ``skytrack_autonomy.contrib.nfz_flow.split_nfz``. The parts of the
  line outside the zone are flown in ``coverage`` mode (hug the line),
  and the leg that crosses the zone in ``transit`` mode (shortest way
  around).
* Checking before take-off that no target lies inside a zone. Such a
  target cannot be reached: the planner would stop at the wall forever.

What happens
============
::

    START ──── C′ ~ ~ (transit: goes around) ~ ~ D′ ──── END
                     ╭────────────╮
                     │ demo_zone  │
                     ╰────────────╯

If the zone file does not exist, the mission writes a demo zone (a
4 × 4 m square on the line, 0–20 m high), uses it, and deletes it when
the mission ends. An existing zone file is used as-is and never
changed.

Run::

    python -m local_planner.examples.no_fly_zone_mission
"""
from __future__ import annotations

import json
import os
from typing import Any, Iterator, List, Tuple

from local_planner import boot_drone, brake, fly_to, land, takeoff
from skytrack_autonomy.contrib.nfz_flow import ON_PATH, load_zones, split_nfz
from skytrack_autonomy.contrib.senses.no_fly_zone_sense import (
    parse_nfz_document,
)

ALT_M = 3.0
TAG = "[NFZ]"

# Must match the node's ``nfz_file`` parameter.
NFZ_FILE = os.environ.get("NFZ_FILE", "/app/nfz/no_fly_zones.json")

# The line to fly, (north, east) in metres from home.
START = (2.0, 0.0)
END = (16.0, 0.0)

# Demo zone, only written when NFZ_FILE does not exist. local_enu
# vertices are [east, north] (order fixed by the file contract).
DEMO_ZONES = {
    "zones": [
        {
            "id": "demo_zone",
            "frame": "local_enu",
            "units": "m",
            "polygon": [[-2.0, 7.0], [2.0, 7.0], [2.0, 11.0], [-2.0, 11.0]],
            "z_range": [0, 20],
        },
    ],
}


def _write_demo_zones(ctx: Any) -> bool:
    """Write ``DEMO_ZONES`` to ``NFZ_FILE``. True if the file was written."""
    try:
        os.makedirs(os.path.dirname(NFZ_FILE), exist_ok=True)
        tmp = NFZ_FILE + ".tmp"
        with open(tmp, "w", encoding="utf-8") as fh:
            json.dump(DEMO_ZONES, fh, indent=2)
        os.replace(tmp, NFZ_FILE)     # atomic: the node never reads half a file
    except OSError as exc:
        ctx.world.log_warn(
            f"{TAG} cannot write {NFZ_FILE} ({exc}) — the planner will NOT "
            f"see the demo zone; the route is still split for it")
        return False
    ctx.world.log_info(f"{TAG} wrote demo zone to {NFZ_FILE}")
    return True


def _plan_route(ctx: Any, zones: list) -> Tuple[List[Tuple[float, float]], List[str]]:
    """Split START→END at zone crossings and log the result."""
    route, kinds = split_nfz([START, END], zones, alt_m=ALT_M)
    if route[-1] != END:
        # END is inside a zone: refuse the mission before take-off.
        raise ValueError(f"{TAG} END {END} lies inside a no-fly zone")

    ctx.world.log_info(
        f"{TAG} {len(zones)} zone(s); {START} → {END} split into "
        f"{len(route) - 1} leg(s):")
    for i, (north, east) in enumerate(route):
        leg = f"  --[{kinds[i]}]-->" if i < len(kinds) else ""
        ctx.world.log_info(f"{TAG}   N{north:6.2f} E{east:6.2f}{leg}")
    return route, kinds


def no_fly_zone_mission(ctx: Any) -> Iterator[Any]:
    """Fly START→END around a no-fly zone, come home, land."""
    wrote_demo = False
    if os.path.exists(NFZ_FILE):
        zones = load_zones(NFZ_FILE, log=ctx.world.log_info)
    else:
        wrote_demo = _write_demo_zones(ctx)
        # Use the demo zones even if writing failed, so the split is shown.
        zones = list(parse_nfz_document(DEMO_ZONES).zones)

    try:
        route, kinds = _plan_route(ctx, zones)

        # The node re-reads the zone file once a second; arming and
        # climbing take longer, so the zone is loaded before the first fly_to.
        yield takeoff(alt_m=ALT_M)

        # The first leg (home → START) just gets to the line.
        yield fly_to(north=START[0], east=START[1], alt_m=ALT_M,
                     name="to_start")
        for i, ((north, east), kind) in enumerate(zip(route[1:], kinds), start=1):
            mode = "coverage" if kind == ON_PATH else "transit"
            ctx.world.log_info(f"{TAG} leg {i}/{len(kinds)}: {kind} → {mode}")
            yield fly_to(north=north, east=east, alt_m=ALT_M, mode=mode,
                         name=f"leg_{i}_{kind}")

        # Home is on the other side of the zone: plain fly_to goes around.
        yield fly_to(north=0.0, east=0.0, alt_m=ALT_M, name="return_home")
        yield brake(name="pre_land")
        yield land()
    finally:
        # Runs when the mission ends, fails or is cancelled.
        if wrote_demo:
            try:
                os.remove(NFZ_FILE)
                ctx.world.log_info(f"{TAG} removed demo zone file")
            except OSError:
                pass


no_fly_zone_mission.requires_senses = ["pose", "obstacle", "status"]


def main() -> None:
    with boot_drone() as drone:
        drone.fly(no_fly_zone_mission)
        print(f"[no_fly_zone_mission] modes:  {drone.list_modes()}")
        print(f"[no_fly_zone_mission] senses: {drone.list_senses()}")
        drone.run()


if __name__ == "__main__":
    main()
