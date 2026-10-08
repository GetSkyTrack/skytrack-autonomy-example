"""Avoidance algorithm mission — switch the obstacle-avoidance algorithm
per leg, and plug in your own path finder.

Level 3 · Mission logic

What you learn
==============
* ``fly_to`` builds its planner from ``ctx.config.motion`` **when the
  leg starts**. So a mission can swap the motion config just for one
  leg (``with_motion(...)`` below) and put it back afterwards. The
  rest of the app is not affected.
* What can be switched for one leg:

  - ``pathfinding_algorithm`` — the path finder:

    * ``BOUNDED_ASTAR``: A* inside a sphere around the drone
      (``bounded_astar_radius_m``). Constant cost; far space counted
      as free and fixed by replanning.
    * ``BOUNDED_THETA_STAR``: any-angle Theta* limited to the box of
      known obstacles, then straight to the goal.
    * ``LAZY_THETA_STAR``: any-angle Theta* (C++). Cost grows with the
      leg length.

  - ``obstacle_replan_strategy`` — ``EAGER`` (replan as soon as a new
    obstacle blocks the path) or ``DEFERRED`` (wait until the drone is
    ``deferred_replan_distance`` metres away from it).
  - ``replan_trigger_mode`` — ``TRAJECTORY`` (the rest of the path
    hits an obstacle) or ``PROXIMITY`` (an obstacle comes close to the
    drone).
  - ``fly_to(mode="direct")`` — no planner: straight line, brake if an
    obstacle is ahead.

* **Your own path finder** (``USE_CUSTOM_PATHFINDER``):
  ``simple_astar`` is a small A* on the voxel grid. It is put in place
  of one built-in algorithm for one leg. This uses a **private** table
  of the framework (``_PATH_ALGO_DISPATCH``): fine for experiments, but
  it may change between versions. To ship an algorithm, add it to
  ``skytrack_autonomy`` (``PathAlgo`` enum + adapter).

Route (world ``warehouse``; a pillar stands at about N +0.6..+2.4,
E +4.8..+6.0 from home, so legs between home and ``BEHIND_PILLAR``
must go around it)::

    home ──A*──> BEHIND_PILLAR ──Theta*──> home ──direct──> CLEAR_NORTH
         ──A* EAGER/PROXIMITY──> BEHIND_PILLAR ──Lazy Theta* or yours──> home

Run::

    python -m local_planner.examples.avoidance_algorithm_mission
"""
from __future__ import annotations

import heapq
import math
from dataclasses import replace
from typing import Any, Callable, Iterator, List, Optional, Tuple

from local_planner import boot_drone, brake, fly_to, land, takeoff
from skytrack_autonomy.core.config.enums import (
    PathAlgo,
    ReplanStrategy,
    ReplanTriggerMode,
)

ALT_M = 3.0
SPEED_M_S = 2.0
TAG = "[AVOID_ALGO]"

HOME = (0.0, 0.0)
BEHIND_PILLAR = (1.5, 10.0)        # (north, east); the pillar is on the way
CLEAR_NORTH = (4.0, 0.0)           # free straight line from home

# Fly the last leg with ``simple_astar`` instead of LAZY_THETA_STAR.
USE_CUSTOM_PATHFINDER = False

Voxel = Tuple[int, int, int]


# ── Per-leg motion config ──────────────────────────────────────────

def with_motion(ctx: Any, step: Any, **changes: Any) -> Iterator[Any]:
    """Yield ``step`` with ``ctx.config.motion`` changed by ``changes``,
    then put the old config back (also if the step is cancelled)."""
    old = ctx.config
    ctx.config = replace(old, motion=replace(old.motion, **changes))
    pretty = ", ".join(f"{k}={getattr(v, 'name', v)}" for k, v in changes.items())
    ctx.world.log_info(f"{TAG} {getattr(step, 'name', step)}: {pretty}")
    try:
        yield step
    finally:
        ctx.config = old


def leg(north_east: Tuple[float, float], name: str, **kwargs: Any) -> Any:
    north, east = north_east
    return fly_to(north=north, east=east, alt_m=ALT_M,
                  target_speed=SPEED_M_S, name=name, **kwargs)


# ── Your own path finder ───────────────────────────────────────────

_STEPS = [(dx, dy, dz)
          for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)
          if (dx, dy, dz) != (0, 0, 0)]


def simple_astar(start: Voxel, goal: Voxel, obstacles: Any,
                 *, margin: int = 15, max_expansions: int = 200_000
                 ) -> Optional[List[Voxel]]:
    """26-connected A* on the voxel grid.

    ``obstacles`` is the inflated obstacle set (supports ``in``). The
    search stays inside the start/goal box plus ``margin`` voxels, and
    never goes below the lower of start and goal (NED z: larger index
    = lower), so it cannot route under an obstacle. Returns the voxel
    path, ``[start]`` when start == goal, or ``None``.
    """
    start, goal = tuple(start), tuple(goal)
    if start in obstacles or goal in obstacles:
        return None
    if start == goal:
        return [start]

    lo = [min(s, g) - margin for s, g in zip(start, goal)]
    hi = [max(s, g) + margin for s, g in zip(start, goal)]
    hi[2] = max(start[2], goal[2])               # no lower than the endpoints

    def h(v: Voxel) -> float:
        return math.dist(v, goal)

    open_heap = [(h(start), 0.0, start)]
    came_from = {start: None}
    cost = {start: 0.0}
    expansions = 0
    while open_heap:
        _, g, cur = heapq.heappop(open_heap)
        if cur == goal:
            path = [cur]
            while came_from[path[-1]] is not None:
                path.append(came_from[path[-1]])
            return path[::-1]
        if g > cost[cur]:
            continue                             # stale heap entry
        expansions += 1
        if expansions > max_expansions:
            return None
        for dx, dy, dz in _STEPS:
            nxt = (cur[0] + dx, cur[1] + dy, cur[2] + dz)
            if not all(lo[i] <= nxt[i] <= hi[i] for i in range(3)):
                continue
            if nxt in obstacles:
                continue
            ng = g + math.sqrt(dx * dx + dy * dy + dz * dz)
            if ng < cost.get(nxt, math.inf):
                cost[nxt] = ng
                came_from[nxt] = cur
                heapq.heappush(open_heap, (ng + h(nxt), ng, nxt))
    return None


def install_pathfinder(finder: Callable[[Voxel, Voxel, Any], Optional[List[Voxel]]],
                       slot: PathAlgo) -> Callable[[], None]:
    """Make ``slot`` run ``finder``. Returns a function that undoes it.

    EXPERIMENTAL — patches a private framework table.
    """
    from skytrack_autonomy.contrib.skills.fly_trajectory import pipeline

    table = pipeline._PATH_ALGO_DISPATCH
    original = table[slot]
    table[slot] = (lambda drone, target, obs, radius_voxels, stats, corridor:
                   (finder(drone, target, obs), None))

    def restore() -> None:
        table[slot] = original
    return restore


def custom_pathfinder_leg(ctx: Any, step: Any) -> Iterator[Any]:
    """Fly ``step`` with ``simple_astar`` in the LAZY_THETA_STAR slot."""
    restore = install_pathfinder(simple_astar, PathAlgo.LAZY_THETA_STAR)
    try:
        yield from with_motion(ctx, step,
                               pathfinding_algorithm=PathAlgo.LAZY_THETA_STAR)
    finally:
        restore()


# ── Mission ────────────────────────────────────────────────────────

def avoidance_algorithm_mission(ctx: Any) -> Iterator[Any]:
    """Fly around the pillar with a different avoidance setup per leg."""
    yield takeoff(alt_m=ALT_M)

    # 1. Bounded A*, defaults for replanning.
    yield from with_motion(
        ctx, leg(BEHIND_PILLAR, "astar_out"),
        pathfinding_algorithm=PathAlgo.BOUNDED_ASTAR)
    yield brake(name="settle_1")

    # 2. Bounded Theta*: any-angle path, fewer corners.
    yield from with_motion(
        ctx, leg(HOME, "theta_back"),
        pathfinding_algorithm=PathAlgo.BOUNDED_THETA_STAR)
    yield brake(name="settle_2")

    # 3. No planner: straight line, brake if something is ahead.
    yield leg(CLEAR_NORTH, "direct_north", mode="direct")
    yield brake(name="settle_3")

    # 4. A* that reacts at once to new obstacles near the drone.
    yield from with_motion(
        ctx, leg(BEHIND_PILLAR, "astar_eager_out"),
        pathfinding_algorithm=PathAlgo.BOUNDED_ASTAR,
        obstacle_replan_strategy=ReplanStrategy.EAGER,
        replan_trigger_mode=ReplanTriggerMode.PROXIMITY)
    yield brake(name="settle_4")

    # 5. Lazy Theta* (C++), or your own path finder.
    if USE_CUSTOM_PATHFINDER:
        yield from custom_pathfinder_leg(ctx, leg(HOME, "custom_astar_back"))
    else:
        yield from with_motion(
            ctx, leg(HOME, "lazy_theta_back"),
            pathfinding_algorithm=PathAlgo.LAZY_THETA_STAR)

    yield brake(name="pre_land")
    yield land()


avoidance_algorithm_mission.requires_senses = ["pose", "obstacle", "status"]


def main() -> None:
    with boot_drone() as drone:
        drone.fly(avoidance_algorithm_mission)
        print(f"[avoidance_algorithm_mission] modes:  {drone.list_modes()}")
        print(f"[avoidance_algorithm_mission] senses: {drone.list_senses()}")
        drone.run()


if __name__ == "__main__":
    main()
