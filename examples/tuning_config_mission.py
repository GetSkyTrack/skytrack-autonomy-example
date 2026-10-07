"""Tuning config mission — change the flight algorithm's settings.

Level 3 · Mission logic

What you learn
==============
The planner's settings come from three places. This example uses all
three:

1. **Planner YAML** (read once, when the node starts). The node loads
   the file named by the ROS parameter ``config_file`` (default: the
   app's ``local_planner/config/default.yaml``). It holds the path
   finder, trajectory, replanning, speed caps, obstacle map (voxel
   size, sensor range, **obstacle retention**), vehicle size and
   landing scan. ``build_config_file()`` copies ``default.yaml``,
   applies ``YAML_OVERRIDES`` and checks the result with the same
   strict loader the node uses: a typo or a wrong type fails here,
   before take-off.
2. **ROS parameters** (when the node starts; some can also be changed
   while flying with ``ros2 param set /local_planner <name> <value>``):
   ``target_speed``, ``obstacle_avoidance_mode``, the arrival window,
   ``obstacle_backend``, ``nfz_file``. See ``ROS_PARAMS``.
3. **Per-leg ``fly_to`` arguments**, which apply to one leg only:
   ``target_speed``, ``mode``, ``replan_mode``, ``yaw_mode``,
   ``yaw_rate_deg_s``.

Mission: take off, fly three legs with different per-leg settings,
come home, land. At start it logs the settings the node actually uses
(``ctx.config``).

Notes
=====
* A YAML that sets only some fields is **not** merged with
  ``default.yaml``: missing fields take the code defaults (for example
  ``retention_s: inf`` and ``LAZY_THETA_STAR``). That is why
  ``build_config_file()`` starts from a copy of ``default.yaml``.
* ``motion.target_speed`` in the YAML has no effect: the node replaces
  it with the ``target_speed`` ROS parameter (default 5.0). Set the
  cruise speed with that ROS parameter or per leg.
* The YAML is read only when the node starts. Changing it needs a
  new run.

Run::

    python -m local_planner.examples.tuning_config_mission
"""
from __future__ import annotations

import copy
import os
import tempfile
from pathlib import Path
from typing import Any, Dict, Iterator, List, Optional

import yaml

from local_planner import boot_drone, brake, fly_to, land, takeoff
from skytrack_autonomy.core.config import load_planner_config

ALT_M = 3.0
TAG = "[TUNING]"

# ── 1. Planner YAML overrides ──────────────────────────────────────
# Only the keys you change. Everything else comes from default.yaml.
# Values are examples, not recommendations.
YAML_OVERRIDES: Dict[str, Any] = {
    "motion": {
        # Path finder: BOUNDED_ASTAR | BOUNDED_THETA_STAR | LAZY_THETA_STAR
        "pathfinding_algorithm": "BOUNDED_ASTAR",
        "bounded_astar_radius_m": 10.0,      # search sphere around the drone
        # Trajectory: acceleration / braking along the path.
        "target_accel": 1.5,                 # m/s²
        "target_decel": 1.0,                 # m/s²
        # Replanning: SLOW stops to replan, FAST replans while flying.
        "replan_mode_setting": "SLOW",
        "obstacle_replan_strategy": "DEFERRED",
        "deferred_replan_distance": 2.5,     # m: replan this close to a new obstacle
        # Speed caps near unknown / known obstacles.
        "limit_speed_to_sensor_range": False,
        "explore_approach_enabled": True,
        "explore_standoff_m": 1.5,           # m kept from an obstacle on the path
        # Path tracking and heading.
        "tracking_corridor_m": 0.5,          # m of drift before a DRIFT replan
        "yaw_rate_deg_s": 45.0,
        "rotation_skip_below_deg": 30.0,
    },
    "obstacle": {
        # How long (s) an obstacle voxel stays in the map after it was
        # last seen. .inf = forever (static SDF world); 30-60 s for the
        # depth camera; 0 = only the current frame (unsafe).
        "retention_s": 30.0,
        "obstacle_sensor_range": 10.0,       # m, also sets the speed-cap horizon
        "obstacle_mode": "ACCUMULATED",      # or CURRENT
    },
    "vehicle": {
        # Physical size incl. props + safety margin. The obstacle
        # inflation and voxel size are derived from these.
        "safety_margin_xy_m": 0.45,          # default 0.37 → 0.98 m clearance
    },
    "arrival": {
        "tolerance": 0.3,                    # m
        "speed_threshold": 0.15,             # m/s
        "settling_time": 0.2,                # s
        "stall_timeout": 1.5,                # s
    },
}

# ── 2. ROS parameters (node start) ─────────────────────────────────
# Types must match the node's declaration: write 3.0, not 3, for floats.
ROS_PARAMS: Dict[str, Any] = {
    "target_speed": 3.0,                     # m/s cruise (default 5.0)
    "obstacle_avoidance_mode": "avoid",      # avoid | brake | off
    # Arrival window: these win over the YAML ``arrival`` section.
    "target_tolerance": 0.3,
    # "obstacle_backend": "depth_camera",    # sdf (default) | depth_camera
    # "nfz_file": "/app/nfz/no_fly_zones.json",
}

# ── 3. Per-leg fly_to settings ─────────────────────────────────────
LEGS: List[Dict[str, Any]] = [
    # Slow, keep the current heading.
    dict(name="slow_hold_heading", north=6.0, east=0.0,
         target_speed=1.0, yaw_mode="hold"),
    # Stay close to the straight line, replan while flying.
    dict(name="coverage_fast_replan", north=6.0, east=6.0,
         target_speed=2.0, mode="coverage", replan_mode="fast"),
    # Face along the path, turn slowly at corners.
    dict(name="course_slow_turns", north=0.0, east=6.0,
         yaw_mode="course", yaw_rate_deg_s=20.0),
]


# ── Building the config file ───────────────────────────────────────

def _deep_merge(base: Dict[str, Any], overrides: Dict[str, Any]) -> Dict[str, Any]:
    out = copy.deepcopy(base)
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _deep_merge(out[key], value)
        else:
            out[key] = value
    return out


def default_config_path() -> Optional[Path]:
    """The app's ``default.yaml``, or None outside the app."""
    try:
        import local_planner
        path = Path(local_planner.__file__).resolve().parent / "config" / "default.yaml"
    except (ImportError, AttributeError, TypeError):
        return None
    return path if path.exists() else None


def build_config_file(base: Optional[Path] = None,
                      overrides: Dict[str, Any] = YAML_OVERRIDES) -> Path:
    """Write ``base`` + ``overrides`` to a temp YAML and validate it.

    Raises ``ConfigError`` (unknown key, wrong type, bad enum value)
    before the drone does anything.
    """
    raw: Dict[str, Any] = {}
    if base is not None:
        raw = yaml.safe_load(base.read_text()) or {}
    merged = _deep_merge(raw, overrides)
    fd, name = tempfile.mkstemp(prefix="tuning_", suffix=".yaml")
    with os.fdopen(fd, "w") as fh:
        yaml.safe_dump(merged, fh, sort_keys=False)
    try:
        load_planner_config(name)            # strict check, same as the node
    except Exception:
        os.unlink(name)
        raise
    return Path(name)


def ros_args(config_file: Path, params: Dict[str, Any] = ROS_PARAMS) -> List[str]:
    """``--ros-args -p name:=value ...`` for ``rclpy.init(args=...)``."""
    args = ["tuning_config_mission", "--ros-args",
            "-p", f"config_file:={config_file}"]
    for key, value in params.items():
        if isinstance(value, bool):
            value = str(value).lower()
        args += ["-p", f"{key}:={value}"]
    return args


# ── Mission ────────────────────────────────────────────────────────

def _log_effective_config(ctx: Any) -> None:
    """Log what the node really uses (after YAML + ROS params)."""
    cfg = getattr(ctx, "config", None)
    if cfg is None or not hasattr(cfg, "motion"):
        return
    m, a = cfg.motion, cfg.arrival
    log = ctx.world.log_info
    log(f"{TAG} motion: algo={m.pathfinding_algorithm.name} "
        f"speed={m.target_speed} m/s accel={m.target_accel} "
        f"decel={m.target_decel} replan={m.replan_mode_setting.name} "
        f"corridor={m.tracking_corridor_m} m")
    log(f"{TAG} arrival: tol={a.tolerance} m speed<{a.speed_threshold} m/s "
        f"settle={a.settling_time} s stall={a.stall_timeout} s")
    log(f"{TAG} obstacle avoidance: "
        f"{'on' if cfg.obstacle_avoidance_enabled else 'off'}")


def tuning_config_mission(ctx: Any) -> Iterator[Any]:
    """Take off, fly three legs with per-leg settings, come home, land."""
    _log_effective_config(ctx)
    yield takeoff(alt_m=ALT_M)

    for leg in LEGS:
        leg = dict(leg)
        name = leg.pop("name")
        ctx.world.log_info(f"{TAG} {name}: {leg}")
        yield fly_to(alt_m=ALT_M, name=name, **leg)

    yield fly_to(north=0.0, east=0.0, alt_m=ALT_M, name="return_home")
    yield brake(name="pre_land")
    yield land()


tuning_config_mission.requires_senses = ["pose", "obstacle", "status"]


def main() -> None:
    import rclpy

    config_file = build_config_file(default_config_path())
    print(f"[tuning_config_mission] config_file: {config_file}")
    # boot_drone() only calls rclpy.init() when ROS is not up yet, so
    # init here with our parameters; they apply to the planner node.
    rclpy.init(args=ros_args(config_file))
    try:
        with boot_drone() as drone:
            drone.fly(tuning_config_mission)
            print(f"[tuning_config_mission] modes:  {drone.list_modes()}")
            print(f"[tuning_config_mission] senses: {drone.list_senses()}")
            drone.run()
    finally:
        config_file.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
