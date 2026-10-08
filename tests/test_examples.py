"""Every example follows the app's mission-file conventions and its
mission can be stepped through end to end."""
import ast
import importlib
from pathlib import Path
from unittest.mock import MagicMock

import pytest

from skytrack_autonomy.core.skill_base import SkillStep
from skytrack_autonomy.core.testing import make_fake_ctx
from skytrack_autonomy.core.testing.senses import FakeBatterySense

EXAMPLES_DIR = Path(__file__).resolve().parent.parent / "examples"
FILES = sorted(p for p in EXAMPLES_DIR.glob("*_mission.py"))
NAMES = [p.stem for p in FILES]
MODE_ONLY = {"custom_mode_mission"}          # no drone.fly() mission


def load(name):
    return importlib.import_module(f"examples.{name}")


def fake_ctx(battery_percent=90.0):
    """Fake context with every sense and service the examples use."""
    from examples.custom_sense_mission import GeofenceSense

    fence = GeofenceSense(radius_m=25, max_alt_m=15)
    ctx = make_fake_ctx(senses={
        "battery": FakeBatterySense(percent=battery_percent),
        "camera": MagicMock(name="camera"),
        "geofence": fence,
    })
    fence.attach(ctx.world)
    for service in ("recorder", "snapshot", "sprayer", "detector", "telemetry"):
        ctx.services.register(service, MagicMock(name=service))
    return ctx


def steps_of(gen):
    steps = list(gen)
    for s in steps:
        assert isinstance(s, SkillStep) or hasattr(s, "start"), s
    return steps


def skill_names(steps):
    return [type(getattr(s, "skill", s)).__name__ for s in steps]


# ── Conventions ────────────────────────────────────────────────────

@pytest.mark.parametrize("path", FILES, ids=NAMES)
def test_file_follows_mission_conventions(path):
    tree = ast.parse(path.read_text())
    doc = ast.get_docstring(tree) or ""
    assert f"python -m local_planner.examples.{path.stem}" in doc
    assert "Level " in doc, "docstring should state the example's level"
    functions = {n.name for n in tree.body if isinstance(n, ast.FunctionDef)}
    assert "main" in functions
    assert "from __future__ import annotations" in path.read_text()


@pytest.mark.parametrize("name", [n for n in NAMES if n not in MODE_ONLY])
def test_mission_declares_senses(name):
    mission = getattr(load(name), name)
    senses = mission.requires_senses
    assert isinstance(senses, list) and "pose" in senses


# ── Every mission runs start to finish on a fake drone ─────────────

@pytest.mark.parametrize("name", [n for n in NAMES if n not in MODE_ONLY])
def test_mission_starts_with_takeoff_and_ends_with_land(name):
    steps = steps_of(getattr(load(name), name)(fake_ctx()))
    names = skill_names(steps)
    assert names[0] == "TakeoffSkill"
    assert names[-1] == "LandSkill"
    assert names[-2] in ("BrakeAndSettleSkill", "LandSkill") or name == "hello_mission"


def test_lawnmower_alternates_line_direction():
    m = load("lawnmower_mission")
    lines = m.sweep_lines()
    assert len(lines) == 5
    assert lines[0][0][0] == 0 and lines[1][0][0] == m.AREA_NORTH_M


def test_battery_check_goes_home_early_when_low():
    m = load("battery_check_mission")
    full = steps_of(m.battery_check_mission(fake_ctx(90)))
    low = steps_of(m.battery_check_mission(fake_ctx(10)))
    assert len(low) < len(full)
    assert not any(s.name.startswith("patrol_") for s in low)


def test_site_survey_skips_inspection_when_battery_low():
    m = load("site_survey_mission")
    low = steps_of(m.site_survey_mission(fake_ctx(10)))
    assert not any(getattr(s, "name", "").startswith("photo_") for s in low)
    full = steps_of(m.site_survey_mission(fake_ctx(90)))
    assert sum(getattr(s, "name", "").startswith("photo_") for s in full) == 3


def test_custom_mode_program_and_command():
    m = load("custom_mode_mission")
    ctx = fake_ctx()
    ctx.flags.inspect_requested = False
    m.InspectRequest(params=m.InspectParams(laps=2)).apply(ctx)
    assert ctx.flags.inspect_requested is True
    mode = m.InspectMode(ctx=ctx)
    names = skill_names(steps_of(mode.program(ctx)))
    assert names[0] == "TakeoffSkill" and names[-1] == "LandSkill"
    m.InspectRequest(active=False).apply(ctx)
    assert ctx.flags.inspect_requested is False


def test_no_fly_zone_splits_the_line_and_goes_around():
    m = load("no_fly_zone_mission")
    steps = steps_of(m.no_fly_zone_mission(fake_ctx()))
    names = [getattr(s, "name", "") for s in steps]
    legs = [n for n in names if n.startswith("leg_")]
    # START ─ C′ (on path) ~ D′ (transit around the zone) ─ END (on path)
    assert [n.split("_", 2)[2] for n in legs] == ["on_path", "transit", "on_path"]


def test_no_fly_zone_refuses_end_inside_a_zone(monkeypatch):
    m = load("no_fly_zone_mission")
    monkeypatch.setattr(m, "END", (9.0, 0.0))     # inside demo_zone
    with pytest.raises(ValueError, match="inside a no-fly zone"):
        steps_of(m.no_fly_zone_mission(fake_ctx()))


def test_tuning_config_builds_a_valid_yaml():
    from skytrack_autonomy.core.config import ConfigError, load_planner_config
    m = load("tuning_config_mission")
    path = m.build_config_file()
    try:
        cfg = load_planner_config(path)
        assert cfg.obstacle.retention_s == m.YAML_OVERRIDES["obstacle"]["retention_s"]
        assert cfg.motion.pathfinding_algorithm.name == "BOUNDED_ASTAR"
        assert f"config_file:={path}" in m.ros_args(path)
    finally:
        path.unlink()
    with pytest.raises(ConfigError):                # typos fail before take-off
        m.build_config_file(overrides={"obstacle": {"retension_s": 30}})


def test_tuning_config_flies_one_step_per_leg():
    m = load("tuning_config_mission")
    names = [getattr(s, "name", "") for s in steps_of(m.tuning_config_mission(fake_ctx()))]
    for leg in m.LEGS:
        assert any(n.startswith(leg["name"]) for n in names)


def test_avoidance_algorithm_switches_config_per_leg_and_restores():
    m = load("avoidance_algorithm_mission")
    ctx = fake_ctx()
    original = ctx.config
    seen = {}
    gen = m.avoidance_algorithm_mission(ctx)
    for step in gen:                       # config as the step would start with it
        seen[getattr(step, "name", "")] = ctx.config.motion
    assert seen["astar_out"].pathfinding_algorithm is m.PathAlgo.BOUNDED_ASTAR
    assert seen["theta_back"].pathfinding_algorithm is m.PathAlgo.BOUNDED_THETA_STAR
    assert seen["astar_eager_out"].obstacle_replan_strategy is m.ReplanStrategy.EAGER
    assert seen["astar_eager_out"].replan_trigger_mode is m.ReplanTriggerMode.PROXIMITY
    assert ctx.config is original          # put back after the mission


def test_simple_astar_goes_around_a_wall():
    m = load("avoidance_algorithm_mission")
    wall = {(5, y, z) for y in range(-3, 4) for z in range(-2, 3)}
    path = m.simple_astar((0, 0, 0), (10, 0, 0), wall)
    assert path[0] == (0, 0, 0) and path[-1] == (10, 0, 0)
    assert not set(path) & wall
    assert all(v[2] <= 0 for v in path)    # never below the endpoints
    assert m.simple_astar((0, 0, 0), (5, 0, 0), wall) is None   # goal blocked


def test_install_pathfinder_restores_the_builtin():
    from skytrack_autonomy.contrib.skills.fly_trajectory import pipeline
    m = load("avoidance_algorithm_mission")
    slot = m.PathAlgo.LAZY_THETA_STAR
    before = pipeline._PATH_ALGO_DISPATCH[slot]
    restore = m.install_pathfinder(m.simple_astar, slot)
    path, _ = pipeline._PATH_ALGO_DISPATCH[slot]((0, 0, 0), (3, 0, 0), set(), 0, {}, None)
    assert path[-1] == (3, 0, 0)
    restore()
    assert pipeline._PATH_ALGO_DISPATCH[slot] is before
