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
