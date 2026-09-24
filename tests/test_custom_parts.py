"""Unit tests for the custom sense, skills and service in Level 5."""
import csv
from types import SimpleNamespace

from skytrack_autonomy.core.lib.scheduling import ScheduleGroup
from skytrack_autonomy.core.testing import FakeClock, make_fake_ctx


def pose(x, y, z):
    return SimpleNamespace(x=x, y=y, z=z, vx=0, vy=0, vz=0, heading=0.0)


def test_geofence_sense():
    from examples.custom_sense_mission import GeofenceSense

    fence = GeofenceSense(radius_m=10, max_alt_m=5)
    ctx = make_fake_ctx()
    fence.attach(ctx.world)
    assert fence.is_inside and fence.distance_from_home_m is None

    ctx.world.push_local_position(pose(3, 4, -2))
    assert fence.distance_from_home_m == 5 and fence.is_inside

    ctx.world.push_local_position(pose(0, 0, -8))      # too high
    assert not fence.is_inside and fence.breaches == 1
    assert fence.allows(6, 8, 3) and not fence.allows(9, 9, 3)


def test_hover_for_seconds_skill():
    from examples.custom_skill_mission import HoverForSeconds

    clock = FakeClock()
    ctx = make_fake_ctx(clock=clock)
    skill = HoverForSeconds(2)
    skill.start(ctx)
    clock.advance(0.15)
    ctx.scheduler.tick(ScheduleGroup.CONTROL, ctx.world.now())
    assert len(ctx.world.setpoints) >= 1 and not skill.is_done
    clock.advance(2.0)
    assert skill.is_done
    skill.cancel(ctx, "done")
    skill.cancel(ctx, "twice is fine")


def test_climb_by_skill_targets_new_altitude():
    from examples.custom_skill_mission import ClimbBy

    clock = FakeClock()
    ctx = make_fake_ctx(clock=clock)
    start_z = ctx.senses.pose.current_position.z
    skill = ClimbBy(3.0)
    skill.start(ctx)
    clock.advance(0.15)
    ctx.scheduler.tick(ScheduleGroup.CONTROL, ctx.world.now())
    assert abs(ctx.world.setpoints[-1].position[2] - (start_z - 3.0)) < 1e-6
    assert not skill.is_done
    skill.cancel(ctx, "test")


def test_telemetry_logger_service(tmp_path):
    from examples.custom_service_mission import TelemetryLogger

    clock = FakeClock()
    ctx = make_fake_ctx(clock=clock)
    tlm = TelemetryLogger(output_dir=str(tmp_path), hz=5.0)
    tlm.attach(ctx)
    for _ in range(3):
        clock.advance(0.25)
        ctx.scheduler.tick(ScheduleGroup.MEDIA, ctx.world.now())
    tlm.mark("hello")
    tlm.shutdown()
    tlm.shutdown()                                     # idempotent

    rows = list(csv.reader(open(tlm.path)))
    assert rows[0] == ["t", "north", "east", "alt", "armed", "note"]
    assert rows[-1][-1] == "hello" and tlm.rows == len(rows) - 1 >= 3
