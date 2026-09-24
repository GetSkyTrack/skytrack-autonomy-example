# Skills

A **skill** is one time-limited motion: "take off", "fly to this
point", "circle for 30 s", "hover for 5 s". While it runs it owns the
drone's setpoints; when it's finished it says so, and the mission moves
on to the next step.

Skills are the **only** part of the framework that moves the drone.
Senses read, services work in the background, modes decide *which*
skills run — but every metre flown comes from a skill.

**On this page:** [Skills vs steps](#skills-vs-steps) ·
[Lifecycle](#lifecycle) · [Built-in skills](#built-in-skills) ·
[Writing a skill](#writing-a-skill) · [Patterns](#patterns) ·
[Pitfalls](#pitfalls) · [Testing](#testing)

---

## Skills vs steps

You rarely touch a skill class directly. Mission helpers like
`takeoff()`, `fly_to()` and `land()` each build a skill and wrap it in a
**`SkillStep`**:

```python
step = fly_to(north=5, east=0, alt_m=3, name="leg_A")
step.skill      # FlyTrajectorySkill instance
step.name       # "leg_A" — shown in logs and the mission report
step.is_done    # None → the runtime reads step.skill.is_done
step.on_enter   # optional hook run just before the skill starts
```

A mission can yield either:

| You yield | What happens |
|---|---|
| A step from a helper: `yield fly_to(...)` | Normal case. |
| A bare skill: `yield HoverForSeconds(3)` | Wrapped in a `SkillStep` for you; the step name is the skill's `name`. |
| A custom step: `yield SkillStep(skill=..., is_done=..., name=...)` | When you need your own finish condition (see [patterns](#finish-on-your-own-condition)). |

---

## Lifecycle

```
mission yields a step
        │
        ▼
 SkillRunner.start(skill) ── cancels the previous skill ("superseded")
        │                   └─ emits SKILL_STARTED to the mission report
        ▼
 skill.start(ctx, params)  ── read initial state, schedule a setpoint
        │                     publisher on the CONTROL group
        ▼
 every program tick (5 Hz): is the step done?
        │   └─ step.is_done(ctx) if given, else skill.is_done
        │
        ├── no  → keep going (the publisher keeps sending setpoints)
        │
        └── yes → SKILL_COMPLETED to the report
                  mission generator resumes → next step
                  (the next start() cancels this skill)
```

Key facts:

- **One skill at a time.** Each mode has one `SkillRunner`; starting a
  new skill always cancels the old one first.
- **`cancel()` is synchronous.** It's called when the mission advances,
  is pre-empted (emergency stop, pause) or ends. Do all cleanup inside
  it — there is no later tick.
- **Done-ness is polled at 5 Hz** (`ControlMode.PROGRAM_STEP_HZ`). A
  skill that finishes between polls is noticed up to 0.2 s later.
- **The report records every step.** `SKILL_STARTED` /
  `SKILL_COMPLETED` (with `duration_s` and `outcome`) appear in
  `~/.ros/recordings/mission_raw_*.json` automatically — you don't
  report anything yourself.

---

## Built-in skills

| Helper | Skill class | Finishes when |
|---|---|---|
| `takeoff(alt_m=)` | `TakeoffSkill` | Altitude reached (within tolerance) or timeout |
| `fly_to(...)`, `fly_to_ned(...)` | `FlyTrajectorySkill` | Arrived and settled at the target |
| `orbit(...)` | `OrbitSkill` | `duration_s` elapsed |
| `helix(...)` | `HelixSkill` | `duration_s` elapsed |
| `yaw_to(north=, east=)` | `YawToSkill` | Heading within threshold of the target |
| `brake()` | `BrakeAndSettleSkill` | Horizontal speed below threshold, or timeout |
| `capture(...)` | `CaptureSkill` | A frame is saved, or `timeout_s` (5 s) passes |
| `land()` | `LandSkill` | Touchdown and disarm, or timeout |
| — | `FlyToGlobalSkill` | Arrived at a lat/lon target |

Full signatures: [catalog.md](../catalog.md#mission-steps).

---

## Writing a skill

The contract is four members:

```python
class HoverForSeconds:
    name = "hover_for_seconds"          # label for logs and the report

    def start(self, ctx, params=None):  # begin
        ...

    def cancel(self, ctx, reason):      # stop — synchronous, idempotent
        ...

    @property
    def is_done(self) -> bool:          # polled at 5 Hz
        ...
```

A complete, flight-tested version is in
[custom_skill_mission.py](../../examples/custom_skill_mission.py) and
explained line by line in the
[custom_skill walkthrough](../walkthroughs/custom_skill_mission.md).

### The setpoint loop

Almost every skill does the same thing in `start()`:

```python
def start(self, ctx, params=None):
    self._ctx = ctx
    self._t_start = ctx.world.now()
    p = ctx.senses.pose.current_position               # NED metres
    self._hold = (p.x, p.y, p.z, getattr(p, "heading", 0.0))

    ctx.world.publish_enable_to_fly(True)              # accept setpoints
    self._handle = ctx.scheduler.schedule(
        self._publish, hz=10.0, group=ScheduleGroup.CONTROL,
        name=self.name, now=ctx.world.now())
```

and undoes it in `cancel()`:

```python
def cancel(self, ctx, reason):
    if self._handle is not None:
        ctx.scheduler.unschedule(self._handle)
        self._handle = None
```

The publisher sends a full setpoint every tick:

```python
def _publish(self):
    x, y, z, yaw = self._hold
    self._ctx.world.publish_trajectory_setpoint(
        position=[x, y, z], velocity=[0.0, 0.0, 0.0],
        acceleration=[0.0, 0.0, 0.0], yaw=yaw, yaw_rate=0.0)
```

### Schedule groups

| Group | Use for | Why |
|---|---|---|
| `CONTROL` | Setpoint publishing | Realtime thread; nothing slow shares it |
| `DECISION` | Planning, anything that may take tens of ms | Separate from control so it can't delay setpoints |
| `MEDIA` | Services: frames, recording, logging | Never a skill's setpoints |
| `DEFAULT` | Unclassified callbacks | Its own thread |

Skills publish on **CONTROL**, at **10 Hz or more** — PX4 drops out of
OFFBOARD if setpoints stop.

---

## Patterns

### Finish on your own condition

Reuse a built-in skill's motion but decide yourself when it's done by
wrapping it in a `SkillStep`. Hover until something outside the flight
happens:

```python
def hover_until(condition, *, timeout_s, ctx, name):
    deadline = ctx.world.now() + timeout_s
    return SkillStep(
        skill=brake(name=name).skill,                 # holds position
        is_done=lambda c: condition(c) or c.world.now() >= deadline,
        name=name,
    )

yield hover_until(lambda c: spray.state, timeout_s=3, ctx=ctx, name="wait_valve")
```

Used in [spray_mission.py](../../examples/spray_mission.py) (wait for
the valve) and [detect_mission.py](../../examples/detect_mission.py)
(wait for the detector — see the
[detect walkthrough](../walkthroughs/detect_mission.md)). **Always add a
timeout** so a missing answer can't hold the drone forever.

### Relative motion

Compute the target from the pose in `start()`, not in `__init__()` —
the skill object is created when the mission *yields* it, but the pose
you want is the one when it *starts*. `ClimbBy` in the custom skill
example does this.

### Always yield the steps you mean

A step only runs if the mission yields it. `fly_to(...)` on its own line
without `yield` builds a step and throws it away.

---

## Pitfalls

| Mistake | What happens | Fix |
|---|---|---|
| Using `time.time()` | Tests can't control the clock; sim time and wall time drift | Use `ctx.world.now()` |
| Sleeping or blocking in `start()` / the publisher | Blocks the control thread; PX4 may leave OFFBOARD | Schedule work; never `time.sleep` |
| Forgetting `unschedule` in `cancel()` | The old publisher keeps sending setpoints and fights the next skill | Always unschedule; set the handle to `None` |
| Setpoints below ~2 Hz | PX4 offboard timeout | 10 Hz on `CONTROL` |
| `is_done` that can never be true | Mission hangs on that step | Add a timeout path |
| Wrong NED sign | `z` is **negative up**: climbing 3 m is `z - 3` | Use `alt_m` helpers where possible |
| `capture()` without a camera | Gives up after 5 s; see [known issues](../known-issues.md#a-camera-mission-on-a-drone-without-a-camera-can-stall) | Check the drone has a camera first |

---

## Testing

Skills are plain Python — test them with a fake context and a fake
clock, no drone needed:

```python
from skytrack_autonomy.core.lib.scheduling import ScheduleGroup
from skytrack_autonomy.core.testing import FakeClock, make_fake_ctx

clock = FakeClock()
ctx = make_fake_ctx(clock=clock)
skill = HoverForSeconds(2)
skill.start(ctx)

clock.advance(0.15)
ctx.scheduler.tick(ScheduleGroup.CONTROL, ctx.world.now())
assert len(ctx.world.setpoints) >= 1        # it published
assert not skill.is_done

clock.advance(2.0)
assert skill.is_done
skill.cancel(ctx, "done")
```

More in [tests/test_custom_parts.py](../../tests/test_custom_parts.py).
