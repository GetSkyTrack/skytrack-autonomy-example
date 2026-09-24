# Walkthrough: `custom_skill_mission.py`

**Level 5 · Build your own** — [source](../../examples/custom_skill_mission.py)

Two new motion primitives, written from scratch and flown in a mission:

- `HoverForSeconds(duration_s)` — hold the current position for a time.
- `ClimbBy(delta_m)` — go straight up (or down) by a distance.

They're deliberately simple, so the skill **contract** is easy to see.
Background: [Skills guide](../guides/skills.md).

---

## 1. The contract, in one class

```python
class HoverForSeconds:
    name = "hover_for_seconds"
    PUBLISH_HZ = 10.0               # setpoints must keep flowing

    def __init__(self, duration_s: float) -> None:
        self._duration_s = float(duration_s)
        self._ctx = None
        self._handle = None
        self._t_start = None
        self._hold = None
```

`__init__` only stores **parameters**. It runs when the mission
*yields* the skill — at that moment the drone may still be somewhere
else, so nothing about the flight is read here.

## 2. `start()` — capture state, begin publishing

```python
def start(self, ctx, params=None):
    self._ctx = ctx
    self._t_start = ctx.world.now()          # never time.time()
    p = ctx.senses.pose.current_position
    self._hold = (p.x, p.y, p.z, getattr(p, "heading", 0.0))
    ctx.world.publish_enable_to_fly(True)
    self._handle = ctx.scheduler.schedule(
        self._publish, hz=self.PUBLISH_HZ, group=ScheduleGroup.CONTROL,
        name=self.name, now=ctx.world.now())
    ctx.world.log_info(f"[HOVER] holding for {self._duration_s:.0f}s")
```

| Line | Why |
|---|---|
| `ctx.world.now()` | The framework's clock — sim time in simulation, fake time in tests. |
| Read the pose **here** | The position to hold is where the drone is when the step *starts*. |
| `getattr(p, "heading", 0.0)` | Keep the current heading so the drone doesn't spin. |
| `publish_enable_to_fly(True)` | Tell the planner stack to follow our setpoints. |
| `schedule(..., group=CONTROL, hz=10)` | The realtime thread; PX4 needs a steady stream. |
| Keep `self._handle` | So `cancel()` can stop exactly this publisher. |

## 3. `cancel()` — stop, synchronously, idempotently

```python
def cancel(self, ctx, reason):
    if self._handle is not None:
        ctx.scheduler.unschedule(self._handle)
        self._handle = None
```

Called when the mission moves on, and also when the step is
pre-empted (emergency stop, pause). The `if … is not None` guard makes
a second call harmless — the unit test calls it twice on purpose.

## 4. `is_done` — the finish condition

```python
@property
def is_done(self) -> bool:
    if self._t_start is None:
        return False
    return self._ctx.world.now() - self._t_start >= self._duration_s
```

Polled at 5 Hz. It must be cheap and must never raise before
`start()` — hence the `None` check.

## 5. `ClimbBy` — relative motion

`ClimbBy` has the same shape; the differences are in `start()` and
`is_done`:

```python
def start(self, ctx, params=None):
    ...
    p = ctx.senses.pose.current_position
    # NED: up is negative z, so climbing subtracts.
    self._target = (p.x, p.y, p.z - self._delta_m, getattr(p, "heading", 0.0))
    ...

@property
def is_done(self) -> bool:
    ...
    p = self._ctx.senses.pose.current_position
    return p is not None and abs(p.z - self._target[2]) <= self._tolerance_m
```

- **Relative to where it starts**, so `ClimbBy(3)` means "3 m above
  wherever you are", computed at start time.
- **Done by position, not time** — within `tolerance_m` (0.2 m) of the
  target altitude.
- **The NED sign**: `z` is down, so climbing *subtracts*. This is the
  most common bug in hand-written skills.

## 6. Using them

```python
def custom_skill_mission(ctx):
    yield takeoff(alt_m=ALT_M)
    yield fly_to(north=5, east=0, alt_m=ALT_M, name="to_lookout")

    yield HoverForSeconds(3)                  # ← custom skill
    yield ClimbBy(3.0)                        # ← custom skill
    yield HoverForSeconds(5)
    yield ClimbBy(-3.0)

    yield fly_to(north=0, east=0, alt_m=ALT_M, name="return_home")
    yield brake(name="pre_land")
    yield land()
```

A bare skill can be yielded directly — the runtime wraps it in a
`SkillStep` named after the skill's `name`.

---

## What happened in simulation

**Succeeded**, 9 steps. The report's step timings against what was asked:

| Step | Asked | Measured | Why the difference |
|---|---|---|---|
| `hover_for_seconds` | 3 s | 3.0 s | — |
| `climb_by` | +3 m | 3.6 s, reached 6.0 m | Time to climb 3 m |
| `hover_for_seconds` | 5 s | 5.3 s | `is_done` is checked at 5 Hz, so up to 0.2 s late, plus the step transition |
| `climb_by` | −3 m | 3.5 s, reached 2.8 m | Started from ~5.8 m (inside the 0.2 m tolerance of 6.0), so −3 m lands at ~2.8 m |

Log lines, in order:

```
[HOVER] holding for 3s
[CLIMB] to 6.0 m
[HOVER] holding for 5s
[CLIMB] to 2.8 m
```

**Lessons from the numbers:**

- Time-based skills are accurate to about **0.2–0.3 s**. Don't use them
  where tenths of a second matter.
- Relative moves **accumulate error**: each `ClimbBy` starts from
  wherever the last one stopped *within tolerance*. For an exact
  altitude, write an absolute skill (`ClimbTo(alt_m)`) or use
  `fly_to(..., alt_m=...)`.

---

## Try it yourself

1. Write `ClimbTo(alt_m)`: same as `ClimbBy`, but the target is
   `-alt_m` instead of `p.z - delta_m`. Replace the second `ClimbBy` and
   check it lands exactly back at 3 m.
2. Give `HoverForSeconds` a slow yaw: add `yaw_rate_deg_s` and increase
   the yaw in `_publish()` each tick.
3. Unit-test your new skill the way `test_climb_by_skill_targets_new_altitude`
   does in [tests/test_custom_parts.py](../../tests/test_custom_parts.py).

**Next:** [site_survey_mission](site_survey_mission.md) — putting it all
together.
