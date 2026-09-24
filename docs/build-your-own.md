# Build your own skills, senses, services, modes and worlds

> This page is the one-page summary. Each building block also has a
> full guide: [skills](guides/skills.md) · [senses](guides/senses.md) ·
> [services](guides/services.md) · [modes](guides/modes.md).

`skytrack-autonomy` is built from five kinds of part. When the built-in
ones ([catalog.md](catalog.md)) don't cover what you need, write your
own. This guide gives the contract for each part and links to a working
example.

| You want to… | Write a | Example |
|---|---|---|
| Move the drone in a new way | [Skill](#skill) | [custom_skill_mission.py](../examples/custom_skill_mission.py) |
| Read or derive new state | [Sense](#sense) | [custom_sense_mission.py](../examples/custom_sense_mission.py) |
| Run something in the background (log, record, drive a payload) | [Service](#service) | [custom_service_mission.py](../examples/custom_service_mission.py) |
| Add a behaviour the operator can trigger at any time | [Mode](#mode) | [custom_mode_mission.py](../examples/custom_mode_mission.py) |
| Connect a new autopilot, simulator or sensor | [World adapter](#world-adapter) | Framework template: `skytrack_autonomy/examples/wire_with_px4.py` |

## Where custom parts live

Define custom parts in the mission file that uses them, as the Level 5
examples do, and register them in `main()` **before** `drone.fly()`:

```python
def main() -> None:
    with boot_drone() as drone:
        drone.add_sense(GeofenceSense(radius_m=25, max_alt_m=15))  # senses first
        drone.add_service(TelemetryLogger(hz=5.0))                 # then services
        drone.add_mode(InspectMode, gate="inspect_requested")      # modes
        drone.fly(my_mission)                                      # then the mission
        drone.run()
```

To reuse a part in another mission, import it from its file, as
[site_survey_mission.py](../examples/site_survey_mission.py) does:
`from .custom_sense_mission import GeofenceSense`.

## Which part do I need?

```
Does it move the drone?
├── yes → Is it one step in a sequence?        → Skill
│         Should it start on its own trigger?  → Mode (its program yields skills)
└── no  → Does it only observe?                → Sense
          Does it do work / have side effects? → Service
          Does it talk to hardware / a sim?    → World adapter
```

The golden rule: **only skills publish setpoints.** Senses only read,
services never move the drone, and modes move it only through the
skills they yield.

---

## Skill

A skill is one time-limited motion: it publishes setpoints while it
runs and reports when it has finished.

```python
class HoverForSeconds:
    name = "hover_for_seconds"

    def start(self, ctx, params=None): ...   # begin; schedule a publisher
    def cancel(self, ctx, reason): ...       # stop; unschedule it

    @property
    def is_done(self) -> bool: ...           # polled every tick
```

| Member | Called | Your job |
|---|---|---|
| `name` | — | Label for logs and reports. |
| `start(ctx, params)` | Once, when the step begins | Read the starting state, call `ctx.world.publish_enable_to_fly(True)`, and register a setpoint publisher with `ctx.scheduler.schedule(fn, hz=10, group=ScheduleGroup.CONTROL, ...)`. |
| `cancel(ctx, reason)` | When the step is interrupted or replaced | `ctx.scheduler.unschedule(handle)`. Must be safe to call more than once. |
| `is_done` | Every tick | Return `True` when finished. |

**Use it:** yield an instance directly from a mission or mode:
`yield HoverForSeconds(3)`. To control the step's label or finish
condition, wrap it: `yield SkillStep(skill=..., name="...", is_done=lambda ctx: ...)`.

**Rules**
- Publish with `ctx.world.publish_trajectory_setpoint(position=, velocity=, acceleration=, yaw=, yaw_rate=)` in NED metres, `z` negative up.
- Use the `CONTROL` schedule group for setpoints, at 10 Hz or more.
- Use `ctx.world.now()` for time, never `time.time()`, so tests can drive the clock with `FakeClock`.
- Never block (`time.sleep`) inside a skill.

---

## Sense

A sense subscribes to data from the world and exposes read-only state.

```python
class GeofenceSense:
    name = "geofence"                        # → ctx.senses.geofence

    def attach(self, world):
        world.on_local_position(self._on_pose)

    def _on_pose(self, msg): ...             # store, don't compute heavily

    @property
    def is_inside(self) -> bool: ...          # what missions read
```

| Member | Called | Your job |
|---|---|---|
| `name` | — | Registry key: missions read `ctx.senses.<name>`. |
| `attach(world)` | Once, by `drone.add_sense(sense)` | Subscribe: `world.on_local_position`, `on_vehicle_status`, `on_battery_status`, `on_global_position`, `on_attitude`, or a custom channel your world adds (e.g. `on_camera`). |
| properties | Any time | Read-only views of the latest state. Return `None` or a safe default before the first message. |

**Use it:** `drone.add_sense(GeofenceSense(...))`, and list it in the
mission's `requires_senses` so boot fails loudly if it's missing.

**Rules**
- Senses never publish or command anything.
- Keep callbacks cheap. Store the message and do the work lazily in the
  property (see how `CameraSense.decode()` works).
- For tests, write a matching fake and pass it to
  `make_fake_ctx(senses={"geofence": FakeGeofence()})`.

---

## Service

A service runs alongside the whole mission. It can write files, emit
telemetry or drive a payload, but **never publishes a setpoint**.

```python
class TelemetryLogger:
    name = "telemetry"                       # → ctx.services.telemetry

    def attach(self, ctx):                   # wire up; may start work
        self._handle = ctx.scheduler.schedule(
            self._sample, hz=5, group=ScheduleGroup.MEDIA,
            name=self.name, now=ctx.world.now())

    def mark(self, note): ...                # your own mission-facing API

    def shutdown(self): ...                  # flush + release; idempotent
```

| Member | Called | Your job |
|---|---|---|
| `name` | — | Registry key: missions use `ctx.services.<name>`. |
| `attach(ctx)` | Once, by `drone.add_service(service)` | Keep `ctx`; schedule periodic work on `ScheduleGroup.MEDIA`. Start work now if it runs for the whole mission; otherwise wait for your own `start()`. |
| `shutdown()` | Once at teardown (`drone.run()` calls it) | Unschedule, flush and close. **Must be idempotent.** |
| `mission_ended()` | Optional, when the mission program completes | Final bookkeeping before the report closes. |
| your methods | By the mission | E.g. `start()`, `stop()`, `on()`, `off()`, `mark()`. Return `bool` for "accepted or refused" rather than raising. |

**Rules**
- Never call `publish_trajectory_setpoint`.
- Use the `MEDIA` group (not `CONTROL`) so slow work can't delay flight control.
- Guard against being used before `attach()`.

---

## Mode

A mode is a program the Supervisor runs when its **gate** opens and no
higher-priority mode wants control. `drone.fly(mission)` wraps your
mission in a mode automatically. Write a mode yourself when the
behaviour must be triggerable at any time, like "inspect", "return
home" or "hold here".

```python
class QuickInspectMode(ControlMode):
    requires_senses = ["pose", "status"]

    @property
    def id(self) -> str:
        return "quick_inspect"

    def program(self, ctx):                  # yields steps, like a mission
        yield takeoff(alt_m=3)
        yield orbit(center_north=3, center_east=0, alt_m=3, radius_m=3)
        yield land()
```

Register and trigger it:

```python
drone.add_mode(QuickInspectMode, gate="inspect_requested",
               level=Level.MISSION, on_done=OnDone.LAND_AND_HOLD)

drone.ctx.flags.inspect_requested = True     # open the gate…
drone.ctx.notify_state_change()              # …and wake the arbiter
```

| Member | Required | Purpose |
|---|---|---|
| `id` | yes | Unique identifier, shown in logs. |
| `program(ctx)` | usually | Generator of steps. Omit it for a mode that only has entry side effects. |
| `on_setup(supervisor)` | no | Runs once on entry, before `program`. Use `self._ctx` for the context. Note the argument is the supervisor, not `ctx`. |
| `on_teardown()` | no | Runs once on exit, after the program is cancelled. |
| `on_complete()` | no | Runs once when the program finishes on its own. |
| `requires_senses` | no | Senses checked at registration. |

`add_mode` options:

| Option | Meaning |
|---|---|
| `gate="name"` | Boolean on `ctx.flags` that makes this mode want control. Created as `False`. |
| `level=` | `Level.MISSION` (default) or `Level.BACKGROUND`. Other tiers are reserved for built-in safety modes. |
| `beats=` / `loses_to=` | Order relative to another registered mode class instead of a level. |
| `gate_params="name"` | A `ctx` attribute holding parameters for the mode. Cleared with the gate when the program finishes. |
| `on_done=` | `OnDone.IDLE` (default): hover and wait. `OnDone.LAND_AND_HOLD`: stay on the ground until the next command. |

> [!IMPORTANT]
> Setting a flag alone is not enough when the drone is idle, because
> nothing is polling the flags. Always call
> `ctx.notify_state_change()` after changing a gate from outside a
> running mode.

---

## World adapter

The world adapter is the only code that touches the outside world.
Subclass `BaseWorld` (or extend an existing adapter) and implement:

**Outputs:** the framework calls these.

| Method | Purpose |
|---|---|
| `publish_trajectory_setpoint(*, position, velocity, acceleration, yaw, yaw_rate)` | Send a position/velocity target (NED). |
| `send_vehicle_command(*, command, params=())` | Arm, takeoff, land, etc. (MAVLink command IDs). |
| `engage_external_control()` | Enter offboard / guided mode. |
| `engage_hold()` | Hold position. |
| `publish_enable_to_fly(enable)` | Enable or disable setpoint following. |
| `set_spray(on, *, done=None)`, `run_detection(request, *, on_stage=None, done=None)` | Optional payload outputs, needed by `Sprayer` and `Detector`. |

**Inputs:** call `self._emit(channel, msg)` when data arrives:

| Channel | Message fields |
|---|---|
| `"pose"` | `x, y, z` (NED m), `vx, vy, vz`, `heading` (rad) |
| `"vehicle_status"` | `is_armed, mode_name, is_offboard_active, is_auto_locked, is_auto_land` |
| `"battery"`, `"global_position"`, `"attitude"` | See the matching sense |

For a sensor the base class doesn't know about, add your own
`on_<thing>(callback)` method and call the callbacks yourself. That is
how the ROS 2 adapter provides `on_camera` for `CameraSense`.

**Check it:** `assert_world_conformance` from `skytrack_autonomy.core.conformance`
runs the adapter contract tests against your class.

---

## Testing what you build

- **Missions:** step through the generator with `make_fake_ctx()`. No
  drone or ROS is needed. See [tests/test_examples.py](../tests/test_examples.py).
- **Skills, senses, services:** drive time with `FakeClock` and tick the
  scheduler by hand, e.g.
  `ctx.scheduler.tick(ScheduleGroup.CONTROL, ctx.world.now())`.
  See [tests/test_custom_parts.py](../tests/test_custom_parts.py).
- **In the app:** run the mission in SITL before real hardware.
