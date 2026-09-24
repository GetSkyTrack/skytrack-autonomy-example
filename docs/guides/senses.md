# Senses

A **sense** is a read-only view of the world: where the drone is, how
much battery is left, what the camera sees, where the obstacles are.
Missions, skills and modes read senses; senses never command anything.

```python
pose = ctx.senses.pose.current_position     # NED metres, or None
if ctx.senses.battery.percent < 40: ...
frame = ctx.senses.camera.decode()          # HxWxC uint8 numpy array
```

**On this page:** [How senses work](#how-senses-work) ·
[Senses in the app](#senses-in-the-app) · [Reading senses
safely](#reading-senses-safely) · [Writing a sense](#writing-a-sense) ·
[requires_senses](#requires_senses) · [Pitfalls](#pitfalls) ·
[Testing](#testing)

---

## How senses work

```
autopilot / sensors ──► world adapter ──► world.on_<channel>(callback)
                                                   │
                                     sense._on_msg(msg): store it
                                                   │
         mission / skill / mode ──► ctx.senses.<name>.<property>
```

1. At boot, each sense's `attach(world)` subscribes to one or more
   channels on the world adapter.
2. Messages arrive on the adapter's threads; the sense's callback
   **stores** the latest message.
3. Anything with `ctx` reads the stored state through properties.

A sense holds **the latest value only**. It is not a queue: if you read
it twice without a new message in between, you get the same value.

---

## Senses in the app

The app's `boot_drone()` registers these (confirmed with
`drone.list_senses()` in simulation):

| `ctx.senses.` | What it gives | Key properties |
|---|---|---|
| `pose` | Local position, velocity, heading | `current_position` (`.x` north, `.y` east, `.z` down, `.heading` rad), `attitude`, `has_fix` |
| `status` | Autopilot state | `is_armed`, `mode_name`, `is_offboard_active`, `has_status` |
| `battery` | Battery | `percent` (**0–100**), `remaining` (0–1), `voltage_v`, `current_a`, `warning` |
| `global_position` | GPS | `lat`, `lon`, `alt`, `is_usable`, `eph`, `epv` |
| `camera` | Latest camera frame | `has_frame`, `seq`, `decode()` |
| `obstacle` | Voxel obstacle map the planner uses | (used by `fly_to`; rarely read directly) |
| `point_cloud` | Known-world obstacles (e.g. from the Gazebo world) | (feeds `obstacle`) |
| `landing_spot` | Safe landing pads found in the point cloud | `spots`, `nearest(...)` |

Add your own with `drone.add_sense(...)` in `main()`, **before**
`drone.fly()`.

---

## Reading senses safely

Every value can be missing: right after boot, before the first
message, or when the autopilot reports "unknown".

```python
pose = ctx.senses.pose.current_position
if pose is None:
    ctx.world.log_warn("no position fix yet")
else:
    alt_m = -pose.z                 # NED: z is negative up

percent = ctx.senses.battery.percent
if percent is not None and percent < 40:
    ...                             # only act on a *known* low battery
```

[battery_check_mission.py](../../examples/battery_check_mission.py)
shows the full pattern. In simulation it logged
`battery=99.0 … 95.4` over one flight.

### When is a value fresh?

Missions read senses **between** steps — after a `yield` returns, the
step has finished and the pose is the pose at that moment. Values don't
change inside a single line of mission code, but can change between
any two `yield`s.

---

## Writing a sense

The contract:

```python
class GeofenceSense:
    name = "geofence"                     # → ctx.senses.geofence

    def attach(self, world):              # called by drone.add_sense()
        world.on_local_position(self._on_pose)

    def _on_pose(self, msg):              # runs on the adapter's thread
        self._pose = msg                  # store; keep it cheap

    @property
    def is_inside(self) -> bool:          # what missions read
        ...
```

| Member | Called | Job |
|---|---|---|
| `name` | — | Registry key. Must be unique. |
| `attach(world)` | Once, by `drone.add_sense()` | Subscribe to channels. |
| callbacks | On every message, on the adapter's thread | Store the message (and cheap counters). |
| properties / methods | Any time, from any thread | Read-only answers. Handle "no data yet". |

### Channels you can subscribe to

| Method | Message fields |
|---|---|
| `world.on_local_position(cb)` | `x`, `y`, `z` (NED m), `vx`, `vy`, `vz`, `heading` |
| `world.on_vehicle_status(cb)` | `is_armed`, `mode_name`, `is_offboard_active`, … |
| `world.on_battery_status(cb)` | `remaining` (0–1), `voltage_v`, … |
| `world.on_global_position(cb)` | `lat`, `lon`, `alt`, … |
| `world.on_attitude(cb)` | Orientation quaternion |
| `world.on_camera(cb)` | Image message (ROS adapter only) |

> [!NOTE]
> Callbacks receive the **raw** message in the autopilot's local frame.
> The built-in `pose` sense additionally passes it through a frame
> adapter into the mission frame. In the simulation runs the two
> agreed (the geofence distances matched the mission waypoints), but if
> your deployment configures a frame offset, derive positions from
> `ctx.senses.pose.current_position` rather than the raw message.

The complete, flight-tested example is
[custom_sense_mission.py](../../examples/custom_sense_mission.py). In
simulation its `GeofenceSense` let the mission fly waypoints 6.0 m and
7.0 m from home and correctly skipped two outside the 10 m fence.

### Derived answers: methods vs properties

- **Properties** answer "what is true now?" from stored data:
  `fence.is_inside`, `fence.distance_from_home_m`.
- **Methods** answer "what if?" questions from their arguments, without
  needing a message: `fence.allows(north, east, alt_m)`. These let a
  mission decide *before* flying anywhere.

---

## `requires_senses`

Every mission and mode declares the senses it needs:

```python
my_mission.requires_senses = ["pose", "obstacle", "status", "geofence"]
```

The framework checks the list when the mission is registered and
**refuses to start** (a `RuntimeError` listing each missing sense) if
one isn't there. It turns "crashes mid-flight on
`ctx.senses.geofence`" into "fails on the ground".

| Always include | When |
|---|---|
| `"pose"` | Every mission |
| `"status"` | Every mission that takes off or lands |
| `"obstacle"` | Any `fly_to` |
| `"camera"` | `capture`, `VideoRecorder`, `Snapshot`, `Detector` |
| `"battery"` | If you read it |
| your custom names | If you read them |

---

## Pitfalls

| Mistake | What happens | Fix |
|---|---|---|
| Heavy work in the callback (image processing, path planning) | Blocks the adapter's thread; other messages queue up | Store the message; compute lazily in the property (as `CameraSense.decode()` does) |
| Formatting a value that can be `None` (`f"{x:.1f}"`) | `TypeError` mid-mission | Check for `None` first — the tests in this repo caught exactly this in `custom_sense_mission` |
| Treating `battery.percent` as 0–1 | `percent < 0.4` never fires | It's 0–100; `remaining` is 0–1 |
| Adding a sense after `drone.fly()` | `requires_senses` check fails | Add senses first |
| Forgetting the sign of `z` | Altitude reads negative | `alt_m = -pose.z` |
| Publishing or commanding from a sense | Breaks the "only skills move the drone" rule | Move that logic to a skill or service |

---

## Testing

Attach the sense to a fake world and push messages into it:

```python
from types import SimpleNamespace
from skytrack_autonomy.core.testing import make_fake_ctx

fence = GeofenceSense(radius_m=10, max_alt_m=5)
ctx = make_fake_ctx()
fence.attach(ctx.world)

ctx.world.push_local_position(SimpleNamespace(x=3, y=4, z=-2, vx=0, vy=0, vz=0, heading=0.0))
assert fence.distance_from_home_m == 5 and fence.is_inside
```

To test a *mission* that reads your sense, pass a real or fake one in:
`make_fake_ctx(senses={"geofence": fence})`. See
[tests/test_custom_parts.py](../../tests/test_custom_parts.py).
