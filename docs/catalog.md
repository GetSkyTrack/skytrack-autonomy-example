# Catalog: what ships in the box

A reference of every building block `skytrack-autonomy` ships today:
**mission steps, skills, senses, services, modes, commands and world
adapters**. Use it to find what's already available before writing
your own.

New to the framework? Work through the examples in order, starting with
[hello_mission.py](../examples/hello_mission.py).
Want to build something that isn't listed here? See
[build-your-own.md](build-your-own.md). Known gaps are in
[known-issues.md](known-issues.md).

| Building block | What it is | Where you use it |
|---|---|---|
| [Mission steps](#mission-steps) | One-line helpers you `yield` from a mission | Inside `def mission(ctx):` |
| [Skills](#skills) | The motion primitives behind the steps | Yield directly, or build your own |
| [Senses](#senses) | Read-only views of vehicle and world state | `ctx.senses.<name>` |
| [Services](#services) | Background activities that never move the drone | `ctx.services.<name>` |
| [Modes](#modes) | Control programs the Supervisor arbitrates between | `drone.fly(...)`, `drone.add_mode(...)` |
| [Commands](#commands) | Operator inputs that switch modes | `drone.submit(...)` |
| [World adapters](#world-adapters) | The link to a simulator or real autopilot | `boot_drone_for_world(world)` |

**Coordinates:** the friendly form uses `north` / `east` in metres and
`alt_m` positive-up above home. The NED form (`x`, `y`, `z`) uses `z`
negative-up.

---

## Mission steps

Import from `skytrack_autonomy`. Each returns a step you `yield`.

```python
from skytrack_autonomy import takeoff, fly_to, orbit, capture, land

def survey(ctx):
    yield takeoff(alt_m=5)
    yield fly_to(north=20, east=0, alt_m=5, mode="coverage", target_speed=2.0)
    yield orbit(center_north=20, center_east=0, alt_m=5, radius_m=4, duration_s=30)
    yield capture(filename="site.png")
    yield land()
```

| Step | Signature (key arguments) | What it does |
|---|---|---|
| `takeoff` | `takeoff(*, alt_m=3.0)` | Arm and climb to `alt_m`. Must be the first step. |
| `fly_to` | `fly_to(north=, east=, alt_m=, *, mode=, replan_mode=, target_speed=, yaw_mode=, yaw_rate_deg_s=)` | Fly to a point through the obstacle-aware planner. See [fly_to options](#fly_to-options). |
| `fly_to_ned` | `fly_to_ned(x, y, z, *, ...)` | Same as `fly_to`, with NED coordinates (e.g. a position from a sense). |
| `orbit` | `orbit(center_north=, center_east=, alt_m=, *, radius_m=5.0, period_s=20.0, duration_s=60.0)` | Circle a point at constant radius and altitude. |
| `helix` | `helix(center_north=, center_east=, alt_m=, alt_m_end=, *, radius_m=5.0, period_s=20.0, duration_s=60.0)` | Circle a point while ramping altitude from `alt_m` to `alt_m_end`. |
| `yaw_to` | `yaw_to(north=, east=)` | Rotate in place to face a point. Needs a world that reports heading (the stock `SimulatedWorld` doesn't). |
| `brake` / `brake_and_settle` | `brake()` | Hold the current pose until the drone has stopped. |
| `capture` | `capture(*, output_dir="~/.ros/captures", filename=None, timeout_s=5.0)` | Hover and save one camera frame. Needs `CameraSense` and a world with a camera. |
| `land` | `land()` | Land at the current position. |

Every step also accepts `name=` as a label for logs.

> [!NOTE]
> `fly_to` works in the app (`boot_drone()`), which registers a real
> obstacle sense. With `boot_drone_for_world` it currently fails; see
> [known-issues.md](known-issues.md#fly_to-fails).

### fly_to options

| Option | Values | Default |
|---|---|---|
| `mode` | `"transit"`: shortest path around obstacles<br>`"coverage"`: hug the straight line, deviate only when blocked (spraying, scanning, inspection)<br>`"direct"`: no planner, straight line with an obstacle-cone brake | `"transit"` |
| `replan_mode` | `"fast"`: replan in flight without stopping<br>`"slow"`: stop for every replan | from config |
| `target_speed` | Cruise speed in m/s, for this leg only | from config (5.0) |
| `yaw_mode` | `"course"`: face along the path, rotate at sharp corners<br>`"hold"`: never command heading | `"course"` |
| `yaw_rate_deg_s` | Maximum turn rate for this leg (> 0) | from config (45) |

### Helpers for no-fly zones

From `skytrack_autonomy.contrib.nfz_flow`:

| Function | What it does |
|---|---|
| `load_zones(nfz_file)` | Load zones from an NFZ file. Returns `[]` if there is none. |
| `split_nfz(waypoints, zones, alt_m, margin_m=...)` | Split a route where it crosses a zone. |

---

## Skills

Skills are the motion primitives the steps above are built on. Most
users only need the steps. Use a skill class directly when you need
options the step doesn't expose, or as a starting point for your own
([guide](build-your-own.md#skill)).

Import from `skytrack_autonomy.contrib.skills`.

| Skill | Used by step | What it does |
|---|---|---|
| `TakeoffSkill(altitude_m=...)` | `takeoff` | Arm, command takeoff, finish when altitude is reached. |
| `FlyTrajectorySkill(target, ...)` | `fly_to`, `fly_to_ned` | Obstacle-aware path planning and trajectory following with continuous replanning. |
| `FlyToGlobalSkill(lat, lon, alt_m=None)` | — | Fly to a GPS (lat/lon) target. Needs `GlobalPositionSense`. |
| `OrbitSkill(params=...)` | `orbit` | Circular orbit for a set duration. |
| `HelixSkill(params=...)` | `helix` | Orbit with an altitude ramp. |
| `YawToSkill(target_yaw_provider)` | `yaw_to` | Rotate in place while holding position. |
| `BrakeAndSettleSkill(timeout_s=..., speed_threshold=None)` | `brake` | Hold pose until stopped or timed out. |
| `CaptureSkill(output_dir=..., filename=None)` | `capture` | Hover until a frame arrives, save it. `saved_path` holds the result. |
| `LandSkill(timeout_s=...)` | `land` | Command landing, finish on touchdown. |

A skill instance can be yielded directly: `yield HoverForSeconds(5)`.
Example: [custom_skill_mission.py](../examples/custom_skill_mission.py).

---

## Senses

Senses give missions read-only access to state. Read them through
`ctx.senses.<name>`:

```python
def mission(ctx):
    if ctx.senses.battery.percent < 30:
        ctx.world.log_warn("Battery low, landing")
        yield land()
        return
```

Import from `skytrack_autonomy.contrib.senses`. Add optional senses
with `drone.add_sense(...)`.

### Always available

`boot_drone_for_world` registers these automatically.

| Sense | `ctx.senses.` | Key properties |
|---|---|---|
| `PoseSense` | `pose` | `current_position`, `attitude`, `has_fix` |
| `StatusSense` | `status` | `is_armed`, `is_offboard_active`, `mode_name`, `has_status` |
| `BatterySense` | `battery` | `percent`, `remaining`, `voltage_v`, `current_a`, `temperature_c`, `warning` |

### Optional

| Sense | `ctx.senses.` | What it provides |
|---|---|---|
| `CameraSense()` | `camera` | Latest camera frame: `has_frame`, `decode()` → `HxWxC` uint8 image. Needed by `capture`, `VideoRecorder`, `Snapshot`. **The app's `boot_drone()` already registers it**; calling `drone.add_sense(CameraSense())` again is harmless. With `boot_drone_for_world` you must add it, and the world must provide `on_camera` (the in-process `SimulatedWorld` doesn't). |
| `GlobalPositionSense()` | `global_position` | GPS fix: `lat`, `lon`, `alt`, `is_usable`, `eph` / `epv` accuracy. Needed for lat/lon flight. |
| `DepthCameraSense(logger=...)` | `depth_camera` | Depth-camera point cloud in world coordinates. Defaults tuned for OAK-D Lite. |
| `PointCloudSense(logger=...)` | `point_cloud` | Known-world obstacles, e.g. loaded from a Gazebo SDF world. |
| `ObstacleSense(logger=..., sources=[...])` | `obstacle` | The voxel obstacle map `fly_to` plans around. Combines point cloud, depth camera and no-fly-zone sources. |
| `NoFlyZoneSense(logger=..., nfz_file=...)` | `no_fly_zone` | No-fly zones from a file, fed to the planner as obstacles. |
| `LandingSpotSense(logger=..., point_cloud=...)` | `landing_spot` | Flat, safe landing pads found in the point cloud: `spots`, `nearest(...)`. Used by smart landing. |

Examples: [battery_check_mission.py](../examples/battery_check_mission.py),
[custom_sense_mission.py](../examples/custom_sense_mission.py).

---

## Services

Services run in the background alongside the mission. They can write
files or drive payloads but **never publish a setpoint**, so they can't
interfere with flight. Register with `drone.add_service(...)` and use
them via `ctx.services.<name>`.

Import from `skytrack_autonomy`.

| Service | `ctx.services.` | Mission API | What it does |
|---|---|---|---|
| `VideoRecorder(fps=10.0, sink=None)` | `recorder` | `start(clip=...)`, `stop()`, `is_recording` | Records the camera to video. Needs `CameraSense`. |
| `Snapshot()` | `snapshot` | `snap(filename=None)` → path or `None` | Saves one still photo without stopping the drone (unlike `capture`). |
| `Detector(model_name=..., classes=[...])` | `detector` | `request(...)` → `bool`, `is_busy`, `last_result`, `cancel()` | Runs one object detection at a time on the world's detector. The answer arrives later (about 1–1.5 s in simulation); wait for `not is_busy` before reading `last_result`. Verified with `model_name="det-coco-v26n-b-quantized-fp16"`, `classes=["human"]`. |
| `Sprayer()` | `sprayer` | `on()`, `off()` → `bool`, `state`, `is_settled` | Opens and closes the spray valve. |

```python
from skytrack_autonomy import Sprayer

def spray_row(ctx):
    yield takeoff(alt_m=3)
    ctx.services.sprayer.on()
    yield fly_to(north=30, east=0, alt_m=3, mode="coverage")
    ctx.services.sprayer.off()
    yield land()

drone.add_service(Sprayer())
```

### Recording sinks

`VideoRecorder(sink=...)` selects the output format:

| Sink | Output | Needs |
|---|---|---|
| `PngFramesSink` (default; `skytrack_autonomy.contrib.services.sinks`) | Numbered PNG frames + `meta.jsonl` with pose and timestamp | Nothing extra |
| `GStreamerSink(output_dir=..., bitrate_kbps=4000)` | One H.264 `.mp4` | GStreamer plugins `appsrc`, `x264enc`, `mp4mux`. **Not installed in the simulation container**: there it records 0 frames and logs an error per frame. Prefer the default sink unless you know the plugins are present. |

Write your own by implementing the `FrameSink` protocol (`open`, `write`, `close`).

Examples: [record_video_mission.py](../examples/record_video_mission.py),
[spray_mission.py](../examples/spray_mission.py),
[detect_mission.py](../examples/detect_mission.py),
[custom_service_mission.py](../examples/custom_service_mission.py).

---

## Modes

A mode is a control program. The **Supervisor** runs the
highest-priority mode that wants control and preempts lower-priority
ones. Priority tiers, highest first:

| Tier | Built-in modes |
|---|---|
| `Level.SAFETY` | `EmergencyStopMode`, `CanceledMode`, `PausedMode` |
| `Level.PRELAUNCH` | `TakeoffMode` |
| `Level.LANDING` | `SmartLandMode` (settle → pick a safe spot → fly → land) |
| `Level.MISSION` | `OrbitMode`, `HelixMode`, `PanoramaMode`, `GoToGlobalMode`, **your missions and modes** |
| `Level.NAVIGATE` | `NavigateMode` (fly to the current target) |
| `Level.BACKGROUND` | — (for your low-priority helpers) |
| `Level.FALLBACK` | `IdleMode` (do nothing) |

> [!IMPORTANT]
> **What is registered depends on how you boot.** The app's
> `boot_drone()`, which all these examples use, registers the full
> built-in set above. `boot_drone_for_world(world)` (pymavlink, SITL,
> in-process simulator) registers **only `IdleMode`**, plus whatever you
> add with `drone.fly(...)` and `drone.add_mode(...)`. Your own modes may
> use `Level.MISSION` or `Level.BACKGROUND`.

Example: [custom_mode_mission.py](../examples/custom_mode_mission.py). Guide:
[build-your-own.md](build-your-own.md#mode).

---

## Commands

Commands are operator inputs, e.g. from a ground station or chat
interface. Send them with `drone.submit(cmd)`. Import from
`skytrack_autonomy.core.commands`.

> [!WARNING]
> A command only has an effect if the mode it targets is registered. With
> `boot_drone_for_world` that is not the case for any built-in mode, so
> **`EmergencyStop`, `Pause` and `Cancel` do not stop a running
> `drone.fly` mission**. See [known-issues.md](known-issues.md#safety-commands-dont-stop-a-mission).
> Use your autopilot's own failsafes and RC override.

| Command | Fields | Effect |
|---|---|---|
| `EmergencyStop` | `active`, `reason=""` | Hard stop; clears the mission. |
| `Cancel` | — | Cancel the current mission. |
| `Pause` | `active` | Pause / resume. |
| `TakeoffRequest` | `active`, `altitude_m=3.0` | Take off. |
| `SmartLandRequest` | `active` | Land on a safe detected spot. |
| `FlyTo` | `target`, `params=None` | Set a new navigation target. |
| `OrbitRequest` | `active`, `center`, `radius_m=5.0`, `period_s=20.0`, `duration_s=60.0` | Orbit a point. |
| `HelixRequest` | same as orbit + `z_end` | Helix around a point. |
| `PanoramaRequest` | `active`, `num_captures=8`, `total_angle_deg=360.0`, `dwell_s=0.5`, `return_to_start=True` | Rotate in place taking photos. |
| `GoToGlobalRequest` | `active`, `lat`, `lon`, `alt_m`, `direct=False` | Fly to a GPS target (needs `GoToGlobalMode`). |


---

## World adapters

A world adapter connects the framework to a simulator or an autopilot.
Import from `skytrack_autonomy.contrib.world.adapters`.

| Adapter | Install | Use for |
|---|---|---|
| `SimulatedWorld(climb_mps=1.5, horizontal_mps=4.0)` | core | Demos, unit tests, CI. No external dependencies. |
| `PymavlinkWorld("udp:0.0.0.0:14550")` | `[pymavlink]` | Real PX4 / ArduPilot, or anything that speaks MAVLink. |
| `SitlWorld()` | `[sitl]` | PX4 SITL with preset connection settings. |
| ROS 2 adapter (the app, via `boot_drone()`) | — | The production deployment. All the examples here use it. |
| Your own | — | Subclass `BaseWorld`; see the framework's `examples/wire_with_px4.py`. |

Supported autopilots: **PX4** and **ArduPilot**. Flight-mode decoding
for both lives in `skytrack_autonomy.contrib.autopilot`.

---

## See also

- [build-your-own.md](build-your-own.md): write your own skills, senses, services, modes and worlds
- [known-issues.md](known-issues.md): current limitations
- Framework docs (the `docs/` folder of `skytrack-autonomy`): architecture, testing, mission patterns
