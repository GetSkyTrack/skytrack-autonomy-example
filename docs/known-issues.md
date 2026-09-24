# Known issues

Problems found while building and running these examples. The results
come from running every example in the SkyTrack simulation stack
(Gazebo + PX4 SITL + the app's `local_planner`) on 2026-09-22; see
the [test results in the README](../README.md#tested-in-simulation).

Remove an entry once it is fixed.

## Affects the app

### Recorder spams errors when its sink can't start

`VideoRecorder(sink=GStreamerSink(...))` needs the GStreamer plugins
(`appsrc`, `x264enc`, `mp4mux`). The simulation container doesn't have
them, so the sink fails with `no element "appsrc"`. The recorder then
**retries on every frame**, logging one traceback per frame (61 in a
one-minute orbit), and the clip ends with 0 frames. The mission itself
still succeeds.

**Workaround:** use the default sink (numbered PNG frames +
`meta.jsonl`), which needs nothing extra. The examples do this. Use
`GStreamerSink` only on a mission computer where the plugins are
installed.

**Fix needed (framework):** the recorder should stop and report once
when its sink fails to build.

### A camera mission on a drone without a camera can stall

Without a camera, each `capture` step gives up after 5 s
(`MEDIA_CAPTURE_FAILED: no camera frame within 5s`) and the mission
continues. In our run of `capture_photos_mission`, it then **hovered at
the next `fly_to` step until the 400 s timeout** without logging an
error. With a camera, the same mission completes in about 1 min 45 s.

**Workaround:** only run missions marked *Requires: a drone with a
camera* on a drone that has one. Check with `ros2 topic hz /camera`
first.

**Fix needed (framework):** find out why the `fly_to` after failed
captures never completes.

### The app's mission guide gives the wrong battery unit

`local_planner/examples/CLAUDE.md` says `ctx.senses.battery.percent`
is 0.0–1.0 and shows `percent < 0.4`. It is **0–100**: in simulation it
read 99.0 → 95.4 during `battery_check_mission`. A check like
`percent < 0.4` never fires. (`battery.remaining` is the 0–1 value.)

**Fix needed (app docs):** correct `CLAUDE.md`. The examples here use
`percent < 40`.

## Only with `boot_drone_for_world` (not the app)

The app's `boot_drone()` registers all built-in modes (`emergency_stop`,
`canceled`, `paused`, `takeoff`, `smart_land`, `orbit`, `helix`,
`panorama`, `goto_global`, `navigate`, `idle`) and a real
`ObstacleSense`, so the issues below were **verified not to affect the
examples in the app**. They matter if you boot the framework directly
with `boot_drone_for_world(world)` (pymavlink, SITL adapter or the
in-process simulator).

### Safety commands don't stop a mission

**Severity: high (flight safety).** Submitting `EmergencyStop`, `Pause`
or `Cancel` while a `drone.fly(...)` mission runs does **not** stop it:
the modes that react to them are not registered. In the in-process
simulator, a drone orbiting at 3 m kept moving about 6 m in the three
seconds after each command.

**Until fixed:** keep the autopilot's own failsafes and an RC pilot
ready.

### `fly_to` fails

Every `fly_to` / `fly_to_ned` step fails with
`AttributeError: '_NoopSense' object has no attribute 'voxel_size'`,
because a placeholder is registered for the `obstacle` sense.
Registering `ObstacleSense(logger=..., sources=[])` by hand fails too,
with an `IndexError`.

### Built-in modes aren't registered

Only `IdleMode` is registered, so `TakeoffRequest`, `OrbitRequest` and
similar commands do nothing.

### Setting a mode's gate flag doesn't start it when idle

`drone.ctx.flags.<gate> = True` has no effect while nothing is running,
because nothing polls the flags. Call `drone.ctx.notify_state_change()`
afterwards, or submit a `Command` instead: `drone.submit(...)` wakes the
arbiter, as [custom_mode_mission.py](../examples/custom_mode_mission.py)
does.

## Simulator-only limitations

### `SimulatedWorld` (in-process) has no heading, camera or battery

Heading is always 0 and there is no `on_camera`, so `yaw_to` never
finishes and camera services have no frames; `battery.percent` is
`None`. Use the Gazebo simulation stack for anything beyond basic
takeoff / orbit / land.

### The Gazebo test world is empty

Detection runs (about 1–1.5 s per request) but finds nothing, and
photos show only the ground and horizon. Add models to the world to
test detection hits.
