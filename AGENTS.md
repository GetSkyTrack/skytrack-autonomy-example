# SkyTrack Autonomy Agent Instructions

This repository contains mission examples and guides for the `skytrack-autonomy` framework. When writing or modifying code in this repository, follow the patterns and rules below.

## API Patterns & Architecture

*   **Missions**: Defined as Python generator functions. Each `yield` statement corresponds to one sequential step in the mission (e.g., `yield takeoff(alt_m=3.0)`).
*   **Skills**: The *only* components allowed to move the drone. They own the setpoint loop and publish trajectory setpoints.
*   **Senses**: Read-only views of the world (e.g., `pose`, `battery`, `camera`). They subscribe to world messages (e.g., `on_local_position`) and process them passively. Never publish or command from a Sense.
*   **Services**: Background tasks (e.g., `VideoRecorder`, `TelemetryLogger`) that run alongside the mission. They may write files or drive payloads but must never publish setpoints.
*   **Modes**: Control programs managed by the Supervisor. Modes are triggered via a gating mechanism (flags and parameters changed by a `Command`). The Supervisor arbitrates based on priority tiers (`SAFETY`, `PRELAUNCH`, `LANDING`, `MISSION`, `NAVIGATE`, `BACKGROUND`, `FALLBACK`).

## Required Imports

*   **Standard Mission Helpers**: Import mission action methods and drone setup functions from `local_planner`.
    ```python
    from local_planner import boot_drone, brake, brake_and_settle, land, takeoff, fly_to, orbit, helix, yaw_to, capture
    ```
*   **Framework Core & Extensions**: Senses, skills, services, and scheduling are located in `skytrack_autonomy`.
    ```python
    from skytrack_autonomy.core.lib.scheduling import ScheduleGroup
    from skytrack_autonomy.core.commands import Command, EmergencyStop, TakeoffRequest
    from skytrack_autonomy import ControlMode, SkillStep, OnDone
    ```
*   **Senses & Services**: Custom/advanced parts are often imported from `skytrack_autonomy` or its `contrib` sub-packages.

## Mission File Conventions

*   **File Naming**: Mission files in `skytrack-autonomy-examples` belong in the `examples/` directory and must be named `*_mission.py`.
*   **Function Naming**: The main generator function must exactly match the file name.
*   **Docstrings**: Mission docstrings must follow a specific convention including 'Level ', 'What you learn:', 'Requires:', and a 'Run::' command block (e.g., `python -m local_planner.examples.<mission_name>`).

## Flight State Conventions

*   **Coordinate System**: NED (North, East, Down).
*   **Altitude**: The `z` axis is **negative up**. To climb 3 meters, `z` decreases by 3. When using high-level helpers (like `alt_m=3.0`), this conversion is handled for you, but be aware of it when writing custom skills.
*   **Time**: Always use `ctx.world.now()` for time. **Never** use `time.time()` or `time.sleep()`, as this breaks compatibility with simulation clocks (`FakeClock`).

## Mission Action Methods

Use the following step helpers in your generator missions via `yield`:
*   `yield takeoff(alt_m=...)`: Ascends to the specified altitude.
*   `yield fly_to(north=..., east=..., alt_m=..., name=...)`: Flies to a local NED coordinate. Always provide a clear `name=`.
*   `yield orbit(center_north=..., center_east=..., alt_m=..., radius_m=..., duration_s=...)`: Flies in a circle.
*   `yield brake(name=...)`: Stops the drone and holds the position. Do this before capturing photos.
*   `yield brake_and_settle(name=...)`: Use this to brake and settle before `yield land()`. This is required to satisfy framework testing conventions.
*   **Note**: All yielding steps (e.g., `fly_to`, `brake`, `brake_and_settle`, `land`) must include a descriptive `name=` argument.

## Coding Rules

1.  **Strict Threading/Scheduling**:
    *   Setpoints in Skills must be published on `ScheduleGroup.CONTROL` at a minimum frequency of 10 Hz.
    *   Background processing in Services must be scheduled on `ScheduleGroup.MEDIA` or `DEFAULT` so they do not block flight control threads.
2.  **No Blocking Calls**: Never use `time.sleep()` or perform heavy synchronous IO in a control callback. Use `ctx.world.now()` for time.
3.  **Always Verify `requires_senses`**: Any mission or mode must explicitly declare the senses it uses, e.g., `my_mission.requires_senses = ["pose", "status"]`.
4.  **Graceful Cancellation**: Every custom `Skill` must implement a `cancel(self, ctx, reason)` method that unschedules its handles (using `ctx.scheduler.unschedule`) and resets its state idempotently.
5.  **Null Safety**: Sense properties can return `None` (e.g., before the first message arrives). Always check for `None` before formatting or using sensor values.
6.  **Dynamic Skill Interruption**: To interrupt a skill dynamically during a mission (e.g., for mid-flight battery checks), wrap the skill in a `SkillStep` and provide a custom `is_done` function.
7.  **Logging**: Logging within missions must be performed using `ctx.world.log_info("message")`.

## Testing

*   Run local tests using `PYTHONPATH=tests pytest tests -v` to successfully leverage the mock framework located in `tests/skytrack_autonomy/`.

## Hackathon 2026 Guidelines

When working on Hackathon 2026 missions:
*   **Coordinate system**: World coordinates are ENU (x east, y north, z up). Convert to NED for `fly_to`: `fly_to(north=y, east=x)`.
*   **Stress Area Writing**: Write detection results to `/root/.ros/captures/stress_area.json`. Write atomically by writing to `.stress_area.json.tmp` and replacing to avoid truncating files if stopped. The file must contain `{"stress_area": [{"class": "stressed", "polygon": [[x1, y1], ...]}]}` in ENU meters.
*   **Camera Specifications**: The downward-facing camera publishes `rgb8` to `/camera` at 640x480 resolution. Intrinsics are: `fx = fy = 269.968, cx = 320.0, cy = 240.0`.
*   **Spray Grading Specifications**: The target spray dose for stressed crops is 1.0 - 3.0 ml/m². A cell receiving below 1.0 ml/m² counts as entirely untreated. Above 3.0 ml/m² is wasted and costs score. Flow rate is 1.0 L/min, with a spray efficiency of 0.7.
