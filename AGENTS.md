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
    from local_planner import boot_drone, brake, land, takeoff, fly_to, orbit, helix, yaw_to, capture
    ```
*   **Framework Core & Extensions**: Senses, skills, services, and scheduling are located in `skytrack_autonomy`.
    ```python
    from skytrack_autonomy.core.lib.scheduling import ScheduleGroup
    from skytrack_autonomy.core.commands import Command, EmergencyStop, TakeoffRequest
    from skytrack_autonomy import ControlMode, SkillStep, OnDone
    ```
*   **Senses & Services**: Custom/advanced parts are often imported from `skytrack_autonomy` or its `contrib` sub-packages.

## Flight State Conventions

*   **Coordinate System**: NED (North, East, Down).
*   **Altitude**: The `z` axis is **negative up**. To climb 3 meters, `z` decreases by 3. When using high-level helpers (like `alt_m=3.0`), this conversion is handled for you, but be aware of it when writing custom skills.
*   **Time**: Always use `ctx.world.now()` for time. **Never** use `time.time()` or `time.sleep()`, as this breaks compatibility with simulation clocks (`FakeClock`).

## Mission Action Methods

Use the following step helpers in your generator missions via `yield`:
*   `yield takeoff(alt_m=...)`: Ascends to the specified altitude.
*   `yield fly_to(north=..., east=..., alt_m=..., name=...)`: Flies to a local NED coordinate. Always provide a clear `name=`.
*   `yield orbit(center_north=..., center_east=..., alt_m=..., radius_m=..., duration_s=...)`: Flies in a circle.
*   `yield brake(name=...)`: Stops the drone and holds the position. Do this before capturing photos or landing.
*   `yield land()`: Lands the drone and finishes when disarmed.

## Coding Rules

1.  **Strict Threading/Scheduling**:
    *   Setpoints in Skills must be published on `ScheduleGroup.CONTROL` at a minimum frequency of 10 Hz.
    *   Background processing in Services must be scheduled on `ScheduleGroup.MEDIA` or `DEFAULT` so they do not block flight control threads.
2.  **No Blocking Calls**: Never use `time.sleep()` or perform heavy synchronous IO in a control callback.
3.  **Always Verify `requires_senses`**: Any mission or mode must explicitly declare the senses it uses, e.g., `my_mission.requires_senses = ["pose", "status"]`.
4.  **Graceful Cancellation**: Every custom `Skill` must implement a `cancel(self, ctx, reason)` method that unschedules its handles (using `ctx.scheduler.unschedule`) and resets its state idempotently.
5.  **Null Safety**: Sense properties can return `None` (e.g., before the first message arrives). Always check for `None` before formatting or using sensor values.
