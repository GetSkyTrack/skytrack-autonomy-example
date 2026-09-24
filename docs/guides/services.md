# Services

A **service** is a background job that runs alongside the whole
mission: recording video, logging telemetry, running a detector,
opening a spray valve. It may write files or drive a payload, but it
**never moves the drone** — that's a skill's job.

```python
def survey(ctx):
    ctx.services.recorder.start(clip="leg_A")   # background: recording
    yield fly_to(north=10, east=0, alt_m=3)     # foreground: flying
    ctx.services.recorder.stop()
```

**On this page:** [Services vs skills](#services-vs-skills) ·
[Lifecycle](#lifecycle) · [Built-in services](#built-in-services) ·
[Writing a service](#writing-a-service) · [Patterns](#patterns) ·
[Pitfalls](#pitfalls) · [Testing](#testing)

---

## Services vs skills

| | Skill | Service |
|---|---|---|
| Moves the drone | Yes (only skills do) | **Never** |
| How long it lives | One step | The whole mission |
| How many at once | One | Any number |
| How the mission uses it | `yield` it | Call its methods between `yield`s |
| Schedule group | `CONTROL` | `MEDIA` |
| Example | `fly_to`, `HoverForSeconds` | `VideoRecorder`, `TelemetryLogger` |

Rule of thumb: if it must happen **while** the drone does something
else, and doesn't decide where the drone goes, it's a service.

---

## Lifecycle

```
main():  drone.add_service(svc)  ──►  svc.attach(ctx)
                                         wire up; may start periodic work
mission:  ctx.services.<name>.<your_method>()   ← any number of times
mission program finishes          ──►  svc.mission_ended()   (optional)
drone.run() exits (any reason)    ──►  svc.shutdown()        (always)
```

- `attach(ctx)` is called **immediately** by `add_service` — so add
  services before `drone.fly()`, and the mission can use them from its
  first line.
- `shutdown()` runs in a `finally` when `drone.run()` returns — on a
  normal finish, Ctrl-C or an exception. It must be **idempotent**
  (safe to call twice).
- `mission_ended()` is optional. In simulation, the
  `TelemetryLogger`'s `mission_ended()` wrote its `"mission ended"` row
  after landing, just before the file closed.

---

## Built-in services

| Service | `ctx.services.` | Mission API | Needs |
|---|---|---|---|
| `VideoRecorder(output_dir=, fps=, sink=)` | `recorder` | `start(clip=)`, `stop()`, `is_recording`, `frame_count` | Camera |
| `Snapshot(output_dir=)` | `snapshot` | `snap(filename)` → path or `None` | Camera |
| `Detector(model_name=, classes=)` | `detector` | `request(...)` → bool, `is_busy`, `last_result`, `cancel()` | Camera + detection server |
| `Sprayer()` | `sprayer` | `on()`, `off()` → bool, `state`, `is_settled` | Spray valve |

Import `VideoRecorder` and `Snapshot` from `local_planner`; `Sprayer`
and `Detector` from `skytrack_autonomy`.

### What the simulation runs showed

| Service | Result |
|---|---|
| `VideoRecorder` (default PNG sink) | 56 frames on a 20 s orbit with `fps=10`. The sim camera publishes ~3 Hz and the recorder only saves new frames (it compares the camera's `seq`), so you get the camera's rate, not `fps`. 274 frames over the 3-minute site survey. |
| `VideoRecorder(sink=GStreamerSink(...))` | **0 frames**: GStreamer plugins aren't installed in the sim container. See [known issues](../known-issues.md#recorder-spams-errors-when-its-sink-cant-start). |
| `Snapshot` | Saved 640×480 stills without stopping the drone. |
| `Detector` | ~1.1–1.3 s per request; 0 hits in the empty test world. |
| `Sprayer` | Not yet tested (needs a spray-valve drone). |

---

## Writing a service

```python
class TelemetryLogger:
    name = "telemetry"                        # → ctx.services.telemetry

    def attach(self, ctx):                    # by drone.add_service()
        self._ctx = ctx
        self._handle = ctx.scheduler.schedule(
            self._sample, hz=5.0, group=ScheduleGroup.MEDIA,
            name=self.name, now=ctx.world.now())

    def mark(self, note):                     # your mission-facing API
        ...

    def mission_ended(self):                  # optional
        self.mark("mission ended")

    def shutdown(self):                       # always; idempotent
        if self._handle is not None:
            self._ctx.scheduler.unschedule(self._handle)
            self._handle = None
        ...close files...
```

| Member | Required | Job |
|---|---|---|
| `name` | yes | Registry key. |
| `attach(ctx)` | yes | Keep `ctx`; schedule periodic work on `MEDIA`. |
| `shutdown()` | yes | Unschedule, flush, close. Idempotent. |
| `mission_ended()` | no | Final bookkeeping. |
| your methods | — | What the mission calls. |

Complete example: [custom_service_mission.py](../../examples/custom_service_mission.py).
In simulation it wrote 217 rows at about 5 Hz, with the notes
`airborne`, `orbit start`, `orbit end` and `mission ended` on the right
rows.

---

## Patterns

### Always-on vs toggled

- **Always-on** (telemetry logger, watchdog): start the periodic work in
  `attach()`.
- **Toggled by the mission** (recorder): `attach()` only prepares;
  `start()` / `stop()` schedule and unschedule the work.

### Asynchronous requests

Some services start work that finishes later (`Detector.request`,
`Sprayer.on`). The call returns immediately with "accepted or refused";
the result shows up in a property. Make the mission **hover until the
answer arrives**, with a timeout:

```python
if det.request(model_name=MODEL, classes=["human"]):
    yield SkillStep(
        skill=brake(name="wait").skill,
        is_done=lambda c: not det.is_busy or c.world.now() >= deadline,
        name="detect_1",
    )
    result = det.last_result
```

Walkthrough: [detect_mission](../walkthroughs/detect_mission.md).

### Return `bool` instead of raising

Mission-facing methods should return `True`/`False` for "accepted or
refused" (as `Sprayer.on()` and `Detector.request()` do) so a missing
payload doesn't crash the flight. Raise only for programming errors,
like calling before `attach()`.

---

## Pitfalls

| Mistake | What happens | Fix |
|---|---|---|
| Publishing a setpoint from a service | Two things fighting over the drone | Move motion into a skill |
| Slow work on `CONTROL` | Setpoints delayed; PX4 may drop OFFBOARD | Use `MEDIA` |
| `shutdown()` not idempotent | Crash at teardown (it can be called twice) | Guard every resource with `if ... is not None` |
| Adding the service after `drone.fly()` | `ctx.services.<name>` missing when the mission starts | Add services first |
| A sink/driver that fails on every tick | Log floods (seen with `GStreamerSink`) | Fail once, log once, stop |
| Assuming a payload exists | Refused requests, empty output | Check return values; document `Requires` |

---

## Testing

Attach to a fake context and tick the `MEDIA` group by hand:

```python
clock = FakeClock()
ctx = make_fake_ctx(clock=clock)
tlm = TelemetryLogger(output_dir=str(tmp_path), hz=5.0)
tlm.attach(ctx)
for _ in range(3):
    clock.advance(0.25)
    ctx.scheduler.tick(ScheduleGroup.MEDIA, ctx.world.now())
tlm.shutdown()
tlm.shutdown()          # must be safe
```

To test a mission that *uses* services without real ones, register
stand-ins: `ctx.services.register("sprayer", MagicMock())` — see
[tests/test_examples.py](../../tests/test_examples.py).
