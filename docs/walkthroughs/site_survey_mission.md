# Walkthrough: `site_survey_mission.py`

**Level 6 · Putting it together** — [source](../../examples/site_survey_mission.py)

A complete job: sweep a 16 × 10 m site while recording video,
photograph three inspection points, stay inside a geofence, log
telemetry, and go home early if the battery runs low. Nothing in this
file is new — it combines the earlier examples. What it teaches is
**how to structure a real mission** so it stays readable.

**Requires:** a drone with a camera, and the three Level 5 files in the
same folder.

---

## 1. Reusing parts from other files

```python
try:
    from .custom_service_mission import TelemetryLogger
    from .custom_sense_mission import GeofenceSense
    from .custom_skill_mission import HoverForSeconds
except ImportError:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from custom_service_mission import TelemetryLogger
    from custom_sense_mission import GeofenceSense
    from custom_skill_mission import HoverForSeconds
```

The custom sense, skill and service are defined once, in their Level 5
files, and imported here.

- `python -m …site_survey_mission` runs the file as part of a package,
  so the **relative import** (`from .custom_…`) works.
- `python3 /path/site_survey_mission.py` runs it as a plain script:
  there's no package, the relative import raises `ImportError`, and the
  **fallback** imports the siblings from the file's own folder.

Both paths were checked inside the simulation container.

## 2. Settings describe the job

```python
ALT_M = 5.0
AREA_NORTH_M = 16.0
AREA_EAST_M = 10.0
LINE_SPACING_M = 2.5
SWEEP_SPEED_M_S = 2.0
MIN_BATTERY_PERCENT = 35.0
FENCE_RADIUS_M = 25.0
FENCE_MAX_ALT_M = 15.0

INSPECTION_POINTS = [
    ("gate", 0.0, 10.0),
    ("tank", 16.0, 10.0),
    ("shed", 16.0, 0.0),
]
```

Everything an operator might change for a different site is here —
area, spacing, speed, safety limits, points of interest.

## 3. One sub-mission per phase

The mission body is three lines of intent:

```python
def site_survey_mission(ctx):
    yield takeoff(alt_m=ALT_M)
    ctx.services.telemetry.mark("airborne")

    yield from survey_area(ctx)
    if not battery_low(ctx):
        yield from inspect_points(ctx)
    yield from go_home(ctx)
```

Each phase is its own generator (`survey_area`, `inspect_points`,
`go_home`), composed with `yield from` as in
[compose_mission](../../examples/compose_mission.py). Reading the top
level tells you the whole job; each phase can be read, changed and
tested alone.

## 4. Safety checks at every decision point

```python
def battery_low(ctx):
    percent = ctx.senses.battery.percent
    return percent is not None and percent < MIN_BATTERY_PERCENT
```

`battery_low` is checked **before every sweep line** and **before every
inspection point**, and once more between the phases. The mission never
commits to more flying than the battery allows; whatever happens, it
still reaches `go_home`.

The geofence is checked the same way, before flying a line:

```python
if not (fence.allows(*start, ALT_M) and fence.allows(*end, ALT_M)):
    ctx.world.log_warn(f"{TAG} line {i} leaves the fence, skipping")
    continue
```

It's a *plan-time* check (`allows(...)` on the target) — the sense
answers "would this point be inside?" before the drone goes there.

## 5. Services wrap the phases

```python
def survey_area(ctx):
    rec, tlm, fence = ctx.services.recorder, ctx.services.telemetry, ctx.senses.geofence
    rec.start(clip="area_sweep")
    tlm.mark("sweep start")
    for i, (start, end) in enumerate(sweep_lines(), start=1):
        ...
    rec.stop()
    tlm.mark("sweep end")
```

Recording covers exactly the sweep, and telemetry notes mark each
phase — so the CSV and the video can be lined up afterwards.

## 6. The inspection loop

```python
for name, north, east in INSPECTION_POINTS:
    if battery_low(ctx):
        ...
        return
    yield fly_to(north=north - 3.0, east=east - 3.0, alt_m=ALT_M, name=f"to_{name}")
    yield yaw_to(north=north, east=east, name=f"face_{name}")
    yield HoverForSeconds(2)                    # let the gimbal settle
    yield capture(filename=f"inspect_{name}.png", name=f"photo_{name}")
    tlm.mark(f"photographed {name}")
```

Stand off 3 m south-west of the point, face it, let the camera settle
(the custom skill), shoot. `return` inside a sub-mission ends just that
phase — the caller carries on to `go_home`.

## 7. `main()` wires everything, in order

```python
def main() -> None:
    with boot_drone() as drone:
        drone.add_sense(CameraSense())
        drone.add_sense(GeofenceSense(radius_m=FENCE_RADIUS_M,
                                      max_alt_m=FENCE_MAX_ALT_M))
        drone.add_service(VideoRecorder(output_dir="~/.ros/recordings", fps=10.0))
        drone.add_service(TelemetryLogger(hz=5.0))
        drone.fly(site_survey_mission)
        ...
        drone.run()
```

Senses, then services, then the mission — and `requires_senses` lists
every sense the mission reads, including the custom `geofence`:

```python
site_survey_mission.requires_senses = [
    "pose", "obstacle", "status", "battery", "camera", "geofence",
]
```

---

## What happened in simulation

**Succeeded**, 26 steps in 3 min 10 s (camera drone).

| Phase | Steps | Time | Output |
|---|---|---|---|
| Takeoff | `takeoff(5.0m)` | 11.7 s | |
| Sweep | 5 × (`line_N_start` + `line_N_sweep`) | ~105 s | **274 video frames** in `~/.ros/recordings/area_sweep/` |
| Inspection | 3 × (`to_*`, `face_*`, `hover_for_seconds`, `photo_*`) | ~49 s | `inspect_gate.png`, `inspect_tank.png`, `inspect_shed.png` |
| Return | `return_home`, `pre_land`, `land` | 24 s | |

Also:

- **Telemetry:** 740 CSV rows, notes at every phase boundary.
- **Geofence:** 0 breaches — all lines and points were inside 25 m.
- **Battery:** 100 % → 80.9 %, so the low-battery branch (35 %) didn't
  trigger. The unit test `test_site_survey_skips_inspection_when_battery_low`
  covers that branch without flying.
- **`hover_for_seconds` (2 s)** measured 2.1–2.2 s each time.
- **`photo_*`** took 0.3–0.6 s: with a camera, a capture is fast.

**Reading the numbers:** about 55 % of the flight was the sweep and
25 % the inspections. A full survey used ~19 % battery in simulation,
so on this airframe the 35 % reserve leaves room for roughly three such
jobs per charge — a real battery will be less generous.

---

## Try it yourself

1. Change `INSPECTION_POINTS` to your own site and add a fourth point.
2. Set `MIN_BATTERY_PERCENT = 95` and fly it: the sweep should stop
   early and the inspections be skipped.
3. Move `FENCE_RADIUS_M` down to `12`: which lines and points are now
   skipped? Check the `[SURVEY]` warnings.
4. Add a `detect` stop at each inspection point, using the pattern from
   [detect_mission](detect_mission.md).
