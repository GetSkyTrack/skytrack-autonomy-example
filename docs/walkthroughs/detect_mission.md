# Walkthrough: `detect_mission.py`

**Level 4 · Camera & services** — [source](../../examples/detect_mission.py)

Fly to two viewpoints, point the camera at an area, and ask the onboard
AI model whether there are people in it. The new idea here is
**waiting for something that isn't flying**: the detector answers
*later*, and the mission must hover until it does, without hanging if
it never does.

**Requires:** a drone with a camera and the `object_detection` server
running.

---

## 1. Registering the service

```python
from skytrack_autonomy import Detector

MODEL = "det-coco-v26n-b-quantized-fp16"
CLASSES = ["human"]

def main() -> None:
    with boot_drone() as drone:
        drone.add_service(Detector(model_name=MODEL, classes=CLASSES))
        drone.fly(detect_mission)
        ...
```

- `Detector` isn't re-exported by `local_planner`, so it's imported from
  `skytrack_autonomy` (like `Sprayer`).
- `model_name` / `classes` passed here are **defaults**; each
  `request()` can override them.
- The service is added **before** `drone.fly()`, so
  `ctx.services.detector` exists from the mission's first line.

## 2. Aim first, then detect

```python
for i, ((north, east), (look_n, look_e)) in enumerate(VIEWPOINTS, start=1):
    yield fly_to(north=north, east=east, alt_m=ALT_M, name=f"to_view_{i}")
    yield yaw_to(north=look_n, east=look_e, name=f"face_area_{i}")
    yield brake(name=f"settle_{i}")
```

Each viewpoint is a pair: *where to be* and *where to look*. `yaw_to`
turns the camera toward the area; `brake` stops residual motion so the
frame isn't blurred. This fly → face → settle sequence is the same one
[capture_photos_mission](../../examples/capture_photos_mission.py) uses
before `capture`.

## 3. Ask, and handle "no"

```python
if not det.request(model_name=MODEL, classes=CLASSES,
                   confidence_threshold=0.5):
    ctx.world.log_warn(f"{TAG} detector refused request at view {i}")
    continue
```

`request()` **returns immediately**. `True` means "accepted, working on
it"; `False` means refused — no model, no classes, no detector on this
drone, or a request already running. The mission logs it and moves on
to the next viewpoint instead of crashing.

## 4. Wait for the answer — with a timeout

```python
def wait_for_detection(ctx: Any, name: str) -> SkillStep:
    """Hover until the detector has finished (or the timeout)."""
    det = ctx.services.detector
    deadline = ctx.world.now() + DETECT_TIMEOUT_S
    return SkillStep(
        skill=brake(name=name).skill,
        is_done=lambda c: not det.is_busy or c.world.now() >= deadline,
        name=name,
    )

yield wait_for_detection(ctx, name=f"detect_{i}")
```

This is the key pattern of the file:

| Piece | Role |
|---|---|
| `brake(...).skill` | The motion while waiting: hold position. Borrowed from a built-in step. |
| `is_done=lambda c: ...` | Our own finish condition, checked at 5 Hz, replacing the brake skill's own. |
| `not det.is_busy` | Done when the detector has an answer. |
| `c.world.now() >= deadline` | …or when 15 s have passed. **Never wait without a timeout.** |
| `name=f"detect_{i}"` | Shows up in the report, so you can see how long each wait took. |

Why not a `while det.is_busy: time.sleep(0.1)` loop? The mission
generator runs on the decision thread; blocking it would stop the
Supervisor, including emergency stop. Yielding a step keeps everything
responsive and keeps sending setpoints.

## 5. Use the result

```python
result = det.last_result
if result is None or not result.success:
    ctx.world.log_warn(f"{TAG} no result at view {i}")
    continue
for d in result.detections:
    ctx.world.log_info(
        f"{TAG} view {i}: {d.class_name} {d.score:.0%} at pixel "
        f"({d.bbox.center_x:.0f}, {d.bbox.center_y:.0f})")
total += result.num_detections
```

`last_result` is a `DetectionOutcome`: `success`, `num_detections`, and
`detections`, each with `class_name`, `score` and a pixel `bbox`. Both
failure shapes (no result, unsuccessful result) are handled before
touching the detections.

---

## What happened in simulation

**Succeeded**, 12 steps, 1 min 7 s of flight. From the mission report:

| Step | Time |
|---|---|
| `takeoff(5.0m)` | 15.3 s |
| `to_view_1` → `face_area_1` → `settle_1` | 11.3 s → 0.3 s → 0.3 s |
| **`detect_1`** | **3.8 s** |
| `to_view_2` → `face_area_2` → `settle_2` | 8.2 s → 1.1 s → 0.3 s |
| **`detect_2`** | **2.2 s** |
| `return_home` → `pre_land` → `land` | 11.1 s → 0.3 s → 12.2 s |

and in the log:

```
[DET] 0 detection(s) for ['human'] in 1273.0ms
[DET] 0 detection(s) for ['human'] in 1073.6ms
[DETECT] total detections: 0
```

**Reading the numbers:**

- The model itself took ~1.1–1.3 s. The `detect_N` steps took longer
  (2.2–3.8 s) because they also include getting a frame to the model
  and the 5 Hz done-check. Budget ~2–4 s per detection stop.
- **0 detections is correct**: the Gazebo test world is empty. To see
  the result-handling branch run, add a person model to the world.
- `face_area_1` took only 0.3 s because the drone was already facing
  roughly north after `to_view_1`; `face_area_2` needed a 45° turn.

---

## Try it yourself

1. Lower `DETECT_TIMEOUT_S` to `0.5`. The waits now end on the timeout
   before the answer arrives — what does `last_result` contain at the
   first viewpoint?
2. Add `"car"` to `CLASSES` and log counts per class.
3. React to a hit: if a person is found, `yield orbit(...)` around the
   viewpoint before moving on.

**Next:** [custom_skill_mission](custom_skill_mission.md) — writing
your own motion.
