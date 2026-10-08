# Prepared examples

Sample code added for three requirements:

| # | Requirement | File |
|---|---|---|
| 1 | Control the camera gimbal | [examples/gimbal_mission.py](../examples/gimbal_mission.py) |
| 2 | Use AI models; line 1 is `"""AImodels = [model1_name, model2_name, model3_name]"""` | [contests/skytrack-hackathon-2026/ai_models_example.py](../contests/skytrack-hackathon-2026/ai_models_example.py) |
| 3 | Write a `.txt` file with the position and priority of each detected person | [contests/skytrack-hackathon-2026/final_example.py](../contests/skytrack-hackathon-2026/final_example.py) |

All three run in the simulation container the same way:

```bash
C=skytrack-simulation-skytrack-autonomy-1
docker exec $C bash -c 'mkdir -p /tmp/sae && touch /tmp/sae/__init__.py'
docker cp <file>.py $C:/tmp/sae/
docker exec -it $C bash -c 'source /app/setup.sh && cd /tmp && python3 -m sae.<file>'
```

---

## 1. Gimbal control — `gimbal_mission.py`

Uses the `Gimbal` service from `skytrack_autonomy`:

| Call | What it does |
|---|---|
| `gimbal.point(pitch_deg=, yaw_deg=)` | Move to an absolute angle (body frame, pitch negative down) |
| `gimbal.rate(yaw=, pitch=)` | Turn at a rate (−100..100) until `stop()` |
| `gimbal.stop()` | Stop turning; the gimbal holds where it is |
| `gimbal.center()` | Back to center |
| `gimbal.result(id)` | `(success, message)` once the controller answered |
| `ctx.senses.gimbal.is_near(...)` | Measured angle is close to a target |

The mission: take off to 5 m → aim down (−90°) → photo → fly a 12 m line → aim ahead (−30°) →
photo → pan right 4 s → photo → center → fly home → land. Each aim hovers until the gimbal
reports the angle, or 5 s pass. Photos go to `~/.ros/captures/gimbal_<view>.png`.

```python
def main() -> None:
    with boot_drone() as drone:
        drone.add_sense(CameraSense())
        drone.add_service(Gimbal())          # also registers the gimbal sense
        drone.fly(gimbal_mission)
        drone.run()
```

**Simulation run (2026-10-08):** the mission completed and saved all 3 photos, but every gimbal
command failed with `No gimbal service on '/skytrack/camera/minipro/gimbal_control'`. That
service has no server in the simulation; the simulated controller serves
`/skytrack/camera/front/gimbal_control` and `/skytrack/camera/cam_0/gimbal_control`. The app
and the simulation disagree on the camera id; the example code is not the cause.

---

## 2. AI models — `ai_models_example.py`

Line 1 lists every model the mission uses:

```python
"""AImodels = [det-coco-v26n-b-quantized-fp16, det-visdrone-v26n-b-quantized-fp16, det-firesmoke-v26n-b-quantized-fp16]"""
```

The mission flies to a viewpoint, runs each model with the `Detector` service one after the
other (the detector serves one request at a time), waits for each answer and logs every box.

```python
AI_MODELS = {
    "det-coco-v26n-b-quantized-fp16": ["person"],
    "det-visdrone-v26n-b-quantized-fp16": ["pedestrian", "people"],
    "det-firesmoke-v26n-b-quantized-fp16": ["fire", "smoke"],
}
```

Built-in models and their classes, from `/opt/skytrack/ai/mapping.json` in the drone container:

| Model | Classes |
|---|---|
| `det-coco-v26n-b-quantized-fp16` | 80 COCO classes; people are `person` |
| `det-visdrone-v26n-b-quantized-fp16` | `pedestrian`, `people`, `bicycle`, `car`, `van`, `truck`, `tricycle`, `awning-tricycle`, `bus`, `motor` |
| `det-firesmoke-v26n-b-quantized-fp16` | `fire`, `smoke` |
| `det-h2026-v26n-b-fp32-640` | `stressed` |

Boxes come back in image pixels. To turn one into a ground position, use "Pixel to ground" in
[spray_drone_specs.md](../contests/skytrack-hackathon-2026/spray_drone_specs.md).

---

## 3. Rescue locations file — `final_example.py`

Shows the standard way to write `rescue_locations.txt`. The data is dummy; replace it with what
your model finds.

### Format

One line per detected person: `index x y z`

| Field | Meaning |
|---|---|
| `index` | Rescue priority, `0` = rescue first |
| `x y z` | Position of the person |

Output of the example:

```
0 10.5 20.3 15.0
1 30.0 40.0 25.5
2 15.2 12.7 30.0
```

### How to write it

Copy `save_rescue_location` from `final_example.py` unchanged and call it once per person,
before `land()`:

```python
save_rescue_location(0, 10.5, 20.3, 15.0)
```

- An index already in the file has its line **updated**; a new index is appended.
- Thread-safe: it can be called from a scheduled callback as well as from the mission.

### Where the file is saved

`rescue_locations.txt` is created **in the same folder as the running Python file**:

```python
RESCUE_LOCATIONS_FILE = Path(__file__).resolve().parent / "rescue_locations.txt"
```

| How it is run | File |
|---|---|
| `python3 -m sae.final_example` from `/tmp` in the drone container | `/tmp/sae/rescue_locations.txt` |
| `python3 /path/to/final_example.py` | `/path/to/rescue_locations.txt` |

The file stays inside the container. Read or copy it from your machine:

```bash
C=skytrack-simulation-skytrack-autonomy-1
docker exec $C cat /tmp/sae/rescue_locations.txt
docker cp $C:/tmp/sae/rescue_locations.txt .
```

Lines from an earlier run stay in the file. Delete it before a new run if needed:
`docker exec $C rm -f /tmp/sae/rescue_locations.txt`.
