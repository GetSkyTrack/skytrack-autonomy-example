# Prepared examples

Sample code added for three requirements:

| # | Requirement | File |
|---|---|---|
| 1 | Control the camera gimbal | [examples/gimbal_mission.py](../examples/gimbal_mission.py) |
| 2 | Use AI models; line 1 is `"""AImodels = [model1_name, model2_name, model3_name]"""` | [contests/skytrack-hackathon-2026/hackathon-example-finale/ai_models_example.py](../contests/skytrack-hackathon-2026/hackathon-example-finale/ai_models_example.py) |
| 3 | Write a `.txt` file with the position and priority of each detected person | [contests/skytrack-hackathon-2026/hackathon-example-finale/final_example.py](../contests/skytrack-hackathon-2026/hackathon-example-finale/final_example.py) |

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
GIMBAL_CAMERA_ID = "cam_0"

def main() -> None:
    sys.argv += ["--ros-args", "-p", f"gimbal_camera_id:={GIMBAL_CAMERA_ID}"]
    with boot_drone() as drone:
        drone.add_sense(CameraSense())
        drone.add_service(Gimbal())          # also registers the gimbal sense
        drone.fly(gimbal_mission)
        drone.run()
```

### Two things to get right

**1. The gimbal camera id.** The app sends gimbal commands to
`/skytrack/camera/<gimbal_camera_id>/gimbal_control`, default `minipro`. In the simulation only
`cam_0` has a server (`ros2 service info /skytrack/camera/cam_0/gimbal_control` →
`Services count: 1`); with `minipro` or `front` every command fails with
`No gimbal service on '...'`. The example sets the `gimbal_camera_id` ROS parameter in `main()`.

**2. Fresh frames before a photo.** `/camera` is the camera on the gimbal (the bridge maps
Gazebo `/minipro/gimbal/image` to it), but it only gives about 1 frame/s in flight, and
`capture` saves the latest frame it already has. A photo taken right after the gimbal arrives
can be an old frame from before the move. `photo()` therefore hovers until the camera has
delivered 2 new frames (`ctx.senses.camera.seq`), then captures:

```python
def photo(ctx, view):
    seq0 = ctx.senses.camera.seq
    yield hover_until(lambda c: c.senses.camera.seq >= seq0 + FRESH_FRAMES,
                      timeout_s=FRAME_WAIT_S, ctx=ctx, name=f"fresh_frame_{view}")
    yield capture(output_dir=OUTPUT_DIR, filename=f"gimbal_{view}.png", name=f"photo_{view}")

yield from photo(ctx, "down")
```

**Simulation run (2026-10-08, `cam_0`):** every gimbal command was answered
(`SIYI simulation command dispatched: ...`), the measured joints followed (pitch −90°, −30°, yaw
pan, back to 0), and each photo shows its view: `gimbal_down.png` looks straight down on the room,
`gimbal_ahead.png` is tilted down, `gimbal_panned.png` is turned to the side. The mission landed
and disarmed.

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

**Simulation run (2026-10-08):** all three models answered (`coco` 647 ms, `visdrone` 766 ms,
`firesmoke` 626 ms), 0 boxes each, which is expected in an empty world; the mission landed.

Boxes come back in image pixels. To turn one into a ground position, use "Pixel to ground" in
[spray_drone_specs.md](../contests/skytrack-hackathon-2026/hackathon-example-simi-finale/spray_drone_specs.md).

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

**Simulation run (2026-10-08):** the log shows
`[RESCUE] saved to /tmp/sae/rescue_locations.txt`, and the file holds exactly the 3 lines above.

Lines from an earlier run stay in the file. Delete it before a new run if needed:
`docker exec $C rm -f /tmp/sae/rescue_locations.txt`.
