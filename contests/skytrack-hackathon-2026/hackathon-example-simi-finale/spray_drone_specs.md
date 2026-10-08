# Spray drone — camera and nozzle

The numbers you need to turn what the camera sees into ground coordinates, and to plan spray lanes.
How a run is scored is covered separately in the grading document.

## Camera

One fixed camera, pointing **straight down**. No gimbal, so it tilts with the drone.

| | |
|---|---|
| Topic | `/camera` (`sensor_msgs/Image`) |
| Encoding | **`rgb8`** — OpenCV expects BGR, so convert before any colour work |
| Resolution | 640 × 480 |
| Frame rate | asks for 5 Hz, in practice **1.5–4 Hz** depending on the machine |
| Field of view | 99.7° across, 83.3° along the flight direction |
| Lens distortion | none |

`camera_info` is not published, so put these intrinsics in your code:

```
fx = fy = 269.968      cx = 320.0      cy = 240.0
```

**Where it sits.** Bolted to the body, so it moves exactly with the drone:

| | |
|---|---|
| 0.125 m | ahead of the drone's centre, along the nose |
| 0.010 m | below the drone's centre |
| pointing | straight down, always |

**Height above ground.** The camera sits 0.217 m above ground when the drone is on the pad, and
pose `z` is measured from there:

```python
H = -pose.z + 0.217      # pose is NED, z points down
```

**What one picture covers**, with `H` in metres:

```
width  (left-right)   = 2.371 · H
length (front-back)   = 1.778 · H
ground size of 1 pixel = H / 269.968
```

| Height | Picture covers | 1 pixel |
|---|---|---|
| 3 m | 7.1 × 5.3 m | 1.1 cm |
| 5 m | 11.8 × 8.9 m | 1.9 cm |
| 10 m | 23.7 × 17.8 m | 3.7 cm |
| 20 m | 47.4 × 35.6 m | 7.4 cm |
| 37 m | 87.7 × 65.8 m | 13.7 cm |

Lane spacing for a survey comes from the first column: lanes further apart than the picture width
leave strips nobody looked at.

**Pixel to ground.** Image top is the drone's nose, image right is its right side:

```python
right = (u - 320.0) / 269.968 * H      # metres right of the camera
back  = (v - 240.0) / 269.968 * H      # metres behind the camera

cos_h, sin_h = math.cos(pose.heading), math.sin(pose.heading)

# the camera is 0.125 m ahead of the drone's centre
cam_east  = pose.y + 0.125 * sin_h
cam_north = pose.x + 0.125 * cos_h

east  = cam_east  + right * cos_h - back * sin_h
north = cam_north - right * sin_h - back * cos_h
```

This assumes the drone is level. While it banks, the picture slides by about `H × tan(tilt)` — at
37 m a 5° lean is 3.2 m off. Take pictures on straight, steady legs.

## Nozzle

One nozzle under the body, spraying **straight down**, 0.11 m below the drone's centre. Like the
camera it is bolted on, so it tilts with the drone. The valve is on or off; there is no flow
control.

| | |
|---|---|
| Flow rate | **1.0 L/min** (≈ 16.7 ml/s) whenever the valve is open |
| Spray efficiency | **0.7** — about 70 % of what leaves the nozzle reaches the ground; the rest drifts or evaporates |
| Spray cone | 30° total |
| Maximum range | 20 m — beyond that nothing lands |
| Target dose | **1.0 – 3.0 ml/m²** on every cell that needs treating |
| Recorded resolution | 0.2 m × 0.2 m cells on the ground |

**Sprayed width**, with `H` the nozzle height above ground (`-pose.z + 0.117`):

```
width = 0.536 · H
```

| Height | Sprayed width |
|---|---|
| 2 m | 1.07 m |
| 3 m | 1.61 m |
| 4 m | 2.14 m |
| 5 m | 2.68 m |

The cone is narrow, so spray altitude decides how many lanes you fly. The footprint is set by the
actual distance from the nozzle to the ground along the cone, so a tilted drone lays down a
slightly larger, displaced footprint than the table says — another reason to spray on straight,
steady legs.

### How much lands where

The nozzle puts out the same amount of liquid every second the valve is open. What sets the dose on
a patch of ground is how widely that liquid is spread:

- **Higher → less per square metre.** The cone covers a wider strip, so the same liquid is spread
  thinner.
- **Faster → less per square metre.** Each patch of ground spends less time under the nozzle.
- **Heaviest along the middle of the lane.** Ground under the centre of a pass stays under the cone
  longest; ground near the edges of the strip is only clipped briefly and gets less.
- **Slow = heavy.** Wherever the valve is open while the drone is slowing down, turning, arriving at
  a waypoint or hovering, the ground below gets far more than the target.
- **Overlap adds up.** Where two lanes overlap, their doses add.
- **Spray that misses the field still counts as spent.** Everything released while the valve is open
  is counted, wherever it lands.

Height and speed together decide both how many lanes you need and whether each lane lands in the
target band: too high or too fast under-doses, too low or too slow over-doses. Pick the pair
deliberately, close the valve for turns and transits, and check what actually landed.

> ### ⚠️ Aim for 1.0 – 3.0 ml/m²
>
> - **Below 1.0 ml/m², a cell counts as untreated** — however much it got.
> - **Above 3.0 ml/m², the extra is wasted** — and it costs you score.
>
> See the grading document for how these feed the score.
