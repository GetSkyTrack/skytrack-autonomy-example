# Dual-camera sample bags

We publish **recorded dual-camera sample bags** for anyone who wants to try the streams. You do **not** need the stereo-depth source code—only ROS 2 and the bag files.

---

## What you receive

A sample is a ROS 2 bag folder (MCAP):

```text
dualcam_YYYYMMDDTHHMMSSZ/
  dualcam_YYYYMMDDTHHMMSSZ_0.mcap
  metadata.yaml
```

If you got a `.tar.gz`:

```bash
tar xzf dualcam_YYYYMMDDTHHMMSSZ.tar.gz
```

### Topics inside

| Topic | Type | Meaning |
| --- | --- | --- |
| `/camera_left` | `sensor_msgs/msg/Image` | Left camera image |
| `/camera_right` | `sensor_msgs/msg/Image` | Right camera image |
| `/depth_camera/depth` | `sensor_msgs/msg/Image` | Depth map (meters, `32FC1`; `0.0` = invalid) |
| `/depth_camera/points` | `sensor_msgs/msg/PointCloud2` | 3D point cloud (forward distance = **X**) |

---

## Requirements

- ROS 2 **Jazzy** (or compatible) with `rosbag2` + MCAP support  
  Example packages: `ros-jazzy-rosbag2`, `ros-jazzy-rosbag2-storage-mcap`

```bash
source /opt/ros/jazzy/setup.bash
```

---

## Play a sample

Point at the bag **folder** (the directory that contains `metadata.yaml`):

```bash
source /opt/ros/jazzy/setup.bash

# Inspect
ros2 bag info /path/to/dualcam_YYYYMMDDTHHMMSSZ

# Play
ros2 bag play /path/to/dualcam_YYYYMMDDTHHMMSSZ
```

Optional:

```bash
ros2 bag play /path/to/dualcam_YYYYMMDDTHHMMSSZ --loop
ros2 bag play /path/to/dualcam_YYYYMMDDTHHMMSSZ --rate 0.5
```

Stop with `Ctrl+C`.

---

## List topics while playing

Open a **second terminal** (bag still playing):

```bash
source /opt/ros/jazzy/setup.bash

ros2 topic list
ros2 topic list -t
```

You should see at least:

- `/camera_left`
- `/camera_right`
- `/depth_camera/depth`
- `/depth_camera/points`

Useful checks:

```bash
ros2 topic hz /camera_left
ros2 topic hz /depth_camera/depth
ros2 topic echo /depth_camera/depth --once
ros2 topic info /depth_camera/points
```

---

## Quick notes

- Play the **folder**, not a random `.mcap` path by itself.
- `ros2 topic list` is empty until playback is running.
- Depth/points are aligned with the **left** camera. On points, use **X** as forward range (not Z).
- Metric depth is meant for roughly **0.01–5 m**. Far outdoor look-out can be noisy—treat meters carefully.

Questions about a sample: send the folder name (`dualcam_…`) and the output of `ros2 bag info`.
