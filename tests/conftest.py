"""Test setup: stand in for the app's ``local_planner`` package.

In the app, ``local_planner`` re-exports the ``skytrack_autonomy`` SDK
and adds ``boot_drone()``, which needs ROS. Here we install a stand-in
module with the same SDK names, so the examples import and their
missions can be stepped through without ROS. ``boot_drone`` is not
available in tests; ``main()`` is never called.
"""
import sys
import types
from pathlib import Path

import skytrack_autonomy

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

_EXPORTS = [
    "CameraSense", "CaptureSkill", "Command", "ControlMode", "Drone",
    "FrameSink", "GStreamerSink", "Level", "OnDone", "Service", "Skill",
    "SkillStep", "Snapshot", "VideoRecorder",
    "brake", "brake_and_settle", "capture", "fly_to", "fly_to_ned",
    "helix", "land", "orbit", "takeoff", "yaw_to",
    "FlyToGlobalSkill", "GlobalPositionSense", "GoToGlobalMode",
    "GoToGlobalParams", "GoToGlobalRequest", "geodetic_delta_to_ned",
]


def _boot_drone():
    raise RuntimeError("boot_drone() needs the app's ROS stack")


stub = types.ModuleType("local_planner")
for _name in _EXPORTS:
    setattr(stub, _name, getattr(skytrack_autonomy, _name))
stub.boot_drone = _boot_drone
sys.modules.setdefault("local_planner", stub)
