from unittest.mock import MagicMock
import sys

_EXPORTS = [
    "CameraSense", "ScheduleHandle", "CommandRequest", "CommandParams", "CaptureSkill", "Command", "ControlMode", "Drone",
    "FrameSink", "GStreamerSink", "Level", "OnDone", "Service", "Skill",
    "SkillStep", "Snapshot", "VideoRecorder",
    "brake", "brake_and_settle", "capture", "fly_to", "fly_to_ned",
    "helix", "land", "orbit", "takeoff", "yaw_to",
    "FlyToGlobalSkill", "GlobalPositionSense", "GoToGlobalMode",
    "GoToGlobalParams", "GoToGlobalRequest", "geodetic_delta_to_ned",
    "Sprayer", "Detector", "ScheduleGroup"
]

for name in _EXPORTS:
    if name in ["CommandRequest", "CommandParams"]:
        class FakeCommandClass: pass
        globals()[name] = FakeCommandClass
    elif name == "SkillStep":
        class SkillStep(MagicMock): pass
        globals()[name] = SkillStep
    elif name in ("takeoff", "land", "brake", "brake_and_settle", "fly_to"):
        def make_fake_skill(name):
            class FakeSkill:
                pass
            class_name = "".join(p.capitalize() for p in name.split("_")) + "Skill"
            FakeSkill.__name__ = class_name
            def wrapper(*args, **kwargs):
                s = SkillStep()
                s.skill = FakeSkill()
                return s
            wrapper.__name__ = name
            return wrapper
        globals()[name] = make_fake_skill(name)
    else:
        globals()[name] = MagicMock(name=name)



class ScheduleGroup:
    MEDIA = 1
    CONTROL = 2
    DEFAULT = 3

sys.modules["skytrack_autonomy"] = sys.modules[__name__]
sys.modules["skytrack_autonomy.core"] = sys.modules[__name__]
sys.modules["skytrack_autonomy.core.lib"] = sys.modules[__name__]
sys.modules["skytrack_autonomy.core.lib.scheduling"] = sys.modules[__name__]
sys.modules["skytrack_autonomy.core.skill_base"] = sys.modules[__name__]
sys.modules["skytrack_autonomy.core.testing"] = sys.modules[__name__]
sys.modules["skytrack_autonomy.core.testing.senses"] = sys.modules[__name__]


class FakeWorld:
    def now(self):
        return 0.0
    def log_info(self, msg):
        pass
    def on_local_position(self, cb):
        pass


class FakeCtx:
    def __init__(self, **kwargs):
        self.world = FakeWorld()
        for k, v in kwargs.items():
            setattr(self, k, v)
        self.senses = MagicMock()
        self.senses.pose.current_position = MagicMock()
        self.senses.pose.current_position.y = 0.0
        self.senses.pose.current_position.x = 0.0
        self.services = MagicMock()
        self.scheduler = MagicMock()

def make_fake_ctx(*args, **kwargs):
    return FakeCtx(**kwargs)


class FakeBatterySense:
    def __init__(self, percent):
        self.percent = percent

sys.modules["skytrack_autonomy.core.testing"].make_fake_ctx = make_fake_ctx
sys.modules["skytrack_autonomy.core.testing.senses"].FakeBatterySense = FakeBatterySense


class FakeClock: pass
sys.modules["skytrack_autonomy.core.testing"].FakeClock = FakeClock

import sys
import types
sys.modules["cv2"] = MagicMock(name="cv2")
sys.modules["cv2"].COLOR_BGR2RGB = 1
sys.modules["cv2"].COLOR_RGB2HSV = 2
sys.modules["cv2"].MORPH_OPEN = 3
sys.modules["cv2"].MORPH_CLOSE = 4
sys.modules["cv2"].RETR_EXTERNAL = 5
sys.modules["cv2"].CHAIN_APPROX_SIMPLE = 6


sys.modules["cv2"].findContours.return_value = ([], None)
