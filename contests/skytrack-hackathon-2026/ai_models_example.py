"""AImodels = [det-coco-v26n-b-quantized-fp16, det-visdrone-v26n-b-quantized-fp16, det-firesmoke-v26n-b-quantized-fp16]"""

# ══════════════════════════════════════════════════════════════════════
# AI models example: run several detection models from a Python mission.
#
# Line 1 must list every model the mission uses, in this exact form:
#     """AImodels = [model1_name, model2_name, model3_name]"""
#
# 1. Take off and fly to a viewpoint; hover so the picture is sharp.
# 2. Run each model in AI_MODELS one after the other (the detector
#    serves one request at a time) and wait for its answer.
# 3. Log every box: class, score and pixel position.
# 4. Return home and land.
#
# Boxes are in image pixels, not world coordinates. To turn a box into a
# ground position, see "Pixel to ground" in spray_drone_specs.md.
#
# Run::
#
#     python -m local_planner.examples.ai_models_example
# ══════════════════════════════════════════════════════════════════════
from __future__ import annotations

from typing import Any, Iterator

from local_planner import SkillStep, boot_drone, brake, fly_to, land, takeoff
from skytrack_autonomy import Detector

# Model name → classes to ask for. The names must be the model's dataset
# classes (see /opt/skytrack/ai/mapping.json in the drone container).
AI_MODELS = {
    "det-coco-v26n-b-quantized-fp16": ["person"],
    "det-visdrone-v26n-b-quantized-fp16": ["pedestrian", "people"],
    "det-firesmoke-v26n-b-quantized-fp16": ["fire", "smoke"],
}
CONFIDENCE = 0.4
DETECT_TIMEOUT_S = 15.0
ALT_M = 8.0
VIEWPOINT = (10.0, 0.0)             # (north, east)
TAG = "[AI]"


def detect(ctx: Any, model: str, classes: list[str]):
    """Run one detection and hover until it answers.

    Use with ``yield from``; returns the outcome, or ``None`` if the
    request was refused, failed or timed out.
    """
    detector = ctx.services.detector
    before = detector.count
    if not detector.request(model_name=model, classes=classes,
                            confidence_threshold=CONFIDENCE):
        ctx.world.log_warn(f"{TAG} {model}: request refused")
        return None
    deadline = ctx.world.now() + DETECT_TIMEOUT_S
    yield SkillStep(
        skill=brake(name=f"detect_{model}").skill,
        is_done=lambda c: detector.count > before or c.world.now() >= deadline,
        name=f"detect_{model}",
    )
    outcome = detector.last_result
    if detector.count == before or outcome is None or not outcome.success:
        detector.cancel()
        ctx.world.log_warn(f"{TAG} {model}: no result")
        return None
    return outcome


def ai_models_mission(ctx: Any) -> Iterator[Any]:
    """Fly to the viewpoint, run every model, log what each one sees."""
    log = ctx.world.log_info

    yield takeoff(alt_m=ALT_M)
    yield fly_to(north=VIEWPOINT[0], east=VIEWPOINT[1], alt_m=ALT_M, name="to_viewpoint")
    yield brake(name="settle")

    for model, classes in AI_MODELS.items():
        outcome = yield from detect(ctx, model, classes)
        if outcome is None:
            continue
        log(f"{TAG} {model}: {outcome.num_detections} box(es) "
            f"in {outcome.inference_time_ms or 0:.0f} ms")
        for det in outcome.detections:
            log(f"{TAG}   {det.class_name} {det.score:.0%} at pixel "
                f"({det.bbox.center_x:.0f}, {det.bbox.center_y:.0f})")

    yield fly_to(north=0.0, east=0.0, alt_m=ALT_M, name="return_home")
    yield brake(name="pre_land")
    yield land()


ai_models_mission.requires_senses = ["pose", "obstacle", "status"]


def main() -> None:
    with boot_drone() as drone:
        drone.add_service(Detector())   # model and classes are given per request
        drone.fly(ai_models_mission)
        drone.run()


if __name__ == "__main__":
    main()
