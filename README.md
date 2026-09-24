# skytrack-autonomy examples

[![Python](https://img.shields.io/badge/python-3.10%2B-blue)](pyproject.toml)
[![License](https://img.shields.io/badge/license-Apache--2.0-green)](LICENSE)

Mission examples for
`skytrack-autonomy`, from
a two-line takeoff-and-land to a complete site survey, plus guides for
writing your own senses, skills, services and modes.

Every example is a mission file in the format the app runs
(`local_planner/examples/*_mission.py`): one generator function, a
`requires_senses` list, and a `main()` that boots the drone with
`boot_drone()` and calls `drone.run()`.

```python
from local_planner import boot_drone, brake, land, takeoff

def hello_mission(ctx):
    yield takeoff(alt_m=3.0)
    yield brake(name="hover")
    yield land()

hello_mission.requires_senses = ["pose", "status"]

def main() -> None:
    with boot_drone() as drone:
        drone.fly(hello_mission)
        drone.run()
```

> [!WARNING]
> These missions fly a real or simulated aircraft. Run them in SITL
> first, keep an RC pilot ready, and follow your local aviation rules.

## Running the examples

### In the app

Put the file in the app's `local_planner/examples/` folder and run it
either way:

```bash
python -m local_planner.examples.hello_mission          # as a module
python3 /path/to/local_planner/examples/hello_mission.py  # as a script
```

`site_survey_mission.py` uses the custom parts from the three Level 5
files (`custom_sense_mission.py`, `custom_skill_mission.py`,
`custom_service_mission.py`), so keep those in the same folder.

### In the simulation container

In the `skytrack-simulation-skytrack-autonomy-1` container the installed
package folder is **read-only**, so copy the examples to `/tmp` and run
them from there:

```bash
C=skytrack-simulation-skytrack-autonomy-1
docker exec $C mkdir -p /tmp/sae
for f in examples/*.py; do docker cp "$f" "$C:/tmp/sae/"; done

docker exec -it $C bash -c 'source /app/setup.sh && cd /tmp && python3 -m sae.hello_mission'
```

A mission returns by itself after it lands and disarms. The one
exception is `custom_mode_mission`, which waits for more commands until
you press Ctrl-C.

> [!CAUTION]
> Killing a mission mid-flight (Ctrl-C, `timeout`) leaves the drone in
> the air with no setpoints. Land it before starting the next one.

Before running a camera example, check the drone has a camera:
`ros2 topic hz /camera` should show frames (about 3 Hz in simulation).

### Where the results go

| What | Where |
|---|---|
| Mission report (every step, with outcome) | `~/.ros/recordings/mission_raw_mission_<time>.json` |
| Photos (`capture`, `Snapshot`) | `~/.ros/captures/` |
| Video frames (`VideoRecorder`) | `~/.ros/recordings/<clip>/` (PNG frames + `meta.jsonl`) |
| Telemetry CSV (`custom_service_mission`) | `~/.ros/telemetry/flight_<time>.csv` |
| Node log | `~/.ros/log/local_planner/` |

The quickest success check is the report's last event:
`MISSION_END` with `final_status: "Succeeded"`. Each step also has a
`SKILL_STARTED` / `SKILL_COMPLETED` pair named after its `name=`.

## Examples, simple to advanced

| Level | Example | What you learn |
|---|---|---|
| **1 · Basics** | [hello_mission.py](examples/hello_mission.py) | The file shape: mission, `requires_senses`, `main()`. Take off and land. |
| | [square_mission.py](examples/square_mission.py) | `fly_to(north=, east=, alt_m=)`, step names. |
| | [waypoints_mission.py](examples/waypoints_mission.py) | Route as data, `for` loops, `target_speed`, logging. |
| **2 · Flight patterns** | [orbit_helix_mission.py](examples/orbit_helix_mission.py) | Circle a point; spiral up. |
| | [heading_control_mission.py](examples/heading_control_mission.py) | `yaw_mode`, `yaw_rate_deg_s`, `yaw_to`. |
| | [lawnmower_mission.py](examples/lawnmower_mission.py) | Generated sweep lines, `mode="coverage"`, `replan_mode="fast"`. |
| **3 · Mission logic** | [compose_mission.py](examples/compose_mission.py) | Reusable sub-missions with `yield from`. |
| | [battery_check_mission.py](examples/battery_check_mission.py) | Read pose / status / battery; branch; return home early. |
| **4 · Camera & services** | [capture_photos_mission.py](examples/capture_photos_mission.py) | `CameraSense`, `yaw_to` → `brake` → `capture`. |
| | [record_video_mission.py](examples/record_video_mission.py) | `VideoRecorder` + `Snapshot` services. |
| | [spray_mission.py](examples/spray_mission.py) | `Sprayer`; hover until the valve confirms with a `SkillStep`. |
| | [detect_mission.py](examples/detect_mission.py) | `Detector`; wait for the result; react to it. |
| **5 · Build your own** | [custom_sense_mission.py](examples/custom_sense_mission.py) | Write a **sense**: `GeofenceSense`. |
| | [custom_skill_mission.py](examples/custom_skill_mission.py) | Write **skills**: `HoverForSeconds`, `ClimbBy`. |
| | [custom_service_mission.py](examples/custom_service_mission.py) | Write a **service**: CSV `TelemetryLogger`. |
| | [custom_mode_mission.py](examples/custom_mode_mission.py) | Write a **mode** + **command**, triggered at any time (optionally from chat). |
| **6 · Putting it together** | [site_survey_mission.py](examples/site_survey_mission.py) | Sweep + video + photos + geofence + telemetry + battery return. |

Every file starts with a docstring saying what it teaches, what it
requires (camera, spray valve) and how to run it.

## Tested in simulation

All examples were run on 2026-09-22 in the SkyTrack simulation stack
(Gazebo + PX4 SITL + the app's `local_planner`), one after another.

| Example | Result | Time | Notes |
|---|---|---|---|
| hello | ✅ Succeeded | — | exited by itself after landing |
| square | ✅ Succeeded | 1 min | |
| waypoints | ✅ Succeeded | 1 min 9 s | 4/4 waypoints |
| orbit_helix | ✅ Succeeded | 1 min 16 s | |
| heading_control | ✅ Succeeded | 1 min 12 s | all 3 heading policies + `yaw_to` |
| lawnmower | ✅ Succeeded | 2 min 17 s | 5 sweep lines |
| compose | ✅ Succeeded | 1 min 31 s | |
| battery_check | ✅ Succeeded | 1 min 11 s | battery read 99 → 95 % |
| capture_photos | ✅ Succeeded | 1 min 45 s | camera drone; 3 photos |
| record_video | ✅ Succeeded | 1 min 10 s | camera drone; 56 frames, 2 stills |
| detect | ✅ Succeeded | 1 min 22 s | camera drone; 2 detections run, 0 hits (empty world) |
| spray | ⏸ Not tested | | needs a drone with a spray valve |
| custom_sense | ✅ Succeeded | 59 s | skipped the 2 waypoints outside the fence |
| custom_skill | ✅ Succeeded | 1 min 4 s | |
| custom_service | ✅ Succeeded | 1 min 15 s | 217 CSV rows with all notes |
| custom_mode | ✅ Completed | ~1 min flight | submitted command → mode → 2 laps → landed |
| site_survey | ✅ Succeeded | 3 min 22 s | camera drone; 26 steps, 274 frames, 3 photos |

Problems found during testing are in
[docs/known-issues.md](docs/known-issues.md).

## Documentation

| Doc | What's in it |
|---|---|
| [docs/catalog.md](docs/catalog.md) | Everything that ships: mission steps, skills, senses, services, modes, commands, world adapters |
| **Guides** | One per building block: how it works, the built-ins, how to write your own, pitfalls, testing |
| · [docs/guides/skills.md](docs/guides/skills.md) | Motion primitives: lifecycle, setpoint loop, schedule groups, custom finish conditions |
| · [docs/guides/senses.md](docs/guides/senses.md) | Read-only state: senses in the app, reading safely, `requires_senses` |
| · [docs/guides/services.md](docs/guides/services.md) | Background jobs: lifecycle, recorder / snapshot / detector / sprayer, async requests |
| · [docs/guides/modes.md](docs/guides/modes.md) | Triggerable behaviours: Supervisor priorities, commands, chat tools |
| **Walkthroughs** | Example files explained step by step, with what happened in simulation |
| · [hello_mission](docs/walkthroughs/hello_mission.md) | The anatomy of a mission file |
| · [lawnmower_mission](docs/walkthroughs/lawnmower_mission.md) | Generating a route; transit vs coverage legs; what leg timings tell you |
| · [detect_mission](docs/walkthroughs/detect_mission.md) | Waiting for an asynchronous answer, with a timeout |
| · [custom_skill_mission](docs/walkthroughs/custom_skill_mission.md) | Writing skills; measured accuracy of time- and position-based skills |
| · [site_survey_mission](docs/walkthroughs/site_survey_mission.md) | Structuring a real job: phases, safety checks, services, reuse |
| [docs/build-your-own.md](docs/build-your-own.md) | The contract and rules for your own sense, skill, service, mode or world adapter |
| [docs/known-issues.md](docs/known-issues.md) | Problems found while testing, what they affect (app or not) and the workarounds |

## The file shape, in one checklist

- [ ] Docstring: one-line summary, level, what you learn, `Requires` (if it needs a camera or payload), `Run::` command
- [ ] `from __future__ import annotations`
- [ ] Imports from `local_planner`; extras (`Sprayer`, `Detector`, scheduling) from `skytrack_autonomy`
- [ ] Settings as `UPPER_CASE` constants at the top
- [ ] Mission `def <name>(ctx: Any) -> Iterator[Any]:` with the same name as the file
- [ ] Every step has a clear `name=`; `brake()` before `capture()` and `land()`
- [ ] `<name>.requires_senses = [...]` (always `"pose"`; `"obstacle"` if using `fly_to`; `"camera"` for imaging)
- [ ] `main()`: `with boot_drone() as drone:` → senses → services → modes → `drone.fly()` → `drone.run()`
- [ ] `if __name__ == "__main__": main()`

## Tests

The tests don't need ROS: they replace `local_planner` with a stand-in
that re-exports the framework, check every file against the checklist,
step through every mission on a fake drone, and unit-test the custom
parts.

```bash
pip install -e ../skytrack-autonomy   # the framework, from a local checkout
pip install -e ".[dev]"
pytest
```

## License

Apache-2.0, the same as skytrack-autonomy.
