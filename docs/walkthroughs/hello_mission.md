# Walkthrough: `hello_mission.py`

**Level 1 · Basics** — [source](../../examples/hello_mission.py)

The smallest complete mission. It does almost nothing — take off, hover,
land — which makes it the best file to learn the **shape** every other
example follows. Once this layout is familiar, every other file is
"the same, with more steps".

---

## The file, top to bottom

### 1. Docstring

```python
"""Hello mission — take off, hover, land. The smallest complete mission.

Level 1 · Basics

What you learn
==============
* ...

Run::

    python -m local_planner.examples.hello_mission
"""
```

The first line is what shows up in listings. The `Run::` line is the
exact command; the tests check that it matches the file name.

### 2. Imports

```python
from __future__ import annotations

from typing import Any, Iterator

from local_planner import boot_drone, brake, land, takeoff
```

- `from __future__ import annotations` — required by the app's
  checklist; lets type hints refer to anything without import-order
  worries.
- Everything a mission author needs comes from **`local_planner`**: the
  step helpers (`takeoff`, `brake`, `land`) and `boot_drone`.

### 3. Settings

```python
ALT_M = 3.0          # takeoff altitude, metres above home (positive up)
```

Numbers you might tune live at the top as `UPPER_CASE` constants, with
their unit. Nobody should have to read the mission body to change the
altitude.

### 4. The mission

```python
def hello_mission(ctx: Any) -> Iterator[Any]:
    """Climb to 3 m, settle, land."""
    yield takeoff(alt_m=ALT_M)
    ctx.world.log_info("[HELLO] airborne")
    yield brake(name="hover")
    yield land()
```

This is the whole idea of the framework:

| Line | What happens |
|---|---|
| `def hello_mission(ctx)` | A mission is a **generator function**. `ctx` gives access to senses, services and the world. The function name matches the file name. |
| `yield takeoff(alt_m=ALT_M)` | Hands a step to the runtime and **pauses** here until the drone has reached altitude. |
| `ctx.world.log_info(...)` | Ordinary code between steps runs instantly, after the previous step finished. Tag log lines (`[HELLO]`) so you can grep for them. |
| `yield brake(name="hover")` | Hold position until the drone is still. `name=` labels the step in logs and the report. |
| `yield land()` | Land and wait for disarm. When the generator returns, the mission is over. |

Why `brake()` before `land()`? PX4 prefers to receive the land command
from a hover rather than while moving. Every example ends with
`brake` → `land`.

### 5. Required senses

```python
hello_mission.requires_senses = ["pose", "status"]
```

Checked before the mission starts. This mission only takes off and
lands, so it needs `pose` and `status`; anything with `fly_to` also
needs `"obstacle"`.

### 6. `main()`

```python
def main() -> None:
    with boot_drone() as drone:
        drone.fly(hello_mission)
        print(f"[hello_mission] modes:  {drone.list_modes()}")
        print(f"[hello_mission] senses: {drone.list_senses()}")
        drone.run()


if __name__ == "__main__":
    main()
```

| Line | What happens |
|---|---|
| `with boot_drone() as drone` | Starts ROS and the planner node, gives you a `Drone`. On exit, shuts ROS down cleanly — even after an error. |
| `drone.fly(hello_mission)` | Registers the mission as a mode (`fly:hello_mission`) and arms it to start after a short delay. |
| `print(... list_modes / list_senses)` | A sanity check in the console. |
| `drone.run()` | Runs everything. **Returns by itself** once the mission has finished and the drone is disarmed. |

---

## What happened in simulation

Running `python3 -m sae.hello_mission` in the simulation container
printed:

```
[hello_mission] modes:  ['emergency_stop', 'canceled', 'paused', 'takeoff',
  'smart_land', 'orbit', 'helix', 'panorama', 'goto_global',
  'fly:hello_mission', 'navigate', 'idle']
[hello_mission] senses: ['battery', 'camera', 'global_position',
  'landing_spot', 'obstacle', 'point_cloud', 'pose', 'status']
...
[FLY:HELLO_MISSION] Program complete
[Supervisor] Mode fly:hello_mission → idle (idle_active)
[Drone] mission complete and disarmed — exiting
```

Two things worth noticing:

- Your mission sits in the mode list as `fly:hello_mission`, **below**
  the safety modes. An emergency stop always wins.
- The app already registers eight senses, including `camera` — you
  only add senses the app doesn't have.

---

## Try it yourself

1. Change `ALT_M` to `5.0` and watch the takeoff step take longer.
2. Add `yield orbit(center_north=3, center_east=0, alt_m=ALT_M, radius_m=2, duration_s=10)`
   before the `brake` (import `orbit` from `local_planner`).
3. Remove `"status"` from `requires_senses` — does it still start? Why
   is it still a good idea to list it?

**Next:** [lawnmower_mission](lawnmower_mission.md) — generating a route in code.
