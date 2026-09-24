# Walkthrough: `lawnmower_mission.py`

**Level 2 · Flight patterns** — [source](../../examples/lawnmower_mission.py)

Sweep a rectangle in parallel lines, like mowing a lawn. The pattern
behind spraying a field, mapping an area or searching for something.
The interesting part isn't the flying — it's that **the route is
computed**, and that each line uses a different flight style from the
moves between lines.

---

## 1. Settings describe the job, not the route

```python
ALT_M = 4.0
AREA_NORTH_M = 12.0          # length of each sweep line
AREA_EAST_M = 8.0            # total width covered
LINE_SPACING_M = 2.0         # distance between sweep lines
SWEEP_SPEED_M_S = 1.5
```

There's no list of waypoints. You describe the **area** and the
**spacing**; code turns that into a route. Change `LINE_SPACING_M` to
match your sprayer's swath or camera footprint and the route follows.

## 2. Generating the route

```python
def sweep_lines() -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
    """(start, end) of each line, alternating direction (boustrophedon)."""
    lines = []
    east = 0.0
    flip = False
    while east <= AREA_EAST_M + 1e-6:
        start, end = (0.0, east), (AREA_NORTH_M, east)
        lines.append((end, start) if flip else (start, end))
        east += LINE_SPACING_M
        flip = not flip
    return lines
```

| Detail | Why |
|---|---|
| A plain function, not part of the mission | It can be unit-tested without a drone — `test_lawnmower_alternates_line_direction` does exactly that. |
| `flip` alternates direction | Line 1 goes north, line 2 comes back south, … The drone never flies an empty return leg. |
| `+ 1e-6` | Floating-point safety: `0.0 + 2.0 × 4` must still count as `<= 8.0`. |

With the defaults this gives **5 lines** at east = 0, 2, 4, 6, 8 m:

```
 north 12 ┤  ↑   ↓   ↑   ↓   ↑
          │  │   │   │   │   │
          │  │   │   │   │   │
 north 0  ┤  ↑   ↓   ↑   ↓   ↑
          └──┴───┴───┴───┴───┴── east
             0   2   4   6   8
```

## 3. Two flight styles in one loop

```python
for i, (start, end) in enumerate(lines, start=1):
    # Get to the start of the line the quickest way.
    yield fly_to(north=start[0], east=start[1], alt_m=ALT_M,
                 name=f"line_{i}_start")
    # Then fly the line itself as straight as possible.
    yield fly_to(north=end[0], east=end[1], alt_m=ALT_M,
                 mode="coverage", replan_mode="fast",
                 target_speed=SWEEP_SPEED_M_S, name=f"line_{i}_sweep")
    ctx.world.log_info(f"{TAG} line {i}/{len(lines)} done")
```

| Leg | `mode` | Why |
|---|---|---|
| `line_N_start` (repositioning) | default `"transit"` | Shortest safe path; where exactly it goes doesn't matter. |
| `line_N_sweep` (the work) | `"coverage"` | Hug the straight line and only deviate around an obstacle. A transit path could cut corners and leave a gap in coverage. |

and on the sweep:

- `replan_mode="fast"` — replan while moving instead of stopping, so
  the line is one smooth pass.
- `target_speed=1.5` — slower than cruise, for even coverage. It
  applies to **this leg only**; the next `fly_to` is back at the
  default speed.

Step names include the line number (`line_3_sweep`), so the mission
report reads like a checklist.

## 4. Finish

```python
yield fly_to(north=0, east=0, alt_m=ALT_M, name="return_home")
yield brake(name="pre_land")
yield land()
```

The standard ending: fly home, settle, land.

---

## What happened in simulation

The mission **Succeeded** in 2 min 8 s of flight (2 min 17 s including
boot). All 14 steps completed in order:

```
takeoff(4.0m), line_1_start, line_1_sweep, line_2_start, line_2_sweep,
line_3_start, line_3_sweep, line_4_start, line_4_sweep, line_5_start,
line_5_sweep, return_home, pre_land, land
```

The same sweep pattern runs inside
[site_survey_mission](site_survey_mission.md), whose report has
per-step timings for 12 m lines at `target_speed=2.0`:

| Step | Time | Straight-line ideal |
|---|---|---|
| `line_N_start` (2.5 m sideways hop) | ~6.6 s | ~1 s at cruise |
| `line_N_sweep` (12 m) | 13.8–19.4 s | 6 s at 2 m/s |

**Reading the numbers:** a short `fly_to` is dominated by planning,
acceleration and the arrival check, not distance — a 2.5 m hop took
about half as long as a whole 12 m line. For missions with many short legs, fewer and
longer legs are much faster.

---

## Try it yourself

1. Set `LINE_SPACING_M = 4.0`. How many lines now? Update the expected
   count in `test_lawnmower_alternates_line_direction` and run
   `pytest -k lawnmower` to check.
2. Remove `mode="coverage"` from the sweep leg. In an empty world the
   paths should look similar; add an obstacle on a line and compare.
3. Make the area start somewhere else: add `ORIGIN = (5.0, -4.0)` and
   offset every point in `sweep_lines()`.

**Next:** [detect_mission](detect_mission.md) — waiting for something
that isn't flying.
