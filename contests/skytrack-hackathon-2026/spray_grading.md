# How spraying is graded

How a spray run is turned into a score. The drone, camera and nozzle are described in the spec
document; this one only covers what happens to your run after it lands.

## What needs spraying

The **target** is the stressed crop inside the survey area assigned to your
mission. No-fly zones are cut out of it: you are never asked to treat crop you are not allowed to
fly over. Stressed crop outside your survey area is not part of your target, and neither is healthy
crop, bare ground, water or anything else.

Every target cell counts, including the ones you never reached. A cell you missed entirely is
scored as a cell with zero dose, not left out.

## The dose band

> ### ⚠️ Aim for 1.0 – 3.0 ml/m² on every target cell
>
> - **Below 1.0 ml/m², a cell counts as untreated** — however much it got.
> - **Above 3.0 ml/m², the extra is wasted** — and it costs you score.

## The metrics

All metrics are fractions from 0 to 1 (0.25 means 25 %).

| Metric | Question it answers | Better |
|---|---|---|
| **Coverage** | Of the target cells, what share got at least 1.0 ml/m²? | higher |
| **Under-dose** | Across all target cells, how far short of 1.0 ml/m² were they, on average? | lower |
| **Precision** | Of every cell that got *any* spray, what share was a target cell? | higher |
| **Off-target waste** | Of the liquid that reached the ground, what share landed outside the target? | lower |
| **Over-dose** | Across all target cells, how far above 3.0 ml/m² were they, on average? | lower |

A few things worth knowing about them:

- **Coverage is all-or-nothing per cell.** A cell at 0.95 ml/m² scores the same as a cell you never
  flew over. **Under-dose** is its softer partner: it gives partial credit for near-misses, measured
  as the average shortfall below 1.0 ml/m², as a fraction of 1.0. A target cell with no spray at
  all contributes its full shortfall.
- **Precision counts cells, not liquid.** A single stray drop on a cell outside the target makes
  that cell "sprayed", so leaving the valve open on transit legs and turns hurts it quickly.
- **Off-target waste counts liquid, not cells.** A few cells soaked during a hover next to the
  field can waste more than a light scatter over a wide area. Precision and waste catch the two
  different ways of spraying the wrong ground, so both are used.
- **Over-dose only looks at target cells.** Excess on off-target ground is already counted as
  off-target waste.

## The final score

Runs are ranked by one number, built from the metrics above:

```
base      = 0.7 · coverage + 0.3 · (1 − under_dose)

off_target = 0.5 · (1 − precision) + 0.5 · min(off_target_waste, 1)
overdose   = min(over_dose, 1)

final_score = base · (1 − 0.6 · off_target) · (1 − 0.3 · overdose)
```

- **The base rewards treating the target.** Mostly coverage, with some credit for coming close.
- **The penalties multiply, they don't subtract.** Great coverage cannot buy back careless
  spraying. Spraying the wrong ground can take away up to 60 % of your base; over-dosing the
  right ground, up to 30 %.
- **Spraying the wrong ground is punished harder than over-dosing the right ground.**

**Example** (illustrative numbers, not a real run): coverage 0.60, under-dose 0.30, precision 0.85,
off-target waste 0.10, over-dose 0.05.

```
base        = 0.7 · 0.60 + 0.3 · 0.70                = 0.630
off_target  = 0.5 · 0.15 + 0.5 · 0.10                = 0.125  → × (1 − 0.075) = × 0.925
overdose    = 0.05                                    → × (1 − 0.015) = × 0.985
final_score = 0.630 · 0.925 · 0.985                  ≈ 0.574
```
