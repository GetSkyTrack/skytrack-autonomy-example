# Modes

A **mode** is a program the drone can switch to. The **Supervisor**
keeps a priority-ordered list of modes and, whenever something changes,
runs the highest-priority one that wants control. Emergency stop beats
landing, landing beats your mission, your mission beats "idle".

You've already used modes without writing one: `drone.fly(mission)`
wraps your mission in a one-shot mode called `fly:<mission name>`.
Write a mode yourself when a behaviour must be **triggerable at any
time** — by an operator button, a command, the chat assistant, or
another part of your code.

**On this page:** [Missions vs modes](#missions-vs-modes) ·
[How the Supervisor picks a mode](#how-the-supervisor-picks-a-mode) ·
[Modes in the app](#modes-in-the-app) · [Writing a
mode](#writing-a-mode) · [Commands](#commands) ·
[Pitfalls](#pitfalls) · [Testing](#testing)

---

## Missions vs modes

| | `drone.fly(mission)` | `drone.add_mode(Mode, gate=...)` |
|---|---|---|
| Runs | Once, shortly after boot | Whenever its gate opens |
| Triggered by | Itself (auto-armed) | A command, a flag, the chat tool |
| Parameters | Constants in the file | Passed in the command (`gate_params`) |
| `drone.run()` returns | After it lands and disarms | Never on its own — use `run(exit_on_complete=False)` and Ctrl-C |
| Example | every `*_mission.py` | [custom_mode_mission.py](../../examples/custom_mode_mission.py) |

Both are written the same way inside: a generator that yields steps.

---

## How the Supervisor picks a mode

Each mode has a **priority** and a **gate** (a condition). The
Supervisor elects the highest-priority mode whose gate is open:

```
priority  mode                 gate open when…
────────  ───────────────────  ─────────────────────────────
SAFETY    emergency_stop       EmergencyStop(active=True) submitted
SAFETY    canceled             Cancel() submitted
SAFETY    paused               Pause(active=True) submitted
PRELAUNCH takeoff              TakeoffRequest submitted
LANDING   smart_land           SmartLandRequest submitted
MISSION   orbit/helix/panorama their *Request commands
MISSION   goto_global          GoToGlobalRequest
MISSION   fly:<your mission>   armed by drone.fly()
MISSION   <your modes>         ctx.flags.<your gate> is True
NAVIGATE  navigate             a navigation target is set
FALLBACK  idle                 always
```

Within one tier, the Supervisor gives each mode its own slot in
registration order.

Arbitration re-runs:

1. when a command is submitted (`drone.submit(...)`),
2. when code calls `ctx.notify_state_change()`,
3. at the end of every `DECISION` tick while something is running.

When a higher-priority mode takes over, the current mode's `on_exit`
runs: its active skill is **cancelled** and `on_teardown()` is called.
That's how an emergency stop interrupts any mission instantly.

---

## Modes in the app

The app's `boot_drone()` registers these (from `drone.list_modes()` in
simulation):

```
emergency_stop, canceled, paused, takeoff, smart_land, orbit, helix,
panorama, goto_global, fly:<your mission>, navigate, idle
```

> [!WARNING]
> `boot_drone_for_world(world)` registers only `idle`. Outside the app,
> `EmergencyStop` / `Pause` / `Cancel` therefore **do not stop a
> mission**. See [known issues](../known-issues.md#safety-commands-dont-stop-a-mission).

---

## Writing a mode

Three pieces: the parameters, a command that opens the gate, and the
mode itself. From [custom_mode_mission.py](../../examples/custom_mode_mission.py):

```python
@dataclass(frozen=True)
class InspectParams:                       # 1. what the behaviour needs
    north: float = 6.0
    east: float = 0.0
    alt_m: float = 4.0
    laps: int = 1


@dataclass(frozen=True)
class InspectRequest(Command):             # 2. the trigger
    active: bool = True
    params: InspectParams = InspectParams()

    def apply(self, ctx):
        if self.active:
            ctx.inspect_params = self.params
            ctx.flags.inspect_requested = True     # open the gate
        else:
            ctx.flags.inspect_requested = False
            ctx.inspect_params = None


class InspectMode(ControlMode):            # 3. the behaviour
    requires_senses = ["pose", "obstacle", "status"]

    @property
    def id(self):
        return "inspect"

    def program(self, ctx):
        p = ctx.inspect_params
        yield takeoff(alt_m=p.alt_m)
        yield orbit(center_north=p.north, center_east=p.east, alt_m=p.alt_m,
                    radius_m=3.0, period_s=15.0, duration_s=15.0 * p.laps)
        yield brake()
        yield land()
```

Register and trigger in `main()`:

```python
drone.add_mode(InspectMode, gate="inspect_requested",
               gate_params="inspect_params", on_done=OnDone.LAND_AND_HOLD)
drone.submit(InspectRequest(params=InspectParams(laps=2)))
drone.run(exit_on_complete=False)
```

In simulation this went `idle → inspect`, took off, flew 2 laps
(30 s), landed and returned to `idle`.

### Members

| Member | Required | Purpose |
|---|---|---|
| `id` | yes | Unique name, shown in logs (`Mode idle → inspect`). |
| `program(ctx)` | usually | Generator of steps — same as a mission. |
| `requires_senses` | recommended | Checked when the mode is registered. |
| `on_setup(supervisor)` | no | Once on entry, before `program`. **Gets the supervisor, not `ctx`** — use `self._ctx`. |
| `on_teardown()` | no | Once on exit, after the skill is cancelled. |
| `on_complete()` | no | Once when the program finishes by itself (not when pre-empted). |

### `add_mode` options

| Option | Meaning |
|---|---|
| `gate="inspect_requested"` | Boolean on `ctx.flags`; created as `False`. |
| `gate_params="inspect_params"` | Attribute on `ctx` holding the parameters. Cleared together with the gate when the program finishes. |
| `level=Level.MISSION` | Default. `Level.BACKGROUND` for helpers that should only run when nothing else wants to. Other levels are reserved for built-in safety modes. |
| `beats=` / `loses_to=` | Order relative to another mode class instead of a level. |
| `on_done=OnDone.IDLE` | Default: stay where you are (hovering, if airborne). |
| `on_done=OnDone.LAND_AND_HOLD` | For modes that end on the ground: stay down until the next command. |

---

## Commands

A `Command` is a frozen dataclass with one method, `apply(ctx)`, which
changes flags and parameters. The Supervisor applies it and
re-arbitrates immediately.

```python
drone.submit(InspectRequest(params=InspectParams(laps=2)))   # from code
```

Built-in commands (import from `skytrack_autonomy.core.commands`):
`EmergencyStop`, `Cancel`, `Pause`, `TakeoffRequest`, `SmartLandRequest`,
`FlyTo`, `OrbitRequest`, `HelixRequest`, `PanoramaRequest`,
`GoToGlobalRequest`. Fields: [catalog.md](../catalog.md#commands).

### Letting the chat assistant trigger a mode

Because a command is a typed dataclass, the app can offer it to the
chat assistant as a tool:

```python
drone.expose_chat_tool(
    name="inspect",
    description="Circle a point and come back.",
    command=InspectRequest,
)
drone.start_chat()
```

The app's `chatbot_example_mission.py` does exactly this with a patrol.

---

## Pitfalls

| Mistake | What happens | Fix |
|---|---|---|
| Setting `ctx.flags.<gate> = True` while idle | Nothing — no one re-arbitrates | Submit a `Command`, or call `ctx.notify_state_change()` |
| `on_setup(self, ctx)` using `ctx.world` | `AttributeError`: the argument is the supervisor | `self._ctx.world` |
| Mode program without `takeoff` when triggered on the ground | Steps run against a landed drone | Start with `takeoff` (or check `ctx.senses.status.is_armed`) |
| `drone.run()` with only modes | Default `run()` waits for `fly()` missions to finish, and there are none | `drone.run(exit_on_complete=False)` |
| Choosing `Level.SAFETY` for your mode | `ValueError` | Use `MISSION` or `BACKGROUND` |
| Long blocking work in `program` between yields | Freezes the decision thread | Put it in a service; wait with a `SkillStep` |

---

## Testing

Apply the command to a fake context and step through the program:

```python
ctx = make_fake_ctx()
ctx.flags.inspect_requested = False
InspectRequest(params=InspectParams(laps=2)).apply(ctx)
assert ctx.flags.inspect_requested

mode = InspectMode(ctx=ctx)
skills = [type(s.skill).__name__ for s in mode.program(ctx)]
assert skills[0] == "TakeoffSkill" and skills[-1] == "LandSkill"
```

See `test_custom_mode_program_and_command` in
[tests/test_examples.py](../../tests/test_examples.py).
