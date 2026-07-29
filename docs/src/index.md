# ElectricMachineWinch.jl

```@meta
CurrentModule = ElectricMachineWinch
```

`ElectricMachineWinch.jl` is a *bridge* package for testing detailed
induction-machine electric drives as winch models in the
KiteSimulators/KiteModels ecosystem. It provides a winch type compatible with
`WinchModels`,

```julia
DetailedIMWinch <: WinchModels.AbstractWinchModel
```

and connects it to interchangeable electric-drive controllers built from the
discrete induction-machine blocks of
[`IM_AWES_bench`](https://github.com/c-nicomar/IM_AWES_bench.jl). The package
contains almost no control math of its own — the controllers here are thin
adapters around `IM_AWES_bench` blocks.

## The layering rule

The whole reason this package exists is a strict split of ownership:

**KiteModels owns the mechanical state, this package owns only the electrical
state.**

- **KiteModels** owns reel-out speed, tether mechanics, and the equivalent winch
  mechanical state.
- **ElectricMachineWinch** owns the electric-drive controller, observer, and
  induction-machine electrical states.
- **IM_AWES_bench** provides the validated controller, observer, and
  machine-model building blocks.

The machine speed is therefore *imposed* from the measured reel-out speed and
never integrated here, and the tether force is converted into an equivalent
machine-side load torque:

```julia
ωm    = gear_ratio / drum_radius * v_reelout
Tload = drum_radius / gear_ratio * tether_force
```

Only the electrical machine states are integrated internally, by
[`rk4_step_electrical_only`](@ref). This avoids introducing a second
mechanical-speed state in parallel with the state already owned by KiteModels.

## Sign convention

The machine-side mechanical model uses:

```text
J * dωm/dt = Te + Tload - B * ωm
```

with:

```julia
Tload = drum_radius / gear_ratio * tether_force
ωm    = gear_ratio / drum_radius * v_reelout
```

For positive-speed generation, the typical signs are:

```text
v_reelout > 0
Tload > 0
Te < 0
```

A positive load torque pulls the winch toward positive reel-out speed. The
load-feedforward sign must be consistent with this convention; the MTPA
constructor exposes `load_ff_sign`, with the current default:

```julia
load_ff_sign = -1.0
```

When a test or a run shows the speed going the wrong way, check this convention
before suspecting the controller.

## Architecture

```text
KiteControllers autopilot
        │
        │ reel-out speed reference
        ▼
KiteModels / KPS4
        │
        │ v_ro_set, v_ro_meas, tether_force
        ▼
ElectricMachineWinch.DetailedIMWinch
        │
        ├── IdealTorqueController
        │
        ├── FOCSpeedF1Controller
        │   └── IM_AWES_bench F1 outer speed/flux loop
        │
        └── FOCSpeedMTPAController
            └── IM_AWES_bench constrained-MTPA outer speed loop
        │
        ▼
electromagnetic torque
        │
        ▼
KiteModels mechanical update
```

All drive controllers subtype [`AbstractIMDriveController`](@ref) and implement
[`drive_step!`](@ref), which returns the same [`DriveStepOutput`](@ref) for
every controller. This keeps `DetailedIMWinch` and the KiteModels interface
independent of the selected controller.

## Controllers

| Symbol | Type | When to use it |
| --- | --- | --- |
| `:ideal` | [`IdealTorqueController`](@ref) | First integration test. A speed PI plus a first-order torque actuator, with no electrical plant at all — use it to debug the KiteModels coupling before adding observer, current controller and machine model. |
| `:foc_speed_f1` | [`FOCSpeedF1Controller`](@ref) | The reference detailed drive. Full discrete FOC with the F1 flux strategy, and the only controller that currently supports field weakening. |
| `:foc_speed_mtpa` | [`FOCSpeedMTPAController`](@ref) | Energy-optimal current allocation with practical constraints (flux floor, torque reserve, current circle). No field weakening yet. |

See [Controllers](controllers.md) for the details and the full parameter sets.

## Quickstart

The package targets Julia 1.12 and depends on `IM_AWES_bench` and
`WinchModels`. `IM_AWES_bench` is unregistered and is resolved from the git URL
declared in `[sources]` in `Project.toml`, so a plain checkout needs no manual
setup:

```bash
julia --project -e 'using Pkg; Pkg.instantiate()'
```

Start a REPL in the package environment (`Revise` is loaded if available) with:

```bash
bin/run_julia
```

Build a winch and inspect its last-step values:

```julia
using ElectricMachineWinch

wm = make_electric_winch(
    controller = :ideal,
    drum_radius = 0.5,
    gear_ratio = 10.0,
    J_eq = 0.3685,
    B_eq = 0.01298,
    Ts = 1e-3,
    Te_max = 124.0,
)

last_summary(wm)
```

Run the test suite:

```bash
julia --project=test test/runtests.jl
```

## Contents

```@contents
Pages = ["getting_started.md", "controllers.md", "integration.md",
         "diagnostics.md", "extending.md"]
Depth = 2
```

## API

```@contents
Pages = ["api/winch.md", "api/controllers.md", "api/plant.md"]
Depth = 2
```
