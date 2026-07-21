# Getting started with `ElectricMachineWinch.jl`

This document explains exactly what the generated bridge package does and how to
start testing it.

---

## 1. Why this package exists

`KiteSimulators.jl` should remain mostly untouched. The clean way to include your
electric machine model is to provide a new object that behaves like a
`WinchModels` winch.

KiteModels internally asks the winch model for acceleration:

```julia
calc_acceleration(wm, v_reelout, tether_force; set_speed = v_set)
```

So this bridge package defines:

```julia
DetailedIMWinch <: WinchModels.AbstractWinchModel
```

and implements:

```julia
WinchModels.calc_acceleration(wm::DetailedIMWinch, ...)
```

This lets KiteModels call your winch with no change to the autopilot logic.

---

## 2. Recommended folder layout

Place the folder next to your local projects:

```text
JuliaModels/
├── KiteSimulators.jl/
├── WinchModels.jl/
├── IM_AWES_bench/
└── ElectricMachineWinch.jl/
```

The setup script assumes this layout.

---

## 3. Configure local dependencies

Open a terminal in `ElectricMachineWinch.jl` and run:

```bash
julia --project=. scripts/setup_local_deps.jl
```

This does:

```julia
Pkg.develop(path="../IM_AWES_bench")
Pkg.develop(path="../WinchModels.jl")
```

After this, the package should be able to load:

```julia
using ElectricMachineWinch
```

---

## 4. First standalone test: ideal torque controller

Run:

```bash
julia --project=. test/manual_test_ideal_torque.jl
```

This test does not use KiteSimulators. It checks only:

```text
v_set -> speed PI -> torque actuator -> winch acceleration -> v_reelout
```

You should see printed values similar to:

```text
t=0.1  v=...  a=...  Te=...  Te_ref=...  sat=false
```

The important thing is not the exact number. The important thing is:

- the script runs without errors;
- the speed remains finite;
- torque remains finite;
- acceleration remains finite.

If this fails, debug the bridge package before connecting KiteSimulators.

---

## 5. Second standalone test: full FOC speed controller

Run:

```bash
julia --project=. test/manual_test_foc_speed_f1.jl
```

This checks:

```text
v_set
  -> FOC outer speed/flux controller
  -> current controller
  -> alpha-beta voltage
  -> IM electrical plant
  -> electromagnetic torque
  -> winch acceleration
```

This is the first test involving your detailed `IM_AWES_bench` model.

If the speed goes the wrong way, check sign conventions first. The current sign
convention is:

```text
J*dω = Te + Tload - B*ω
```

where:

```julia
Tload = drum_radius / gear_ratio * tether_force
ωm = gear_ratio / drum_radius * v_reelout
```

During positive-speed generation, you usually expect:

```text
v_reelout > 0
Tload > 0
Te < 0
```

---

## 6. Connecting to KiteSimulators autopilot

In your `KiteSimulators.jl` project, add the bridge package to the environment:

```julia
using Pkg
Pkg.activate(".")
Pkg.develop(path="../ElectricMachineWinch.jl")
Pkg.develop(path="../IM_AWES_bench")
Pkg.develop(path="../WinchModels.jl")
Pkg.instantiate()
```

Then copy the original autopilot example:

```text
examples/autopilot.jl -> examples/autopilot_im_winch.jl
```

Add near the top:

```julia
using ElectricMachineWinch
```

Find:

```julia
app.kps4 = KPS4(app.kcu)
```

Immediately after it, insert first the simple test winch:

```julia
app.kps4.wm = make_electric_winch(
    controller = :ideal,
    drum_radius = 0.5,
    gear_ratio = 10.0,
    J_eq = 0.3685,
    B_eq = 0.01298,
    Ts = app.dt,
    Te_max = 124.0,
)
```

Run:

```julia
include("examples/autopilot_im_winch.jl")
```

Only after the ideal torque controller works should you switch to the FOC model:

```julia
app.kps4.wm = make_electric_winch(
    controller = :foc_speed_f1,
    drum_radius = 0.5,
    gear_ratio = 10.0,
    J_eq = 0.3685,
    B_eq = 0.01298,
    Ts = 100e-6,
    plant_substeps = 1,
    Vs_max = 310.0,
    Is_max = 40.0,
    Te_max = 124.0,
    speed_ts_wm = 0.5,
    use_load_feedforward = false,
)
```

---

## 7. What each source file does

### `src/types.jl`

Defines the core objects:

- `DriveStepOutput`
- `InductionMachinePlant`
- `AbstractIMDriveController`
- `DetailedIMWinch`

This file sets the architecture.

### `src/plant_steps.jl`

Defines the electrical-only induction-machine RK4 step. It uses the same alpha-beta
electrical equations as your benchmark, but does not let the IM plant integrate
its own mechanical speed.

This is important because KiteModels already owns the reel-out speed state.

### `src/controllers/ideal_torque.jl`

Simple controller for debugging the KiteSimulators coupling before using the full
machine model.

### `src/controllers/foc_speed_f1.jl`

The real controller wrapper around your validated blocks from `IM_AWES_bench`.

### `src/winch_interface.jl`

Defines `WinchModels.calc_acceleration` for `DetailedIMWinch`. This is the actual
interface used by KiteModels.

### `src/constructors.jl`

Convenience constructor `make_electric_winch(...)` to quickly create either the
ideal-torque winch or the FOC-speed-F1 winch.

---

## 8. How to test different machine controllers later

Do not modify KiteSimulators. Add a new controller type:

```julia
mutable struct MyNewController <: AbstractIMDriveController
    # states and parameters
end
```

Then implement:

```julia
function drive_step!(
    c::MyNewController,
    plant::InductionMachinePlant;
    ωm_ref,
    ωm,
    Tload,
    Ts,
    plant_substeps = 1,
)
    # controller + plant logic
    return DriveStepOutput(Te = ..., Te_ref = ...)
end
```

Then use:

```julia
wm = DetailedIMWinch(controller = MyNewController(...))
```

or extend `make_electric_winch(...)` with a new controller symbol.

---

## 9. Current limitation

The first integration version updates the electric drive inside
`calc_acceleration`. This is the easiest way to connect to KiteModels without
changing the autopilot structure.

However, DAE solvers can call `calc_acceleration` more than once per macro step.
So treat this version as the practical first integration/debugging version.

The later research-grade version should update the electric drive once per
simulator sample and make `calc_acceleration` mostly read-only.
