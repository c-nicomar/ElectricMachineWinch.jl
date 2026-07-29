# Getting started with `ElectricMachineWinch.jl`

This document explains what the bridge package does and how to start testing it.

---

## 1. Why this package exists

`KiteSimulators.jl` should remain mostly untouched. The clean way to include a
detailed electric machine model is to provide a new object that behaves like a
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

It also provides `step_drive_from_kite!`, which is the preferred entry point for the
detailed discrete controllers (see section 7).

---

## 2. Requirements and folder layout

The package targets Julia 1.12 and depends on `IM_AWES_bench` and `WinchModels`.

`IM_AWES_bench` is resolved from the git URL declared in `[sources]` in
`Project.toml`, so no manual setup is needed for a plain checkout. If you also want to
edit `IM_AWES_bench` locally, place the projects side by side:

```text
JuliaModels/
├── ElectricMachineWinch.jl/
├── IM_AWES_bench.jl/
├── WinchModels.jl/
└── KiteSimulators.jl/
```

---

## 3. Configure dependencies

Default (git source) setup:

```bash
julia --project -e 'using Pkg; Pkg.instantiate()'
```

To switch `IM_AWES_bench` to the local sibling checkout `../IM_AWES_bench`:

```bash
bin/dev
```

To switch back to the git source:

```bash
bin/free
```

Both scripts edit the `[sources]` section of `Project.toml` in place, so that file
will show as modified afterwards.

Check that the package loads:

```julia
using ElectricMachineWinch
```

`bin/run_julia` starts a REPL in this project with `Revise` loaded if available.
Revise does not track changes to struct definitions, module includes, or exports, so
restart Julia after changing those.

---

## 4. First standalone test: ideal torque controller

```bash
julia --project=test test/test_ideal_torque.jl
```

or, in a running REPL:

```julia
include("test/test_ideal_torque.jl")
```

This test does not use KiteSimulators. It checks only:

```text
v_set -> speed PI -> torque actuator -> winch acceleration -> v_reelout
```

The important thing is not the exact numbers, but that the script runs without
errors and that speed, torque, and acceleration stay finite.

If this fails, debug the bridge package before connecting KiteSimulators.

---

## 5. Second standalone test: full FOC speed controller

```bash
julia --project=test test/test_foc_speed_f1.jl
```

This checks:

```text
v_set
  -> rotor-flux observer
  -> FOC outer speed/flux controller
  -> current controller
  -> inverse Park -> alpha-beta voltage
  -> electrical-only IM RK4 plant
  -> electromagnetic torque
  -> winch acceleration
```

This is the first test involving the detailed `IM_AWES_bench` model.

Run the whole suite with:

```bash
julia --project=test test/runtests.jl
```

If the speed goes the wrong way, check sign conventions first. The convention is:

```text
J*dωm/dt = Te + Tload - B*ωm
```

where:

```julia
Tload = drum_radius / gear_ratio * tether_force
ωm    = gear_ratio / drum_radius * v_reelout
```

During positive-speed generation, you usually expect:

```text
v_reelout > 0
Tload > 0
Te < 0
```

---

## 6. Constructing a winch

`make_electric_winch` selects the controller and builds a coherent set of
`IM_AWES_bench` parameter objects:

```julia
wm = make_electric_winch(
    controller = :ideal,
    drum_radius = 0.5,
    gear_ratio = 10.0,
    J_eq = 0.3685,
    B_eq = 0.01298,
    Ts = 1e-3,
    Te_max = 124.0,
)
```

Available controllers are `:ideal`, `:foc_speed_f1`, and `:foc_speed_mtpa`. See
[Controllers](controllers.md) for the full parameter sets of the two FOC variants.

---

## 7. Connecting to KiteSimulators autopilot

In your `KiteSimulators.jl` project, add the bridge package to the environment:

```julia
using Pkg
Pkg.activate(".")
Pkg.develop(path="../ElectricMachineWinch.jl")
Pkg.instantiate()
```

Then copy the original autopilot example:

```text
examples/autopilot.jl -> examples/autopilot_im_winch.jl
```

and add near the top:

```julia
using ElectricMachineWinch
```

Immediately after `app.kps4 = KPS4(app.kcu)`, insert the winch. Start with the simple
ideal-torque controller:

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

Only after that works, switch to the FOC model:

```julia
app.kps4.wm = make_electric_winch(
    controller = :foc_speed_f1,
    drum_radius = 0.2,
    gear_ratio = 4.26,
    J_eq = 0.204,
    B_eq = 0.0804,
    Ts = 1e-4,
    plant_substeps = 1,
    Vs_max = 310.0,
    Is_max = 40.0 * sqrt(2),
    Te_max = 124.0,
    speed_ts_wm = 1.0,
    use_load_feedforward = true,
    use_field_weakening = true,
    wm_base_fw = 120.0,
)
```

### Stepping the drive

Update the electric drive exactly once per KiteSimulators macro-step, then hand the
resulting torque to the mechanical update:

```julia
Te_cmd = ElectricMachineWinch.step_drive_from_kite!(
    app.kps4.wm,
    v_ro_set,
    v_ro_meas,
    F_tether;
    dt_outer = app.dt,
)

KiteModels.next_step!(app.kps4, integrator; set_torque = Te_cmd, dt = app.dt)
```

`step_drive_from_kite!` runs enough electrical/controller substeps to cover the
macro-step, holding the kite-side variables constant during it. Choose `dt_outer` so
that `dt_outer / wm.Ts` is an integer (or very close to one); otherwise the effective
sample time drifts away from the `Ts` stored in the observer and controller parameter
objects.

This pattern is required for detailed stateful controllers because a DAE solver may
evaluate the mechanical residual more than once per macro-step. Advancing the
discrete observer/controller states inside every residual evaluation would produce a
nonphysical number of controller updates. `wm.n_acceleration_calls` counts the calls
so this is easy to detect.

The direct `WinchModels.calc_acceleration(...; set_speed = ...)` path, which steps the
drive inside the call, remains available for simple integration tests and
compatibility.

### Working examples

- [`examples/autopilot_im_winch_FOC_F1.jl`](https://github.com/c-nicomar/ElectricMachineWinch.jl/blob/main/examples/autopilot_im_winch_FOC_F1.jl)
  — a complete autopilot script using the F1 controller, including debug logging and
  CSV export of the drive signals.
- [`examples/autopilot_im_winch_patch.jl`](https://github.com/c-nicomar/ElectricMachineWinch.jl/blob/main/examples/autopilot_im_winch_patch.jl)
  — not runnable on its own; it is the minimal patch to apply to a fresh copy of the
  upstream autopilot example.

See [KiteModels integration](integration.md) for the full picture of the two
integration paths.

---

## 8. What each source file does

### `src/types.jl`

Defines the core objects and thereby the architecture:

- `DriveStepOutput` — the common return value of every controller
- `InductionMachinePlant` — electrical plant parameters and state
- `AbstractIMDriveController` — the controller interface
- `DetailedIMWinch` — the `WinchModels`-compatible winch, plus `reset!` and
  `last_summary`

### `src/plant_steps.jl`

The electrical-only induction-machine RK4 step. It uses the same alpha-beta
electrical equations as the benchmark, but does not let the IM plant integrate its
own mechanical speed — KiteModels already owns the reel-out speed state. Also
contains `im_torque`, `inverse_park_voltage`, and `phase_power_alpha_beta`.

### `src/controllers/ideal_torque.jl`

Speed PI plus a first-order torque actuator with rate and magnitude limits. No
electrical plant. Used to debug the KiteSimulators coupling before adding the machine
model.

### `src/controllers/foc_speed_f1.jl`

Adapter around the validated FOC speed/flux F1 blocks from `IM_AWES_bench`. Supports
field weakening.

### `src/controllers/foc_speed_mtpa.jl`

Adapter around the constrained-MTPA outer speed controller from `IM_AWES_bench`. The
full outer-loop output is retained in `wm.controller.last_outer` for diagnostics
(`isd_mtpa`, `isd_floor`, `isd_reserve`, `Te_current_limited`, ...).

### `src/winch_interface.jl`

`WinchModels.calc_acceleration` for `DetailedIMWinch`, and `step_drive_from_kite!`.

### `src/constructors.jl`

`make_electric_winch(...)`, which selects the controller and validates the
configuration.

---

## 9. Logging and diagnostics

The latest controller-agnostic values are stored on the winch itself:

```julia
wm.Te, wm.Te_ref
wm.isd_ref, wm.isq_ref, wm.isd, wm.isq
wm.vsd, wm.vsq, wm.vsα, wm.vsβ
wm.Pelec, wm.Pmech, wm.saturated
```

A compact NamedTuple is available with:

```julia
last_summary(wm)
```

---

## 10. Adding another controller

Do not modify KiteSimulators. Add a new controller type:

```julia
mutable struct MyNewController <: AbstractIMDriveController
    # states and parameters
end
```

Implement:

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

Then include and export it in `src/ElectricMachineWinch.jl`, add a selector branch in
`src/constructors.jl`, and add a test file modeled on `test/test_foc_speed_f1.jl`.
The rest of the KiteControllers integration stays unchanged.

---

## 11. Current limitations

- The MTPA controller does not implement field weakening yet, so
  `use_field_weakening` and `wm_base_fw` apply only to `:foc_speed_f1`. A full-speed
  comparison of the two controllers is not strictly equivalent once the cycle enters
  the field-weakening region.
- There is no `test/test_foc_speed_mtpa.jl` yet; it can be added following
  `test/test_foc_speed_f1.jl` with `controller = :foc_speed_mtpa`.
- Using `calc_acceleration(...; set_speed = ...)` inside a DAE solve advances the
  discrete controller states once per residual evaluation. Prefer
  `step_drive_from_kite!` for research runs.
