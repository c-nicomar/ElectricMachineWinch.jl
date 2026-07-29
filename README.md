# ElectricMachineWinch.jl

`ElectricMachineWinch.jl` is a package for testing detailed induction-machine electric drives as winch models in the KiteSimulators/KiteModels ecosystem.

The package provides a winch type compatible with `WinchModels`:

```julia
DetailedIMWinch <: WinchModels.AbstractWinchModel
```

and connects it to interchangeable electric-drive controllers implemented with the discrete induction-machine blocks from `IM_AWES_bench`.

The currently supported controller selections are:

```julia
:ideal
:foc_speed_f1
:foc_speed_mtpa
```

---

## Purpose

The package separates the AWES mechanical simulation from the detailed electric-drive simulation:

- **KiteModels** owns reel-out speed, tether mechanics, and the equivalent winch mechanical state.
- **ElectricMachineWinch** owns the electric-drive controller, observer, and induction-machine electrical states.
- **IM_AWES_bench** provides the validated controller, observer, and machine-model building blocks.

The machine speed used by the electrical model is imposed from the measured reel-out speed:

```julia
ωm = gear_ratio / drum_radius * v_reelout
```

The tether force is converted to an equivalent machine-side load torque:

```julia
Tload = drum_radius / gear_ratio * tether_force
```

Only the electrical machine states are integrated internally. This avoids introducing a second mechanical-speed state in parallel with the state already owned by KiteModels.

---

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

All drive controllers subtype:

```julia
AbstractIMDriveController
```

and implement:

```julia
drive_step!(
    controller,
    plant;
    ωm_ref,
    ωm,
    Tload,
    Ts,
    plant_substeps,
)
```

Each controller returns the same:

```julia
DriveStepOutput
```

This keeps `DetailedIMWinch` and the KiteModels interface independent of the selected controller.

---

## Controllers

### `IdealTorqueController`

A simplified torque actuator with a speed PI.

Use it first to verify:

```text
autopilot speed reference
    -> winch controller
    -> electromagnetic torque
    -> KiteModels mechanics
```

without simultaneously debugging the induction-machine observer, current controller, and electrical plant.

Select it with:

```julia
controller = :ideal
```

---

### `FOCSpeedF1Controller`

Wrapper around the discrete FOC speed-control path from `IM_AWES_bench`:

```text
rotor_flux_observer_step!
    -> outer_speed_flux_f1_step!
    -> current_controller_step!
    -> inverse Park transformation
    -> electrical-only alpha-beta IM RK4 step
    -> electromagnetic torque
```

The F1 controller uses its configured flux-producing-current strategy and supports the field-weakening options exposed by `make_electric_winch`:

```julia
use_field_weakening
wm_base_fw
```

Select it with:

```julia
controller = :foc_speed_f1
```

---

### `FOCSpeedMTPAController`

Wrapper around the constrained-MTPA discrete speed controller from `IM_AWES_bench`:

```text
rotor_flux_observer_step!
    -> outer_speed_flux_mtpa_step!
    -> current_controller_step!
    -> inverse Park transformation
    -> electrical-only alpha-beta IM RK4 step
    -> electromagnetic torque
```

The MTPA torque-current relationship is:

```text
Te = Kt_isd * isd * isq

Kt_isd = 1.5 * pole_pairs * Lm^2 / Lrr
```

The pure MTPA solution is supplemented by practical constraints:

- minimum d-axis current;
- rotor-flux floor;
- torque reserve;
- d-axis current slew-rate limit;
- total dq-current circle;
- electromagnetic-torque limit.

Select it with:

```julia
controller = :foc_speed_mtpa
```

The full outer-controller output is retained in:

```julia
wm.controller.last_outer
```

which exposes MTPA-specific diagnostics such as:

```julia
isd_mtpa
isd_floor
isd_reserve
isd_desired
isq_max_disp
Te_current_limited
torque_current_limited
```

### Current MTPA limitation

The present MTPA controller does **not** yet include field weakening.

Therefore:

```julia
use_field_weakening
wm_base_fw
```

currently apply only to `:foc_speed_f1`.

For full-speed AWES cycles, first verify whether the MTPA case reaches the inverter voltage limit during high-speed reel-in. A future combined MTPA/field-weakening controller should coordinate the energy-optimal current allocation below base speed with the voltage-feasibility constraint above base speed.

---

## Repository structure

```text
ElectricMachineWinch.jl/
├── Project.toml
├── Manifest-v1.12.toml.default
├── README.md
├── bin/
│   ├── dev
│   ├── free
│   └── run_julia
├── data/
│   ├── settings.yaml
│   ├── system.yml
│   ├── fpc_settings.yaml
│   ├── fpp_settings.yaml
│   ├── wc_settings.yaml
│   └── gui.yaml
├── docs/
│   └── GETTING_STARTED.md
├── examples/
│   ├── Project.toml
│   ├── autopilot_im_winch_patch.jl
│   └── autopilot_im_winch_FOC_F1.jl
├── src/
│   ├── ElectricMachineWinch.jl
│   ├── types.jl
│   ├── plant_steps.jl
│   ├── winch_interface.jl
│   ├── constructors.jl
│   └── controllers/
│       ├── ideal_torque.jl
│       ├── foc_speed_f1.jl
│       └── foc_speed_mtpa.jl
└── test/
    ├── Project.toml
    ├── runtests.jl
    ├── test_ideal_torque.jl
    └── test_foc_speed_f1.jl
```

---

## Requirements

The package currently targets Julia 1.12 and depends on:

- `IM_AWES_bench`
- `WinchModels`

For local development, the easiest layout is:

```text
JuliaModels/
├── ElectricMachineWinch.jl/
├── IM_AWES_bench.jl/
├── WinchModels.jl/
├── KiteSimulators.jl/
└── AWES_IM_KiteSimulators_reproducibility.jl/
```

---

## Local setup

Check that the package loads:

```julia
using ElectricMachineWinch
```

After changing controller types, struct definitions, module includes, or exports, restart Julia before testing again.

---

## Standalone checks

Run the individual test files from the package root:

```bash
julia --project=test test/test_ideal_torque.jl
julia --project=test test/test_foc_speed_f1.jl
```

Run the package test suite with:

```julia
using Pkg
Pkg.activate(".")
Pkg.test()
```

A dedicated `test_foc_speed_mtpa.jl` can be added following the same structure as `test_foc_speed_f1.jl`, with:

```julia
controller = :foc_speed_mtpa
```

---

## Constructing a winch

### Ideal controller

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
```

### F1 controller

```julia
wm = make_electric_winch(
    controller = :foc_speed_f1,

    drum_radius = 0.2,
    gear_ratio = 4.26,
    J_eq = 0.204,
    B_eq = 0.0804,

    Ts = 100e-6,
    plant_substeps = 1,

    Vs_max = 310.0,
    Is_max = 40.0 * sqrt(2.0),
    Te_max = 124.0,

    speed_ts_wm = 1.0,
    speed_tau_f_wm = 10e-3,
    speed_ts_dist_wm = 3.0,
    wm_dot_max = 100.0,

    use_load_feedforward = true,

    use_field_weakening = true,
    wm_base_fw = 120.0,
)
```

### Constrained-MTPA controller

```julia
wm = make_electric_winch(
    controller = :foc_speed_mtpa,

    drum_radius = 0.2,
    gear_ratio = 4.26,
    J_eq = 0.204,
    B_eq = 0.0804,

    Ts = 100e-6,
    plant_substeps = 1,

    Vs_max = 310.0,
    Is_max = 40.0 * sqrt(2.0),
    Te_max = 124.0,

    speed_ts_wm = 1.0,
    speed_tau_f_wm = 10e-3,
    speed_ts_dist_wm = 3.0,
    wm_dot_max = 100.0,

    use_load_feedforward = true,
    load_ff_sign = -1.0,

    pole_pairs = 2.0,
    Lm = 40.84e-3,
    Lrr = 45.12e-3,

    isd_nom = 23.04579328,
    isd_min = 5.0,
    id_dot_max = 600.0,
    lambda_rd_floor = 0.35,
    Te_reserve = 45.0,
)
```

`Is_max` is interpreted as the maximum magnitude of the dq current vector. Use the actual peak-vector limit of the modeled machine and converter. The expression `40.0 * sqrt(2.0)` is appropriate only when 40 A is an RMS phase-current rating that is intentionally converted to a peak value.

For the MTPA controller, `J_eq` and `B_eq` are passed into both:

- `DetailedIMWinch`;
- the MTPA outer speed controller.

This keeps the speed-loop tuning and the inertia/friction feedforward consistent with the mechanical plant seen by KiteModels.

---

## KiteControllers integration

A version of the `autopilot.jl` script from KiteControllers that is using this package can be found in the examples directory.


### The way the integration works

For detailed discrete controllers, update the electric drive exactly once per KiteSimulators macro-step:

```julia
Te_cmd = ElectricMachineWinch.step_drive_from_kite!(
    app.kps4.wm,
    v_ro_set,
    v_ro_meas,
    F_tether;
    dt_outer = app.dt,
)
```

Then pass the resulting torque to the mechanical update:

```julia
KiteModels.next_step!(
    app.kps4,
    integrator;
    set_torque = Te_cmd,
    dt = app.dt,
)
```

`step_drive_from_kite!` internally executes enough electrical/controller substeps to cover the KiteSimulators macro-step while holding the kite-side variables constant during that macro-step.

For the current discrete adapters, choose `dt_outer` so that `dt_outer / wm.Ts` is an integer, or extremely close to one. This keeps the effective electrical sample time consistent with the sample time stored in the observer and controller parameter objects.

This pattern is preferred for detailed stateful controllers because numerical solvers may evaluate a mechanical residual or acceleration function more than once per macro-step. Advancing discrete observer/controller states inside every residual evaluation would produce a nonphysical number of controller updates.

The direct `WinchModels.calc_acceleration(...; set_speed=...)` interface remains available for simple integration tests and compatibility.

---

## Sign convention

The machine-side mechanical model uses:

```text
J * dωm/dt = Te + Tload - B * ωm
```

with:

```julia
Tload = drum_radius / gear_ratio * tether_force
ωm = gear_ratio / drum_radius * v_reelout
```

For positive-speed generation, the typical signs are:

```text
v_reelout > 0
Tload > 0
Te < 0
```

The load-feedforward sign must be consistent with this convention. The MTPA constructor exposes:

```julia
load_ff_sign
```

with the current default:

```julia
load_ff_sign = -1.0
```

---

## Logging and diagnostics

The latest common drive values are stored directly in `DetailedIMWinch`:

```julia
wm.Te
wm.Te_ref

wm.isd_ref
wm.isq_ref
wm.isd
wm.isq

wm.vsd
wm.vsq
wm.vsα
wm.vsβ

wm.Pelec
wm.Pmech
wm.saturated
```

A compact summary is available with:

```julia
last_summary(wm)
```

For an MTPA controller:

```julia
mtpa = wm.controller.last_outer
```

Useful fields include:

```julia
mtpa.isd_mtpa
mtpa.isd_floor
mtpa.isd_reserve
mtpa.isd_desired
mtpa.isd_ref
mtpa.isq_ref
mtpa.isq_max_disp
mtpa.Te_ref_out
mtpa.Te_current_limited
mtpa.torque_current_limited
```

These signals make it possible to distinguish:

- the pure MTPA solution;
- the active minimum-flux or torque-reserve constraint;
- the ramped d-axis reference;
- current-circle limitation;
- requested versus current-feasible torque.

---

## Comparing F1 and MTPA fairly

When comparing:

```julia
:foc_speed_f1
```

and:

```julia
:foc_speed_mtpa
```

keep the following identical unless the parameter itself is under study:

- kite and wind profile;
- initial conditions;
- drum radius and gear ratio;
- equivalent inertia and friction;
- speed-loop tuning;
- sampling period and plant substeps;
- current, voltage, and torque limits;
- current-controller parameters;
- rotor-flux observer;
- load-feedforward configuration;
- KiteSimulators macro-step.

Useful comparison metrics include:

- reel-out and reel-in speed RMSE;
- electromagnetic-torque tracking;
- RMS dq-current magnitude;
- integral of `isd^2 + isq^2` as a current/copper-loss proxy;
- electrical and mechanical energy;
- current- and voltage-saturation time;
- MTPA flux-floor and torque-reserve activity;
- cycle completion and average generated power.

Because the current F1 controller supports field weakening while the current MTPA controller does not, a full-speed comparison is not yet strictly equivalent whenever the AWES cycle enters the field-weakening region. A first comparison should either remain below base speed or clearly report the voltage-limited intervals.

---

## Extending the package with another controller

Create a new controller subtype:

```julia
mutable struct MyController <: AbstractIMDriveController
    # states and parameters
end
```

Implement:

```julia
function drive_step!(
    controller::MyController,
    plant::InductionMachinePlant;
    ωm_ref,
    ωm,
    Tload,
    Ts,
    plant_substeps = 1,
)
    # observer/controller/plant update

    return DriveStepOutput(
        Te = ...,
        Te_ref = ...,
    )
end
```

Then:

1. include and export the controller in `src/ElectricMachineWinch.jl`;
2. add a selector branch in `src/constructors.jl`;
3. add a standalone test;
4. use the new controller symbol in `make_electric_winch`.

The rest of the KiteControllers integration can remain unchanged.
