# Controllers

```@meta
CurrentModule = ElectricMachineWinch
```

Three drive controllers are available today. All of them subtype
[`AbstractIMDriveController`](@ref), implement [`drive_step!`](@ref), and return
a [`DriveStepOutput`](@ref), so switching between them changes nothing on the
KiteModels side.

## `IdealTorqueController`

A simplified torque actuator with a speed PI.

Use it first to verify:

```text
autopilot speed reference
    -> winch controller
    -> electromagnetic torque
    -> KiteModels mechanics
```

without simultaneously debugging the induction-machine observer, current
controller, and electrical plant.

Select it with:

```julia
controller = :ideal
```

## `FOCSpeedF1Controller`

Wrapper around the discrete FOC speed-control path from `IM_AWES_bench`:

```text
rotor_flux_observer_step!
    -> outer_speed_flux_f1_step!
    -> current_controller_step!
    -> inverse Park transformation
    -> electrical-only alpha-beta IM RK4 step
    -> electromagnetic torque
```

The F1 controller uses its configured flux-producing-current strategy and
supports the field-weakening options exposed by
[`make_electric_winch`](@ref):

```julia
use_field_weakening
wm_base_fw
```

Select it with:

```julia
controller = :foc_speed_f1
```

## `FOCSpeedMTPAController`

Wrapper around the constrained-MTPA discrete speed controller from
`IM_AWES_bench`:

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

See [Logging and diagnostics](diagnostics.md) for the full list.

### Current MTPA limitation

The present MTPA controller does **not** yet include field weakening.

Therefore:

```julia
use_field_weakening
wm_base_fw
```

currently apply only to `:foc_speed_f1`.

For full-speed AWES cycles, first verify whether the MTPA case reaches the
inverter voltage limit during high-speed reel-in. A future combined
MTPA/field-weakening controller should coordinate the energy-optimal current
allocation below base speed with the voltage-feasibility constraint above base
speed.

## Constructing a winch

[`make_electric_winch`](@ref) selects the controller and builds a coherent set
of `IM_AWES_bench` parameter objects.

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

### Parameter notes

`Is_max` is interpreted as the maximum magnitude of the dq current *vector*, not
as an RMS phase rating. Use the actual peak-vector limit of the modeled machine
and converter. The expression `40.0 * sqrt(2.0)` is appropriate only when 40 A
is an RMS phase-current rating that is intentionally converted to a peak value.

For the MTPA controller, `J_eq` and `B_eq` are passed into both:

- `DetailedIMWinch`;
- the MTPA outer speed controller.

This keeps the speed-loop tuning and the inertia/friction feedforward consistent
with the mechanical plant seen by KiteModels.

Custom F1 parameter objects (`obs_p`, `outer_f1_p`, `current_p`) are
all-or-nothing, and each must carry a `Ts` matching the winch `Ts`. The
validation is deliberately strict, to prevent mixing a new-machine observer with
the default small-machine loops.
