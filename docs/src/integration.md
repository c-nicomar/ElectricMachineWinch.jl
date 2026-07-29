# KiteModels integration

```@meta
CurrentModule = ElectricMachineWinch
```

A version of the `autopilot.jl` script from KiteControllers that uses this
package is available as
[`examples/autopilot_im_winch_FOC_F1.jl`](https://github.com/c-nicomar/ElectricMachineWinch.jl/blob/main/examples/autopilot_im_winch_FOC_F1.jl).
[`examples/autopilot_im_winch_patch.jl`](https://github.com/c-nicomar/ElectricMachineWinch.jl/blob/main/examples/autopilot_im_winch_patch.jl)
is not runnable on its own — it is the minimal patch to apply to a fresh copy of
the upstream autopilot example.

## Two integration paths

There are two ways into KiteModels, and they are not equivalent.

### 1. `step_drive_from_kite!` — preferred

Update the electric drive exactly once per KiteSimulators macro-step:

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

[`step_drive_from_kite!`](@ref) internally executes `round(Int, dt_outer/wm.Ts)`
electrical/controller substeps to cover the KiteSimulators macro-step, holding
the kite-side variables constant during that macro-step.

Choose `dt_outer` so that `dt_outer / wm.Ts` is an integer, or extremely close
to one. Otherwise the effective electrical sample time drifts away from the `Ts`
baked into the observer and controller parameter objects.

### 2. `calc_acceleration(...; set_speed = ...)` — convenient, but be careful

The direct [`WinchModels.calc_acceleration`](@ref) interface advances the drive
*inside* the call. It is convenient, it is what the tests use, and it remains
available for simple integration tests and compatibility.

It is not the right path for the detailed discrete controllers, because a DAE
solver may evaluate the mechanical residual or acceleration function more than
once per macro-step. Advancing discrete observer/controller states inside every
residual evaluation would produce a nonphysical number of controller updates.
`wm.n_acceleration_calls` counts the calls, which makes this easy to detect.

`calc_acceleration` with `set_torque` — rather than `set_speed` — is the
read-only companion to `step_drive_from_kite!`: it applies the given torque and
returns the resulting reel-out acceleration without stepping the controller.

## Adding the winch to a KiteSimulators project

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

Immediately after `app.kps4 = KPS4(app.kcu)`, insert the winch. Start with the
simple ideal-torque controller:

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

and switch the stepping over to `step_drive_from_kite!` as shown above.
