# Logging and diagnostics

```@meta
CurrentModule = ElectricMachineWinch
```

## Controller-agnostic values

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

which also reports `n_acceleration_calls`, the debug counter that reveals
excessive DAE residual calls (see [KiteModels integration](integration.md)).

## MTPA-specific diagnostics

For an MTPA controller, the full outer-loop output of the last step is retained:

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
- KiteControllers macro-step.

Useful comparison metrics include:

- reel-out and reel-in speed RMSE;
- electromagnetic-torque tracking;
- RMS dq-current magnitude;
- integral of `isd^2 + isq^2` as a current/copper-loss proxy;
- electrical and mechanical energy;
- current- and voltage-saturation time;
- MTPA flux-floor and torque-reserve activity;
- cycle completion and average generated power.

Because the current F1 controller supports field weakening while the current
MTPA controller does not, a full-speed comparison is not yet strictly equivalent
whenever the AWES cycle enters the field-weakening region. A first comparison
should either remain below base speed or clearly report the voltage-limited
intervals.
