# ElectricMachineWinch.jl

Bridge package for testing a detailed induction-machine electric drive as a
KiteSimulators/WinchModels winch model.

The package is intentionally small. It does **not** replace KiteSimulators,
KiteModels, or WinchModels. It only provides a new winch type:

```julia
DetailedIMWinch <: WinchModels.AbstractWinchModel
```

and a corresponding method:

```julia
WinchModels.calc_acceleration(wm::DetailedIMWinch, v_reelout, tether_force; set_speed, set_torque)
```

That is the method KiteModels already calls internally when advancing the kite
power system model.

## Structure

```text
ElectricMachineWinch.jl/
├── Project.toml
├── README.md
├── docs/
│   └── GETTING_STARTED.md
├── examples/
│   └── autopilot_im_winch_patch.jl
├── scripts/
│   └── setup_local_deps.jl
├── src/
│   ├── ElectricMachineWinch.jl
│   ├── types.jl
│   ├── plant_steps.jl
│   ├── winch_interface.jl
│   ├── constructors.jl
│   └── controllers/
│       ├── ideal_torque.jl
│       └── foc_speed_f1.jl
└── test/
    ├── manual_test_ideal_torque.jl
    ├── manual_test_foc_speed_f1.jl
    └── runtests.jl
```

## Controllers included

### `IdealTorqueController`

Simple first-order torque actuator with a speed PI. Use this first because it
proves that the KiteSimulators/WinchModels interface works before debugging the
full electrical machine model.

### `FOCSpeedF1Controller`

Wrapper around your `IM_AWES_bench` validated controller blocks:

```text
rotor_flux_observer_step!
outer_speed_flux_f1_step!
current_controller_step!
electrical-only alpha-beta IM RK4 step
```

The machine mechanical speed is imposed from the KiteModels reel-out speed:

```julia
ωm = gear_ratio / drum_radius * v_reelout
```

Only the electrical states are integrated internally. This avoids duplicating the
mechanical speed dynamics already owned by KiteModels.

## Setup summary

Assuming this folder sits next to `IM_AWES_bench` and `WinchModels.jl`:

```bash
cd ElectricMachineWinch.jl
julia --project=. scripts/setup_local_deps.jl
julia --project=. test/manual_test_ideal_torque.jl
julia --project=. test/manual_test_foc_speed_f1.jl
```

Then add the package to your KiteSimulators environment:

```julia
using Pkg
Pkg.activate("../KiteSimulators.jl")
Pkg.develop(path="../ElectricMachineWinch.jl")
Pkg.develop(path="../IM_AWES_bench")
Pkg.develop(path="../WinchModels.jl")
```

Copy `KiteSimulators.jl/examples/autopilot.jl` to
`KiteSimulators.jl/examples/autopilot_im_winch.jl` and insert the patch shown in
`examples/autopilot_im_winch_patch.jl` after `app.kps4 = KPS4(app.kcu)`.
