"""
    ElectricMachineWinch

Bridge package that exposes a detailed induction-machine electric drive as a
`WinchModels.AbstractWinchModel`, so that KiteModels/KiteControllers can use it
in place of a simple winch.

The layering rule behind the whole package is:

**KiteModels owns the mechanical state, this package owns only the electrical
state.**

The machine speed is therefore imposed rather than integrated here:

    ωm    = gear_ratio / drum_radius * v_reelout
    Tload = drum_radius / gear_ratio * tether_force

and the machine-side mechanics follow `J*dωm/dt = Te + Tload - B*ωm`.

Main entry points:

- [`make_electric_winch`](@ref) builds a [`DetailedIMWinch`](@ref) with one of
  the controllers `:ideal`, `:foc_speed_f1` or `:foc_speed_mtpa`.
- [`step_drive_from_kite!`](@ref) advances the drive once per KiteControllers
  macro-step; this is the preferred integration path.
- `WinchModels.calc_acceleration` is the `WinchModels` interface method.
- [`last_summary`](@ref) returns the most useful last-step values.
"""
module ElectricMachineWinch

using WinchModels
using InductionMachineDrives

const IMB = InductionMachineDrives

export DetailedIMWinch
export InductionMachinePlant
export DriveStepOutput
export AbstractIMDriveController
export IdealTorqueController
export FOCSpeedF1Controller
export drive_step!
export make_default_foc_speed_controller
export make_electric_winch
export step_drive_from_kite!
export reset!
export last_summary
export FOCSpeedMTPAController
export make_default_foc_speed_mtpa_controller

include("types.jl")
include("plant_steps.jl")
include("controllers/ideal_torque.jl")
include("controllers/foc_speed_f1.jl")
include("winch_interface.jl")
include("constructors.jl")
include("controllers/foc_speed_mtpa.jl")

end # module
