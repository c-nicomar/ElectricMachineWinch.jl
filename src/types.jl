# Core type definitions for the ElectricMachineWinch bridge.
#
# The design intentionally separates:
#   1. the KiteSimulators-compatible winch wrapper,
#   2. the electrical machine plant state,
#   3. the replaceable controller object.

"""
    DriveStepOutput

Common output returned by every electric-drive controller implementation.
This lets the winch wrapper stay controller-agnostic: FOC, ideal torque,
future MPC, scalar V/f, etc. can all return the same information.
"""
Base.@kwdef struct DriveStepOutput
    Te::Float64 = 0.0
    Te_ref::Float64 = 0.0

    isd_ref::Float64 = 0.0
    isq_ref::Float64 = 0.0
    isd::Float64 = 0.0
    isq::Float64 = 0.0

    vsd::Float64 = 0.0
    vsq::Float64 = 0.0
    vsα::Float64 = 0.0
    vsβ::Float64 = 0.0

    Pelec::Float64 = 0.0
    Pmech::Float64 = 0.0

    saturated::Bool = false
end

"""
    InductionMachinePlant

Small mutable wrapper around the validated IM_AWES_bench plant parameters and
state. This deliberately contains only the electrical machine state/parameters.
The reel-out mechanics are owned by the KiteModels/WinchModels interface.
"""
mutable struct InductionMachinePlant
    p::IMB.IMPlantParams
    x::IMB.IMPlantState
end

function InductionMachinePlant(; p = IMB.IMPlantParams(), x = IMB.IMPlantState())
    return InductionMachinePlant(p, x)
end

"""
    AbstractIMDriveController

All machine-controller variants should subtype this and implement:

    drive_step!(controller, plant; ωm_ref, ωm, Tload, Ts, plant_substeps)

The method must return a `DriveStepOutput`.
"""
abstract type AbstractIMDriveController end

"""
    drive_step!(controller, plant; ωm_ref, ωm, Tload, Ts, plant_substeps) -> DriveStepOutput

Advance one electric-drive controller sample and return the resulting
[`DriveStepOutput`](@ref).

This is *the* extension point of the package: every
[`AbstractIMDriveController`](@ref) implements exactly this method, and because
the return value is the same for all of them, [`DetailedIMWinch`](@ref) and the
KiteModels interface stay controller-agnostic.

Arguments:
- `controller`: the controller object, whose discrete states are updated in place
- `plant`: [`InductionMachinePlant`](@ref) holding the electrical machine
  parameters and state; also updated in place by controllers that use it

Keyword arguments:
- `ωm_ref`: machine-side speed reference [rad/s], `gear_ratio/drum_radius * v_ro_set`
- `ωm`: measured/imposed machine-side speed [rad/s],
  `gear_ratio/drum_radius * v_reelout`
- `Tload`: machine-side load torque [Nm], `drum_radius/gear_ratio * tether_force`
- `Ts`: controller sample time [s]
- `plant_substeps`: RK4 substeps of the electrical plant per controller sample

Sign convention, which every implementation must follow:

    J*dωm/dt = Te + Tload - B*ωm

A positive `Tload` pulls toward positive reel-out. During positive-speed
generation expect `ωm > 0`, `Tload > 0` and a negative electromagnetic torque
`Te`.

The mechanical speed is *imposed*, never integrated: implementations synchronize
`plant.x.ωm` with `ωm` and integrate the electrical states only, with
[`rk4_step_electrical_only`](@ref).
"""
function drive_step! end

"""
    reset!(wm::DetailedIMWinch)
    reset!(controller::AbstractIMDriveController)

Clear all stored last-step values and discrete controller/observer states, so
that the same object can be reused for another run.

`reset!(wm)` zeroes the logging fields and the `n_acceleration_calls` counter of
the winch and then resets its controller. For controllers, the fallback
`reset!(::AbstractIMDriveController) = nothing` covers stateless implementations;
controllers with discrete states add their own method.

Note that the electrical plant state is *not* reset by these methods.
"""
function reset! end

"""
    DetailedIMWinch

KiteSimulators/KiteModels-compatible winch model.

It subtypes `WinchModels.AbstractWinchModel`, so when KiteModels calls
`calc_acceleration(wm, ...)`, Julia dispatches to the method defined in this
package.

Fields:
- `drum_radius`: winch drum radius [m]
- `gear_ratio`: machine speed / drum speed [-]
- `J_eq`: equivalent inertia referred to the machine shaft [kg m²]
- `B_eq`: equivalent viscous friction referred to the machine shaft [N m s/rad]
- `Ts`: electrical/controller sample time [s]
- `plant_substeps`: RK4 substeps per controller sample
- `controller`: swappable controller object
- `plant`: induction-machine plant wrapper
- last values: stored for logging/debugging
"""
mutable struct DetailedIMWinch{C<:AbstractIMDriveController,P} <: WinchModels.AbstractWinchModel
    drum_radius::Float64
    gear_ratio::Float64

    J_eq::Float64
    B_eq::Float64

    Ts::Float64
    plant_substeps::Int

    controller::C
    plant::P

    Te::Float64
    Te_ref::Float64

    isd_ref::Float64
    isq_ref::Float64
    isd::Float64
    isq::Float64

    vsd::Float64
    vsq::Float64
    vsα::Float64
    vsβ::Float64

    Pelec::Float64
    Pmech::Float64

    saturated::Bool

    # Debug counter: useful to detect excessive DAE residual calls.
    n_acceleration_calls::Int
end

function DetailedIMWinch(;
    drum_radius::Float64 = 0.5,
    gear_ratio::Float64 = 10.0,
    J_eq::Float64 = 0.3685,
    B_eq::Float64 = 0.01298,
    Ts::Float64 = 100e-6,
    plant_substeps::Int = 1,
    controller::AbstractIMDriveController = IdealTorqueController(),
    plant = InductionMachinePlant(),
)
    return DetailedIMWinch(
        drum_radius,
        gear_ratio,
        J_eq,
        B_eq,
        Ts,
        plant_substeps,
        controller,
        plant,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        false,
        0,
    )
end

function reset!(wm::DetailedIMWinch)
    wm.Te = 0.0
    wm.Te_ref = 0.0
    wm.isd_ref = 0.0
    wm.isq_ref = 0.0
    wm.isd = 0.0
    wm.isq = 0.0
    wm.vsd = 0.0
    wm.vsq = 0.0
    wm.vsα = 0.0
    wm.vsβ = 0.0
    wm.Pelec = 0.0
    wm.Pmech = 0.0
    wm.saturated = false
    wm.n_acceleration_calls = 0
    reset!(wm.controller)
    return nothing
end

# Fallback reset for controllers with no internal states.
reset!(::AbstractIMDriveController) = nothing

"""
    last_summary(wm)

Small NamedTuple with the most useful last-step values. Handy in REPL tests.
"""
function last_summary(wm::DetailedIMWinch)
    return (
        Te = wm.Te,
        Te_ref = wm.Te_ref,
        isd = wm.isd,
        isq = wm.isq,
        isd_ref = wm.isd_ref,
        isq_ref = wm.isq_ref,
        vsd = wm.vsd,
        vsq = wm.vsq,
        Pelec = wm.Pelec,
        Pmech = wm.Pmech,
        saturated = wm.saturated,
        n_acceleration_calls = wm.n_acceleration_calls,
    )
end
