# Simple, robust controller used as the first integration test.
# It does not use the electrical IM plant. It converts a speed error into a
# torque reference and applies first-order torque dynamics plus limits.

"""
    IdealTorqueController

Minimal controller for debugging the KiteSimulators integration before adding
full IM electrical dynamics.

Mechanical convention:

    J*dω = Te + Tload - B*ω

A positive tether/load torque `Tload` pulls the winch in the positive reel-out
speed direction. During generation/braking at positive reel-out speed, `Te` is
usually negative.
"""
Base.@kwdef mutable struct IdealTorqueController <: AbstractIMDriveController
    Te::Float64 = 0.0

    # Speed PI gains. Start conservative.
    Kp_speed::Float64 = 15.0
    Ki_speed::Float64 = 5.0
    ui_speed::Float64 = 0.0

    # Torque actuator and limits.
    Te_max::Float64 = 124.0
    Te_rate_max::Float64 = 2_000.0       # [Nm/s]
    tau_Te::Float64 = 20e-3              # [s]

    # Whether to cancel the tether/load torque in the speed controller.
    use_load_feedforward::Bool = true
end

function reset!(c::IdealTorqueController)
    c.Te = 0.0
    c.ui_speed = 0.0
    return nothing
end

function drive_step!(
    c::IdealTorqueController,
    plant::InductionMachinePlant;
    ωm_ref::Float64,
    ωm::Float64,
    Tload::Float64,
    Ts::Float64,
    plant_substeps::Int = 1,
)
    e = ωm_ref - ωm
    c.ui_speed += c.Ki_speed * e * Ts

    # Load feedforward sign follows the same convention as your IM model:
    # Te should oppose a positive tether torque if the goal is constant speed.
    Te_ff = c.use_load_feedforward ? -Tload : 0.0
    Te_ref_unsat = c.Kp_speed * e + c.ui_speed + Te_ff
    Te_ref = clamp(Te_ref_unsat, -c.Te_max, c.Te_max)

    # Basic anti-windup by back-calculating when saturated.
    if Te_ref != Te_ref_unsat
        c.ui_speed += (Te_ref - Te_ref_unsat) * 0.1
    end

    # First-order torque actuator with torque-rate limit.
    Te_dyn = c.Te + Ts / max(c.tau_Te, Ts) * (Te_ref - c.Te)
    dTe = clamp(Te_dyn - c.Te, -c.Te_rate_max * Ts, c.Te_rate_max * Ts)
    c.Te += dTe

    return DriveStepOutput(
        Te = c.Te,
        Te_ref = Te_ref,
        Pmech = c.Te * ωm,
        saturated = abs(Te_ref_unsat) > c.Te_max,
    )
end
