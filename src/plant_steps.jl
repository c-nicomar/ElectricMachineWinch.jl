# Electrical-only induction-machine RK4 step.
#
# This mirrors the validated InductionMachineDrives alpha-beta IM equations, but it does
# NOT integrate mechanical speed using Te + Tload. KiteModels already owns the
# reel-out speed state. Here we impose the machine speed corresponding to the
# current kite reel-out speed and only integrate electrical currents/rotor states.

function _im_fluxes(x::IMB.IMPlantState, p::IMB.IMPlantParams)
    Ls = p.Lls + p.Lm
    Lr = p.Llr + p.Lm

    ψsα = Ls * x.isα + p.Lm * x.irα
    ψsβ = Ls * x.isβ + p.Lm * x.irβ

    ψrα = p.Lm * x.isα + Lr * x.irα
    ψrβ = p.Lm * x.isβ + Lr * x.irβ

    return ψsα, ψsβ, ψrα, ψrβ
end

"""
    im_torque(x, p)

Electromagnetic torque of the IM model, using the same expression as
InductionMachineDrives. Reimplemented here to avoid relying on non-exported internals.
"""
function im_torque(x::IMB.IMPlantState, p::IMB.IMPlantParams)
    ψsα, ψsβ, _, _ = _im_fluxes(x, p)
    return 1.5 * p.p * (ψsα * x.isβ - ψsβ * x.isα)
end

Base.@kwdef struct _ElectricalDerivatives
    disα::Float64 = 0.0
    disβ::Float64 = 0.0
    dirα::Float64 = 0.0
    dirβ::Float64 = 0.0
    dθm::Float64 = 0.0
end

function _electrical_derivatives(
    x::IMB.IMPlantState,
    p::IMB.IMPlantParams;
    vsα::Float64,
    vsβ::Float64,
    imposed_ωm::Float64,
)
    Ls = p.Lls + p.Lm
    Lr = p.Llr + p.Lm
    detL = Ls * Lr - p.Lm^2

    _, _, ψrα, ψrβ = _im_fluxes(x, p)

    dψsα = vsα - p.Rs * x.isα
    dψsβ = vsβ - p.Rs * x.isβ

    dψrα = -p.Rr * x.irα - p.p * imposed_ωm * ψrβ
    dψrβ = -p.Rr * x.irβ + p.p * imposed_ωm * ψrα

    disα = ( Lr * dψsα - p.Lm * dψrα) / detL
    dirα = (-p.Lm * dψsα + Ls * dψrα) / detL

    disβ = ( Lr * dψsβ - p.Lm * dψrβ) / detL
    dirβ = (-p.Lm * dψsβ + Ls * dψrβ) / detL

    return _ElectricalDerivatives(
        disα = disα,
        disβ = disβ,
        dirα = dirα,
        dirβ = dirβ,
        dθm = imposed_ωm,
    )
end

function _add_scaled_electrical(
    x::IMB.IMPlantState,
    dx::_ElectricalDerivatives,
    h::Float64;
    imposed_ωm::Float64,
)
    return IMB.IMPlantState(
        isα = x.isα + h * dx.disα,
        isβ = x.isβ + h * dx.disβ,
        irα = x.irα + h * dx.dirα,
        irβ = x.irβ + h * dx.dirβ,
        ωm = imposed_ωm,
        θm = x.θm + h * dx.dθm,
    )
end

"""
    rk4_step_electrical_only(x, p, h; vsα, vsβ, imposed_ωm)

Integrate only the electrical αβ IM dynamics and mechanical angle for one RK4
substep. Mechanical speed is imposed from KiteModels reel-out speed.
"""
function rk4_step_electrical_only(
    x::IMB.IMPlantState,
    p::IMB.IMPlantParams,
    h::Float64;
    vsα::Float64,
    vsβ::Float64,
    imposed_ωm::Float64,
)
    x0 = IMB.IMPlantState(
        isα = x.isα,
        isβ = x.isβ,
        irα = x.irα,
        irβ = x.irβ,
        ωm = imposed_ωm,
        θm = x.θm,
    )

    k1 = _electrical_derivatives(x0, p; vsα = vsα, vsβ = vsβ, imposed_ωm = imposed_ωm)
    k2 = _electrical_derivatives(_add_scaled_electrical(x0, k1, h / 2; imposed_ωm = imposed_ωm), p; vsα = vsα, vsβ = vsβ, imposed_ωm = imposed_ωm)
    k3 = _electrical_derivatives(_add_scaled_electrical(x0, k2, h / 2; imposed_ωm = imposed_ωm), p; vsα = vsα, vsβ = vsβ, imposed_ωm = imposed_ωm)
    k4 = _electrical_derivatives(_add_scaled_electrical(x0, k3, h; imposed_ωm = imposed_ωm), p; vsα = vsα, vsβ = vsβ, imposed_ωm = imposed_ωm)

    return IMB.IMPlantState(
        isα = x0.isα + h / 6 * (k1.disα + 2k2.disα + 2k3.disα + k4.disα),
        isβ = x0.isβ + h / 6 * (k1.disβ + 2k2.disβ + 2k3.disβ + k4.disβ),
        irα = x0.irα + h / 6 * (k1.dirα + 2k2.dirα + 2k3.dirα + k4.dirα),
        irβ = x0.irβ + h / 6 * (k1.dirβ + 2k2.dirβ + 2k3.dirβ + k4.dirβ),
        ωm = imposed_ωm,
        θm = x0.θm + h / 6 * (k1.dθm + 2k2.dθm + 2k3.dθm + k4.dθm),
    )
end

"""
    inverse_park_voltage(vsd, vsq, theta_e)

Inverse Park transformation of the stator voltage: rotate the dq voltage
`(vsd, vsq)` by the electrical field angle `theta_e` [rad] into the stationary
αβ frame. Returns the tuple `(vsα, vsβ)` fed to
[`rk4_step_electrical_only`](@ref).
"""
function inverse_park_voltage(vsd::Float64, vsq::Float64, theta_e::Float64)
    vsα = cos(theta_e) * vsd - sin(theta_e) * vsq
    vsβ = sin(theta_e) * vsd + cos(theta_e) * vsq
    return vsα, vsβ
end

"""
    phase_power_alpha_beta(vsα, vsβ, isα, isβ)

Instantaneous three-phase electrical power [W] of the stator, computed by
expanding the αβ voltages and currents back into phase quantities and summing
`va*ia + vb*ib + vc*ic`. Reported as `Pelec` in [`DriveStepOutput`](@ref).
"""
function phase_power_alpha_beta(vsα::Float64, vsβ::Float64, isα::Float64, isβ::Float64)
    ia = isα
    ib = -0.5 * isα + sqrt(3) / 2 * isβ
    ic = -0.5 * isα - sqrt(3) / 2 * isβ

    va = vsα
    vb = -0.5 * vsα + sqrt(3) / 2 * vsβ
    vc = -0.5 * vsα - sqrt(3) / 2 * vsβ

    return va * ia + vb * ib + vc * ic
end
