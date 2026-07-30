# WinchModels-compatible interface.
#
# This is the single method KiteModels needs. KiteModels calls
# `calc_acceleration(s.wm, v_reel_out, force; set_speed=..., set_torque=...)`.
# Because DetailedIMWinch subtypes AbstractWinchModel, Julia dispatches here.

function _store_last_output!(wm::DetailedIMWinch, out::DriveStepOutput)
    wm.Te = out.Te
    wm.Te_ref = out.Te_ref
    wm.isd_ref = out.isd_ref
    wm.isq_ref = out.isq_ref
    wm.isd = out.isd
    wm.isq = out.isq
    wm.vsd = out.vsd
    wm.vsq = out.vsq
    wm.vsα = out.vsα
    wm.vsβ = out.vsβ
    wm.Pelec = out.Pelec
    wm.Pmech = out.Pmech
    wm.saturated = out.saturated
    return nothing
end

"""
    WinchModels.calc_acceleration(wm::DetailedIMWinch, v_reelout, tether_force; set_speed, set_torque, use_brake)

Return reel-out acceleration [m/s²] for KiteModels.

Inputs from KiteModels:
- `v_reelout`: current reel-out velocity [m/s]
- `tether_force`: tether tension / force at the winch [N]
- `set_speed`: reel-out speed reference [m/s], used by the autopilot path
- `set_torque`: optional machine-side torque command [Nm], useful for direct tests

Internal sign convention:

    J*dω = Te + Tload - B*ω

where:
- `Tload = drum_radius / gear_ratio * tether_force`
- positive `Tload` pulls toward positive reel-out speed
- during positive-speed generation, `Te` is normally negative
"""
function WinchModels.calc_acceleration(
    wm::DetailedIMWinch,
    v_reelout,
    tether_force;
    set_torque = nothing,
    set_speed = nothing,
    use_brake = false,
)
    wm.n_acceleration_calls += 1

    R = wm.drum_radius
    n = wm.gear_ratio

    # Kite linear speed -> machine-side angular speed.
    ωm = n / R * Float64(v_reelout)

    # Tether force -> machine-side load torque.
    Tload = R / n * Float64(tether_force)

    if set_speed !== nothing
        # Autopilot mode: speed reference comes from KiteControllers.
        ωm_ref = n / R * Float64(set_speed)

        out = drive_step!(
            wm.controller,
            wm.plant;
            ωm_ref = ωm_ref,
            ωm = ωm,
            Tload = Tload,
            Ts = wm.Ts,
            plant_substeps = wm.plant_substeps,
        )
        _store_last_output!(wm, out)

    elseif set_torque !== nothing
        # Direct torque mode. Useful for unit tests and later for the cleaner
        # architecture where the electric drive is updated once outside the DAE.
        wm.Te = Float64(set_torque)
        wm.Te_ref = Float64(set_torque)
        wm.Pmech = wm.Te * ωm

    else
        # No new command: hold previous torque. This should rarely happen, but
        # makes the model robust to calls without set_speed/set_torque.
    end

    # Machine-side mechanical dynamics.
    dωm = (wm.Te + Tload - wm.B_eq * ωm) / wm.J_eq

    # Machine angular acceleration -> reel-out linear acceleration.
    return R / n * dωm
end

"""
    step_drive_from_kite!(wm, v_ro_set, v_ro_meas, tether_force; dt_outer)

Update the electric drive/controller once from the KiteControllers loop, but
internally run the electrical/FOC model with its own smaller timestep `wm.Ts`.

Returns the latest machine-side electromagnetic torque [Nm].
"""
function step_drive_from_kite!(
    wm::DetailedIMWinch,
    v_ro_set::Real,
    v_ro_meas::Real,
    tether_force::Real;
    dt_outer::Real = wm.Ts,
)
    R = wm.drum_radius
    n = wm.gear_ratio

    dt_outer_f = Float64(dt_outer)

    # Number of electric-drive updates during one kite macro-step.
    n_sub = max(1, round(Int, dt_outer_f / wm.Ts))

    # Adjust the internal step very slightly so the substeps exactly cover dt_outer.
    Ts_eff = dt_outer_f / n_sub

    last_out = DriveStepOutput()

    for _ in 1:n_sub
        # For the first implementation, hold kite-side variables constant
        # during the macro-step.
        ωm_ref = n / R * Float64(v_ro_set)
        ωm     = n / R * Float64(v_ro_meas)
        Tload  = R / n * Float64(tether_force)

        last_out = drive_step!(
            wm.controller,
            wm.plant;
            ωm_ref = ωm_ref,
            ωm = ωm,
            Tload = Tload,
            Ts = Ts_eff,
            plant_substeps = wm.plant_substeps,
        )
    end

    _store_last_output!(wm, last_out)

    return wm.Te
end
