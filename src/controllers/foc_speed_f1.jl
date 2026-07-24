# Wrapper around the user's validated IM_AWES_bench FOC speed controller.
#
# This controller preserves the existing structure:
#   rotor flux observer -> outer speed/flux F1 loop -> current controller ->
#   alpha-beta voltage -> electrical-only IM RK4 plant -> electromagnetic torque.

"""
    FOCSpeedF1Controller

Controller object holding the observer, outer speed/flux F1 loop, and current
controller states and parameters from `IM_AWES_bench`.
"""
mutable struct FOCSpeedF1Controller <: AbstractIMDriveController
    obs_state::IMB.RotorFluxObserverDiscreteState
    obs_p::IMB.RotorFluxObserverDiscreteParams

    outer_state::IMB.OuterSpeedFluxF1State
    outer_p::IMB.OuterSpeedFluxF1Params

    current_state::IMB.CurrentControllerDiscreteState
    current_p::IMB.CurrentControllerDiscreteParams
end

function FOCSpeedF1Controller(;
    obs_p = IMB.RotorFluxObserverDiscreteParams(),
    outer_p = IMB.OuterSpeedFluxF1Params(),
    current_p = IMB.CurrentControllerDiscreteParams(),
)
    return FOCSpeedF1Controller(
        IMB.RotorFluxObserverDiscreteState(),
        obs_p,
        IMB.OuterSpeedFluxF1State(
            id_ref_ant = outer_p.isd_nom,
        ),
        outer_p,
        IMB.CurrentControllerDiscreteState(),
        current_p,
    )
end

function reset!(c::FOCSpeedF1Controller)
    c.obs_state = IMB.RotorFluxObserverDiscreteState()
    c.outer_state = IMB.OuterSpeedFluxF1State(
        id_ref_ant = c.outer_p.isd_nom,
    )
    c.current_state = IMB.CurrentControllerDiscreteState()
    return nothing
end

"""
    make_default_foc_speed_controller(; Ts, Vs_max, Is_max, ...)

Convenience constructor that creates a coherent FOC speed controller using the
same default machine constants as your uploaded benchmark package.
"""
function make_default_foc_speed_controller(;
    Ts::Float64 = 100e-6,
    Vs_max::Float64 = 310.0,
    Is_max::Float64 = 40.0*sqrt(2),
    speed_ts_wm::Float64 = 0.5,
    speed_tau_f_wm::Float64 = 10e-3,
    speed_ts_dist_wm::Float64 = 3.0,
    wm_dot_max::Float64 = 100.0,
    Te_max::Float64 = 124.0419647,
    use_load_feedforward::Bool = false,
    use_field_weakening::Bool = false,
    wm_base_fw::Float64 = 120.0,
)
    obs_p = IMB.RotorFluxObserverDiscreteParams(Ts = Ts)

    outer_p = IMB.OuterSpeedFluxF1Params(
        Ts = Ts,
        Is_max = Is_max,
        Te_max = Te_max,
        wm_dot_max = wm_dot_max,
        tau_f_wm = speed_tau_f_wm,
        ts_wm = speed_ts_wm,
        ts_dist_wm = speed_ts_dist_wm,
        use_load_feedforward = use_load_feedforward,
        use_field_weakening = use_field_weakening,
        wm_base_fw = wm_base_fw,
    )

    current_p = IMB.CurrentControllerDiscreteParams(
        Ts = Ts,
        Vs_max = Vs_max,
        Is_max = Is_max,
    )

    return FOCSpeedF1Controller(
        obs_p = obs_p,
        outer_p = outer_p,
        current_p = current_p,
    )
end

function drive_step!(
    c::FOCSpeedF1Controller,
    plant::InductionMachinePlant;
    ωm_ref::Float64,
    ωm::Float64,
    Tload::Float64,
    Ts::Float64,
    plant_substeps::Int = 1,
)
    # 1. Synchronize machine mechanical speed with KiteModels.
    plant.x = IMB.IMPlantState(
        isα = plant.x.isα,
        isβ = plant.x.isβ,
        irα = plant.x.irα,
        irβ = plant.x.irβ,
        ωm = ωm,
        θm = plant.x.θm,
    )

    # 2. Rotor-flux observer.
    obs = IMB.rotor_flux_observer_step!(
        c.obs_state,
        c.obs_p;
        i_alpha = plant.x.isα,
        i_beta = plant.x.isβ,
        theta_m = plant.x.θm,
        omega_m = ωm,
        reset = false,
    )

    # 3. Outer speed + F1 flux loop.
    outer = IMB.outer_speed_flux_f1_step!(
        c.outer_state,
        c.outer_p;
        wm_ref = ωm_ref,
        wm_med = ωm,
        TL_est = Tload,
        reset = false,
    )
  

    # 4. Inner current controller.
    ctrl = IMB.current_controller_step!(
        c.current_state,
        c.current_p;
        isd_ref = outer.isd_ref,
        isq_ref = outer.isq_ref,
        isd_med = obs.i_sd_e,
        isq_med = obs.i_sq_e,
        omega_e = obs.omega_e,
        lambda_rd = obs.lambda_rd_e,
        reset = false,
    )

    # 5. dq voltage command to stationary alpha-beta.
    vsα, vsβ = inverse_park_voltage(ctrl.vsd, ctrl.vsq, obs.theta_e)

    # 6. Integrate the electrical plant only.
    h = Ts / plant_substeps
    for _ in 1:plant_substeps
        plant.x = rk4_step_electrical_only(
            plant.x,
            plant.p,
            h;
            vsα = vsα,
            vsβ = vsβ,
            imposed_ωm = ωm,
        )
    end

    # 7. Compute outputs.
    Te = im_torque(plant.x, plant.p)
    Pelec = phase_power_alpha_beta(vsα, vsβ, plant.x.isα, plant.x.isβ)
    Pmech = Te * ωm

    return DriveStepOutput(
        Te = Te,
        Te_ref = outer.Te_ref_out,
        isd_ref = outer.isd_ref,
        isq_ref = outer.isq_ref,
        isd = obs.i_sd_e,
        isq = obs.i_sq_e,
        vsd = ctrl.vsd,
        vsq = ctrl.vsq,
        vsα = vsα,
        vsβ = vsβ,
        Pelec = Pelec,
        Pmech = Pmech,
        saturated = ctrl.saturado,
    )
end
