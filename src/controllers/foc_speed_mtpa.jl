# Wrapper around the constrained-MTPA speed controller implemented in
# InductionMachineDrives.
#
# Control path:
#
#   rotor-flux observer
#       -> outer speed + constrained-MTPA loop
#       -> inner dq current controller
#       -> inverse Park transformation
#       -> electrical-only induction-machine RK4 plant
#       -> electromagnetic torque
#
# The mechanical speed is imposed from KiteModels through the
# ElectricMachineWinch interface. This controller advances only the electrical
# machine states; it does not integrate a second mechanical-speed state.

"""
    FOCSpeedMTPAController

ElectricMachineWinch adapter for the discrete constrained-MTPA speed controller
from `InductionMachineDrives`.

The object owns the states and parameters of:

1. the discrete rotor-flux observer;
2. the outer speed/MTPA controller;
3. the inner discrete current controller.

`last_outer` retains the complete `OuterSpeedFluxMTPAOutput` from the most
recent controller step so that the KiteControllers example can log MTPA-specific
diagnostics such as `isd_mtpa`, `isd_floor`, `isd_reserve`,
`Te_current_limited`, and `torque_current_limited`.
"""
mutable struct FOCSpeedMTPAController <: AbstractIMDriveController
    obs_state::IMB.RotorFluxObserverDiscreteState
    obs_p::IMB.RotorFluxObserverDiscreteParams

    outer_state::IMB.OuterSpeedFluxMTPAState
    outer_p::IMB.OuterSpeedFluxMTPAParams

    current_state::IMB.CurrentControllerDiscreteState
    current_p::IMB.CurrentControllerDiscreteParams

    last_outer::IMB.OuterSpeedFluxMTPAOutput
end

"""
    FOCSpeedMTPAController(; obs_p, outer_p, current_p)

Construct an MTPA speed-controller adapter from explicit observer, outer-loop,
and current-controller parameter structures.
"""
function FOCSpeedMTPAController(;
    obs_p::IMB.RotorFluxObserverDiscreteParams =
        IMB.RotorFluxObserverDiscreteParams(),

    outer_p::IMB.OuterSpeedFluxMTPAParams =
        IMB.OuterSpeedFluxMTPAParams(),

    current_p::IMB.CurrentControllerDiscreteParams =
        IMB.CurrentControllerDiscreteParams(),
)
    outer_state = IMB.OuterSpeedFluxMTPAState(
        id_ref_ant = outer_p.isd_nom,
    )

    return FOCSpeedMTPAController(
        IMB.RotorFluxObserverDiscreteState(),
        obs_p,
        outer_state,
        outer_p,
        IMB.CurrentControllerDiscreteState(),
        current_p,
        IMB.OuterSpeedFluxMTPAOutput(),
    )
end

"""
    reset!(controller::FOCSpeedMTPAController)

Reset all discrete controller/observer states and clear the stored MTPA
diagnostics.
"""
function reset!(c::FOCSpeedMTPAController)
    c.obs_state = IMB.RotorFluxObserverDiscreteState()

    c.outer_state = IMB.OuterSpeedFluxMTPAState(
        id_ref_ant = c.outer_p.isd_nom,
    )

    c.current_state = IMB.CurrentControllerDiscreteState()
    c.last_outer = IMB.OuterSpeedFluxMTPAOutput()

    return nothing
end

"""
    make_default_foc_speed_mtpa_controller(; kwargs...)

Create a coherent constrained-MTPA FOC speed controller for use inside
`DetailedIMWinch`.

The parameters `J` and `B` must describe the same equivalent machine-side
mechanics as `DetailedIMWinch.J_eq` and `DetailedIMWinch.B_eq`. The
`make_electric_winch` constructor should therefore pass its `J_eq` and `B_eq`
values into this function.

The MTPA torque model is

    Te = Kt_isd * isd * isq

where

    Kt_isd = 1.5 * pole_pairs * Lm^2 / Lrr.

`Is_max` is interpreted as the maximum magnitude of the dq current vector.
"""
function make_default_foc_speed_mtpa_controller(;
    # Common discrete sampling period.
    Ts::Float64 = 100e-6,

    # Inner-controller limits.
    Vs_max::Float64 = 310.0,
    Is_max::Float64 = 40.0 * sqrt(2.0),
    Te_max::Float64 = 124.0419647,

    # Machine constants used by the MTPA torque-current map.
    pole_pairs::Float64 = 2.0,
    Lm::Float64 = 40.84e-3,
    Lrr::Float64 = 45.12e-3,

    # Equivalent machine-side mechanical parameters used by the speed PI.
    J::Float64 = 0.3685,
    B::Float64 = 0.01298,

    # Speed-loop design.
    speed_ts_wm::Float64 = 0.5,
    speed_tau_f_wm::Float64 = 10e-3,
    speed_ts_dist_wm::Float64 = 3.0,
    wm_dot_max::Float64 = 100.0,

    # Constrained-MTPA current settings.
    isd_nom::Float64 = 23.04579328,
    isd_min::Float64 = 5.0,
    id_dot_max::Float64 = 600.0,
    lambda_rd_floor::Float64 = 0.35,
    Te_reserve::Float64 = 45.0,

    # Tether/load-torque feedforward.
    use_load_feedforward::Bool = false,
    load_ff_sign::Float64 = -1.0,
)
    Ts > 0.0 ||
        throw(ArgumentError("Ts must be positive."))

    Vs_max > 0.0 ||
        throw(ArgumentError("Vs_max must be positive."))

    Is_max > 0.0 ||
        throw(ArgumentError("Is_max must be positive."))

    Te_max > 0.0 ||
        throw(ArgumentError("Te_max must be positive."))

    pole_pairs > 0.0 ||
        throw(ArgumentError("pole_pairs must be positive."))

    Lm > 0.0 ||
        throw(ArgumentError("Lm must be positive."))

    Lrr > Lm ||
        throw(ArgumentError("Lrr must be greater than Lm."))

    J > 0.0 ||
        throw(ArgumentError("J must be positive."))

    B >= 0.0 ||
        throw(ArgumentError("B must be non-negative."))

    0.0 <= isd_min <= Is_max ||
        throw(ArgumentError("isd_min must lie in [0, Is_max]."))

    0.0 <= isd_nom <= Is_max ||
        throw(ArgumentError("isd_nom must lie in [0, Is_max]."))

    id_dot_max > 0.0 ||
        throw(ArgumentError("id_dot_max must be positive."))

    lambda_rd_floor >= 0.0 ||
        throw(ArgumentError("lambda_rd_floor must be non-negative."))

    Te_reserve >= 0.0 ||
        throw(ArgumentError("Te_reserve must be non-negative."))

    obs_p = IMB.RotorFluxObserverDiscreteParams(
        Ts = Ts,
    )

    outer_p = IMB.OuterSpeedFluxMTPAParams(
        Ts = Ts,

        p = pole_pairs,
        Lm = Lm,
        Lrr = Lrr,

        J = J,
        B = B,

        isd_nom = isd_nom,
        Is_max = Is_max,
        isd_min = isd_min,
        Te_max = Te_max,

        wm_dot_max = wm_dot_max,
        id_dot_max = id_dot_max,

        lambda_rd_floor = lambda_rd_floor,
        Te_reserve = Te_reserve,

        tau_f_wm = speed_tau_f_wm,
        ts_wm = speed_ts_wm,
        ts_dist_wm = speed_ts_dist_wm,

        use_load_feedforward = use_load_feedforward,
        load_ff_sign = load_ff_sign,
    )

    current_p = IMB.CurrentControllerDiscreteParams(
        Ts = Ts,
        Vs_max = Vs_max,
        Is_max = Is_max,
    )

    return FOCSpeedMTPAController(
        obs_p = obs_p,
        outer_p = outer_p,
        current_p = current_p,
    )
end

"""
    drive_step!(
        controller::FOCSpeedMTPAController,
        plant::InductionMachinePlant;
        ωm_ref,
        ωm,
        Tload,
        Ts,
        plant_substeps = 1,
    )

Advance the complete electrical drive by one controller sample.

Arguments
---------
- `ωm_ref`: machine mechanical-speed reference `[rad/s]`.
- `ωm`: machine mechanical speed imposed by KiteModels `[rad/s]`.
- `Tload`: equivalent machine-side tether/load torque `[N*m]`.
- `Ts`: controller/electrical step `[s]`.
- `plant_substeps`: electrical RK4 substeps inside `Ts`.

Returns
-------
A common `DriveStepOutput` used by `DetailedIMWinch`.
"""
function drive_step!(
    c::FOCSpeedMTPAController,
    plant::InductionMachinePlant;
    ωm_ref::Float64,
    ωm::Float64,
    Tload::Float64,
    Ts::Float64,
    plant_substeps::Int = 1,
)
    Ts > 0.0 ||
        throw(ArgumentError("Ts must be positive."))

    plant_substeps >= 1 ||
        throw(ArgumentError("plant_substeps must be at least 1."))

    # The three discrete blocks must use the same sample time as the winch
    # interface. This check catches inconsistent manual construction.
    Ts_tolerance = max(1e-12, 1e-9 * Ts)

    abs(c.obs_p.Ts - Ts) <= Ts_tolerance ||
        throw(ArgumentError(
            "Observer Ts ($(c.obs_p.Ts)) does not match drive-step Ts ($Ts).",
        ))

    abs(c.outer_p.Ts - Ts) <= Ts_tolerance ||
        throw(ArgumentError(
            "Outer-controller Ts ($(c.outer_p.Ts)) does not match " *
            "drive-step Ts ($Ts).",
        ))

    abs(c.current_p.Ts - Ts) <= Ts_tolerance ||
        throw(ArgumentError(
            "Current-controller Ts ($(c.current_p.Ts)) does not match " *
            "drive-step Ts ($Ts).",
        ))

    # ------------------------------------------------------------------
    # 1. Synchronize the electrical machine model with the mechanical
    #    speed state owned by KiteModels.
    # ------------------------------------------------------------------
    plant.x = IMB.IMPlantState(
        isα = plant.x.isα,
        isβ = plant.x.isβ,
        irα = plant.x.irα,
        irβ = plant.x.irβ,
        ωm = ωm,
        θm = plant.x.θm,
    )

    # ------------------------------------------------------------------
    # 2. Rotor-flux observer.
    # ------------------------------------------------------------------
    obs = IMB.rotor_flux_observer_step!(
        c.obs_state,
        c.obs_p;
        i_alpha = plant.x.isα,
        i_beta = plant.x.isβ,
        theta_m = plant.x.θm,
        omega_m = ωm,
        reset = false,
    )

    # ------------------------------------------------------------------
    # 3. Outer speed loop plus constrained-MTPA current allocation.
    # ------------------------------------------------------------------
    outer = IMB.outer_speed_flux_mtpa_step!(
        c.outer_state,
        c.outer_p;
        wm_ref = ωm_ref,
        wm_med = ωm,
        TL_est = Tload,
        reset = false,
    )

    c.last_outer = outer

    # ------------------------------------------------------------------
    # 4. Inner dq current controller.
    # ------------------------------------------------------------------
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

    # ------------------------------------------------------------------
    # 5. dq voltage command to stationary alpha-beta coordinates.
    # ------------------------------------------------------------------
    vsα, vsβ = inverse_park_voltage(
        ctrl.vsd,
        ctrl.vsq,
        obs.theta_e,
    )

    # ------------------------------------------------------------------
    # 6. Advance only the electrical machine states.
    #
    #    KiteModels owns the winch/machine mechanical-speed state, so the
    #    machine speed is imposed during every electrical RK4 substep.
    # ------------------------------------------------------------------
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

    # ------------------------------------------------------------------
    # 7. Common drive outputs.
    # ------------------------------------------------------------------
    Te = im_torque(
        plant.x,
        plant.p,
    )

    Pelec = phase_power_alpha_beta(
        vsα,
        vsβ,
        plant.x.isα,
        plant.x.isβ,
    )

    Pmech = Te * ωm

    outer_saturated =
        outer.sat_isd != 0 ||
        outer.sat_isq != 0 ||
        outer.sat_Te != 0 ||
        outer.torque_current_limited != 0

    return DriveStepOutput(
        Te = Te,

        # Keep the same meaning used by the existing F1 adapter:
        # requested outer-loop electromagnetic torque before actual plant
        # tracking dynamics.
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

        saturated = ctrl.saturado || outer_saturated,
    )
end
