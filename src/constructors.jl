# Convenience constructors for common test configurations.

"""
    make_electric_winch(; controller=:ideal, ...)

Create a `DetailedIMWinch` with one of the available drive controllers:

- `controller = :ideal`
  Ideal electromagnetic-torque source for initial integration/debugging.

- `controller = :foc_speed_f1`
  Existing InductionMachineDrives FOC speed controller with the F1 flux strategy.

- `controller = :foc_speed_mtpa`
  Discrete FOC speed controller with constrained MTPA current allocation.

The mechanical parameters `J_eq` and `B_eq` are used both by the winch model
and by the MTPA speed controller. This keeps the speed-loop tuning and
mechanical feedforward consistent with the plant seen by KiteModels.

Notes
-----
- `use_field_weakening` and `wm_base_fw` currently apply only to
  `:foc_speed_f1`.
- The present `:foc_speed_mtpa` controller does not yet implement field
  weakening.
- `Is_max` is interpreted as the maximum magnitude of the dq current vector.
"""
function make_electric_winch(;
    controller = :ideal,

    # Winch and equivalent machine-side mechanics.
    drum_radius::Float64 = 0.5,
    gear_ratio::Float64 = 10.0,
    J_eq::Float64 = 0.3685,
    B_eq::Float64 = 0.01298,

    # Electrical/control discretization.
    Ts::Float64 = 100e-6,
    plant_substeps::Int = 1,

    # Common drive limits.
    Vs_max::Float64 = 310.0,
    Is_max::Float64 = 40.0,
    Te_max::Float64 = 124.0,

    # Common speed-loop settings.
    speed_ts_wm::Float64 = 0.5,
    speed_tau_f_wm::Float64 = 10e-3,
    speed_ts_dist_wm::Float64 = 3.0,
    wm_dot_max::Float64 = 100.0,

    # Load-torque feedforward.
    use_load_feedforward::Bool = false,
    load_ff_sign::Float64 = -1.0,

    # F1-only field-weakening settings.
    use_field_weakening::Bool = false,
    wm_base_fw::Float64 = 120.0,

    # MTPA machine constants.
    pole_pairs::Float64 = 2.0,
    Lm::Float64 = 40.84e-3,
    Lrr::Float64 = 45.12e-3,

    # MTPA current-allocation settings.
    isd_nom::Float64 = 23.04579328,
    isd_min::Float64 = 5.0,
    id_dot_max::Float64 = 600.0,
    lambda_rd_floor::Float64 = 0.35,
    Te_reserve::Float64 = 45.0,

    # Optional explicit InductionMachineDrives parameter objects.
    #
    # When all three F1 controller parameter objects are supplied, the
    # constructor uses them directly instead of the small-machine defaults.
    #
    # `plant_p` is independent from the controller parameter objects so that
    # plant/controller mismatch and robustness studies remain possible.
    plant_p::Union{Nothing,IMB.IMPlantParams} = nothing,
    obs_p::Union{Nothing,IMB.RotorFluxObserverDiscreteParams} = nothing,
    outer_f1_p::Union{Nothing,IMB.OuterSpeedFluxF1Params} = nothing,
    current_p::Union{Nothing,IMB.CurrentControllerDiscreteParams} = nothing,
)
    # ------------------------------------------------------------
    # Basic winch-level validation
    # ------------------------------------------------------------
    drum_radius > 0.0 ||
        throw(ArgumentError("drum_radius must be positive."))

    gear_ratio > 0.0 ||
        throw(ArgumentError("gear_ratio must be positive."))

    J_eq > 0.0 ||
        throw(ArgumentError("J_eq must be positive."))

    B_eq >= 0.0 ||
        throw(ArgumentError("B_eq must be non-negative."))

    Ts > 0.0 ||
        throw(ArgumentError("Ts must be positive."))

    plant_substeps >= 1 ||
        throw(ArgumentError("plant_substeps must be at least 1."))

    Vs_max > 0.0 ||
        throw(ArgumentError("Vs_max must be positive."))

    Is_max > 0.0 ||
        throw(ArgumentError("Is_max must be positive."))

    Te_max > 0.0 ||
        throw(ArgumentError("Te_max must be positive."))

    # ------------------------------------------------------------
    # Validate explicit F1 parameter selection.
    #
    # Custom F1 configuration is all-or-nothing. This prevents accidentally
    # combining a new-machine observer with the default small-machine outer
    # or current controller.
    # ------------------------------------------------------------
    has_custom_f1 =
        obs_p !== nothing ||
        outer_f1_p !== nothing ||
        current_p !== nothing

    missing_custom_f1 =
        obs_p === nothing ||
        outer_f1_p === nothing ||
        current_p === nothing

    if controller != :foc_speed_f1 && has_custom_f1
        throw(ArgumentError(
            "obs_p, outer_f1_p, and current_p are only valid when " *
            "controller = :foc_speed_f1.",
        ))
    end

    if controller == :foc_speed_f1 && has_custom_f1 && missing_custom_f1
        throw(ArgumentError(
            "Custom F1 configuration requires all three parameter objects: " *
            "obs_p, outer_f1_p, and current_p.",
        ))
    end

    if controller == :foc_speed_f1 && has_custom_f1
        isapprox(obs_p.Ts, Ts; rtol = 1e-12, atol = 0.0) ||
            throw(ArgumentError(
                "obs_p.Ts ($(obs_p.Ts)) must match winch Ts ($Ts).",
            ))

        isapprox(outer_f1_p.Ts, Ts; rtol = 1e-12, atol = 0.0) ||
            throw(ArgumentError(
                "outer_f1_p.Ts ($(outer_f1_p.Ts)) must match winch Ts ($Ts).",
            ))

        isapprox(current_p.Ts, Ts; rtol = 1e-12, atol = 0.0) ||
            throw(ArgumentError(
                "current_p.Ts ($(current_p.Ts)) must match winch Ts ($Ts).",
            ))
    end

    # ------------------------------------------------------------
    # Select and construct the drive controller
    # ------------------------------------------------------------
    c = if controller == :ideal
        IdealTorqueController(
            Te_max = Te_max,
        )

    elseif controller == :foc_speed_f1
        if has_custom_f1
            FOCSpeedF1Controller(
                obs_p = obs_p,
                outer_p = outer_f1_p,
                current_p = current_p,
            )
        else
            make_default_foc_speed_controller(
                Ts = Ts,
                Vs_max = Vs_max,
                Is_max = Is_max,
                Te_max = Te_max,

                speed_ts_wm = speed_ts_wm,
                use_load_feedforward = use_load_feedforward,

                use_field_weakening = use_field_weakening,
                wm_base_fw = wm_base_fw,
            )
        end

    elseif controller == :foc_speed_mtpa
        make_default_foc_speed_mtpa_controller(
            Ts = Ts,

            Vs_max = Vs_max,
            Is_max = Is_max,
            Te_max = Te_max,

            pole_pairs = pole_pairs,
            Lm = Lm,
            Lrr = Lrr,

            # Keep the controller mechanical model consistent with
            # DetailedIMWinch and KiteModels.
            J = J_eq,
            B = B_eq,

            speed_ts_wm = speed_ts_wm,
            speed_tau_f_wm = speed_tau_f_wm,
            speed_ts_dist_wm = speed_ts_dist_wm,
            wm_dot_max = wm_dot_max,

            isd_nom = isd_nom,
            isd_min = isd_min,
            id_dot_max = id_dot_max,
            lambda_rd_floor = lambda_rd_floor,
            Te_reserve = Te_reserve,

            use_load_feedforward = use_load_feedforward,
            load_ff_sign = load_ff_sign,
        )

    else
        throw(ArgumentError(
            "Unknown controller = $controller. " *
            "Use :ideal, :foc_speed_f1, or :foc_speed_mtpa.",
        ))
    end

    # ------------------------------------------------------------
    # Electrical-machine plant
    #
    # KiteModels owns the mechanical integration during autopilot
    # operation, but these values are retained in the plant parameter
    # structure for consistency and standalone use.
    # ------------------------------------------------------------
    selected_plant_p = if plant_p === nothing
        IMB.IMPlantParams(
            J = J_eq,
            B = B_eq,
        )
    else
        plant_p
    end

    return DetailedIMWinch(
        drum_radius = drum_radius,
        gear_ratio = gear_ratio,

        J_eq = J_eq,
        B_eq = B_eq,

        Ts = Ts,
        plant_substeps = plant_substeps,

        controller = c,
        plant = InductionMachinePlant(
            p = selected_plant_p,
        ),
    )
end