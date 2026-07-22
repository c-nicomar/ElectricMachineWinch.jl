# Convenience constructors for common test configurations.

"""
    make_electric_winch(; controller=:ideal, ...)

Create a `DetailedIMWinch` with either:
- `controller = :ideal` for first integration/debugging tests
- `controller = :foc_speed_f1` for the full IM_AWES_bench FOC-speed loop

Keyword arguments are intentionally explicit so you can quickly compare cases.
"""
function make_electric_winch(;
    controller = :ideal,
    drum_radius::Float64 = 0.5,
    gear_ratio::Float64 = 10.0,
    J_eq::Float64 = 0.3685,
    B_eq::Float64 = 0.01298,
    Ts::Float64 = 100e-6,
    plant_substeps::Int = 1,
    Vs_max::Float64 = 310.0,
    Is_max::Float64 = 40.0,
    Te_max::Float64 = 124.0,
    speed_ts_wm::Float64 = 0.5,
    use_load_feedforward::Bool = false,
    use_field_weakening::Bool = false,
    wm_base_fw::Float64 = 120.0,
)
    c = if controller == :ideal
        IdealTorqueController(Te_max = Te_max)
    elseif controller == :foc_speed_f1
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
    else
        error("Unknown controller = $controller. Use :ideal or :foc_speed_f1.")
    end

    plant_p = IMB.IMPlantParams(
        J = J_eq,
        B = B_eq,
    )

    return DetailedIMWinch(
        drum_radius = drum_radius,
        gear_ratio = gear_ratio,
        J_eq = J_eq,
        B_eq = B_eq,
        Ts = Ts,
        plant_substeps = plant_substeps,
        controller = c,
        plant = InductionMachinePlant(p = plant_p),
    )
end
