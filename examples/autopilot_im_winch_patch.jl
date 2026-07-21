# This is NOT a complete copy of KiteSimulators/examples/autopilot.jl.
# It is the minimal patch you apply to a copied autopilot example.
#
# Recommended workflow:
#   1. In KiteSimulators.jl, copy examples/autopilot.jl to examples/autopilot_im_winch.jl
#   2. Add `using ElectricMachineWinch` near the other using statements.
#   3. After `app.kps4 = KPS4(app.kcu)`, insert one of the blocks below.
#
# First integration test: simple ideal torque controller

using ElectricMachineWinch

app.kps4.wm = make_electric_winch(
    controller = :ideal,
    drum_radius = 0.5,
    gear_ratio = 10.0,
    J_eq = 0.3685,
    B_eq = 0.01298,
    Ts = app.dt,        # easiest first test: same sample time as autopilot
    Te_max = 124.0,
)

# Second test: full FOC speed controller + IM electrical plant
#
# app.kps4.wm = make_electric_winch(
#     controller = :foc_speed_f1,
#     drum_radius = 0.5,
#     gear_ratio = 10.0,
#     J_eq = 0.3685,
#     B_eq = 0.01298,
#     Ts = 100e-6,
#     plant_substeps = 1,
#     Vs_max = 310.0,
#     Is_max = 40.0,
#     Te_max = 124.0,
#     speed_ts_wm = 0.5,
#     use_load_feedforward = false,
# )
