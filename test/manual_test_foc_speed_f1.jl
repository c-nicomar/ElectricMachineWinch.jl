# Manual standalone test for the custom winch with the full FOCSpeedF1Controller.
# Run from inside ElectricMachineWinch.jl:
#
#   julia --project=. test/manual_test_foc_speed_f1.jl
#
# This test does not require KiteSimulators. It checks the coupling of:
#   set_speed -> FOC speed loop -> current controller -> electrical IM -> Te -> acceleration

using ElectricMachineWinch
using WinchModels

wm = make_electric_winch(
    controller = :foc_speed_f1,
    drum_radius = 0.5,
    gear_ratio = 10.0,
    J_eq = 0.3685,
    B_eq = 0.01298,
    Ts = 100e-6,
    plant_substeps = 1,
    Vs_max = 310.0,
    Is_max = 40.0,
    Te_max = 124.0,
    speed_ts_wm = 0.5,
    use_load_feedforward = false,
)

v = 0.0
F = 500.0
v_set = 5.0
dt = wm.Ts

println("Manual FOC-speed-F1 winch test")
println("Expected behavior: currents/torque build up gradually; v should respond toward v_set if signs are correct.")
println()

for k in 1:50_0000
    a = WinchModels.calc_acceleration(wm, v, F; set_speed = v_set)
    global v += dt * a

    if k % 2500 == 0
        s = last_summary(wm)
        println(
            "t=", round(k*dt, digits=3),
            "  v=", round(v, digits=4),
            "  a=", round(a, digits=4),
            "  Te=", round(s.Te, digits=3),
            "  Te_ref=", round(s.Te_ref, digits=3),
            "  isd=", round(s.isd, digits=3),
            "  isq=", round(s.isq, digits=3),
            "  sat=", s.saturated,
        )
    end
end
