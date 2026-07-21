# Manual standalone test for the custom winch with the simple IdealTorqueController.
# Run from inside ElectricMachineWinch.jl:
#
#   julia --project=. test/manual_test_ideal_torque.jl
#
# This test does not require KiteSimulators. It only checks:
#   set_speed -> controller torque -> winch acceleration -> speed response

using ElectricMachineWinch
using WinchModels

wm = make_electric_winch(
    controller = :ideal,
    drum_radius = 0.5,
    gear_ratio = 10.0,
    J_eq = 0.3685,
    B_eq = 0.01298,
    Ts = 100e-6,
    Te_max = 124.0,
)

v = 0.0                  # reel-out speed [m/s]
F = 500.0                # tether force [N]
v_set = 0.0              # reel-out speed reference [m/s]
dt = wm.Ts

println("Manual ideal-torque winch test")
println("Expected behavior: v should move toward v_set; Te usually opposes load if needed.")
println()

for k in 1:20_000
    a = WinchModels.calc_acceleration(wm, v, F; set_speed = v_set)
    global v += dt * a

    if k % 1000 == 0
        s = last_summary(wm)
        println(
            "t=", round(k*dt, digits=3),
            "  v=", round(v, digits=4),
            "  a=", round(a, digits=4),
            "  Te=", round(s.Te, digits=3),
            "  Te_ref=", round(s.Te_ref, digits=3),
            "  sat=", s.saturated,
        )
    end
end
