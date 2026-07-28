# Unit test for the custom winch with the simple IdealTorqueController.
#
# Checks the coupling:
#   set_speed -> controller torque -> winch acceleration -> speed response

using Test
using ElectricMachineWinch
using WinchModels

@testset "IdealTorqueController winch coupling" begin
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
    v_set = 0.0               # reel-out speed reference [m/s]
    dt = wm.Ts

    local a
    for k in 1:20_000
        a = WinchModels.calc_acceleration(wm, v, F; set_speed = v_set)
        v += dt * a
    end
    s = last_summary(wm)

    # Speed settles at the reference despite the constant tether load.
    @test isapprox(v, v_set; atol = 1e-3)
    @test isfinite(a)

    # Tload = drum_radius/gear_ratio * F = 25 N.m; at steady state Te opposes it.
    @test isapprox(s.Te, -25.0; atol = 0.5)
    @test isapprox(s.Te_ref, -25.0; atol = 0.5)
    @test isfinite(s.Te)
    @test !s.saturated

    @test wm.n_acceleration_calls == 20_000
end
