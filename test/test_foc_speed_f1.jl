# Unit test for the custom winch with the full FOCSpeedF1Controller.
#
# Checks the coupling:
#   set_speed -> FOC speed loop -> current controller -> electrical IM -> Te -> acceleration

using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

using Test
using ElectricMachineWinch
using WinchModels

@testset "FOCSpeedF1Controller winch coupling" begin
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

    local a
    for k in 1:500_000
        a = WinchModels.calc_acceleration(wm, v, F; set_speed = v_set)
        v += dt * a
    end
    s = last_summary(wm)

    # Reel-out speed converges to the reference (signs are correct).
    @test isapprox(v, v_set; atol = 1e-2)
    @test isfinite(a)

    # Generation at positive reel-out speed: Te opposes motion.
    @test s.Te < 0.0
    @test isfinite(s.Te)
    @test isfinite(s.isd)
    @test isfinite(s.isq)
    @test !s.saturated

    @test wm.n_acceleration_calls == 500_000
end
