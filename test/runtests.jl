# Lightweight package smoke test.
# This is intentionally minimal because the real package depends on local
# development paths for WinchModels and IM_AWES_bench.

using Test
using ElectricMachineWinch
using WinchModels

@testset "ElectricMachineWinch smoke tests" begin
    wm = make_electric_winch(controller = :ideal)
    a = WinchModels.calc_acceleration(wm, 0.0, 500.0; set_speed = 1.0)
    @test isfinite(a)
    @test isfinite(wm.Te)
    @test wm.n_acceleration_calls == 1
end
