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

include("test_ideal_torque.jl")
include("test_foc_speed_f1.jl")
nothing
