using Pkg
if Base.active_project() != joinpath(@__DIR__, "Project.toml")
    Pkg.activate(joinpath(@__DIR__))
end

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

@testset "docstring coverage of exports" begin
    # docs/make.jl runs with checkdocs = :all, which verifies that every
    # docstring that exists is published -- it is blind to an export with no
    # docstring at all. This testset is what keeps coverage at 100%.
    meta = Base.Docs.meta(ElectricMachineWinch)
    undocumented = [
        name for name in names(ElectricMachineWinch)
        if !haskey(meta, Base.Docs.Binding(ElectricMachineWinch, name))
    ]
    @test isempty(undocumented)
end

include("test_ideal_torque.jl")
include("test_foc_speed_f1.jl")
nothing
