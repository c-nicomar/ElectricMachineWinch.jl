# Setup script for local development.
#
# Put ElectricMachineWinch.jl next to your local projects, for example:
#
#   repos/
#     ElectricMachineWinch.jl/
#     IM_AWES_bench/
#     WinchModels.jl/
#     KiteSimulators.jl/
#
# Then run from inside ElectricMachineWinch.jl:
#
#   julia --project=. scripts/setup_local_deps.jl

using Pkg

root = normpath(joinpath(@__DIR__, "..", ".."))

im_path = joinpath(root, "IM_AWES_bench.jl")
winch_path = joinpath(root, "WinchModels.jl")

if !isdir(im_path)
    error("Could not find IM_AWES_bench at: $im_path")
end

if !isdir(winch_path)
    error("Could not find WinchModels.jl at: $winch_path")
end

Pkg.activate(joinpath(@__DIR__, ".."))
Pkg.develop(path = im_path)
Pkg.develop(path = winch_path)
Pkg.instantiate()
Pkg.precompile()

println("Local dependencies configured successfully.")
println("Now try:")
println("  julia --project=. test/manual_test_ideal_torque.jl")
println("  julia --project=. test/manual_test_foc_speed_f1.jl")
