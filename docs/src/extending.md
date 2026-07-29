# Extending the package with another controller

```@meta
CurrentModule = ElectricMachineWinch
```

Adding a controller requires no change to KiteControllers, KiteModels or
KiteControllers — the whole integration is written against
[`DriveStepOutput`](@ref).

Create a new controller subtype:

```julia
mutable struct MyController <: AbstractIMDriveController
    # states and parameters
end
```

Implement [`drive_step!`](@ref):

```julia
function drive_step!(
    controller::MyController,
    plant::InductionMachinePlant;
    ωm_ref,
    ωm,
    Tload,
    Ts,
    plant_substeps = 1,
)
    # observer/controller/plant update

    return DriveStepOutput(
        Te = ...,
        Te_ref = ...,
    )
end
```

If the controller carries discrete states, also implement
[`reset!`](@ref) for it. The fallback
`reset!(::AbstractIMDriveController) = nothing` covers stateless controllers.

Then:

1. include and export the controller in
   [`src/ElectricMachineWinch.jl`](https://github.com/c-nicomar/ElectricMachineWinch.jl/blob/main/src/ElectricMachineWinch.jl);
2. add a selector branch in
   [`src/constructors.jl`](https://github.com/c-nicomar/ElectricMachineWinch.jl/blob/main/src/constructors.jl);
3. add a standalone test, modeled on
   [`test/test_foc_speed_f1.jl`](https://github.com/c-nicomar/ElectricMachineWinch.jl/blob/main/test/test_foc_speed_f1.jl);
4. use the new controller symbol in [`make_electric_winch`](@ref).

The rest of the KiteControllers integration can remain unchanged.

## Things to get right

- **The sign convention.** `J*dωm/dt = Te + Tload - B*ωm`; see
  [the home page](index.md). Anything else, and the speed will run away in a
  direction that looks like a controller bug.
- **The machine speed is imposed**, not integrated. Use
  [`rk4_step_electrical_only`](@ref) rather than the `IM_AWES_bench` plant step,
  which would integrate a second, duplicate mechanical speed state.
- **Include order matters** in `src/ElectricMachineWinch.jl`:
  `controllers/foc_speed_mtpa.jl` is included *after* `constructors.jl`.
- Revise does not pick up changes to struct definitions, module includes, or
  exports — restart Julia after those.
