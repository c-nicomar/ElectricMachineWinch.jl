# ElectricMachineWinch.jl

[![Dev](https://img.shields.io/badge/docs-dev-blue.svg)](https://c-nicomar.github.io/ElectricMachineWinch.jl/dev/)
[![CI](https://github.com/c-nicomar/ElectricMachineWinch.jl/actions/workflows/CI.yml/badge.svg)](https://github.com/c-nicomar/ElectricMachineWinch.jl/actions/workflows/CI.yml)


`ElectricMachineWinch.jl` is a bridge package for testing detailed induction-machine electric drives as winch models in the KiteControllers/KiteModels ecosystem. It provides a winch type compatible with `WinchModels`,

```julia
DetailedIMWinch <: WinchModels.AbstractWinchModel
```

and connects it to interchangeable electric-drive controllers implemented with the discrete induction-machine blocks from [`InductionMachineDrives`](https://github.com/c-nicomar/InductionMachineDrives.jl).

The design rule behind the package is that **KiteModels owns the mechanical state, and this package owns only the electrical state**: the machine speed is imposed from the reel-out speed, never integrated here.

## Controllers

| Symbol | When to use it |
| --- | --- |
| `:ideal` | Ideal torque source with a speed PI and no electrical plant. Use it to debug the KiteModels coupling first. |
| `:foc_speed_f1` | Full discrete FOC with the F1 flux strategy. The only controller with field weakening today. |
| `:foc_speed_mtpa` | Constrained-MTPA current allocation. No field weakening yet. |

## Quickstart

The package targets Julia 1.12. `InductionMachineDrives` is unregistered and resolved from the git URL in `[sources]`, so a fresh checkout needs no manual configuration — just run the setup script once:

```bash
git clone https://github.com/c-nicomar/ElectricMachineWinch.jl
cd ElectricMachineWinch.jl
bin/install
```

`bin/install` checks that Julia 1.12 is active, restores the reference manifest, and instantiates and precompiles the root, `test` and `examples` projects.

```julia
using ElectricMachineWinch

wm = make_electric_winch(
    controller = :ideal,
    drum_radius = 0.5,
    gear_ratio = 10.0,
    J_eq = 0.3685,
    B_eq = 0.01298,
    Ts = 1e-3,
    Te_max = 124.0,
)
```

Run the test suite with:

```bash
julia --project=test test/runtests.jl
```

## Documentation

The manual is at
[c-nicomar.github.io/ElectricMachineWinch.jl/dev](https://c-nicomar.github.io/ElectricMachineWinch.jl/dev/):

- [Getting started](https://c-nicomar.github.io/ElectricMachineWinch.jl/dev/getting_started/) — setup, the standalone tests, and the KiteControllers walkthrough
- [Controllers](https://c-nicomar.github.io/ElectricMachineWinch.jl/dev/controllers/) — the three controllers and their full parameter sets
- [KiteModels integration](https://c-nicomar.github.io/ElectricMachineWinch.jl/dev/integration/) — `step_drive_from_kite!` versus `calc_acceleration`
- [Diagnostics](https://c-nicomar.github.io/ElectricMachineWinch.jl/dev/diagnostics/) — logged signals and a fair F1/MTPA comparison
- [Extending](https://c-nicomar.github.io/ElectricMachineWinch.jl/dev/extending/) — adding another controller
- [API](https://c-nicomar.github.io/ElectricMachineWinch.jl/dev/api/winch/) — the docstrings

Build it locally with `bin/build_docs`, or serve it with live reload using `bin/serve_docs`.
