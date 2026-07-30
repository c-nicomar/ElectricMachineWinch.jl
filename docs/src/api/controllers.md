# Drive controllers

```@meta
CurrentModule = ElectricMachineWinch
```

Every controller implements the [`drive_step!`](@ref) contract documented under
[Core types](winch.md#Core-types).

## Ideal torque controller

```@autodocs
Modules = [ElectricMachineWinch]
Pages   = ["controllers/ideal_torque.jl"]
```

## FOC speed controller, F1 flux strategy

```@autodocs
Modules = [ElectricMachineWinch]
Pages   = ["controllers/foc_speed_f1.jl"]
```

## FOC speed controller, constrained MTPA

```@autodocs
Modules = [ElectricMachineWinch]
Pages   = ["controllers/foc_speed_mtpa.jl"]
```
