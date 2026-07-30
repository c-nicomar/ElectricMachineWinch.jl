# Drive controllers

```@meta
CurrentModule = ElectricMachineWinch
```

Every controller implements the [`drive_step!`](@ref) contract documented under
[Types](winch.md#Types).

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

## FOC speed controller, constrained MTPA: Types

```@autodocs
Modules = [ElectricMachineWinch]
Pages   = ["controllers/foc_speed_mtpa.jl"]
Order   = [:type]
```

## FOC speed controller, constrained MTPA: Functions

```@autodocs
Modules = [ElectricMachineWinch]
Pages   = ["controllers/foc_speed_mtpa.jl"]
Order   = [:function]
```
