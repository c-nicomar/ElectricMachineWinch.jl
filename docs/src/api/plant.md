# Electrical plant

```@meta
CurrentModule = ElectricMachineWinch
```

The electrical-only αβ induction-machine step. It mirrors the validated
`InductionMachineDrives` machine equations, but does **not** integrate the mechanical
speed: KiteModels already owns the reel-out speed state, so the machine speed is
imposed here.

```@autodocs
Modules = [ElectricMachineWinch]
Pages   = ["plant_steps.jl"]
```
