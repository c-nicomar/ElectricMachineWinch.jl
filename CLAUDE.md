# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this package is

`ElectricMachineWinch.jl` is a *bridge* package: it wraps the detailed discrete
induction-machine drive blocks from `IM_AWES_bench` into a
`WinchModels.AbstractWinchModel` (`DetailedIMWinch`) that KiteModels/KiteSimulators
can use in place of a simple winch. It contains almost no control math of its own —
the controllers here are thin adapters around `IM_AWES_bench` blocks.

Julia 1.12 is the target. Root `Project.toml` declares a `[workspace]` with the
`examples` and `test` sub-projects.

## Commands

Run tests (preferred: in the shared REPL via the kaimon `ex` tool, since each test
file activates `test/` itself):

```julia
include("test/runtests.jl")        # full suite
include("test/test_foc_speed_f1.jl")   # one file / testset
```

From a shell:

```bash
julia --project=test test/runtests.jl
julia --project=test test/test_ideal_torque.jl
bin/run_julia                      # REPL in the root project, Revise loaded if present
bin/run_julia test/test_foc_speed_f1.jl
```

Switching the `IM_AWES_bench` dependency between the git URL in `[sources]` and a
local sibling checkout at `../IM_AWES_bench`:

```bash
bin/dev     # comment out [sources] entry + Pkg.develop("../IM_AWES_bench")
bin/free    # restore [sources] entry + Pkg.resolve
```

Both scripts rewrite `Project.toml` in place, so expect that file to show up as
modified after running them.

Documentation (Documenter.jl, deployed to GitHub Pages by
`.github/workflows/Documenter.yml`):

```bash
bin/build_docs      # instantiate docs/ and build into docs/build/
bin/serve_docs      # live-reload preview on http://localhost:8000
```

`Manifest-v1.12.toml` is gitignored; `Manifest-v1.12.toml.default` is the committed
reference copy.

Revise does not pick up changes to struct definitions, module includes, or exports —
restart Julia after those.

## Architecture

### Layer split

The core design decision: **KiteModels owns the mechanical state, this package owns
only the electrical state.** The machine speed is *imposed*, never integrated here:

```
ωm    = gear_ratio / drum_radius * v_reelout
Tload = drum_radius / gear_ratio * tether_force
```

`src/plant_steps.jl` therefore reimplements the αβ IM RK4 step as
*electrical-only* (`rk4_step_electrical_only`) rather than calling the
`IM_AWES_bench` plant step, which would integrate a second, duplicate mechanical
speed state.

### Controller contract

Every controller subtypes `AbstractIMDriveController` and implements exactly:

```julia
drive_step!(controller, plant; ωm_ref, ωm, Tload, Ts, plant_substeps) -> DriveStepOutput
```

`DriveStepOutput` (`src/types.jl`) is the same for all controllers, which is what
keeps `DetailedIMWinch` and the KiteModels interface controller-agnostic. Optional
`reset!(::AbstractIMDriveController)` has a no-op fallback.

Controllers: `:ideal` (speed PI + first-order torque actuator, no electrical plant —
use it to debug the KiteModels coupling first), `:foc_speed_f1`, `:foc_speed_mtpa`.
All are selected through `make_electric_winch` in `src/constructors.jl`.

The two FOC adapters share the same pipeline and differ only in the outer loop:
observer → outer speed/flux loop (`outer_speed_flux_f1_step!` /
`outer_speed_flux_mtpa_step!`) → current controller → inverse Park →
electrical-only RK4 → `im_torque`.

### Two integration paths into KiteModels

1. `WinchModels.calc_acceleration(wm, v_reelout, force; set_speed=...)` —
   advances the drive *inside* the call. Convenient and used by the tests, but a DAE
   solver may call it several times per macro-step, giving a nonphysical number of
   discrete controller updates. `wm.n_acceleration_calls` exists to detect this.
2. `step_drive_from_kite!(wm, v_ro_set, v_ro_meas, F; dt_outer)` — the preferred
   path for the detailed discrete controllers. Called once per KiteSimulators
   macro-step; internally runs `round(Int, dt_outer/wm.Ts)` substeps with kite-side
   variables held constant, then the returned torque is passed to
   `KiteModels.next_step!(...; set_torque = Te_cmd)`. Choose `dt_outer` so
   `dt_outer / wm.Ts` is an integer, otherwise the effective sample time drifts from
   the `Ts` baked into the observer/controller parameter objects.

   `calc_acceleration` with `set_torque` (rather than `set_speed`) is the read-only
   companion for this path.

### Sign convention

Machine-side mechanics are `J*dωm/dt = Te + Tload - B*ωm`. Positive `Tload` pulls
toward positive reel-out. During positive-speed generation expect
`v_reelout > 0`, `Tload > 0`, `Te < 0`. Load-feedforward signs (`load_ff_sign`,
default `-1.0`) must match this. When a test or run shows speed going the wrong way,
check this convention before suspecting the controller.

### Gotchas

- Include order in `src/ElectricMachineWinch.jl` matters:
  `controllers/foc_speed_mtpa.jl` is included *after* `constructors.jl`.
- `make_electric_winch` validation is deliberately strict: custom F1 parameter
  objects (`obs_p`, `outer_f1_p`, `current_p`) are all-or-nothing and each must have
  `Ts` matching the winch `Ts`, to prevent mixing a new-machine observer with default
  small-machine loops.
- `Is_max` is a dq-current-vector *peak* magnitude, not an RMS phase rating.
- `J_eq`/`B_eq` are forwarded into the MTPA outer controller as `J`/`B`, so the
  speed-loop feedforward stays consistent with the plant KiteModels sees.
- Field weakening (`use_field_weakening`, `wm_base_fw`) currently applies to
  `:foc_speed_f1` only; the MTPA controller has none yet.
- MTPA-specific diagnostics live in `wm.controller.last_outer` (the full
  `OuterSpeedFluxMTPAOutput`); the controller-agnostic last values live directly on
  `wm` and via `last_summary(wm)`.

### Adding a controller

Subtype `AbstractIMDriveController`, implement `drive_step!` returning
`DriveStepOutput`, then: include + export it in `src/ElectricMachineWinch.jl`, add a
selector branch in `src/constructors.jl`, and add a test file following
`test/test_foc_speed_f1.jl`. The KiteControllers integration needs no changes.

## Documentation

The manual lives in `docs/src/` and is built by `docs/make.jl`. `README.md` is
deliberately short: the prose was *moved* into `docs/src/{controllers,integration,
diagnostics,extending}.md`, so do not copy sections back — the sign convention in
particular must have one source of truth (`docs/src/index.md`).

- `docs/` is **not** a workspace member, so it does not inherit the root `[sources]`
  entry for the unregistered `IM_AWES_bench`; `docs/Project.toml` repeats it. After
  `bin/dev` the docs build still uses the pinned git `rev` — run `bin/free` first, or
  `Pkg.develop` it in the docs environment too.
- `checkdocs = :all`: every docstring in the module must be spliced into a page. The
  three `docs/src/api/*.md` pages must stay exhaustive over `src/` — a new source
  file needs an `@autodocs` block, or the build fails.
- The docstring for `WinchModels.calc_acceleration` is a foreign binding written in
  this package; it is picked up by the `Pages = ["winch_interface.jl"]` block. Never
  add `WinchModels` to `modules` in `make.jl`.

## Examples

`examples/autopilot_im_winch_FOC_F1.jl` is a full KiteControllers `autopilot.jl`
adapted to this package (it also contains the debug-logging/CSV helpers).
`examples/autopilot_im_winch_patch.jl` is not runnable — it is the minimal diff to
apply to a fresh copy of the upstream autopilot example.
