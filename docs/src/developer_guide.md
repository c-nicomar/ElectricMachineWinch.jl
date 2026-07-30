# Developer guide

This page documents the developer tools and scripts in the `bin/` directory, and
explains how to run tests, build documentation, and work with the workspace
effectively. For a task-oriented walkthrough of the package itself, see
[Getting started](getting_started.md).

## Prerequisites

- **Julia 1.12** — required by the project. Install it with
  [juliaup](https://github.com/JuliaLang/juliaup):
  ```bash
  juliaup add 1.12 && juliaup default 1.12
  ```

## Workspace layout

The root `Project.toml` declares a `[workspace]` with `examples` and `test` as
member sub-projects. They resolve against the shared, version-specific
`Manifest-v1.12.toml` rather than their own manifest — a `Manifest.toml` inside
either sub-project would shadow it and break resolution, which is why
`bin/install` deletes any it finds.

`IM_AWES_bench` is unregistered and resolved from the git URL declared in
`[sources]` in `Project.toml`. `docs/` is deliberately **not** a workspace
member (see [`bin/build_docs`](#bin-build_docs) below), so `docs/Project.toml`
repeats that `[sources]` entry independently.

## Scripts in `bin/`

All scripts are designed to be run from the repository root (or from the `bin/`
directory itself — they auto-detect and `cd ..` when needed).

### `bin/install`

Set up the workspace environment. Run this once after cloning the repository.

It performs the following steps:

1. **Checks the Julia version** — verifies that `julia` resolves to Julia 1.12
   and exits with a clear error otherwise.
2. **Refuses to run on a `bin/dev`'d checkout** — if `IM_AWES_bench` is
   currently developed from a local sibling checkout (detected via a
   commented-out `[sources]` entry or a `path =` entry in the manifest),
   restoring the default manifest would silently undo that. Run `bin/free`
   first, then `bin/dev` again afterwards if wanted.
3. **Restores the known-good manifest** — copies
   `Manifest-v1.12.toml.default` to `Manifest-v1.12.toml`, so every developer
   starts with the exact same dependency resolution.
4. **Removes stale sub-project manifests** — deletes `Manifest.toml`,
   `test/Manifest.toml` and `examples/Manifest.toml` if present.
5. **Instantiates and precompiles** — runs `Pkg.instantiate()` on the root
   project, then `Pkg.precompile()` on the `test` and `examples` projects.

```bash
bin/install
```

### `bin/run_julia`

Start a Julia REPL configured for the root project (`--project .`, `-t auto`),
loading Revise if it is available.

```bash
bin/run_julia                       # interactive REPL
bin/run_julia test/foo.jl [args...] # run a script; extra args are forwarded to it
```

Revise does not pick up changes to struct definitions, module includes, or
exports — restart Julia after those instead of relying on Revise to catch up.

### `bin/dev` and `bin/free`

Switch the `IM_AWES_bench` dependency between the git URL in `[sources]` and a
local sibling checkout at `../IM_AWES_bench`, for editing both packages at
once.

```bash
bin/dev     # comment out the [sources] entry, Pkg.develop("../IM_AWES_bench"), Pkg.resolve
bin/free    # restore the [sources] entry, Pkg.resolve back onto the git source
```

Both scripts rewrite `Project.toml` in place, so expect that file to show up
as modified after running them. `bin/free` does not use `Pkg.free` —
`IM_AWES_bench` is unregistered, so there is nothing registered to free back
to; `Pkg.resolve()` alone drops the local `path =` entry in favour of the
restored `[sources]` entry.

Note that switching with `bin/dev` only affects the root/`test`/`examples`
workspace. `bin/build_docs`, described next, is a separate environment and
still resolves `IM_AWES_bench` from the pinned git `rev` unless you also run
`Pkg.develop` inside `docs/`.

### `bin/build_docs`

Build the Documenter.jl documentation into `docs/build/`.

It first instantiates the `docs/` environment (which resolves independently
from the workspace — it has its own `Project.toml` and gitignored
`Manifest.toml`) and then runs `docs/make.jl`.

```bash
bin/build_docs
```

Open the result at `docs/build/index.html`.

### `bin/serve_docs`

Serve the documentation locally with live reload at `http://localhost:8000`,
opening the browser automatically.

```bash
bin/serve_docs              # open browser automatically
bin/serve_docs --no-browser # skip browser launch (e.g. over SSH)
```

Internally this calls `Documenter.servedocs(...)`, which rebuilds whenever a
file in `docs/src/` (or `src/`, since docstrings are watched too) changes.
Requires `LiveServer`, which is auto-installed into the global environment if
absent.

## Running tests

Preferred: in the shared REPL, since each test file activates its own `test/`
environment at the top and running via `include` avoids paying for a fresh
`Pkg.instantiate` each time:

```julia
include("test/runtests.jl")            # full suite
include("test/test_foc_speed_f1.jl")   # one file / testset
```

From a shell:

```bash
julia --project=test test/runtests.jl
julia --project=test test/test_ideal_torque.jl
```

`test/runtests.jl` includes a smoke test (`:ideal` controller through
`WinchModels.calc_acceleration`), a docstring-coverage check that mirrors the
`checkdocs = :all` build setting, and the per-controller testsets
(`test_ideal_torque.jl`, `test_foc_speed_f1.jl`). See
[Adding a controller](extending.md) for the checklist a new controller's test
file should follow.

## Documentation conventions

- New `.md` pages go into `docs/src/`.
- To add a page to the navigation sidebar, edit the `pages` array in
  `docs/make.jl`.
- API docstrings are auto-generated by `Documenter` from the source code via
  `@autodocs` blocks in `docs/src/api/*.md`. `checkdocs = :all` means every
  docstring in the module must be spliced into one of those pages — a new
  source file needs an `@autodocs` block added, or the build fails.
- The sign convention (`docs/src/index.md`) has one source of truth; do not
  copy that prose into other pages.
