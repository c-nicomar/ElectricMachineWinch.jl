# Plan: Documenter.jl documentation for `ElectricMachineWinch`, published on GitHub Pages

Goal: turn `README.md`, `docs/GETTING_STARTED.md` and the docstrings in `src/` into a
browsable HTML manual, built by
[Documenter.jl](https://documenter.juliadocs.org/stable/) and deployed by GitHub
Actions to

```
https://c-nicomar.github.io/ElectricMachineWinch.jl/dev/       # built from main
https://c-nicomar.github.io/ElectricMachineWinch.jl/stable/    # built from the latest git tag
```

Status: **executed on 2026-07-29**, except for the GitHub-side configuration in
Step 8, which needs repository admin. The local build is green with
`checkdocs = :all`; see the checklist in §7 for exactly what is left.

Scope: adds `docs/make.jl`, `docs/Project.toml`, `docs/src/`, one GitHub Actions
workflow and two `bin/` helpers. Touches `src/` only to add docstrings and one
export, and `README.md` only to add badges and re-point links. No numerical
behaviour changes.

The sister project `../IM_AWES_bench/PlanDocu.md` is the model for this plan and has
already been executed there — reuse its `docs/make.jl`, `bin/build_docs`,
`bin/serve_docs` and `.github/workflows/Documenter.yml` as starting points. Section 9
below lists where this repository genuinely differs; do not assume the two are
interchangeable.

---

## 1. Verified facts about the current tree

Checked against the working tree at commit `1071690` (branch `main`), Julia 1.12.6.
Re-check if the code moves under you.

- **There is no `docs/make.jl`, no `docs/src/`, no `docs/Project.toml`.** `docs/`
  holds exactly one file, `GETTING_STARTED.md` (396 lines). Documenter requires every
  page to live under `docs/src/`, so it has to move.
- **There is no `.github/` directory** — no CI of any kind exists yet.
- The remote is `https://github.com/c-nicomar/ElectricMachineWinch.jl`, so Pages
  lands under `c-nicomar.github.io/ElectricMachineWinch.jl`. As with the sister
  project, the repository name carries the `.jl` suffix while the package directory
  does not.
- The root project declares `[workspace] projects = ["examples", "test"]`. Both
  members carry `[sources] ElectricMachineWinch = {path = ".."}`.
- **`IM_AWES_bench` is unregistered** and is resolved from
  `[sources] IM_AWES_bench = {url = "https://github.com/c-nicomar/IM_AWES_bench.jl", rev = "main"}`
  in the *root* `Project.toml`. This single fact drives §2.2 — it is the main way
  this repository differs from the sister project.
- `bin/dev` and `bin/free` rewrite that `[sources]` line to switch between the git
  URL and a local `../IM_AWES_bench` checkout. There is **no `bin/install`** here
  (unlike the sister project), so `Manifest-v1.12.toml.default` is a manual
  convenience only; nothing restores it automatically. It is currently byte-identical
  to the working `Manifest-v1.12.toml`, which is gitignored.
- `[compat]` in the root `Project.toml` says `julia = "1.10"`, but `README.md` says
  the package targets 1.12 and the committed manifest is `Manifest-v1.12.toml`. Pick
  one before writing a CI matrix; this plan assumes 1.12 (§7).
- `examples/Project.toml` pulls in `KiteControllers`, `KiteModels`, `KiteViewers`,
  `MakieControlPlots` and `NativeFileDialog`. None of that may enter the docs
  environment (§6, pitfall table).
- `.gitignore` line 31 is `desktop.iniIM_AWES_jl.zip` — a missing newline merged two
  patterns, so neither is actually ignored. `docs/build/` is not ignored either.
  Drive-by fix in Step 1.

### Docstring coverage — measured, not counted

Measured on 2026-07-29 by loading the package (`names` vs `Base.Docs.meta`), not by
counting `"""` markers:

- `length(names(ElectricMachineWinch))` = **14** — 13 exported names plus the module.
- **All 13 exported names carry a docstring. There are zero undocumented exports.**
- `Base.Docs.meta(ElectricMachineWinch)` holds **17** bindings. The four beyond the
  exports are `im_torque`, `rk4_step_electrical_only`, `step_drive_from_kite!`, and
  `WinchModels.calc_acceleration` — the last written inside this package but keyed to
  a *foreign* binding.

So the giant docstring pass that dominates the sister project's plan **does not exist
here**. What is left is four specific gaps, all of them about the *interface* rather
than about coverage:

1. **No module docstring.** There is no `"""…"""` above `module ElectricMachineWinch`
   in `src/ElectricMachineWinch.jl`.
2. **`drive_step!` and `reset!` are documented only in their MTPA flavour.** Both
   docstrings live in `src/controllers/foc_speed_mtpa.jl:78` and `:241` and describe
   `FOCSpeedMTPAController`. `drive_step!` is *the* extension point of this package
   (`AbstractIMDriveController` in `src/types.jl:52` refers to it), so it needs a
   generic docstring stating the contract — arguments, units, sign convention, and
   the `DriveStepOutput` return — independent of any one controller.
3. **`step_drive_from_kite!` is documented but not exported.** It is the recommended
   integration entry point, and `examples/autopilot_im_winch_FOC_F1.jl:266` has to
   call it fully qualified. Either export it (additive, no breakage) or say plainly
   in the manual that it must be qualified. Recommendation: export it.
4. `inverse_park_voltage` and `phase_power_alpha_beta` in `src/plant_steps.jl` are
   undocumented internals. Low priority; document them if `api/plant.md` publishes
   them.

Consequence: unlike the sister project, `checkdocs = :all` is within reach here from
day one (§4, Step 4). Use it — with 17 bindings there is no reason to settle for
`:exports`.

---

## 2. Design decisions, with rationale

### 2.1 `docs/` is *not* a workspace member

Same conclusion as the sister project, for a slightly weaker reason. Adding `"docs"`
to `[workspace] projects` folds Documenter and its whole dependency tree into the
shared `Manifest-v1.12.toml`, which every `bin/run_julia` session and every test run
resolves against, and which is mirrored by the committed
`Manifest-v1.12.toml.default`. There is no `bin/install` here to break, so the
failure is quieter than in the sister project — but Documenter still has no business
in the environment used to run the winch model.

Instead `docs/` gets a standalone `Project.toml`, like `test/` and `examples/` but
outside the workspace. Its `docs/Manifest.toml` resolves independently and is already
covered by the `Manifest.toml` line in `.gitignore`.

### 2.2 The docs environment must repeat the `IM_AWES_bench` source entry

This is the trap specific to this repository. Pkg honours `[sources]` **only in the
active project and its workspace members**. `docs/` is deliberately neither. So a
`docs/Project.toml` that lists only

```toml
[sources]
ElectricMachineWinch = {path = ".."}
```

resolves `ElectricMachineWinch`, then fails on its unregistered dependency
`IM_AWES_bench`. The root project's git-URL entry is *not* inherited.

The fix is to declare `IM_AWES_bench` in the docs environment as well — both in
`[deps]` and in `[sources]` — even though `make.jl` never loads it directly. See
Step 1 for the file, and verify it by instantiating from a clean `docs/` (no
`Manifest.toml` present).

Corollary for `bin/dev` users: after `bin/dev` the root resolves `IM_AWES_bench` from
`../IM_AWES_bench`, but the docs environment still resolves the pinned git `rev`.
Docstrings and behaviour then come from two different versions. If you are actively
changing both packages, `Pkg.develop(path="../IM_AWES_bench")` in the docs
environment too, or run `bin/free` before building docs.

### 2.3 No package extension — `make.jl` stays trivial

The sister project has to load `ModelingToolkit` and `OrdinaryDiffEq` in `make.jl` so
its extension becomes visible, at the cost of minutes of precompilation per build.
This package has no `[weakdeps]` and no `ext/`: `using ElectricMachineWinch` brings
in the entire public surface. Builds will be fast (tens of seconds), and CI needs no
special cache warming.

`make.jl` should additionally `using WinchModels`, but only because of the
`WinchModels.calc_acceleration` docstring (§4, Step 5).

### 2.4 The README is the manual — split it, do not duplicate it

`README.md` is 644 lines and already reads like a manual: purpose, architecture,
three controller descriptions, sign conventions, integration recipe, diagnostics, a
comparison methodology, and an extension guide. `docs/GETTING_STARTED.md` is a
396-line tutorial that overlaps it in places.

Two sources of truth for the sign convention is precisely the failure mode the
sister project's plan warns about. So: **split `README.md` into `docs/src/` pages
(§3), and shrink the README to a short overview, the badges, and links to the hosted
pages.** `git mv` `GETTING_STARTED.md` as-is; it was rewritten on 2026-07-29 and is
current.

The alternative — `cp README.md docs/src/index.md` inside `make.jl` — keeps one file
but produces a single enormous page and breaks every relative link. Rejected.

### 2.5 Deploy with `GITHUB_TOKEN`, not an SSH deploy key

Unchanged from the sister project: a workflow with `permissions: contents: write` and
the built-in `GITHUB_TOKEN` needs zero repository secrets. The only thing it cannot
do is deploy previews for pull requests from forks, which is acceptable here.

---

## 3. Target file layout

```
.github/
  workflows/
    Documenter.yml          # new
bin/
  build_docs                # new  (adapt ../IM_AWES_bench/bin/build_docs)
  serve_docs                # new  (adapt ../IM_AWES_bench/bin/serve_docs)
docs/
  .gitignore                # new: build/, Manifest.toml
  Project.toml              # new
  make.jl                   # new
  src/
    index.md                # new: purpose, architecture diagram, sign convention
    getting_started.md      # <- docs/GETTING_STARTED.md  (git mv)
    controllers.md          # <- README "Controllers" + "Constructing a winch"
    integration.md          # <- README "KiteControllers integration"
    diagnostics.md          # <- README "Logging and diagnostics" + "Comparing F1 and MTPA fairly"
    extending.md            # <- README "Extending the package with another controller"
    api/
      winch.md              # new: types.jl, winch_interface.jl, constructors.jl
      controllers.md        # new: controllers/*.jl
      plant.md              # new: plant_steps.jl
```

Every file in `src/` is covered by exactly one `@autodocs` block, which is what makes
the strict `checkdocs` of Step 4 safe.

---

## 4. Step-by-step

### Step 1 — create the docs environment

`docs/Project.toml`:

```toml
[deps]
Documenter = "e30172f5-a6a5-5a46-863b-614d45cd2de4"
ElectricMachineWinch = "0c7c3992-6b03-4dd7-9ce6-b78056c4dd77"
IM_AWES_bench = "d85c8359-f9b8-4611-8006-e67b7a824205"
WinchModels = "7dcfa46b-7979-4771-bbf4-0aee0da42e1f"

[sources]
ElectricMachineWinch = {path = ".."}
IM_AWES_bench = {url = "https://github.com/c-nicomar/IM_AWES_bench.jl", rev = "main"}

[compat]
Documenter = "1"
julia = "1.12"
```

`IM_AWES_bench` appears in `[deps]` only so its `[sources]` entry is unambiguous —
`make.jl` never loads it directly (§2.2).

`docs/.gitignore`:

```
build/
Manifest.toml
```

While in `.gitignore`: fix the merged line 31 in the **root** `.gitignore`
(`desktop.iniIM_AWES_jl.zip` → two lines).

Resolve from a clean state and confirm §2.2 is handled:

```bash
rm -f docs/Manifest.toml
julia --project=docs -e 'using Pkg; Pkg.instantiate(); using ElectricMachineWinch'
```

### Step 2 — move and split the existing prose

```bash
mkdir -p docs/src/api
git mv docs/GETTING_STARTED.md docs/src/getting_started.md
```

Then create `controllers.md`, `integration.md`, `diagnostics.md` and `extending.md`
by **moving** the corresponding sections out of `README.md` (§2.4), and reduce the
README to: one-paragraph purpose, the badge row, the controller-symbol list, a
minimal `make_electric_winch` example, and links to the hosted pages.

Fix links afterwards:

- Cross-references between pages become `[text](controllers.md)`; Documenter
  rewrites `.md` to `.html`.
- Links to source files (`src/…`, `examples/…`, `test/…`) point at files that are not
  part of the doc build. Rewrite as absolute GitHub URLs, e.g.
  `https://github.com/c-nicomar/ElectricMachineWinch.jl/blob/main/examples/autopilot_im_winch_FOC_F1.jl`.
  Both `CLAUDE.md` and the current `GETTING_STARTED.md` use relative
  `[file](path)` links heavily — grep for them:

```bash
grep -n '](\(\.\./\|src/\|bin/\|test/\|examples/\|docs/\|data/\)' docs/src/*.md
```

### Step 3 — write `docs/src/index.md`

Landing page. Content, in order:

1. One-paragraph purpose: a bridge package exposing
   `DetailedIMWinch <: WinchModels.AbstractWinchModel` backed by `IM_AWES_bench`
   drive blocks.
2. The layering rule, which is the package's whole reason to exist — **KiteModels
   owns the mechanical state, this package owns only the electrical state**, hence
   the imposed speed and the electrical-only RK4 step.
3. The sign convention, prominently:

   ```
   J*dωm/dt = Te + Tload - B*ωm
   ωm    = gear_ratio / drum_radius * v_reelout
   Tload = drum_radius / gear_ratio * tether_force
   ```

4. The three controller symbols and when to use each.
5. Quickstart: `bin/run_julia`, `make_electric_winch`, `julia --project=test test/runtests.jl`.
6. A table of contents:

````markdown
```@contents
Pages = ["getting_started.md", "controllers.md", "integration.md",
         "diagnostics.md", "extending.md"]
Depth = 2
```
````

The ASCII architecture diagram from `README.md` belongs here too.

### Step 4 — write `docs/make.jl`

```julia
using Documenter
using ElectricMachineWinch
# Only so that the WinchModels.calc_acceleration docstring resolves (see Step 5).
using WinchModels

DocMeta.setdocmeta!(
    ElectricMachineWinch, :DocTestSetup,
    :(using ElectricMachineWinch); recursive = true,
)

makedocs(;
    modules  = [ElectricMachineWinch],
    authors  = "Carolina Nicolás <canicola@ing.uc3m.es> and contributors",
    sitename = "ElectricMachineWinch.jl",
    format = Documenter.HTML(;
        canonical  = "https://c-nicomar.github.io/ElectricMachineWinch.jl",
        edit_link  = "main",
        prettyurls = get(ENV, "CI", "false") == "true",
        assets     = String[],
    ),
    pages = [
        "Home"                 => "index.md",
        "Getting started"      => "getting_started.md",
        "Controllers"          => "controllers.md",
        "KiteModels integration" => "integration.md",
        "Diagnostics"          => "diagnostics.md",
        "Extending"            => "extending.md",
        "API" => [
            "Winch and interface" => "api/winch.md",
            "Controllers"         => "api/controllers.md",
            "Electrical plant"    => "api/plant.md",
        ],
    ],
    # Coverage is already complete (§1), so run the strict check over every
    # docstring in the module, not just the exported ones.
    checkdocs = :all,
)

deploydocs(;
    repo         = "github.com/c-nicomar/ElectricMachineWinch.jl",
    devbranch    = "main",
    push_preview = true,
)
```

Notes:

- Fix `authors` to the real authors. While there: `Project.toml` still says
  `authors = ["Generated bridge package for AWES electric-machine winch integration"]`,
  which is a placeholder, not a person.
- `repo` in `deploydocs` must be the GitHub repository name **including `.jl`**, no
  scheme, no `.git`. This mismatch is the most common reason a build goes green and
  nothing is ever published.
- If `checkdocs = :all` turns out to trip over the foreign
  `WinchModels.calc_acceleration` binding, fall back to `:exports` rather than
  deleting the check — but read Step 5 first.

### Step 5 — API pages

Each page selects docstrings by source file, so the ordering stays meaningful.
`docs/src/api/winch.md`:

````markdown
# Winch model and KiteModels interface

```@meta
CurrentModule = ElectricMachineWinch
```

## Core types

```@autodocs
Modules = [ElectricMachineWinch]
Pages   = ["types.jl"]
```

## KiteModels interface

```@autodocs
Modules = [ElectricMachineWinch]
Pages   = ["winch_interface.jl"]
```

## Construction

```@autodocs
Modules = [ElectricMachineWinch]
Pages   = ["constructors.jl"]
```
````

`api/controllers.md` uses `Pages = ["controllers/ideal_torque.jl",
"controllers/foc_speed_f1.jl", "controllers/foc_speed_mtpa.jl"]` — keep the
directory prefix, a bare basename is a suffix match and is needlessly ambiguous.
`api/plant.md` uses `Pages = ["plant_steps.jl"]`.

**The one thing to verify early.** The docstring at `src/winch_interface.jl:24`
documents `WinchModels.calc_acceleration`, a binding owned by another package.
Julia stores it in `Docs.meta(ElectricMachineWinch)` (measured, §1), and the
sister project's build shows `@autodocs` publishing exactly this kind of
foreign-binding docstring — its extension documents `IM_AWES_bench.*` names and the
build is strict-clean. So the `Pages = ["winch_interface.jl"]` block above is
expected to pick it up. Confirm it on the first build: search the generated
`docs/build/api/winch.html` for `calc_acceleration`. If it is missing, splice it
explicitly instead:

````markdown
```@docs
WinchModels.calc_acceleration
```
````

and do **not** add `WinchModels` to `modules` — that would drag every `WinchModels`
export into `checkdocs`.

**The docstring pass** is small here (§1): a module docstring, generic `drive_step!`
and `reset!` docstrings, exporting `step_drive_from_kite!`, and optionally the two
`plant_steps.jl` helpers. Budget an hour, not days. Every docstring that mentions
torque must match `J*dωm/dt = Te + Tload - B*ωm`; cross-check against the
"Sign convention" section that moves into `index.md`.

Since the package has no MTK dependency and constructs cheaply, a couple of
`jldoctest` blocks are realistic — e.g. `make_electric_winch(controller = :ideal)`
followed by `last_summary(wm)`. Do this only after the build is green; doctests that
print `Float64` vectors are a maintenance trap.

### Step 6 — build and preview locally

```bash
julia --project=docs docs/make.jl     # output in docs/build/, open index.html
```

`deploydocs` is a no-op outside CI, so this is safe.

Copy `bin/build_docs` and `bin/serve_docs` from `../IM_AWES_bench/bin/` and adapt:
drop the `KMP_DUPLICATE_LIB_OK` export and the `bin/sysimage.so` warnings (neither
exists in this repository), and keep the `--startup-file=no` and the LiveServer
bootstrap. For the docstring pass, `servedocs` needs to watch the package source
too:

```julia
using LiveServer; servedocs(launch_browser = true, include_dirs = ["src"])
```

`chmod +x bin/build_docs bin/serve_docs`.

### Step 7 — the GitHub Actions workflow

Copy `.github/workflows/Documenter.yml` from the sister project verbatim, minus the
`KMP_DUPLICATE_LIB_OK` env var:

```yaml
name: Documentation

on:
  push:
    branches: [main]
    tags: ['*']
  pull_request:
  workflow_dispatch:

concurrency:
  group: ${{ github.workflow }}-${{ github.ref }}
  cancel-in-progress: true

jobs:
  docs:
    name: Build and deploy documentation
    runs-on: ubuntu-latest
    permissions:
      contents: write        # push to gh-pages
      pull-requests: write   # PR preview comments
      statuses: write
    steps:
      - uses: actions/checkout@v4
      - uses: julia-actions/setup-julia@v2
        with:
          version: '1.12'
      - uses: julia-actions/cache@v2
      - name: Instantiate docs environment
        run: julia --project=docs -e 'using Pkg; Pkg.instantiate()'
      - name: Build and deploy
        env:
          GITHUB_TOKEN: ${{ secrets.GITHUB_TOKEN }}
          JULIA_DEBUG: Documenter
        run: julia --project=docs docs/make.jl
```

Points that matter:

- `version: '1.12'` matches the committed manifest. `'1'` would drift.
- The runner clones `IM_AWES_bench` from the git URL in `docs/Project.toml`
  `[sources]`. That repository is public, so no token juggling is needed — but a
  moving `rev = "main"` means the docs build is not reproducible across time. If that
  ever bites, pin a commit SHA in `docs/Project.toml` only.
- Do not add `julia-actions/julia-buildpkg`; `Pkg.instantiate()` on `docs/` already
  develops the package through `[sources]`.

### Step 8 — one-time GitHub configuration

1. **Allow Actions to write.** Settings → Actions → General → *Workflow permissions*
   → "Read and write permissions".
2. **Merge to `main`** and watch the Actions tab. On success an orphan `gh-pages`
   branch appears containing `dev/` and `versions.js`.
3. **Enable Pages.** Settings → Pages → *Source*: "Deploy from a branch", branch
   `gh-pages`, folder `/ (root)`.
4. **Verify** `https://c-nicomar.github.io/ElectricMachineWinch.jl/dev/`.
5. **`stable/` requires a tag** — `v0.1.0`, matching `version = "0.1.0"` in
   `Project.toml`. Until then, link to `/dev/`.

**Expect steps 1 and 3 to be blocked.** Both need admin on
`c-nicomar/ElectricMachineWinch.jl`; on the sister repository `ufechner7` has
push/triage only, and the same is likely here. Confirm before promising a live URL —
everything up to and including a green `gh-pages` branch can be done without admin,
but publishing cannot.

If the run reports "Deploy: skipped", read the `JULIA_DEBUG=Documenter` output; it
names exactly which of the repo/branch/token/event checks failed.

### Step 9 — wire the docs into the repository

- Badges at the top of `README.md`:

  ```markdown
  [![Dev](https://img.shields.io/badge/docs-dev-blue.svg)](https://c-nicomar.github.io/ElectricMachineWinch.jl/dev/)
  [![Stable](https://img.shields.io/badge/docs-stable-blue.svg)](https://c-nicomar.github.io/ElectricMachineWinch.jl/stable/)
  ```

- Point the README's remaining links at the hosted pages (§2.4).
- Update `CLAUDE.md`: it references `docs/GETTING_STARTED.md`, which becomes
  `docs/src/getting_started.md`, and its *Commands* section should gain
  `bin/build_docs` / `bin/serve_docs`.
- The sister project's `docs/src/index.md` already mentions this package; once
  `/dev/` is live, add the reciprocal link there.

---

## 5. Optional follow-ups, in priority order

1. **A test CI workflow.** There is none. `test/runtests.jl` currently only ever runs
   on a developer's laptop, and `test/test_foc_speed_f1.jl` is a 500 000-step
   integration check — exactly the kind of thing that silently rots. Higher value
   than anything below.
2. **A `test_foc_speed_mtpa.jl`**, still missing (`README.md` and
   `getting_started.md` both promise it). Not documentation work, but it is on the
   critical path for documenting MTPA honestly.
3. **`linkcheck = true`** in `makedocs`, to catch the absolute GitHub URLs from
   Step 2 going stale. Enable only after the workflow is green — it makes builds
   network-dependent and flaky.
4. **Doctests** for the cheap constructors (Step 5).
5. **Plots in the docs.** The example autopilot produces CSV debug logs. Embedding
   figures means adding a plotting stack and a headless backend to the docs
   environment; commit a handful of static PNGs under `docs/src/assets/` instead.

---

## 6. Pitfalls specific to this repository

| Pitfall | Symptom | Avoidance |
| --- | --- | --- |
| `docs/Project.toml` without the `IM_AWES_bench` `[sources]` entry | `Pkg.instantiate()` fails: `IM_AWES_bench` not found in a registry | Repeat the git-URL entry in the docs env (§2.2) |
| Building docs after `bin/dev` | Docs describe the pinned git `rev`, not the local checkout you are editing | `Pkg.develop` it in `docs/` too, or `bin/free` first |
| Adding Documenter to the root `Project.toml` | A bridge package gains a docs dependency | Documenter only in `docs/Project.toml` |
| Adding `"docs"` to `[workspace] projects` | Documenter enters the shared `Manifest-v1.12.toml`; the committed `.default` drifts out of date | Keep `docs/` outside the workspace (§2.1) |
| Pulling `examples/` deps into the docs env | `KiteViewers`/Makie/`NativeFileDialog` in a headless CI runner | Never `using` them in `make.jl`; link example scripts as GitHub URLs |
| `deploydocs(repo = "github.com/c-nicomar/ElectricMachineWinch")` | Build green, nothing ever published | The repository name includes `.jl` |
| `devbranch` left at the default `"master"` | Deploy skipped on every push | `devbranch = "main"` |
| `README.md` sections copied rather than moved | Two sources of truth for the sign convention; they will diverge | Move the sections, shrink the README (§2.4) |
| An `@autodocs` page that skips a `src/` file | Under `checkdocs = :all`, orphaned docstrings fail the build | Keep the three API pages exhaustive over `src/` |
| Assuming `checkdocs` enforces coverage | A future export with no docstring passes in silence | Coverage is at 13/13 today — add a `names()` vs `Docs.meta()` testset to keep it there |
| Root `[compat] julia = "1.10"` vs a 1.12 manifest and 1.12 CI | Ambiguous support claim; a 1.10 user hits a resolve failure instead of a clear message | Decide, then make `Project.toml`, CI and `README.md` agree |

---

## 7. Checklist

- [x] `docs/Project.toml` (with both `[sources]` entries) + `docs/.gitignore`
      created; clean `Pkg.instantiate()` verified; root `.gitignore` line 31 was
      already fixed in commit `4a2c885`
- [x] `docs/GETTING_STARTED.md` `git mv`-ed to `docs/src/getting_started.md`
- [x] `README.md` split into `controllers.md`, `integration.md`, `diagnostics.md`,
      `extending.md`; README shrunk; relative links rewritten
- [x] `docs/src/index.md` written (purpose, layering rule, sign convention, TOC)
- [x] `docs/make.jl` written; `authors` and `Project.toml` `authors` corrected
- [x] Three `docs/src/api/*.md` pages written, exhaustive over `src/`
- [x] `WinchModels.calc_acceleration` confirmed present in the built
      `api/winch.html` (§Step 5) — the `Pages = ["winch_interface.jl"]`
      `@autodocs` block picks it up, no explicit `@docs` splice needed
- [x] Docstring pass: module docstring, generic `drive_step!`, generic `reset!`,
      `step_drive_from_kite!` exported; `inverse_park_voltage` and
      `phase_power_alpha_beta` documented too, since `api/plant.md` publishes them
- [x] `julia --project=docs docs/make.jl` builds clean locally with
      `checkdocs = :all`
- [x] `bin/build_docs` and `bin/serve_docs` added and executable
- [x] `.github/workflows/Documenter.yml` added (first `.github/` file in the repo)
- [x] A `names()` vs `Docs.meta()` coverage testset added to `test/runtests.jl`
      (§6, "Assuming `checkdocs` enforces coverage")
- [ ] Workflow permissions set to read/write — **needs repository admin**
- [ ] First `main` build green, `gh-pages` branch created
- [ ] Pages source set to `gh-pages` / root, `/dev/` reachable — **needs repository
      admin**
- [x] `README.md` badges added, `CLAUDE.md` paths updated
- [ ] `v0.1.0` tagged so `/stable/` exists
- [ ] Reciprocal link added to `../IM_AWES_bench/docs/src/index.md` once `/dev/`
      is live (§9)

---

## 8. Effort estimate

| Work | Estimate |
| --- | --- |
| Steps 1, 3, 4, 6–8 (environment, `make.jl`, helpers, workflow, GitHub setup) | half a day, most of it waiting on builds |
| Step 2 README split and link rewriting | 2–3 hours, and the part most likely to be got wrong |
| Step 5 API pages | about an hour |
| Step 5 docstring pass | about an hour (13/13 exports already documented) |
| Step 9 README/`CLAUDE.md` rewiring | under an hour |

Roughly one day total — materially less than the sister project, because the
docstring coverage work is already done and there is no ModelingToolkit extension to
keep visible.

---

## 9. How this differs from `../IM_AWES_bench/PlanDocu.md`

Read that plan first; then apply these deltas.

| | `IM_AWES_bench` | `ElectricMachineWinch` |
| --- | --- | --- |
| Docstring coverage | 29 of 36 exports undocumented at the start — days of work | 13 of 13 documented — hours of interface polish (§1) |
| `checkdocs` | `:exports`, coverage enforced by review | `:all` is affordable from day one |
| Package extension | `ext/` + MTK/OrdinaryDiffEq triggers, `Base.get_extension` guard, slow builds | none; `make.jl` is a plain `using` |
| Unregistered deps | none — the package `[deps]` is empty | `IM_AWES_bench` via a git `[sources]` entry that the docs env must repeat (§2.2) |
| Existing prose | four `docs/README_*.md` files, moved as-is | one `GETTING_STARTED.md` moved as-is, plus a 644-line `README.md` that must be split (§2.4) |
| `bin/install` / manifest restore | exists; drives the "docs outside the workspace" argument | does not exist; the argument still holds, more weakly (§2.1) |
| CI env vars | `KMP_DUPLICATE_LIB_OK` needed | not needed |
