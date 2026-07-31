# Improve dev script

## Default directory for Julia package source code

Add this line to `~/.bashrc` (once, per machine):

    export JULIA_PKG_DEVDIR=/path/to/checkouts

where `/path/to/checkouts` is the directory holding your Julia package
checkouts — the parent of both this repo and `InductionMachineDrives`.
For example, with the checkouts at `/home/ufechner/repos/ElectricMachineWinch`
and `/home/ufechner/repos/InductionMachineDrives`:

    export JULIA_PKG_DEVDIR=/home/ufechner/repos

Changes take effect in new shells; run `source ~/.bashrc` to apply them to
the current one. Note `~/.bashrc` is not read by non-interactive shells, so
also set it in `~/.profile` if `bin/dev` is invoked outside a terminal.

## Change to `bin/dev`

At the top of the script, before `Project.toml` is rewritten, verify that:

- `JULIA_PKG_DEVDIR` is set and non-empty, and
- `$JULIA_PKG_DEVDIR/InductionMachineDrives` exists and `realpath`s to the
  same path as `../InductionMachineDrives` (the checkout the final
  `Pkg.develop` call uses).

Compare the two with `realpath -m ... || true` so a missing path does not
trip `set -e` before the hint is printed. On failure, print the required
`export` line plus the mismatching paths to stderr and `exit 1`.

Also update the `bin/dev` description in `CLAUDE.md` to mention the new
requirement.
