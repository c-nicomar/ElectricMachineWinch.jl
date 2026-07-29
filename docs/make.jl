using Documenter
using ElectricMachineWinch

# Only so that the docstring this package writes for the foreign binding
# WinchModels.calc_acceleration resolves. WinchModels is deliberately NOT in
# `modules` below -- that would drag every WinchModels export into checkdocs.
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
        # Clean URLs in CI; file:// friendly locally.
        prettyurls = get(ENV, "CI", "false") == "true",
        assets     = String[],
    ),
    pages = [
        "Home"                   => "index.md",
        "Getting started"        => "getting_started.md",
        "Controllers"            => "controllers.md",
        "KiteModels integration" => "integration.md",
        "Diagnostics"            => "diagnostics.md",
        "Extending"              => "extending.md",
        "API" => [
            "Winch and interface" => "api/winch.md",
            "Controllers"         => "api/controllers.md",
            "Electrical plant"    => "api/plant.md",
        ],
    ],
    # Docstring coverage is complete (13 of 13 exports), and every file in src/
    # is covered by exactly one @autodocs block, so the strict check over *all*
    # docstrings in the module is affordable -- not just the exported ones.
    #
    # Note what this does NOT do: checkdocs compares docstrings that exist
    # against docstrings spliced into the manual. A future export with no
    # docstring at all is invisible to it.
    checkdocs = :all,
)

deploydocs(;
    # The GitHub repository name includes the .jl suffix, unlike the package.
    repo         = "github.com/c-nicomar/ElectricMachineWinch.jl",
    devbranch    = "main",
    push_preview = true,
)
