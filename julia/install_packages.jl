#!/usr/bin/env julia

import Pkg

Pkg.activate(@__DIR__)

packages = [
    "RxInfer",
    "JSON3",
    "Distributions"
]

for pkg in packages
    try
        Pkg.add(pkg)
    catch err
        @warn "Failed to add package" pkg exception = (err, catch_backtrace())
        rethrow(err)
    end
end

Pkg.instantiate()
Pkg.precompile()

println("Julia package setup complete.")
println("Active project: ", Base.active_project())