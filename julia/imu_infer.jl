#!/usr/bin/env julia

import Pkg
Pkg.activate(@__DIR__)

using JSON3
using LinearAlgebra
using RxInfer
using Statistics

include(joinpath(@__DIR__, "imu_model.jl"))
using .ImuModel

function vec_to_matrix(v::Vector, rows::Int, cols::Int)
    expected = rows * cols
    actual = length(v)
    @assert actual == expected
    return reshape(collect(Float64, v), cols, rows)'
end

function read_packet(path::String)
    raw = JSON3.read(read(path, String))

    n = Int(raw.n)
    state_dim = Int(raw.state_dim)
    obs_dim = Int(raw.obs_dim)

    m0 = collect(Float64, raw.m0)
    P0 = Matrix(vec_to_matrix(collect(Float64, raw.P0), state_dim, state_dim))

    F = [Matrix(vec_to_matrix(collect(Float64, item), state_dim, state_dim)) for item in raw.F]
    c = [collect(Float64, item) for item in raw.c]
    Q = [Matrix(vec_to_matrix(collect(Float64, item), state_dim, state_dim)) for item in raw.Q]

    H = [Matrix(vec_to_matrix(collect(Float64, item), obs_dim, state_dim)) for item in raw.H]
    d = [collect(Float64, item) for item in raw.d]
    R = [Matrix(vec_to_matrix(collect(Float64, item), obs_dim, obs_dim)) for item in raw.R]

    y = [collect(Float64, item) for item in raw.y]

    return (
        n = n,
        state_dim = state_dim,
        obs_dim = obs_dim,
        m0 = m0,
        P0 = P0,
        F = F,
        c = c,
        Q = Q,
        H = H,
        d = d,
        R = R,
        y = y
    )
end

function infer_packet(packet)
    model = build_imu_model(
        n = packet.n,
        F = packet.F,
        c = packet.c,
        Q = packet.Q,
        H = packet.H,
        d = packet.d,
        R = packet.R,
        m0 = packet.m0,
        P0 = packet.P0
    )

    result = infer(
        model = model,
        data = (y = packet.y,),
        returnvars = (s = KeepLast(),)
    )

    s_last = result.posteriors[:s][end]
    μ = mean(s_last)
    Σ = cov(s_last)

    return Vector{Float64}(μ), Matrix{Float64}(Σ)
end

function write_result(path::String, mean_vec::Vector{Float64}, cov_mat::Matrix{Float64})
    result = Dict(
        "mean" => mean_vec,
        "covariance" => vec(cov_mat')
    )

    open(path, "w") do io
        JSON3.pretty(io, result)
    end
end

function main()
    if length(ARGS) != 2
        println("Usage: julia imu_infer.jl <input_packet.json> <output_result.json>")
        exit(1)
    end

    input_path = ARGS[1]
    output_path = ARGS[2]

    packet = read_packet(input_path)
    mean_vec, cov_mat = infer_packet(packet)
    write_result(output_path, mean_vec, cov_mat)

    println("Inference complete.")
    println("Output written to: ", output_path)
end

main()