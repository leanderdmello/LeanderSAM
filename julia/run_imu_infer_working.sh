#!/usr/bin/env bash
set -e
julia /workspaces/telerob-main/julia/imu_infer.jl "$1" "$2"