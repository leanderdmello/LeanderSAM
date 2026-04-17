module ImuModel

using RxInfer
using Distributions
using LinearAlgebra

export build_imu_model

@model function build_imu_model(
    y,
    n,
    F,
    c,
    Q,
    H,
    d,
    R,
    m0,
    P0
)
    s[1] ~ MvNormal(mean = m0, covariance = Matrix(P0))
    y[1] ~ MvNormal(mean = H[1] * s[1] + d[1], covariance = Matrix(R[1]))

    for t in 2:n
        s[t] ~ MvNormal(mean = F[t - 1] * s[t - 1] + c[t - 1], covariance = Matrix(Q[t - 1]))
        y[t] ~ MvNormal(mean = H[t] * s[t] + d[t], covariance = Matrix(R[t]))
    end

    return s
end

end