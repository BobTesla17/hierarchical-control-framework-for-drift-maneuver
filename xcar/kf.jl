# Adapted from demo of Kalman.jl

using Kalman, GaussianDistributions, LinearAlgebra
using GaussianDistributions: ⊕ # independent sum of Gaussian r.v.
using Statistics
using Parameters
using BlockDiagonals

mutable struct KF
    x0::Vector
    P0::Matrix
    F::Matrix
    u::Vector
    Q::Matrix
    H::Matrix
    R::Matrix
    dist::Gaussian
end

function KF(x0, P0, F, u, Q, H, R)
    return KF(x0, P0, F, u, Q, H, R, Gaussian(x0, P0))
end

function predict!(kf::KF)
    kf.dist = kf.F * kf.dist ⊕ Gaussian(zero(kf.x0), kf.Q)
end

function update!(kf::KF, y::Vector)
    kf.dist, _, _ = Kalman.correct(Kalman.JosephForm(), kf.dist, (Gaussian(y, kf.R), kf.H))
end

function get_mean(kf::KF)
    return mean(kf.dist)
end

function get_cov(kf::KF)
    return cov(kf.dist)
end

function Q_discrete_white_noise(dim, dt, var)
    @assert dim in [2, 3, 4]
    if dim == 2
        Q =  [.25*dt^4  .5*dt^3;
               .5*dt^3     dt^2]
    elseif dim == 3
        Q =  [.25*dt^4  .5*dt^3  .5*dt^2;
               .5*dt^3     dt^2       dt;
               .5*dt^2       dt        1]
    else
        Q = [(dt^6)/36  (dt^5)/12  (dt^4)/6  (dt^3)/6;
             (dt^5)/12  (dt^4)/4   (dt^3)/2  (dt^2)/2;
             (dt^4)/6   (dt^3)/2    dt^2      dt;
             (dt^3)/6   (dt^2)/2    dt         1.]
    end
    return Q * var
end

function make_tracker()
    R1_std = sqrt(0.000001)
    R2_std = sqrt(0.0016)
    Q_std = sqrt(0.05)
    dt = 1/100
    F_block = [1 dt dt ^ 2; 0 1 dt; 0 0 1]
    F = BlockDiagonal([F_block, F_block, F_block])
    u = zeros(3) 
    H = [1.0 0 0 0   0 0 0   0   0;
         0   0 0 1.0 0 0 0   0   0;
         0   0 0 0   0 0 1.0 0   0;
         0   0 0 0   0 0 0   1.0 0]
    R1 = Matrix(R1_std * I, 3, 3)
    R2 = Matrix(R2_std * I, 1, 1)
    R = BlockDiagonal([R1, R2])
    Q_block = Q_discrete_white_noise(3, dt, Q_std ^ 2)
    Q = BlockDiagonal([Q_block, Q_block, Q_block])
    x0 = zeros(9)
    P0 = Matrix(500.0I, 9, 9)
    return KF(x0, P0, F, u, Q, H, R)
end