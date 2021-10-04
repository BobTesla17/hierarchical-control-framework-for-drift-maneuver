using DifferentialEquations
using Parameters

round_angle = a -> mod(a + π, 2 * π) - π

@with_kw struct RigidBody
    lF = 1.446
    lR = 1.408
    rF = 0.33
    rR = 0.33
    m = 2220.
    h = 0.53
    g = 9.8
    Iz = 1549.034
    mF = 90.
    mR = 80.
end

@with_kw struct Tyre
    # coefficients in the Pacejka's Magic Formula;
    # "Ice" parameter set from https://www.mathworks.com/help/physmod/sdl/ref/tireroadinteractionmagicformula.html
    B = 4.
    C = 2.
    D = 0.1
end

@with_kw struct State
    x = 0.
    y = 0.
    ψ = 0.
    ẋ = 1.
    ẏ = 0.
    ψ̇ = 0.
end

struct Input
    δ
    ωF
    ωR
end

struct StateDerivative
    ẋ
    ẏ
    ψ̇
    ẍ
    ÿ
    ψ̈
end

struct ExtendedState
    V
    β
    VFx
    VFy
    VRx
    VRy
    sFx
    sFy
    sRx
    sRy
    sF
    sR
    μF
    μR
    μFx
    μFy
    μRx
    μRy
    fFz
    fRz
    fFx
    fFy
    fRx
    fRy
end

struct DriftData
    s::State
    u::Input
    es::ExtendedState
    R
end

@with_kw mutable struct Car
    body = RigidBody()
    tyre = Tyre()
    state = State()
end

function compute_extended_state(s::State, u::Input, body::RigidBody, tyre::Tyre)
    # speeds
    V = √(s.ẋ ^ 2 + s.ẏ ^ 2)
    β = round_angle(atan(s.ẏ, s.ẋ) - s.ψ)
    VFx = V * cos(β - u.δ) + s.ψ̇ * body.lF * sin(u.δ)
    VFy = V * sin(β - u.δ) + s.ψ̇ * body.lF * cos(u.δ)
    VRx = V * cos(β)
    VRy = V * sin(β) - s.ψ̇ * body.lR
    
    # slip ratios
    sFx = (VFx - u.ωF * body.rF) / (u.ωF * body.rF)
    sFy = VFy / (u.ωF * body.rF)
    sRx = (VRx - u.ωR * body.rR) / (u.ωR * body.rR)
    sRy = VRy / (u.ωR * body.rR)
    sF = √(sFx ^ 2 + sFy ^ 2)
    sR = √(sRx ^ 2 + sRy ^ 2)
    
    # friction coefficients
    μF = tyre.D * sin(tyre.C * atan(tyre.B * sF))
    μR = tyre.D * sin(tyre.C * atan(tyre.B * sR))
    μFx = -(sFx / sF) * μF
    μFy = -(sFy / sF) * μF
    μRx = -(sRx / sR) * μR
    μRy = -(sRy / sR) * μR
    
    # normal forces
    G = body.m * body.g
    l = body.lF + body.lR
    fFz = (body.lR * G - body.h * G * μRx) / (l + body.h * (μFx * cos(u.δ) - μFy * sin(u.δ) - μRx))
    fRz = G - fFz
    
    # tire forces
    fFx = μFx * fFz
    fFy = μFy * fFz
    fRx = μRx * fRz
    fRy = μRy * fRz
    
    return ExtendedState(V, β, VFx, VFy, VRx, VRy, sFx, sFy, sRx, sRy, sF, sR, μF, μR, μFx, μFy, μRx, μRy, fFz, fRz, fFx, fFy, fRx, fRy)
end


function dynamics(s::State, u::Input, body::RigidBody, tyre::Tyre)
    es = compute_extended_state(s, u, body, tyre)
    
    # the derivatives of the state variables
    ẍ = 1 / body.m * (es.fFx * cos(s.ψ + u.δ) - es.fFy * sin(s.ψ + u.δ) + es.fRx * cos(s.ψ) - es.fRy * sin(s.ψ))
    ÿ = 1 / body.m * (es.fFx * sin(s.ψ + u.δ) + es.fFy * cos(s.ψ + u.δ) + es.fRx * sin(s.ψ) + es.fRy * cos(s.ψ))
    ψ̈ = 1 / body.Iz * ((es.fFy * cos(u.δ) + es.fFx * sin(u.δ)) * body.lF  - es.fRy * body.lR)
    
    return StateDerivative(s.ẋ, s.ẏ, s.ψ̇, ẍ, ÿ, ψ̈)
end


function step!(car::Car, u::Input; dt=0.2)
    tspan = (0.0, dt)
    function ode_dyn!(ṡ::Array, s::Array, u::Array, t)
        st_deriv = dynamics(State(s...), Input(u...), car.body, car.tyre)
        ṡ[1:end] = [st_deriv.ẋ, st_deriv.ẏ, st_deriv.ψ̇, st_deriv.ẍ, st_deriv.ÿ, st_deriv.ψ̈]
    end
    s = car.state
    prob = ODEProblem(ode_dyn!, [s.x, s.y, s.ψ, s.ẋ, s.ẏ, s.ψ̇], tspan, [u.δ, u.ωF, u.ωR])
    sol = DifferentialEquations.solve(prob)
    car.state = State(sol(dt)...)
end

@with_kw mutable struct PID
    Kp
    Ki
    Kd
    sign = -1
    err_sum = 0
    last_err = 0
end

function control!(pid::PID, err)
    pid.err_sum += err
    err_diff = err - pid.last_err
    pid.last_err = err
    return pid.sign * (pid.Kp * err + pid.Ki * pid.err_sum + pid.Kd * err_diff)
end

using JuMP
using Ipopt

struct Circle
    x0
    y0
    r
end


function make_radius_estimator(num_points)
    est = Model(with_optimizer(Ipopt.Optimizer, print_level=0))
    
    # Center as decision variable
    @variable(est, x0, start=0.0)
    @variable(est, y0, start=0.0)

    # Data
    @NLparameter(est, x[i = 1:num_points] == 0.0)
    @NLparameter(est, y[i = 1:num_points] == 0.0)
    est[:x] = x
    est[:y] = y
    
    # Radius as a function of center
    @NLexpression(est, ri[i = 1:num_points], sqrt((x[i] - x0) ^ 2 + (y[i] - y0) ^ 2))
    @NLexpression(est, r, sum(ri[i] for i in 1:num_points) / num_points)
    
    # Objective
    @NLobjective(est, Min, sum(ri[i] ^ 2 for i in 1:num_points) - num_points * r ^ 2)
    
    return est
end

function update!(est::Model, new_x::Float64, new_y::Float64)
    x, y = est[:x], est[:y]
    num_points = length(x)
    for i in num_points - 1:-1:1
        set_value(x[i + 1], value(x[i]))
        set_value(y[i + 1], value(y[i]))
    end
    set_value(x[1], new_x)
    set_value(y[1], new_y)
    optimize!(est)
    set_start_value(est[:x0], value(est[:x0]))
    set_start_value(est[:y0], value(est[:y0]))
    return Circle(value(est[:x0]), value(est[:y0]), value(est[:r]))
end

(δ_ff_snow, δ_ff_ice) = (0.22997754824975136, 0.211634)
(β_ff_snow, β_ff_ice) = (-1.0261624598306827, -1.12966)

using RecursiveArrayTools
using DiffEqParamEstim

function simplified_compute_extended_state(s::State, u::Input, body::RigidBody, fric_coef::Float64)
    # speeds
    V = √(s.ẋ ^ 2 + s.ẏ ^ 2)
    β = round_angle(atan(s.ẏ, s.ẋ) - s.ψ)
    VFx = V * cos(β - u.δ) + s.ψ̇ * body.lF * sin(u.δ)
    VFy = V * sin(β - u.δ) + s.ψ̇ * body.lF * cos(u.δ)
    VRx = V * cos(β)
    VRy = V * sin(β) - s.ψ̇ * body.lR
    
    # slip ratios
    sFx = (VFx - u.ωF * body.rF) / (u.ωF * body.rF)
    sFy = VFy / (u.ωF * body.rF)
    sRx = (VRx - u.ωR * body.rR) / (u.ωR * body.rR)
    sRy = VRy / (u.ωR * body.rR)
    sF = √(sFx ^ 2 + sFy ^ 2)
    sR = √(sRx ^ 2 + sRy ^ 2)
    
    # friction coefficients
    μF = fric_coef
    μR = fric_coef
    μFx = -(sFx / sF) * μF
    μFy = -(sFy / sF) * μF
    μRx = -(sRx / sR) * μR
    μRy = -(sRy / sR) * μR
    
    # normal forces
    G = body.m * body.g
    l = body.lF + body.lR
    fFz = (body.lR * G - body.h * G * μRx) / (l + body.h * (μFx * cos(u.δ) - μFy * sin(u.δ) - μRx))
    fRz = G - fFz
    
    # tire forces
    fFx = μFx * fFz
    fFy = μFy * fFz
    fRx = μRx * fRz
    fRy = μRy * fRz
    
    return ExtendedState(V, β, VFx, VFy, VRx, VRy, sFx, sFy, sRx, sRy, sF, sR, μF, μR, μFx, μFy, μRx, μRy, fFz, fRz, fFx, fFy, fRx, fRy)
end

function simplified_dynamics(s::State, u::Input, body::RigidBody, fric_coef::Float64)
    es = simplified_compute_extended_state(s, u, body, fric_coef)
    
    # the derivatives of the state variables
    ẍ = 1 / body.m * (es.fFx * cos(s.ψ + u.δ) - es.fFy * sin(s.ψ + u.δ) + es.fRx * cos(s.ψ) - es.fRy * sin(s.ψ))
    ÿ = 1 / body.m * (es.fFx * sin(s.ψ + u.δ) + es.fFy * cos(s.ψ + u.δ) + es.fRx * sin(s.ψ) + es.fRy * cos(s.ψ))
    ψ̈ = 1 / body.Iz * ((es.fFy * cos(u.δ) + es.fFx * sin(u.δ)) * body.lF  - es.fRy * body.lR)
    
    return StateDerivative(s.ẋ, s.ẏ, s.ψ̇, ẍ, ÿ, ψ̈)
end

function build_cost_function(data_segment, rigid_body; dt=0.01)
    """data_segment: array of DriftData used for fitting"""
    flatten_state = s::State -> [s.x, s.y, s.ψ, s.ẋ, s.ẏ, s.ψ̇]
    true_data = convert(Array, VectorOfArray([flatten_state(p.s) for p in data_segment]))

    function ode_dyn_guessed_tyre!(ṡ::Array, s::Array, param::Array, t)
        this_step = 1 + Int(floor(t / dt))
        u = data_segment[this_step].u
        st_deriv = simplified_dynamics(State(s...), u, rigid_body, param[1])
        ṡ[1:end] = [st_deriv.ẋ, st_deriv.ẏ, st_deriv.ψ̇, st_deriv.ẍ, st_deriv.ÿ, st_deriv.ψ̈]
    end

    s0 = data_segment[1].s
    tspan = (0.0, dt * (length(data_segment) - 1))
    p = [0.1]  # initial guess of friction coefficient
    prob = ODEProblem(ode_dyn_guessed_tyre!, [s0.x, s0.y, s0.ψ, s0.ẋ, s0.ẏ, s0.ψ̇], tspan, p)
    sample_time = dt .* (0:(length(data_segment) - 1) |> collect)
    cost_function = build_loss_objective(prob, Tsit5(), L2Loss(sample_time, true_data), maxiters=1000, verbose=false)
    return cost_function
end

using Optim

