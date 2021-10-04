using JuMP
using Ipopt

struct Circle
    x0
    y0
    r
end

function make_curv_estimator(num_points)
    est = Model(with_optimizer(Ipopt.Optimizer, print_level=0))
    set_optimizer_attribute(est, "max_cpu_time", 0.005)

    # Center as decision variable 
    @variable(est, x0, start=0.0)
    @variable(est, y0, start=0.0)

    # Data
    @NLparameter(est, x[i = 1:num_points] == 0.0)
    @NLparameter(est, y[i = 1:num_points] == 0.0)
    @NLparameter(est, vx[i = 1:num_points] == 0.0)
    @NLparameter(est, vy[i = 1:num_points] == 0.0)
    @NLparameter(est, vψ[i = 1:num_points] == 0.0)
    est[:x] = x
    est[:y] = y
    est[:vx] = vx
    est[:vy] = vy 
    est[:vψ] = vψ
()
    # Radius as a function of center
    @NLexpression(est, ra[i = 1:num_points], sqrt((x[i] - x0) ^ 2 + (y[i] - y0) ^ 2))
    @NLexpression(est, rb[i = 1:num_points], sqrt(vx[i] ^ 2 + vy[i] ^ 2) / (abs(vψ[i]) + 1e-10))
    @NLexpression(est, r, sum((ra[i] + rb[i]) / 2 for i in 1:num_points) / num_points)
    @NLconstraint(est, r_constr, r <= 1000)
    @NLobjective(est, Min, sum((ra[i] - r) ^ 2 + (rb[i] - r) ^ 2 for i in 1:num_points))
    return est
end

function update!(est::Model, new_x::Float64, new_y::Float64, new_vx::Float64, new_vy::Float64, new_vψ::Float64)
    x, y = est[:x], est[:y]
    vx, vy, vψ = est[:vx], est[:vy], est[:vψ]
    num_points = length(x)
    for i in num_points-1:-1:1
        set_value(x[i + 1], value(x[i]))
        set_value(y[i + 1], value(y[i]))
        set_value(vx[i + 1], value(vx[i]))
        set_value(vy[i + 1], value(vy[i]))
        set_value(vψ[i + 1], value(vψ[i]))
    end
    set_value(x[1], new_x)
    set_value(y[1], new_y)
    set_value(vx[1], new_vx)
    set_value(vy[1], new_vy)
    set_value(vψ[1], new_vψ)
    optimize!(est)
    set_start_value(est[:x0], value(est[:x0]))
    set_start_value(est[:y0], value(est[:y0]))

    return Circle(value(est[:x0]), value(est[:y0]), value(est[:r]))
end
