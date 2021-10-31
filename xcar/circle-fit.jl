using CircleFit

struct Circle
    x0
    y0
    r
end

function make_circle_fit(num_points)
    return zeros(2,num_points) 
end

function kasa!(trace::Matrix, new_x::Float64, new_y::Float64)
    num_points = length(trace[1,:])
    for i in num_points-1:-1:1
        trace[:,i + 1] = trace[:,i]
    end
    trace[:,1] = [new_x, new_y]
    x0, y0, r = CircleFit.kasa(trace[1,:], trace[2,:])
    # x0, y0, r = CircleFit.taubin(trace[1,:], trace[2,:])
    # x0, y0, r = CircleFit.pratt(trace[1,:], trace[2,:])
    return Circle(x0, y0, r)
end