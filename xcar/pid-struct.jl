using Parameters

@with_kw mutable struct PID
    Kp
    Ki
    Kd
    sign = -1
    err_sum = 0
    last_err = 0
end

function controla!(pid::PID, err)
    pid.err_sum += err
    err_diff = err - pid.last_err
    pid.last_err = err
    return pid.sign * (pid.Kp * err + pid.Ki * pid.err_sum + pid.Kd * err_diff)
end

function controlb!(pid::PID, err)
    pid.err_sum += err
    err_diff = err - pid.last_err
    pid.last_err = err
    return pid.sign * tanh(pid.Kp * err + pid.Ki * pid.err_sum + pid.Kd * err_diff) * 0.39
end