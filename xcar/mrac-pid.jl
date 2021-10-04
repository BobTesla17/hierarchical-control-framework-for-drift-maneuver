using DifferentialEquations

@with_kw mutable struct RefSys @deftype Float64

    a1 = 1.
    a2 = 1.
    b = 1.

    am1 = 1.
    am2 = 1.
    am3 = 1.
    bm1 = 1.
    bm2 = 1.
    bm3 = 1.

    γp = 1.
    γi = 1.
    γd = 1.

end

@with_kw PID_State mutable struct @deftype Float64

    Kp = 1.
    ∇Kp = 0.
    ∇∇Kp = 0.
    Ki = 1.
    ∇Ki = 0.
    ∇∇Ki = 0.
    Kd = 1.
    ∇Kd = 0.
    ∇∇Kd = 0.

end

@with_kw PID_Error mutable struct @deftype Float64

    em = 0.
    intem = 0.
    difem = 0.

end

@with_kw mutable struct MRAC_PID
    pid_state = PID_State()
    pid_error = PID_Error()
    ref_sys = RefSys()
end


function PID_dynamics(s::PID_State, u::PID_Error, sys::RefSys)
    ds3 = - (sys.a1 + sys.b * s.Kp) * s.∇∇Kp - (sys.a2 + sys.b * s.Kp) * s.∇Kp - sys.b * s.Ki * s.Kp - sys.γp * sys.b * u.em
    ds6 = - (sys.a1 + sys.b * s.Kp) * s.∇∇Kp - (sys.a2 + sys.b * s.Kp) * s.∇Kp - sys.b * s.Ki * s.Kp - sys.γp * sys.b * u.intem
    ds9 = - (sys.a1 + sys.b * s.Kp) * s.∇∇Kp - (sys.a2 + sys.b * s.Kp) * s.∇Kp - sys.b * s.Ki * s.Kp - sys.γp * sys.b * difem
    return [s.∇Kp, s.∇∇Kp, ds3, ∇Ki, ∇∇Ki, ds6, ∇Kd, ∇∇Kd, ds9]
end

function MRAC_Adaptive!(mrac::MRAC_PID, dt=0.01)
    tspan = (0.0, dt)
    function pid_dyn!(ds::Array, s::Array, u::Array, t)
        ds[1:end] = PID_dynamics(PID_State(s...), PID_Error(u...), mrac.ref_sys)
    end
    s = mrac.pid_state
    u = mrac.pid_error
    prob = ODEProblem(pid_dyn!, [s.Kp, s.∇Kp, s.∇∇Kp, s.Ki, s.∇Ki, s.∇∇Ki, s.Kd, s.∇Kd, s.∇∇Kd], tspan, [u.em, u.intem, u.difem])
    sol = solve(prob)
    mrac.pid_state = State(sol(dt)...)
end

