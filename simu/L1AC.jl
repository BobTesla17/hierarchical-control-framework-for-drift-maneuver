using DifferentialEquations
using Parameters

@with_kw mutable struct L1AC_RefSys @deftype Float64
    c = 1.0 #0.5#1.0#
    m = 0.5 #1.0#0.2#
    gamma = 0.7 #1.#0.2#
end

@with_kw mutable struct L1AC_State @deftype Float64
    u = 6.
    du = 0.
    ddu = 0.
end

@with_kw mutable struct L1AC_Ctrl @deftype Float64
    r = 0.
    dr = 0.
    ddr = 0.
    y = 0.
    dy = 0.
end

@with_kw mutable struct L1AC_Struct 
    sys = L1AC_RefSys()
    state = L1AC_State()
    ctrl = L1AC_Ctrl()
end

function L1AC_dynamics(s::L1AC_State, c::L1AC_Ctrl, sys::L1AC_RefSys)
    dddu = - (sys.m + sys.c) * s.ddu - (sys.m * sys.c + sys.gamma * sys.m) * s.du + sys.c * c.ddr + sys.m * sys.c * c.dr + sys.gamma * sys.m * sys.c * c.r - sys.gamma * sys.c * c.dy - sys.gamma * sys.m * sys.c * c.y 
    return [s.du, s.ddu, dddu]
end

function L1AC!(l1ac::L1AC_Struct, r_ref::Float64, y_plant::Float64, dt=0.01)
    tspan = (0.0,dt)
    function l1ac_dyn!(ds::Array, s::Array, c::Array, t)
        ds[1:end] = L1AC_dynamics(L1AC_State(s...), L1AC_Ctrl(c...), l1ac.sys)
    end
    dr = (r_ref - l1ac.ctrl.r) / dt
    l1ac.ctrl.ddr = (dr - l1ac.ctrl.dr) / dt
    l1ac.ctrl.dr = dr
    l1ac.ctrl.r = r_ref
    l1ac.ctrl.dy = (y_plant - l1ac.ctrl.y) / dt
    l1ac.ctrl.y = y_plant
    s = l1ac.state
    c = l1ac.ctrl
    prob = ODEProblem(l1ac_dyn!, [s.u, s.du, s.ddu], tspan, [c.r, c.dr, c.ddr, c.y, c.dy])
    sol = DifferentialEquations.solve(prob)
    l1ac.state = L1AC_State(sol(dt)...)
    u = l1ac.state.u
    return u 
end