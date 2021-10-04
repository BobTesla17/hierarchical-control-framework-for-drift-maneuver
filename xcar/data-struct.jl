using Parameters

@with_kw mutable struct Pose @deftype Float64

    x = 0
    y = 0
    ψ = 0
    vψ = 0

    t0 = 0
    t1 = 0

    sum_ψ = 0

end

@with_kw mutable struct State @deftype Float64

    x = 0
    y = 0
    ψ = 0
    vx = 0
    vy = 0
    vψ = 0
    ax = 0
    ay = 0
    aψ = 0

end

@with_kw mutable struct Input @deftype Float64

    δ = 0
    ω = 0

end

@with_kw mutable struct EtsCircle @deftype Float64

    x0 = 0
    y0 = 0
    r = 0

end

@with_kw mutable struct ExtendState @deftype Float64

    β = 0
    V = 0

    vx_car = 0
    vy_car = 0

    x0 = 0
    y0 = 0
    R = 0

end

@with_kw mutable struct Reference @deftype Float64

    R_ref = 0
    β_ref = 0
    R_l1ac = 0

end

@with_kw mutable struct Vesc @deftype Float64

    rpm = 0
    servo = 0
    volt = 0

end


struct DriftData
    safe::Bool
    s::State
    u::Input
    es::ExtendState
    ref::Reference
    dt::Float64
    t::Float64
    vesc::Vesc
end