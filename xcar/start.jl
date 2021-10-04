#!/usr/bin/env julia

include("RosMsg.jl")
include("curv-estimator.jl")
include("pid-struct.jl")
include("tool-func.jl")
include("data-struct.jl")
include("kf.jl")

using RobotOS
using ReferenceFrameRotations
using BlockDiagonals
using JLD

function update_safe(msg::RosMsg.BoolMsg)
    global safe
    safe = msg.data
end

function update_rts(msg::RosMsg.PoseStamped)
    global pose
    if pose.t1 == 0
        pose.t0 = msg.header.stamp.secs + msg.header.stamp.nsecs * 10 ^ -9 - 0.01
    else 
        pose.t0 = pose.t1
    end
    pose.t1 = msg.header.stamp.secs + msg.header.stamp.nsecs * 10 ^ -9
    
    pose.x = msg.pose.position.x 
    pose.y = msg.pose.position.y 
    q = Quaternion(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z)
    euler_angle = quat_to_angle(q, :XYZ)
    ψ = euler_angle.a3
    pose.sum_ψ = angle_mapping(pose.sum_ψ, pose.ψ, ψ)
    pose.ψ = ψ
    kf_tracker()
end

function update_imu(msg::RosMsg.Imu)
    global pose
    pose.vψ = msg.angular_velocity.z
end

function compute_extend_state(s::State)
    global curv_est
    vx_car = s.vx * cos(s.ψ) + s.vy * sin(s.ψ)
    vy_car = - s.vx * sin(s.ψ) + s.vy * cos(s.ψ)
    β = atan(vy_car, vx_car)
    V = sqrt(s.vx ^ 2 + s.vy ^ 2)
    est_circle = update!(curv_est, s.x, s.y, s.vx, s.vy, s.vψ)
    return ExtendState(β, V, vx_car, vy_car, est_circle.x0, est_circle.y0, est_circle.r)
end

function kf_tracker()
    global tracker
    global pose
    global state 
    global exstate 
    dt = pose.t1 - pose.t0
    F_block = [1 dt dt ^ 2; 0 1 dt; 0 0 1]
    F = BlockDiagonal([F_block, F_block, F_block])
    tracker.F = F
    observation = Vector([pose.x, pose.y, pose.sum_ψ, pose.vψ])
    predict!(tracker)
    update!(tracker, observation)
    state.x, state.vx, state.ax, state.y, state.vy, state.ay, state.ψ, state.vψ, state.aψ = get_mean(tracker)

    exstate = compute_extend_state(state)
    println(exstate)
end

function pid_ctrl(pub)
    loop_rate = Rate(100.0)
    global ctrl
    global state
    global exstate
    global input
    global reference 
    global Data
    i = 1
    
    while ! is_shutdown() 
        β_ref = - π / 3 #+ clamp(controla!(R_controller, err_R),-π/10,π/10)
        err = round_angle(exstate.β - β_ref)
        if safe == true
            δ = clamp(controla!(β_controller, err), 0, π/8)
            ω = 5.0
        else
            δ = π/8
            ω = 1.0
        end
        ctrl.control.mode = 3
        ctrl.control.servo = δ
        ctrl.control.speed = ω
        reference.R_ref = R_ref
        reference.β_ref = β_ref
        input.δ = δ
        input.ω = ω
        publish(pub, ctrl)
        rossleep(loop_rate)

        i += 1
        data_point = DriftData(safe, state, input, exstate, reference)
        push!(Data, data_point)
    end
end

function main()
    init_node("Start")
    sub1 = Subscriber{RosMsg.PoseStamped}("/vrpn_client_node/racecar/pose", update_rts, queue_size=1)
    sub2 = Subscriber{RosMsg.Imu}("/imu", update_imu, queue_size=1)
    sub3 = Subscriber{RosMsg.BoolMsg}("/Safe", update_safe, queue_size=1)
    pub = Publisher{RosMsg.VescCtrlStamped}("/vesc/ctrl", queue_size=1)
    global Data = []
    global safe = false
    global pose = Pose()
    global state = State()
    global exstate = ExtendState()
    global input = Input()
    global reference = Reference()
    global ctrl = RosMsg.VescCtrlStamped()
    global tracker = make_tracker()
    global curv_est = make_curv_estimator(25)
    global β_controller = PID(Kp=5, Ki=0.00, Kd=70, sign=1)
    global R_controller = PID(Kp=2, Ki=0.01, Kd=50, sign=-1)
    pid_ctrl(pub)
end

if ! isinteractive()
    main()
end