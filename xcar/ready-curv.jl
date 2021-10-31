include("RosMsg.jl")
include("curv-estimator.jl")
include("pid-struct.jl")
include("tool-func.jl")
include("data-struct.jl")
include("kf.jl")
include("L1AC.jl")

using RobotOS
using ReferenceFrameRotations
using BlockDiagonals
using Plots
using Statistics
using JLD

global safe = false
global pose = Pose()
global robo = Pose()
global state = State()
global exstate = ExtendState()
global input = Input()
global reference = Reference()
global vesc = Vesc()
global ctrl = RosMsg.VescCtrlStamped()
global tracker = make_tracker()
global curv_est = make_curv_estimator(25)
est_circle = update!(curv_est, 0., 0., 0., 0., 0.)

#global l1ac = L1AC_Struct()
#R_l1ac = L1AC!(l1ac, convert(Float64,1.0), 1.0, 0.10)

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
    F_block = [1 dt 0.5 * dt ^ 2; 0 1 dt; 0 0 1]
    F = BlockDiagonal([F_block, F_block, F_block])
    tracker.F = F
    observation = Vector([pose.x, pose.y, pose.sum_ψ, pose.vψ])
    predict!(tracker)
    update!(tracker, observation)
    state.x, state.vx, state.ax, state.y, state.vy, state.ay, state.ψ, state.vψ, state.aψ = get_mean(tracker)
    exstate = compute_extend_state(state)
end

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

function update_robo(msg::RosMsg.PoseStamped)
    global robo
    robo.x = msg.pose.position.x
    robo.y = msg.pose.position.y
end

function update_imu(msg::RosMsg.Imu)
    global pose
    pose.vψ = msg.angular_velocity.z
end

function update_servo(msg::RosMsg.Float64Msg)
    global vesc
    vesc.servo = msg.data
end

function update_rpm(msg::RosMsg.VescStateStamped)
    global vesc
    vesc.rpm = msg.state.speed
    vesc.volt = msg.state.voltage_input
    vesc.current_motor = msg.state.current_motor
    vesc.current_input = msg.state.current_input
    vesc.duty_cycle = msg.state.duty_cycle
    vesc.charge_drawn = msg.state.charge_drawn
    vesc.charge_regen = msg.state.charge_regen
    vesc.energe_drawn = msg.state.energe_drawn
    vesc.energe_regen = msg.state.energe_regen
end

function control_wrapper(pub::Publisher, control_fn!, reset_fn)
    """
    - control_fn!: a controller function (i, state, exstate, reference) |-> (δ, ω); may modify reference
    - reset_fn: a function that resets controller memory
    """
    data = []
    loop_rate = Rate(100.0)
    global safe
    global state
    global exstate
    global input
    global reference 
    global ctrl
    global robo
    global vesc
    
    tmp = false      # 防止因手柄命令冲突导致的小车抽搐
    interrupted = false
    dt = 1 / 100
    last_t = time()
    current_t = time()
    i = 1
    
    println("Ready")
    while !interrupted
        try
            current_t = time()   
            if safe
                stamp = time()
                input.δ, input.ω = control_fn!(i, state, input, exstate, reference, robo)
                dt = current_t - last_t
                data_point = DriftData(deepcopy(safe), deepcopy(state), deepcopy(input), deepcopy(exstate), deepcopy(reference), deepcopy(dt), deepcopy(stamp),deepcopy(vesc))
                push!(data, data_point)
                i += 1
                ctrl.control.mode = 3
                ctrl.control.servo = input.δ
                ctrl.control.speed = input.ω
                publish(pub, ctrl)
                tmp = true
            else
                reset_fn()
                i = 1
                input.δ = 0
                input.ω = 0
                if tmp
                    tmp = false
                    ctrl.control.mode = 3
                    ctrl.control.servo = input.δ
                    ctrl.control.speed = input.ω
                    publish(pub, ctrl)
                end
            end
            
            # ctrl.control.mode = 3
            # ctrl.control.servo = input.δ
            # ctrl.control.speed = input.ω
            # publish(pub, ctrl)
            rossleep(loop_rate)
            last_t = current_t
        catch ex
            if isa(ex, InterruptException)
                println("Interrupted")
                interrupted = true
            else
                throw(ex)
            end
        end
    end
    
    # stop car
    ctrl.control.mode = 3
    ctrl.control.servo = 0
    ctrl.control.speed = 0
    for i in 1:20
        publish(pub, ctrl)
        rossleep(loop_rate)
    end
    
    return data
end

println("complete initialization")
