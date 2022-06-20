@time include("ready.jl")

β_controller = PID(Kp=1.0, Ki=0.0006, Kd=50.0, sign=1)
R_controller = PID(Kp=5.0, Ki=0.05, Kd=5.0, sign=-1)
phase = 1
t1 = 0
t12 = 0

reference.R_ref = 1.0
reference.β_ref = -1.2
Cx = 0
Cy = 0


function control_fn!(i, state, input, exstate, reference, robo)

    δ_start = 0.3
    ω_start = 6.0
    t_start = 60

    δ_end = -0.1
    ω_end = 3.2
    t_end = 20

    if i <= 100
        δ = 0.0
        ω = 1.0
    elseif i <= 100 + t_start
        δ = δ_start
        ω = ω_start
    elseif i <= 100 + t_start + t_end
        δ = δ_end
        ω = ω_end
    else 
        δ = 0.0
        ω = 0.0
    end

    return δ, ω
end

function reset_fn()
    β_controller.err_sum = 0
    β_controller.last_err = 0
    R_controller.err_sum = 0
    R_controller.last_err = 0
    phase = 1
    reference.R_ref = 1.0
    reference.β_ref = -1.2
    Cx = 0
    Cy = 0
end

init_node("Controller", disable_signals=true)
sub1 = Subscriber{RosMsg.PoseStamped}("/vrpn_client_node/sensorcar/pose", update_rts, queue_size=1) 
sub2 = Subscriber{RosMsg.Imu}("/imu", update_imu, queue_size=1)
sub3 = Subscriber{RosMsg.BoolMsg}("/Safe", update_safe, queue_size=1)
#sub4 = Subscriber{RosMsg.PoseStamped}("/vrpn_client_node/robomaster/pose", update_robo, queue_size=1)
sub5 = Subscriber{RosMsg.VescStateStamped}("/vesc/sensors/core", update_rpm, queue_size=1)
sub6 = Subscriber{RosMsg.Float64Msg}("/vesc/sensors/servo_position_command", update_rpm, queue_size=1)
pub = Publisher{RosMsg.VescCtrlStamped}("/vesc/ctrl", queue_size=1)
data = control_wrapper(pub, control_fn!, reset_fn);

# p1 = plot([p.es.β for p in data],label="β")
# plot!(p1, [p.ref.β_ref for p in data],label="β_ref")
# p2 = plot([p.es.R for p in data],label="R")
# plot!(p2, [p.ref.R_ref for p in data],label="R_ref")
# ylims!(p1,-2,0)
# ylims!(p2,0,5)
# plot(p1,p2,size = (2800, 1400),xtickfontsize=50,ytickfontsize=50,legendfontsize=50,width=5)

# p3 = plot([p.vesc.rpm for p in data],label="β")
# plot(p3,size = (1400, 1400),xtickfontsize=50,ytickfontsize=50,legendfontsize=50,width=5)


save("park.jld","data",data)

δ_start = 0.3
ω_start = 6.0
t_start = 60

δ_end = -0.1
ω_end = 3.2
t_end = 20

lF = 0.185
lR = 0.12

wid = 5
wid2 = 3

data = data[1:100 + t_start + t_end + 100]
save("park-core.jld","data",data)
fig_traj3 = plot([p.s.x for p in data], [p.s.y for p in data], aspect_ratio=:equal, color=:blue, label="", w=wid2);
plot!(fig_traj3, [p.s.x for p in data[100:100+t_start]], [p.s.y for p in data[100:100+t_start]], color=:red, label="", w=wid2);
plot!(fig_traj3, [p.s.x for p in data[100+t_start:100+t_start+t_end]], [p.s.y for p in data[100+t_start:100+t_start+t_end]], color=:orange, label="", w=wid2);

for p in data[1:Int(floor(length(data) / 8)):length(data)]
    s = p.s
    xF, yF = s.x + lF * cos(s.ψ), s.y + lF * sin(s.ψ);
    xR, yR = s.x - lR * cos(s.ψ), s.y - lR * sin(s.ψ);
    plot!(fig_traj3, [xR, xF], [yR, yF], color=:green, arrow=true, label="");
end
xlabel!(fig_traj3, "x(m)");
ylabel!(fig_traj3, "y(m)");
xlims!(fig_traj3, -1.0, 4.0)
ylims!(fig_traj3, -1.0, 2.5)
fig2 = plot(fig_traj3)
savefig(fig2, "data/drift-park.png");
println(data[end].es.β)