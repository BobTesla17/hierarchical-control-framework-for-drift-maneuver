@time include("ready.jl")
using JLD
data = load("park-core.jld","data")
x_sim, y_sim, ψ_sim, β_sim = load("park-sim.jld", "x","y","ψ","β")

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

fig_β = plot(0.01*collect(1:1:length(data)), [p.es.β for p in data], label="β");
xlabel!(fig_β, "t(s)");

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
fig1 = plot(fig_traj3)
fig2 = plot(fig_β)
savefig(fig1, "drift-park-ex-1.png");
savefig(fig2, "drift-park-ex-2.png");
println(data[end].es.β)

x_ = [p.s.x - data[1].s.x for p in data]
y_ = [p.s.y - data[1].s.y for p in data]
θ = atan((data[50].s.y - data[1].s.y),(data[50].s.x - data[1].s.x))
x = zeros(length(x_))
y = zeros(length(y_))
β = [p.es.β for p in data]
for i in [1:length(x_)]
    x[i] = x_[i] * cos(-θ) - y_[i] * sin(-θ)
    y[i] = x_[i] * sin(-θ) + y_[i] * cos(-θ)
end
ψ = [p.s.ψ - θ for p in data]


fig_traj = plot(x, y, aspect_ratio=:equal, color=:blue, label="", w=wid2);
plot!(fig_traj, x[100:100+t_start], y[100:100+t_start], color=:red, label="", w=wid2);
plot!(fig_traj, x[100+t_start:100+t_start+t_end], y[100+t_start:100+t_start+t_end], color=:orange, label="", w=wid2);

for i in collect(1:Int(floor(length(data) / 8)):length(data))
    xF, yF = x[i] + lF * cos(ψ[i]), y[i] + lF * sin(ψ[i]);
    xR, yR = x[i] - lR * cos(ψ[i]), y[i] - lR * sin(ψ[i]);
    plot!(fig_traj, [xR, xF], [yR, yF], color=:green, arrow=true, label="");
end
xlabel!(fig_traj, "x(m)");
ylabel!(fig_traj, "y(m)");

fig_traj = plot!(x_sim, y_sim, aspect_ratio=:equal, color=:blue, label="", w=wid2);
plot!(fig_traj, x_sim[100:100+t_start], y_sim[100:100+t_start], color=:red, label="", w=wid2);
plot!(fig_traj, x_sim[100+t_start:100+t_start+t_end], y_sim[100+t_start:100+t_start+t_end], color=:orange, label="", w=wid2);

for i in collect(1:Int(floor(length(data) / 8)):length(data))
    xF, yF = x_sim[i] + lF * cos(ψ_sim[i]), y_sim[i] + lF * sin(ψ_sim[i]);
    xR, yR = x_sim[i] - lR * cos(ψ_sim[i]), y_sim[i] - lR * sin(ψ_sim[i]);
    plot!(fig_traj, [xR, xF], [yR, yF], color=:green, arrow=true, label="");
end
xlabel!(fig_traj, "x(m)");
ylabel!(fig_traj, "y(m)");
fig4 = plot(fig_traj);
savefig(fig4, "drift-park-ex-4.png");

fig_x = plot(x, label="x_real", legend = :outertopleft, )
plot!(fig_x, x_sim, label="x_sim", legend = :outertopleft)
plot!(fig_x, [100,100+t_start,100+t_start+t_end], seriestype="vline", label="")
fig_y = plot(y, label="x_real", legend = :outertopleft)
plot!(fig_y, y_sim, label="x_sim", legend = :outertopleft)
fig_ψ = plot(ψ, label="x_real", legend = :outertopleft)
plot!(fig_ψ, ψ_sim, label="x_sim", legend = :outertopleft)
fig_β = plot(β, label="x_real", legend = :outertopleft)
plot!(fig_β, β_sim, label="x_sim", legend = :outertopleft)
fig_X = plot(fig_x, fig_y, fig_ψ, fig_β, layout=(2,2))
savefig(fig_X, "drift-park-ex-5.png");