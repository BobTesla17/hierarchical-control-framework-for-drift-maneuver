using JLD
using Plots
using Statistics

data = load("data.jld","data")

fig = plot(collect(1:2000)/100,[p.es.R for p in data[4001:6000]], label = "R")
plot!(fig,collect(1:2000)/100,[p.ref.R_ref for p in data[4001:6000]], label = "R_ref")
ylims!(fig,0,2)
title!(fig,"mean=0.9865(0.9879),var=0.0011(0.002)")
plot(fig)
savefig(fig,"R_data.png")

@show mean([p.es.R for p in data[4001:6000]]) # 0.9865264531430152
@show var([p.es.R for p in data[4001:6000]]) # 0.001108414221837044

@show mean([sqrt(p.s.x^2+p.s.y^2) for p in data[4001:6000]]) # 0.987881523
@show var([sqrt(p.s.x^2+p.s.y^2) for p in data[4001:6000]]) # 0.00207

fig2 = plot(collect(1:2000)/100,[p.es.β for p in data[4001:6000]], label = "β")
plot!(fig2,collect(1:2000)/100,[p.ref.β_ref for p in data[4001:6000]], label = "β_ref")
ylims!(fig2,-2,0)
title!(fig2,"mean=-1.194,var=0.00045")
plot(fig2)
savefig(fig2,"β_data.png")

@show mean([p.es.β for p in data[4001:6000]]) # -1.194212097406
@show var([p.es.β for p in data[4001:6000]]) # 0.00044738665852627927

