using PackageCompiler

required_packages = [:DifferentialEquations,:RobotOS,:ReferenceFrameRotations,:BlockDiagonals,:JLD,:Plots,:Parameters,:JuMP,:Ipopt,:Statistics,:Kalman,:GaussianDistributions]

create_sysimage(required_packages, sysimage_path="sys_robot.so", precompile_execution_file="ready.jl")
