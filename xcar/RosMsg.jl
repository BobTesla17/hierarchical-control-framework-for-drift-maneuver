module RosMsg

using RobotOS

# PoseStamped: RTS tracker VRPN
# VescCtrlStamped: simulate control of Racecar
@rosimport geometry_msgs.msg:PoseStamped
@rosimport sensor_msgs.msg:Imu
@rosimport vesc_msgs.msg:VescCtrlStamped
@rosimport vesc_msgs.msg:VescStateStamped
@rosimport std_msgs.msg:Bool
@rosimport std_msgs.msg:Float64

rostypegen(@__MODULE__)
import .geometry_msgs.msg:PoseStamped
import .sensor_msgs.msg:Imu
import .vesc_msgs.msg:VescCtrlStamped
import .vesc_msgs.msg:VescStateStamped
import .std_msgs.msg:BoolMsg
import .std_msgs.msg:Float64Msg

end





