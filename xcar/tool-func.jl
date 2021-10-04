round_angle = a -> mod(a + π, 2 * π) - π

function angle_mapping(angle1, angle2, angle3)
    # angle1: total angle
    # angle2: last angle
    # angle3: current angle
    if angle3 - angle2 < -π
        angle1 = angle1 + angle3 - angle2 + 2 * π
    elseif angle3 - angle2 > π
        angle1 = angle1 + angle3 - angle2 - 2 * π
    else
        angle1 = angle1 + angle3 - angle2
    end
    return angle1
end
