from controller import Robot
import q1

import numpy as np

robot = Robot()
timestep = 32
max_speed = 6.28

def init_robot(robot, timestep):
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    motors = {'left': left_motor, 'right': right_motor}

    gps = robot.getDevice('gps')
    gps.enable(timestep)

    inertial = robot.getDevice("inertial unit")
    inertial.enable(timestep)

    prox_sensors = []
    for i in range(8):
        sensor_name = 'ps' + str(i)
        prox_sensors.append(robot.getDevice(sensor_name))
        prox_sensors[i].enable(timestep)
    
    return motors, gps, inertial, prox_sensors

motors, gps, inertial, prox_sensors = init_robot(robot, timestep)

def check_wall(prox_sensors):
    readings=[]
    for idx in range(8):
        readings.append(prox_sensors[idx].getValue()>80)
    return readings

goal_location = (0.75, 0)
prev_distace = float('inf')
side_detected = False
right_side_sensors = False
mode=1

while robot.step(timestep) != -1:

    current_location = gps.getValues()
    current_location[0] = round(current_location[0], 3)
    current_location[1] = round(current_location[1], 3)

    current_orientation = inertial.getRollPitchYaw()[2]

    target_line = q1.get_line_through_points(current_location[:2], goal_location)

    current_distance = q1.get_distance_between_points(current_location[:2], goal_location)

    is_wall = check_wall(prox_sensors)

    if sum(is_wall)==0:
        if abs(np.tan(current_orientation) - float(target_line.slope))< 0.1:
            if current_distance<=prev_distace:
                motors['left'].setVelocity(max_speed)
                motors['right'].setVelocity(max_speed)
                prev_distace = current_distance
            else:
                motors['left'].setVelocity(-max_speed)
                motors['right'].setVelocity(max_speed)
                prev_distace = current_distance
        else:
            motors['left'].setVelocity(-max_speed)
            motors['right'].setVelocity(max_speed)
        mode=1
    else:
        if mode==1:
            motors['left'].setVelocity(0)
            motors['right'].setVelocity(0)  
            prev_distace = current_distance
            mode = 2
        elif mode==2:
            right_wall = is_wall[2]
            front_wall = is_wall[0]
            rcorner_wall = is_wall[1]
            if right_wall and not(is_wall[1] or is_wall[3] or is_wall[6] ) :
                motors['left'].setVelocity(max_speed)
                motors['right'].setVelocity(max_speed)
                prev_distace = current_distance
            elif right_wall and is_wall[0] and is_wall[7]:
                motors['left'].setVelocity(-max_speed)
                motors['right'].setVelocity(max_speed)
            else:
                motors['left'].setVelocity(-max_speed)
                motors['right'].setVelocity(max_speed)