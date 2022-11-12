"""proprotional controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot


import numpy as np
from controller import InertialUnit
import math
import q1

max_speed = 6.28

Threshold = 0.005

def angle_of(A, B):
    return math.degrees(math.atan2((B[1]-A[1]), (B[0]-A[0])) % 360) + 90 

def calculate_line_err(distance):
    # print("distance is : ", distance)
    return distance < Threshold


def calculate_distance(a, b, c):
    x0, y0, _ = a
    x1, y1 ,_= b
    x2, y2 ,_= c
    return abs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1)) / np.sqrt(np.square(x2 - x1) + np.square(y2 - y1))


def calculate_endCond(current_position, destination):
    return np.sqrt((current_position[0] - destination[0]) ** 2 + (current_position[1] - destination[1]) ** 2)


def detect_obstacle(prox_sensors):
    for i in range(len(prox_sensors)):
        if i == 3 or i == 4:
            continue
        if prox_sensors[i].getValue() > 80:
            return True
    return False


def refine_current_orientation(sensed_orientation, five_prev_orientations):
    if abs(abs(np.average(five_prev_orientations)) - abs(sensed_orientation)) < 0.4:
        return sensed_orientation
    else:
        # print("refined to ", five_prev_orientations[0], " sensed was :", sensed_orientation)
        return five_prev_orientations[0]


def update_prev_orientations_list(new_value, five_prev_orientations):
    for i in range(0, 4):
        five_prev_orientations[i + 1] = five_prev_orientations[i]
    five_prev_orientations[0] = new_value
    return five_prev_orientations


def find_desired_orientation(start_point, destination):
    if destination[0] - start_point[0] < 0:
        if destination[1] - start_point[1] < 0:
            desired = np.arctan((destination[0] - start_point[0]) / (destination[1] - start_point[1]))
        else:
            desired = np.pi - np.arctan(-(destination[0] - start_point[0]) / (destination[1] - start_point[1]))
    else:
        if destination[1] - start_point[1] < 0:
            desired = np.arctan((destination[0] - start_point[0]) / (destination[1] - start_point[1]))
        else:
            desired = np.arctan((destination[0] - start_point[0]) / (destination[1] - start_point[1])) - np.pi

    return desired


def euclid_distance(point1, point2):
    distance = np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)
    # print(distance)
    return distance


# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0)
right_motor.setVelocity(0)

    
while robot.step(timestep) != -1:

    gps = robot.getDevice('gps')
    gps.enable(timestep)

    inertial = robot.getDevice("inertial unit")
    inertial.enable(timestep)

    ds_values = []
    for i in range(3):
        ds = robot.getDevice(f'ir{i}'.format(i))
        ds.enable(timestep)
        ds_values.append(0.7611*ds.getValue()**(-0.9313)-0.1252)

    goal_location = (0.8, 0.3)

    current_location = gps.getValues()
    current_location[0] = round(current_location[0], 3)
    current_location[1] = round(current_location[1], 3)

    current_orientation = inertial.getRollPitchYaw()[2]
    current_orientation = current_orientation*180/np.pi
    # print(current_orientation)

    # target_line = q1.get_line_through_points(current_location[:2], goal_location)

    target_angle = angle_of(current_location[:2],goal_location) -90
    # print(target_angle)

    current_distance = q1.get_distance_between_points(current_location[:2], goal_location)
    print(current_distance)

    # print(ds_values)

    if current_distance <=0.1:
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        print('goal reached!!!')
        break

    if ds_values[0]>0.2 and ds_values[1]>0.2 and ds_values[2]>0.2:
        if abs(current_orientation - target_angle)< 10:
            left_motor.setVelocity(max_speed)
            right_motor.setVelocity(max_speed)
        elif current_orientation<target_angle:
            left_motor.setVelocity(-max_speed)
            right_motor.setVelocity(max_speed)
        else:
            left_motor.setVelocity(max_speed)
            right_motor.setVelocity(-max_speed)

        mode=1
        print('free')

    elif ds_values[1]<=0.2:
        left_motor.setVelocity(max_speed/2)
        right_motor.setVelocity(max_speed*ds_values[1])
        print('left')

    elif ds_values[2]<=0.2:
        left_motor.setVelocity(max_speed*ds_values[2])
        right_motor.setVelocity(max_speed/2)
        print('right')

    elif ds_values[0]<=0.2:
        left_motor.setVelocity(max_speed/3)
        right_motor.setVelocity(max_speed*ds_values[0])
        print('left')

