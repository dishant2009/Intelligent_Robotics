from controller import Robot, Motor, GPS, Compass
from q2 import *
from math import *

# create the Robot instance.
robot = Robot()

timestep = int(robot.getBasicTimeStep())
max_speed = 4

target = [-0.6, 0.2, 0]
obs = 100 
target_prox = 0.08
angle_epsilon = 1.5

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

leftMotor.setVelocity(0.5 * max_speed)
rightMotor.setVelocity(0.5 * max_speed)

gps = robot.getDevice('gps')
gps.enable(timestep)

compass = Compass('compass')
compass = robot.getDevice('compass')
compass.enable(timestep)

state = 'start'

while robot.step(timestep) != -1:
    
    curr_pos = [gps.getValues()[0], gps.getValues()[1], gps.getValues()[2]]
    
    heading_angle = get_bearing_in_degrees(compass.getValues())
    
    leftSpeed = 0.5 * max_speed
    rightSpeed = 0.5 * max_speed
    
    
    if state == 'start':
        start_pos = gps.getValues()
        print("angle:",angle_of(curr_pos, target))
        print("heading angle:",heading_angle)
        aligned = angle_of(curr_pos, target) > 0.95*heading_angle and angle_of(curr_pos, target) < 1.05*heading_angle
        
        if not aligned: 
            leftSpeed = -0.5 * max_speed
            rightSpeed = 0.5 * max_speed
            state = 'start'
        else:
            state = 'move'
    
    elif state == 'move':
        if dist_betn(curr_pos, target) < target_prox:
            state = 'reached'
        
        elif not on_line(curr_pos, start_pos, target):
            angle = heading_angle
            goal_angle = angle_of(start_pos, target)
            
            if (angle - goal_angle) > angle_epsilon:
                print('Robot staus: aligning to the goal')
                leftSpeed  = 0.5 * max_speed
                rightSpeed = 0.1 * max_speed
            elif (angle - goal_angle) < -angle_epsilon:
                print('Robot staus: aligning to the goal')
                leftSpeed  = 0.1 * max_speed
                rightSpeed = 0.5 * max_speed
        else:
            print('Robot staus: moving to goal')
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)
        
    elif state == 'reached':
        print("Reached")
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        break
        
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)