from controller import Robot
import q1

from sympy.geometry import Ray

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


def can_move_forward(is_obstacle):
    for i in range(len(is_obstacle)):
        if i == 3 or i == 4:
            continue
        if is_obstacle[i]:
            return False
    return True


def check_obstacle(prox_sensors):
    readings = []
    for i in range(len(prox_sensors)):
        if prox_sensors[i].getValue() >= 80:
            readings.append(True)
        else:
            readings.append(False)
    return readings


def get_turn_angle(current_location, current_orientation, m_line):
    forward_line = Ray(current_location, angle=current_orientation)
    desired_line = m_line
    angle = float(forward_line.angle_between(desired_line))
    if float(desired_line.angle_between(Ray(current_location, angle=angle))) == 0.0:
        pass
    else:
        angle = -angle
    return angle


goal_location = (0.75, 0)
m_line = None
mode = 0
while robot.step(timestep) != -1:
    current_location = [round(gps.getValues()[i], 3) for i in range(2)]
    current_orientation = inertial.getRollPitchYaw()[2]
    is_obstacle = check_obstacle(prox_sensors)

    if m_line == None:
        m_line = Ray(current_location, goal_location)

    if mode == 0:
        turn_angle = get_turn_angle(
            current_location, current_orientation, m_line)

        if q1.get_distance_between_points(current_location, goal_location) <= 0.1:
            print("Reached Goal!!!")
            motors['left'].setVelocity(-max_speed)
            motors['right'].setVelocity(max_speed)
        elif abs(turn_angle) > 0.1:
            motors['left'].setVelocity(max_speed)
            motors['right'].setVelocity(-max_speed)
        elif abs(turn_angle) <= 0.1:
            print("Moving Forward")
            mode = 1

    elif mode == 1:
        if can_move_forward(is_obstacle):
            motors['left'].setVelocity(max_speed)
            motors['right'].setVelocity(max_speed)
        else:
            print("Wall Detected")
            motors['left'].setVelocity(0)
            motors['right'].setVelocity(0)
            hit_point = current_location
            print(hit_point)
            side_detected = False
            mode = 2

    elif mode == 2:
        front_left_wall = is_obstacle[7] or is_obstacle[6]
        front_right_wall = is_obstacle[0] or is_obstacle[1]

        if not side_detected and front_left_wall:
            side_detected = True
            right_side_sensors = True
            motors['left'].setVelocity(-max_speed)
            motors['right'].setVelocity(max_speed)
        elif not side_detected and front_right_wall:
            side_detected = True
            right_side_sensors = False
            motors['left'].setVelocity(max_speed)
            motors['right'].setVelocity(-max_speed)
        elif side_detected and not right_side_sensors and (front_right_wall or front_left_wall):
            motors['left'].setVelocity(max_speed)
            motors['right'].setVelocity(-max_speed)
        elif side_detected and right_side_sensors and (front_right_wall or front_left_wall):
            motors['left'].setVelocity(-max_speed)
            motors['right'].setVelocity(max_speed)
        elif is_obstacle[5] or is_obstacle[2]:
            last_hit_is_valid = True
            motors['left'].setVelocity(max_speed)
            motors['right'].setVelocity(max_speed)
        elif right_side_sensors:
            motors['right'].setVelocity(max_speed/8)
            motors['left'].setVelocity(max_speed)
        elif not right_side_sensors:
            motors['right'].setVelocity(max_speed)
            motors['left'].setVelocity(max_speed/8)

        if float(m_line.angle_between(Ray(hit_point, current_location))) < 0.1 and float(q1.make_Point(hit_point).distance(q1.make_Point(current_location))) >= 0.05:
            mode = 0
            print(f"Turn Towards Goal")
