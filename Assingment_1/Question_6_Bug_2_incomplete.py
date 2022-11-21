from controller import Robot


from sympy.geometry import Ray
import numpy as np

from sympy.geometry import Point, Line, Polygon

def make_Point(p):
    if not isinstance(p, Point):
        return Point(p)
    else:
        return p

def make_Polygon(poly):
    if not isinstance(poly, Polygon):
        return Polygon(*poly)
    else:
        return poly  

def get_line_through_points(p1, p2):
    p1 = make_Point(p1)
    p2 = make_Point(p2)
    return Line(p1, p2)

def get_distance_between_points(p1, p2):
    p1 = make_Point(p1)
    p2 = make_Point(p2)
    return float(p1.distance(p2))

def get_distance_between_point_and_line(p, l):
    p = make_Point(p)
    return float(l.distance(p))

def get_distace_between_point_and_polygon(p, poly):
    p = make_Point(p)
    poly = make_Polygon(poly)
    return float(poly.distance(p))

def get_tangent_vector_to_polygon(poly):
    poly = make_Polygon(poly)
    slopes=[]
    for ls in poly.sides:
        slopes.append(float(ls.slope))
    return slopes

def get_intersection_between_polygons(poly1, poly2):
    poly1 = make_Polygon(poly1)
    poly2 = make_Polygon(poly2)
    return poly1.intersection(poly2)

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
    forward_ray = Ray(current_location, angle=current_orientation)
    desired_ray = m_line
    angle = float(forward_ray.angle_between(desired_ray))
    if float(desired_ray.angle_between(Ray(current_location, angle=angle))) == 0.0:
        pass
    else:
        angle = -angle
    return angle

m_line = None
goal_location = (1, 0) #1, 0
side_detected = False
mode = 0
while robot.step(timestep) != -1:
    current_location = [round(gps.getValues()[i], 3) for i in range(2)]
    if m_line == None:
        m_line = Ray(current_location, goal_location)
    current_orientation = inertial.getRollPitchYaw()[2]
    is_obstacle = check_obstacle(prox_sensors)
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
        else:
            if right_side_sensors:
                motors['right'].setVelocity(max_speed/8)
                motors['left'].setVelocity(max_speed)
            else:
                motors['right'].setVelocity(max_speed)
                motors['left'].setVelocity(max_speed/8)
        if hit_point != current_location and float(m_line.distance(q1.make_Point(current_location))) == 0:
            mode = 0
            print(f"Turn Towards Goal")
