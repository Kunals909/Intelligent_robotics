from controller import Robot, Motor, DistanceSensor
import math
def get_line(A, B):
    m = (B[1] - B[0]) / (A[1] - A[0])
    c = B[0] - m*A[0]
    return m, c

def distance_between(A, B):
    return math.sqrt((A[0] - B[0])**2 + (A[1] - B[1])**2)

def angle_of(A, B):
    return math.degrees(math.atan2((B[1]-A[1]), (B[0]-A[0])) % 360) + 90 
    
def distance_from_line(x, A, B):
    length_AB = distance_between(A, B)
    distance_from_AB = abs((B[1]-A[1])*(A[0] - x[0]) - (B[0]-A[0])*(A[1]-x[1]))
    distance_from_AB /= length_AB
    return distance_from_AB
    
def distance_from_polygon(x, pairs):
    distances = []
    for (A, B) in pairs:
        distances.append(
            distance_from_line(x, A, B)
        )
    return min(distances)


# For Bug algorithms
def on_line(x, A, B, tolerance=0.02):   
    dist = distance_from_line(x, A, B)
    if dist > tolerance:
        return False
    else:
        return True

def get_bearing_in_degrees(north):
    rad = math.atan2(north[0], north[1])
    bearing = (rad - 1.5708) / 3.14 * 180.0
    bearing += 180
    if bearing < 0.0:
        bearing = bearing + 360.0

    return bearing

robot = Robot()

# Constants    
TIME_STEP = 64
MAX_SPEED = 6.28
GOAL_POSITION = [0.4, 0.4, 0.0]
POS_EPSILON = 0.08  
OBS_PROX = 150.0
ANGLE_EPSILON = 0.05

#initialize motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))  # number of radians the motor rotates
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# initialize devices
ps = []
ps_names = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]
for i in range(8):
    ps.append(robot.getDevice(ps_names[i]))
    ps[i].enable(TIME_STEP)

ps_values = [0 for i in range(8)]

gps = robot.getDevice('gps')
gps.enable(TIME_STEP)

compass = robot.getDevice('compass')
compass.enable(TIME_STEP)

state = 'begin'

while robot.step(TIME_STEP) != -1:
    # read sensors outputs
    for i in range(8):
        ps_values[i] = ps[i].getValue()    
    current_position = gps.getValues()
    current_angle = get_bearing_in_degrees(compass.getValues())
    
    # initialize motor speeds at 50% of MAX_SPEED.
    left_speed  = 0.5 * MAX_SPEED
    right_speed = 0.5 * MAX_SPEED
    
    # at time begin
    if state == 'begin':
        start_position = gps.getValues()
        aligned_to_goal = angle_of(current_position, GOAL_POSITION) > 0.95*current_angle
        aligned_to_goal = aligned_to_goal and angle_of(current_position, GOAL_POSITION) < 1.05*current_angle
        
        if not aligned_to_goal:
            print('aligning to the goal')
            left_speed  = -0.50 * MAX_SPEED
            right_speed = 0.50 * MAX_SPEED
            state = 'begin'
        else:
            state = 'move_to_goal'

    elif state == 'move_to_goal':
        obstacle_detected = ps_values[0] > OBS_PROX and ps_values[7] > OBS_PROX
        if obstacle_detected:
            hit_point = gps.getValues()
            hit_angle = get_bearing_in_degrees(compass.getValues())
            state = 'follow_obstacle'
        elif distance_between(current_position, GOAL_POSITION) <= POS_EPSILON:
            state = 'end'
        elif not on_line(current_position, start_position, GOAL_POSITION):
            # move back on line
            heading_angle = current_angle
            goal_angle = angle_of(start_position, GOAL_POSITION)
            
            if (heading_angle - goal_angle) > ANGLE_EPSILON:
                print('aligning to the goal')
                left_speed  = 0.5 * MAX_SPEED
                right_speed = 0.1 * MAX_SPEED
            elif (heading_angle - goal_angle) < -ANGLE_EPSILON:
                print('aligning to the goal')
                left_speed  = 0.1 * MAX_SPEED
                right_speed = 0.5 * MAX_SPEED
        else:
            print('moving to goal')
            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed)
            
    elif state == 'follow_obstacle':
        print('following obstacle boundary')
        front_clear = max(ps_values[0:1]) < OBS_PROX and max(ps_values[6:7]) < OBS_PROX
        aligned_to_goal = angle_of(current_position, GOAL_POSITION) > 0.95*current_angle
        aligned_to_goal = aligned_to_goal and angle_of(current_position, GOAL_POSITION) < 1.05*current_angle
        if front_clear and aligned_to_goal:
            print('Robot staus: goal reachable')
            state = 'move_to_goal'
            left_speed  = 0.20 * MAX_SPEED
            right_speed = 0.50 * MAX_SPEED
            start_position = current_position
            continue
  
        right_side_covered = ps_values[2] > OBS_PROX
        if not right_side_covered:
            left_speed  = -0.5 * MAX_SPEED
            right_speed = 0.5 * MAX_SPEED
        else:
            right_value = max(ps_values[0:2])
            left_value = max(ps_values[5:7])
            if right_value > 2.0*OBS_PROX:
                left_speed  = 0.20 * MAX_SPEED
                right_speed = 0.50 * MAX_SPEED
            elif right_value < OBS_PROX and left_value < OBS_PROX:
                left_speed  = 0.50 * MAX_SPEED
                right_speed = 0.20 * MAX_SPEED
            else:
                left_speed  = 0.50 * MAX_SPEED
                right_speed = 0.50 * MAX_SPEED
                
    elif state == 'end':
        print('goal reached')
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        break
        
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
