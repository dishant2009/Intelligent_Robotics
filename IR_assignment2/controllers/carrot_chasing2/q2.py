import math

def line(A, B):
    m = (B[1] - B[0]) / (A[1] - A[0])
    c = B[0] - m*A[0]
    return m, c

def dist_betn(p1, p2):
    """
    Calculates distance B/W P1, P2
    :return: distance: int
    """
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

def distance_from_line(x, A, B):
    length_AB = dist_betn(A, B)
    distance_from_AB = abs((B[1]-A[1])*(A[0] - x[0]) - (B[0]-A[0])*(A[1]-x[1]))
    distance_from_AB /= length_AB
    return distance_from_AB

def distance_frm_polygon(x, pairs):
    distances = []
    for (A, B) in pairs:
        distances.append(
            distance_from_line(x, A, B)
        )
    return min(distances)


def angle(robot_pos, goal_pos):
    x = robot_pos[0]-goal_pos[0]
    y = robot_pos[1]-goal_pos[1]
    rad = math.atan2(y, x)
    return 180+math.degrees(rad)

def angle_of(A, B):
    return math.degrees(math.atan2((B[1]-A[1]), (B[0]-A[0])) % 360) + 90 
    
def get_bearing_in_degrees(north):
    rad = math.atan2(north[0], north[1])
    bearing = (rad - 1.5708) / 3.14 * 180.0
    bearing += 180
    if bearing < 0.0:
        bearing = bearing + 360.0

    return bearing
    
def on_line(x, A, B, tolerance=0.02):      
    dist = distance_from_line(x, A, B)
    if dist > tolerance:
        return False
    else:
        return 
