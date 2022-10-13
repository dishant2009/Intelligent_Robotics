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