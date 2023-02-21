# Import required libraries
from sympy import *
import numpy as np
import math

# A dictionary to map direction integers to their corresponding string values
Direction_lookup = {0: 'DOWN', 1: 'UP'}

# A function that creates a line with the given slope and angle, generates a random segment on that line,
# gets the side of the segment where the given point is located, finds the perpendicular segment of the line
# that passes through the given point, and returns a list of line and point information.
def create_mt(angle, direction, p):
    # Calculate the slope of the line using the given angle
    slope = math.tan(math.radians(angle))
    # Create a line object with slope and passing through the origin
    mt = Line(Point(0, 0), slope=slope)
    # Generate a random segment on the line and get its endpoints and the segment itself
    first_extreme, second_extreme, mt_segment = rnd_segment_on_line(mt, direction)
    # Calculate the angle of the line in degrees and round to the nearest integer
    mt_angle = round(line_angle(slope, direction))
    # Get the side of the segment where the given point is located
    side = get_side(mt_segment, p)
    # Find the perpendicular segment of the line that passes through the given point
    hp = mt.perpendicular_segment(p)
    # Get the endpoint of the perpendicular segment that is on the line
    h = hp.p2
    # Create a list of line information and point information and return it
    mt_info = [mt, mt_angle, direction]
    point_info = [p, h, hp, side]
    return mt_info, point_info

# A function that returns the position of a point with respect to a segment
# (either on the left side or the right side of the segment).
def get_side(segment, DP):
    if np.cross(DP - segment.p1, segment.p2 - segment.p1) < 0:
        pos = "SX"
    else:
        pos = "DX"
    return pos


def rnd_segment_on_line(line, direction):
    # Generate two random points on the given line
    p1 = line.random_point()
    p2 = line.random_point()

    # Ensure that the two points are distinct
    while p2 == p1:
        p2 = line.random_point()

    # Determine the segment endpoints and direction based on the given direction
    if p1.y == 0 and p2.y == 0:
        if p1.x < p2.x:
            if direction == 1:
                segment = Segment(N(p1), N(p2))
            else:
                segment = Segment(N(p2), N(p1))
        else:
            if direction == 1:
                segment = Segment(N(p2), N(p1))
            else:
                segment = Segment(N(p1), N(p2))
    else:
        if p1.y < p2.y:
            if direction == 1:
                segment = Segment(N(p1), N(p2))
            else:
                segment = Segment(N(p2), N(p1))
        else:
            if direction == 1:
                segment = Segment(N(p2), N(p1))
            else:
                segment = Segment(N(p1), N(p2))

    # Return the segment endpoints and the segment itself
    return p1, p2, segment


def line_angle(slope, direction):
    # Compute the angle of the line based on its slope and direction
    if direction == 1:
        if slope < 0:
            angle = ((math.degrees(math.atan(slope))) + 180) % 360
        else:
            angle = (math.degrees(math.atan(slope))) % 360
    else:
        if slope < 0:
            angle = (math.degrees(math.atan(slope))) % 360
        else:
            angle = ((math.degrees(math.atan(slope))) + 180) % 360

    # Return the angle
    return angle

def intermediates(p1, p2, nb_points=8):
    # Compute the spacing between x and y coordinates of the intermediate points
    x_spacing = (p2[0] - p1[0]) / (nb_points + 1)
    y_spacing = (p2[1] - p1[1]) / (nb_points + 1)

    # Generate the intermediate points as a list of lists
    return [[p1[0] + i * x_spacing, p1[1] + i * y_spacing]
            for i in range(1, nb_points + 1)]
