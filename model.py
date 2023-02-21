################ IMPORT ####################
import math
import geopy.distance
import numpy as np
import pyproj as pyproj
from sympy import *
import pandas as pd
from energy import *


############## FUNCTION ####################
def compute_wind_classes_strict(n_of_class, median=False):
    ## Function that construct the wind classes representation
    wind_classes = {}
    wind_sector = {}
    relative_wind_values = []
    n_sector = n_of_class * 2
    sector = 360 / n_sector

    lower_bound = 1
    upper_bound = sector

    while upper_bound <= 360:
        if not median:
            if abs(math.cos(math.radians(lower_bound))) > abs(math.cos(math.radians(upper_bound))):
                angle = lower_bound
            else:
                angle = upper_bound
        else:
            angle = math.floor(lower_bound + upper_bound / 2)
        wind_classes[(lower_bound, upper_bound + 1)] = angle % 360
        relative_wind_values.append(angle % 360)
        wind_sector[(lower_bound, upper_bound + 1)] = math.floor(lower_bound / sector)
        lower_bound = lower_bound + sector
        upper_bound = upper_bound + sector

    return wind_classes, wind_sector, relative_wind_values


def check_sector_strict(sect, angle):
    l, u = sect
    if l <= angle <= (u - 1):
        return True
    else:
        return False


def query_wind_dict(angle, wind_dict):
    if angle == 0:
        angle = 360
    for key in wind_dict:
        if check_sector_strict(key, angle):
            return wind_dict[key]


def list_delivery_slack(n_class, atan_mt, wind_direction):
    ## Function that return the list of angles to test in an adaptive way
    list_neg_alpha = []
    list_pos_alpha = []

    n_sector = n_class * 2
    sector = 360 / n_sector
    pos_slack = (wind_direction - atan_mt) % sector
    neg_slack = (-(wind_direction - atan_mt)) % sector
    # print(pos_slack)
    # if ((WIND_DIRECTION - atan_MT) % 360) % sector == 0:
    t = int(90 / sector)
    if pos_slack == 0:
        list_neg_alpha.append(0)
        neg_slack = sector
        pos_slack = sector
        t = t - 1
    # print("P ", pos_slack, " N ", neg_slack)
    for i in range(t):
        neg = neg_slack + i * sector
        pos = pos_slack + i * sector - 1
        list_neg_alpha.append(neg)
        list_pos_alpha.append(pos)
    list_pos_alpha.append(90)
    list_neg_alpha.append(90)
    if pos_slack - sector == 0:
        list_pos_alpha[t] = list_pos_alpha[t] - 1
        list_pos_alpha.append(90)
    # print("P ", list_pos_alpha, " N ", list_neg_alpha)
    return list_pos_alpha, list_neg_alpha


def compute_point_Takeoff(deviation, mt, alpha_t, point):
    if deviation.is_finite:
        atan = mt[1] + alpha_t
        TP_line = Line(point, slope=math.tan(math.radians(atan)))
        T = TP_line.intersect(mt[0])
        T = list(T)[0]
        x = T.x
        y = T.y
        x = round(x, 4)
        y = round(y, 4)
        T = Point(x, y)
        TP = Segment(T, point)
    # print(N(T),"\n")
    else:
        if mt[2] == 0:
            x = -math.inf
            y = -math.inf
        else:
            x = math.inf
            y = math.inf
        T = Point(x, y)
        TP = Segment(T, point)

    return T, TP


def compute_point_Landing(deviation, mt, alpha_l, point):
    if deviation.is_finite:
        atan = mt[1] + alpha_l
        PL_line = Line(point, slope=math.tan(math.radians(atan)))
        L = PL_line.intersect(mt[0])
        L = list(L)[0]
        x = L.x
        y = L.y
        x = round(x, 4)
        y = round(y, 4)
        L = Point(x, y)
        PL = Segment(point, L)
    # print(N(L),"\n")
    else:
        if mt[2] == 0:
            x = math.inf
            y = math.inf
        else:
            x = -math.inf
            y = -math.inf
        L = Point(x, y)
        PL = Segment(point, L)

    return L, PL


def compute_takeoff_landing_strict(point, env_param, env_dict, mt, p, n):
    """
		point: [P, H, HP, side]
		env_param: [Drone_speed,wind_speed,wind_direction,payload]
		w_direction: wind direction
		DP_side: point P line side
		p: alpha angles first quadrant
		n: alpha angles fourth quadrant
		mt : [mt,mt_angle, direction]
		env_dict : [dict_classes, dict_sect, dict_unit]
	"""
    takeoff_points = []
    landing_points = []
    for i in range(len(p)):
        if (point[3] == "DX"):
            # TAKEOFF -alpha, LANDING alpha
            atan_landing = (mt[1] + p[i]) % 360
            atan_takeoff = (mt[1] - n[i]) % 360
            HT_segment = (N(point[2].length) / math.tan(math.radians(-n[i])))
            HL_segment = (N(point[2].length) / math.tan(math.radians(p[i])))
            which_alhaT = -n[i]
            which_alhaL = p[i]
        else:
            # TAKEOFF alpha, LANDING -alpha
            atan_landing = (mt[1] - n[i]) % 360
            atan_takeoff = (mt[1] + p[i]) % 360
            HT_segment = (N(point[2].length) / math.tan(math.radians(p[i])))
            HL_segment = (N(point[2].length) / math.tan(math.radians(-n[i])))
            which_alhaT = p[i]
            which_alhaL = -n[i]

        L, PL = compute_point_Landing(HL_segment, mt, which_alhaL, point[0])
        T, TP = compute_point_Takeoff(HT_segment, mt, which_alhaT, point[0])
        relative_w_t = (env_param[2] - atan_takeoff) % 360
        relative_w_l = (env_param[2] - atan_landing) % 360
        # print(w_direction," - ",atan_takeoff," - ",relative_w_t," - ",which_alhaT)
        # print(HL_segment,H)
        # print(T,TP)
        wc_t = query_wind_dict(relative_w_t, env_dict[0])
        wc_l = query_wind_dict(relative_w_l, env_dict[0])

        sector_t = query_wind_dict(relative_w_t, env_dict[1])
        sector_l = query_wind_dict(relative_w_l, env_dict[1])

        # unit_t = env_dict[2][env_param[3], wc_t, env_param[1]]
        unit_t = env_dict[2][env_param[3], wc_t]
        cost_t = unit_t * N(TP.length)
        takeoff_points.append((T, TP, unit_t, cost_t, atan_takeoff, N(TP.length), wc_t, which_alhaT, sector_t))

        # unit_l = env_dict[2][0, wc_l, env_param[1]]
        unit_l = env_dict[2][0, wc_l]
        cost_l = unit_l * N(PL.length)
        landing_points.append((L, PL, unit_l, cost_l, atan_landing, N(PL.length), wc_l, which_alhaL, sector_l))
    return takeoff_points, landing_points


# Deprecated--------------------------------------------------
def xy_to_lonlat(x, y):
    proj_latlon = pyproj.Proj(proj='latlong', datum='WGS84')
    proj_xy = pyproj.Proj(proj="utm", zone=33, datum='WGS84')
    lonlat = pyproj.transform(proj_xy, proj_latlon, x, y)
    return lonlat[0], lonlat[1]


def lonlat_to_xy(lon, lat):
    proj_latlon = pyproj.Proj(proj='latlong', datum='WGS84')
    proj_xy = pyproj.Proj(proj="utm", zone=33, datum='WGS84')
    xy = pyproj.transform(proj_latlon, proj_xy, lon, lat)
    return xy[0], xy[1]


# -------------------------------------------------------------

def meteo2math(wind_direction):
    return (-wind_direction + 360 + 90 - 180) % 360


def math2meteo(wind_direction):
    return (-wind_direction + 360 + 90 - 180) % 360


def xy_to_gps(point_xy):
    scale_factor = 0.001
    home_gps = [43.11571856359583, 12.38504488191669]
    # Bearing in degrees: 0 – North, 90 – East
    tmp1 = geopy.distance.distance(kilometers=scale_factor * point_xy[0]).destination((home_gps[0], home_gps[1]),
                                                                                      bearing=90)
    tmp2 = geopy.distance.distance(kilometers=scale_factor * point_xy[1]).destination((home_gps[0], home_gps[1]),
                                                                                      bearing=0)
    point_gps = [tmp2.latitude, tmp1.longitude]
    return point_gps
