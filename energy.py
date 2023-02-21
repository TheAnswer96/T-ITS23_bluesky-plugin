import sympy as sp
import numpy as np
import mpmath

mpmath.mp.dps = 5

#wind_direction: vento relativo (wind - drone)
#distance: meters
def get_energy(distance, payload_weight, drone_speed, wind_speed, wind_direction):
    """
        Computes the energy required for a drone to travel a given distance while carrying a payload,
        against a given wind speed and direction, based on a set of physical and environmental parameters.

        Args:
        - distance (float): The distance that the drone needs to travel, in meters.
        - payload_weight (float): The weight of the payload that the drone needs to carry, in kilograms.
        - drone_speed (float): The airspeed of the drone, in meters per second.
        - wind_speed (float): The speed of the wind, in meters per second.
        - wind_direction (float): The direction of the wind, relative to the direction of the drone's motion,
          in degrees clockwise from due north.

        Returns:
        - energy (float): The energy required for the drone to travel the given distance with the given payload
          and against the given wind, in kilowatt-hours.
    """
    # Parameter values for the drone, battery, etc.
    m_package = payload_weight
    m_drone = 7
    m_battery = 10

    num_rotors = 8
    diameter = 0.432

    # s_battery = 540000
    # delta = 0.5
    # f = 1.2

    pressure = 100726  # 50 meters above sea level
    R = 287.058
    temperature = 15 + 273.15  # 15 degrees in Kelvin
    rho = pressure / (R * temperature) # Calculate air density based on environmental factors.

    g = 9.81

    eta = 0.7 # Power efficiency

    drag_coefficient_drone = 1.49
    drag_coefficient_battery = 1
    drag_coefficient_package = 2.2

    projected_area_drone = 0.224
    projected_area_battery = 0.015
    projected_area_package = 0.0929

    # Calculate the speed and direction of the drone's motion relative to the air.
    v_north = drone_speed - wind_speed * np.cos(np.deg2rad(wind_direction))
    v_east = - wind_speed * np.sin(np.deg2rad(wind_direction))
    v_air = np.sqrt(v_north ** 2 + v_east ** 2)

    # Calculate the drag force on the drone, battery, and payload.
    F_drag_drone = 0.5 * rho * (v_air ** 2) * drag_coefficient_drone * projected_area_drone
    F_drag_battery = 0.5 * rho * (v_air ** 2) * drag_coefficient_battery * projected_area_battery
    F_drag_package = 0.5 * rho * (v_air ** 2) * drag_coefficient_package * projected_area_package

    F_drag = F_drag_drone + F_drag_battery + F_drag_package

    alpha = np.arctan(F_drag / ((m_drone + m_battery + m_package) * g))

    # Calculate the thrust required for the drone to hover.
    T = (m_drone + m_battery + m_package) * g + F_drag

    # # Power min hover
    # P_min_hover = (T**1.5) / (np.sqrt(0.5 * np.pi * num_rotors * (diameter**2) * rho))

    # v_i = Symbol('v_i')
    # f_0 = v_i - (2*T / (np.pi * num_rotors * (diameter**2) * rho * sp.sqrt((drone_speed*sp.cos(alpha))**2 + (drone_speed*sp.sin(alpha) + v_i)**2)))
    # induced_speed = float(nsolve(f_0, v_i, 5))
    # print(induced_speed)

    tmp_a = 2 * T
    tmp_b = np.pi * num_rotors * (diameter ** 2) * rho
    tmp_c = (drone_speed * sp.cos(alpha)) ** 2
    tmp_d = drone_speed * sp.sin(alpha)
    tmp_e = tmp_a / tmp_b

    coeff = [1, (2 * tmp_d), (tmp_c + tmp_d ** 2), 0, -tmp_e ** 2]
    sol = np.roots(coeff)
    induced_speed = float(max(sol[np.isreal(sol)]).real) # Calculate the induced velocity of the air by the rotors.
    # print(induced_speed)

    # Power min to go forward
    P_min = T * (drone_speed * np.sin(alpha) + induced_speed)

    # expended power
    P = P_min / eta

    # energy efficiency of travel
    mu = P / drone_speed

    # Energy consumed
    E = mu * distance

    # # Range of a drone
    # R = (m_battery * s_battery * delta) / (e * f)
    # Return the energy required in kilowatt-hours.
    return E / 1000.


def compute_prefixes(max_payload, drone_speed, wind_speed, relative_winds):
    # set the distance to 1 meter
    distance = 1
    # create a list of payload weights, starting with 0 and ending with the max payload
    payload_weights = [0]
    payload_weights.append(max_payload)

    # create a dictionary to store the prefix values for each combination of payload weight and relative wind direction
    prefix = {}

    # loop over each payload weight and relative wind direction
    for p in payload_weights:
        for wd in relative_winds:
            # calculate the energy required for the given payload weight, drone speed, wind speed, and relative wind direction
            energy = get_energy(distance, p, drone_speed, wind_speed, wd)
            # store the energy value in the dictionary, using the payload weight and relative wind direction as keys
            prefix[(p, wd)] = energy

    # return the dictionary of prefix values
    return prefix


def compute_prefixes_extended(max_payload, drone_speed, wind_speed, relative_winds):
    # Set the distance to 1 meter
    distance = 1
    # Set the minimum and maximum payload weights
    payload_weights = [0]
    payload_weights.append(max_payload)
    # Set the possible wind directions
    relative_wind_directions = relative_winds

    # Initialize a dictionary to store the energy consumption prefixes
    prefix = {}

    # For each payload weight, wind speed, and relative wind direction combination, compute the energy consumption prefix
    for p in payload_weights:
        for s in wind_speed:
            for wd in relative_wind_directions:
                prefix[(p, wd, s)] = get_energy(distance, p, drone_speed, s, wd)

    # Return the dictionary of energy consumption prefixes
    return prefix
