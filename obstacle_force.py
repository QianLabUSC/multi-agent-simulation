import numpy as np

def get_vertical_vector1(vec):
    """ normal vector """
    return [vec[1], -vec[0]]


def get_vertical_vector2(vec):
    """ normal vector """
    return [-vec[1], vec[0]]


def obstacle_force(x, y, leg_posx, leg_posy, com_x, com_y):
    # force_mag_coefficient =
    force_mag = 10
    # obstacle force direction vector
    force_unit_vector = [leg_posx - x, leg_posy - y] / np.linalg.norm([leg_posx - x, leg_posy - y])
    # magnitude of obstacle force
    force_vector = force_mag * force_unit_vector
    # Leg contact position to CoM direction normal vector
    contact_com_unit_vector = [com_x - leg_posx, com_y - leg_posy] / np.linalg.norm([com_x - leg_posx, com_y - leg_posy])
    # Force on x-axis direction
    f = np.dot(np.array(force_vector), np.array(contact_com_unit_vector)) * contact_com_unit_vector
    f_x, f_y = f[0], f[1]

    # potential vertical vector for Leg contact position to CoM direction
    v1, v2 = get_vertical_vector1(contact_com_unit_vector), get_vertical_vector2(contact_com_unit_vector)

    # decide which one is the vertical vector for Leg contact position to CoM direction
    if np.dot(force_unit_vector, v1) >= 0:
        rotation_vector = v1
    else:
        rotation_vector = v2

    rotation_force = np.dot(np.array(force_vector), rotation_vector)

    # decide the force rotation direction
    if np.sign(rotation_vector[0]) == -np.sign(contact_com_unit_vector[1]):  # clockwise rotation
        theta_sign = 1
    else:
        theta_sign = -1

    return f_x, f_y, theta_sign, rotation_force