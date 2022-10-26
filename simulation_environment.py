import numpy as np
from obstacle_force import obstacle_force
from contact_detect import contact_detect


def robot_dynamic_state(f_x, f_y, total_rotation_force, ori, posx, posy, body_weight):
    # alpha, beta = 0.01, 0.00005
    alpha, beta = 0.005, 0.001

    update_x, update_y = alpha * f_x / body_weight, alpha * f_y / body_weight
    update_theta = beta * total_rotation_force / body_weight  # Total rotation torque, no need for sign operation
    ori += update_theta
    posx += update_x
    posy += update_y

    return update_x, update_y, update_theta


def simulation_environment(x_list, y_list, r, half_body_length, half_body_width, ori, posx, posy, t, T, body_weight, lf_theta_change_record, rf_theta_change_record, lh_theta_change_record, rh_theta_change_record):
    HalfBodyDiag, beta_rad = np.sqrt(half_body_length**2 + half_body_width**2), np.arctan(half_body_width / half_body_length)
    update_x, update_y, update_theta, theta_record_sign = 0, 0, 0, 0
    total_fx, total_fy, total_rf, contact_detect_sign = 0, 0, 0, 0

    y_LF_absolute = posy + HalfBodyDiag * np.cos(beta_rad - ori)
    x_LF_absolute = posx - HalfBodyDiag * np.sin(beta_rad - ori)
    y_RB_absolute = posy - HalfBodyDiag * np.cos(beta_rad - ori)
    x_RB_absolute = posx + HalfBodyDiag * np.sin(beta_rad - ori)
    y_RF_absolute = posy + HalfBodyDiag * np.cos(beta_rad + ori)
    x_RF_absolute = posx + HalfBodyDiag * np.sin(beta_rad + ori)
    y_LB_absolute = posy - HalfBodyDiag * np.cos(beta_rad + ori)
    x_LB_absolute = posx - HalfBodyDiag * np.sin(beta_rad + ori)

    lf_theta_change_record.append(0)
    rf_theta_change_record.append(0)
    lh_theta_change_record.append(0)
    rh_theta_change_record.append(0)

    if t % T <= 0.2*T:  # Leg position for LF, RB

        contact_list, leg_marker = contact_detect([x_LF_absolute, x_RF_absolute], [y_LF_absolute, y_RF_absolute], x_list, y_list, r, 1)

        # if (t % T == 0) and (len(contact_list) != 0):
        #     contact_detect_sign = 1
        if len(contact_list) != 0:  # version_2 only consider virtual bipedal gait leg contact
            for leg in leg_marker:
                if leg == 0:
                    fx, fy, theta_sign, rf = obstacle_force(contact_list[0], contact_list[1], x_LF_absolute, y_LF_absolute, posx, posy)
                    total_fx += fx
                    total_fy += fy
                    total_rf += theta_sign * rf
                    lf_theta_change_record[-1] = lf_theta_change_record[-2] + theta_sign * rf


                if leg == 1:
                    fx, fy, theta_sign, rf = obstacle_force(contact_list[0], contact_list[1], x_RF_absolute, y_RF_absolute, posx, posy)
                    total_fx += fx
                    total_fy += fy
                    total_rf += theta_sign * rf
                    rf_theta_change_record[-1] = rf_theta_change_record[-2] + theta_sign * rf

            update_x, update_y, update_theta = robot_dynamic_state(total_fx, total_fy, total_rf, ori, posx, posy, body_weight)

    if (0.5*T <= t % T) and (t % T < 0.7*T):  # Leg position for RF, LB

        contact_list, leg_marker = contact_detect([x_LB_absolute, x_RB_absolute], [y_LB_absolute, y_RB_absolute], x_list, y_list, r, 2)
        # if (t % T == int(T/2)) and (len(contact_list) != 0):
        #     contact_detect_sign = 1
        if len(contact_list) != 0:  # version_2 only consider virtual bipedal gait leg contact
            for leg in leg_marker:
                if leg == 0:
                    fx, fy, theta_sign, rf = obstacle_force(contact_list[0], contact_list[1], x_LB_absolute, y_LB_absolute, posx, posy)
                    total_fx += fx
                    total_fy += fy
                    total_rf += theta_sign * rf
                    lh_theta_change_record[-1] = lh_theta_change_record[-2] + theta_sign * rf

                if leg == 1:
                    fx, fy, theta_sign, rf = obstacle_force(contact_list[0], contact_list[1], x_RB_absolute, y_RB_absolute, posx, posy)
                    total_fx += fx
                    total_fy += fy
                    total_rf += theta_sign * rf
                    rh_theta_change_record[-1] = rh_theta_change_record[-2] + theta_sign * rf

            update_x, update_y, update_theta = robot_dynamic_state(total_fx, total_fy, total_rf, ori, posx, posy, body_weight)

    return update_x, update_y, update_theta, x_LB_absolute, y_LB_absolute, x_RB_absolute, y_RB_absolute, x_LF_absolute, y_LF_absolute, x_RF_absolute, y_RF_absolute, lf_theta_change_record, rf_theta_change_record, lh_theta_change_record, rh_theta_change_record


def plot_contact_position(lf_contact_x, lf_contact_y, rf_contact_x, rf_contact_y, rh_contact_x, rh_contact_y, lh_contact_x, lh_contact_y, ax):
    ax.plot(lf_contact_x, lf_contact_y, "rs", markersize=10)
    ax.plot(rf_contact_x, rf_contact_y, "gs", markersize=10)
    ax.plot(rh_contact_x, rh_contact_y, "ms", markersize=10)
    ax.plot(lh_contact_x, lh_contact_y, "ks", markersize=10)


def two_agents_simulation(x_list, y_list, r, half_body_length, half_body_width, ori, posx, posy, connector_length, t, T, body_weight, lf_theta_change_record, rf_theta_change_record, lh_theta_change_record, rh_theta_change_record):
    longerBodyDiag, shorterBodyDiag, longerbeta_rad, shorterbeta_rad = np.sqrt((half_body_length*2 + connector_length/2)**2 + half_body_width**2), np.sqrt((connector_length/2)**2 + half_body_width**2), np.arctan(half_body_width / (half_body_length*2 + connector_length/2)), np.arctan(half_body_width / (connector_length/2))
    update_x, update_y, update_theta, theta_record_sign = 0, 0, 0, 0
    total_fx, total_fy, total_rf, contact_detect_sign = 0, 0, 0, 0

    y_LF_absolute_1 = posy + longerBodyDiag * np.cos(longerbeta_rad - ori)
    x_LF_absolute_1 = posx - longerBodyDiag * np.sin(longerbeta_rad - ori)
    y_RB_absolute_1 = posy + shorterBodyDiag * np.cos(shorterbeta_rad + ori)
    x_RB_absolute_1 = posx + shorterBodyDiag * np.sin(shorterbeta_rad + ori)
    y_RF_absolute_1 = posy + longerBodyDiag * np.cos(longerbeta_rad + ori)
    x_RF_absolute_1 = posx + longerBodyDiag * np.sin(longerbeta_rad + ori)
    y_LB_absolute_1 = posy + shorterBodyDiag * np.cos(shorterbeta_rad - ori)
    x_LB_absolute_1 = posx - shorterBodyDiag * np.sin(shorterbeta_rad - ori)

    y_LF_absolute_2 = posy - shorterBodyDiag * np.cos(shorterbeta_rad + ori)
    x_LF_absolute_2 = posx - shorterBodyDiag * np.sin(shorterbeta_rad + ori)
    y_RB_absolute_2 = posy - longerBodyDiag * np.cos(longerbeta_rad - ori)
    x_RB_absolute_2 = posx + longerBodyDiag * np.sin(longerbeta_rad - ori)
    y_RF_absolute_2 = posy - shorterBodyDiag * np.cos(shorterbeta_rad - ori)
    x_RF_absolute_2 = posx + shorterBodyDiag * np.sin(shorterbeta_rad - ori)
    y_LB_absolute_2 = posy - longerBodyDiag * np.cos(longerbeta_rad + ori)
    x_LB_absolute_2 = posx - longerBodyDiag * np.sin(longerbeta_rad + ori)

    # print("s")



    # lf_theta_change_record.append(0)
    # rf_theta_change_record.append(0)
    # lh_theta_change_record.append(0)
    # rh_theta_change_record.append(0)

    if t % T <= 0.2*T:  # Leg position for LF, RB

        contact_list, leg_marker = contact_detect([x_LF_absolute_1, x_RF_absolute_1, x_LF_absolute_2, x_RF_absolute_2], [y_LF_absolute_1, y_RF_absolute_1, y_LF_absolute_2, y_RF_absolute_2], x_list, y_list, r, 1)

        # if (t % T == 0) and (len(contact_list) != 0):
        #     contact_detect_sign = 1
        if len(contact_list) != 0:  # version_2 only consider virtual bipedal gait leg contact
            index = -1
            for leg in leg_marker:
                index += 1
                if leg == 0:
                    fx, fy, theta_sign, rf = obstacle_force(contact_list[index], contact_list[index*2+1], x_LF_absolute_1, y_LF_absolute_1, posx, posy)
                    total_fx += fx
                    total_fy += fy
                    total_rf += theta_sign * rf
                    # lf_theta_change_record[-1] = lf_theta_change_record[-2] + theta_sign * rf


                if leg == 1:
                    fx, fy, theta_sign, rf = obstacle_force(contact_list[index], contact_list[index*2+1], x_RF_absolute_1, y_RF_absolute_1, posx, posy)
                    total_fx += fx
                    total_fy += fy
                    total_rf += theta_sign * rf
                    # rf_theta_change_record[-1] = rf_theta_change_record[-2] + theta_sign * rf

                if leg == 2:
                    fx, fy, theta_sign, rf = obstacle_force(contact_list[index], contact_list[index*2+1], x_LF_absolute_2, y_LF_absolute_2, posx, posy)
                    total_fx += fx
                    total_fy += fy
                    total_rf += theta_sign * rf

                if leg == 3:
                    fx, fy, theta_sign, rf = obstacle_force(contact_list[index], contact_list[index*2+1], x_RF_absolute_2, y_RF_absolute_2, posx, posy)
                    total_fx += fx
                    total_fy += fy
                    total_rf += theta_sign * rf

            update_x, update_y, update_theta = robot_dynamic_state(total_fx, total_fy, total_rf, ori, posx, posy, body_weight)

    if (0.5*T <= t % T) and (t % T < 0.7*T):  # Leg position for RF, LB

        contact_list, leg_marker = contact_detect([x_LB_absolute_1, x_RB_absolute_1, x_LB_absolute_2, x_RB_absolute_2], [y_LB_absolute_1, y_RB_absolute_1, y_LB_absolute_2, y_RB_absolute_2], x_list, y_list, r, 2)
        # if (t % T == int(T/2)) and (len(contact_list) != 0):
        #     contact_detect_sign = 1
        if len(contact_list) != 0:  # version_2 only consider virtual bipedal gait leg contact
            index = -1
            for leg in leg_marker:
                index += 1
                if leg == 0:
                    fx, fy, theta_sign, rf = obstacle_force(contact_list[index], contact_list[index*2+1], x_LB_absolute_1, y_LB_absolute_1, posx, posy)
                    total_fx += fx
                    total_fy += fy
                    total_rf += theta_sign * rf
                    # lh_theta_change_record[-1] = lh_theta_change_record[-2] + theta_sign * rf

                if leg == 1:
                    fx, fy, theta_sign, rf = obstacle_force(contact_list[index], contact_list[index*2+1], x_RB_absolute_1, y_RB_absolute_1, posx, posy)
                    total_fx += fx
                    total_fy += fy
                    total_rf += theta_sign * rf
                    # rh_theta_change_record[-1] = rh_theta_change_record[-2] + theta_sign * rf

                if leg == 2:
                    fx, fy, theta_sign, rf = obstacle_force(contact_list[index], contact_list[index*2+1], x_LB_absolute_2, y_LB_absolute_2, posx, posy)
                    total_fx += fx
                    total_fy += fy
                    total_rf += theta_sign * rf

                if leg == 3:
                    fx, fy, theta_sign, rf = obstacle_force(contact_list[index], contact_list[index*2+1], x_RB_absolute_2, y_RB_absolute_2, posx, posy)
                    total_fx += fx
                    total_fy += fy
                    total_rf += theta_sign * rf

            update_x, update_y, update_theta = robot_dynamic_state(total_fx, total_fy, total_rf, ori, posx, posy, body_weight)

    # if t==1500:
    #     print("s")

    return update_x, update_y, update_theta, x_LB_absolute_2, y_LB_absolute_2, x_RB_absolute_2, y_RB_absolute_2, x_LF_absolute_2, y_LF_absolute_2, x_RF_absolute_2, y_RF_absolute_2, x_LB_absolute_1, y_LB_absolute_1, x_RB_absolute_1, y_RB_absolute_1, x_LF_absolute_1, y_LF_absolute_1, x_RF_absolute_1, y_RF_absolute_1, lf_theta_change_record, rf_theta_change_record, lh_theta_change_record, rh_theta_change_record