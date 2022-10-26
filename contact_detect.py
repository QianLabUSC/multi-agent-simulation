import numpy as np

lf_contact_x, lf_contact_y, rf_contact_x, rf_contact_y, rh_contact_x, rh_contact_y, lh_contact_x, lh_contact_y = [], [], [], [], [], [], [], []


def contact_detect(leg_list_x, leg_list_y, x_list, y_list, r, leg_pair_sign):
    # output contact obstacle list coordinates
    global lf_contact_x, lf_contact_y, rf_contact_x, rf_contact_y, rh_contact_x, rh_contact_y, lh_contact_x, lh_contact_y
    contact_list, leg_contact_sign = [], []
    for index in range(len(leg_list_x)):
        leg_posx, leg_posy = leg_list_x[index], leg_list_y[index]
        # caculation_opt_1, caculation_opt_2 =
        for i in x_list:
            for j in y_list:
                if np.absolute(leg_posx - i) <= r and np.absolute(leg_posy - j) <= r:
                    if np.sqrt((leg_posx - i)**2 + (leg_posy - j)**2) <= r:
                        contact_list += [i, j]
                        if leg_pair_sign == 1 and index == 0:
                            lf_contact_x.append(leg_posx), lf_contact_y.append(leg_posy)
                        if leg_pair_sign == 1 and index == 1:
                            rf_contact_x.append(leg_posx), rf_contact_y.append(leg_posy)
                        if leg_pair_sign == 2 and index == 0:
                            lh_contact_x.append(leg_posx), lh_contact_y.append(leg_posy)
                        if leg_pair_sign == 2 and index == 1:
                            rh_contact_x.append(leg_posx), rh_contact_y.append(leg_posy)
                        leg_contact_sign.append(index)
    return contact_list, leg_contact_sign