import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle
from matplotlib.patches import Rectangle
from simulation_environment import simulation_environment
from simulation_environment import two_agents_simulation
import imageio
import os

# x_list, y_list, r = [-60, -50, -40, -30, -20, -10, 0, 10, 20, 30, 40, 50, 60], [20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130], 2.5

x_list, y_list, r = np.arange(-50, 50, 5.0), np.arange(20, 150, 5.0), 2.5

legactive_s1, legactive_e1, legactive_s2, legactive_e2, legactive_s3, legactive_e3, legactive_s4, legactive_e4 = 0, 0, 0, 0, 0, 0, 0, 0

lf_theta_change_record, rf_theta_change_record, lh_theta_change_record, rh_theta_change_record = [0.0], [0.0], [0.0], [0.0]

filenames = []



# x_list,y_list, r = [-150, -120, -90, -50, -40, -30, -20, -10, 0, 10, 20, 30, 40, 50, 90, 120], [-30, -10, 10, 30, 50, 70, 80, 90, 100, 110, 120, 140, 160, 180, 200], 5

def plot_cicle(x, y, r, ax):
    cir1 = Circle(xy=(x, y), radius=r, alpha=0.5)
    ax.add_patch(cir1)
    ax.plot(x, y, 'm.')
    ax.axis('scaled')
    ax.axis('equal')  # changes limits of x or y axis so that equal increments of x and y have the same length
    ax.set_ylim([0, 100])
    ax.set_xlim([-50, 50])

def gait(Phi1, Phi2, Phi3, duty_circle, T):  # s1, e1:RF leg; s2, e2:LF leg; s3, e3:LH leg; s4, e4:RH leg
    global legactive_s1, legactive_e1, legactive_s2, legactive_e2, legactive_s3, legactive_e3, legactive_s4, legactive_e4
    legactive_s1 = 0
    legactive_e1 = (legactive_s1 + duty_circle*T) % T
    legactive_s2 = Phi1 * T
    legactive_e2 = (legactive_s2 + duty_circle*T) % T
    legactive_s3 = Phi2 * T
    legactive_e3 = (legactive_s3 + duty_circle*T) % T
    legactive_s4 = Phi3 * T
    legactive_e4 = (legactive_s4 + duty_circle*T) % T



def single_trajectory_plot(theta_list, time, T, body_width, body_length, x_LB_record, y_LB_record, x_RB_record, y_RB_record, x_LF_record, y_LF_record, x_RF_record, y_RF_record, ax):
    ax.patches.pop()
    time_stamp = [0, int(0.2*T), int(0.5*T), int(0.7*T)]

    for index in range(0, time, int(T)):   # LF, RB leg pair
        handle1 = ax.add_patch(Rectangle((x_LB_record[index], y_LB_record[index]), body_width, body_length, angle=-theta_list[index], fill=0))
        handle2 = ax.add_patch(Circle((x_LF_record[index], y_LF_record[index]), 1.0, fc='r'))
        handle3 = ax.add_patch(Circle((x_RF_record[index], y_RF_record[index]), 1.0, fc='r'))
        # print(x_LB_record[index], y_LB_record[index], theta_list[index]) # test point
        filename = f'{index}.png'
        filenames.append(filename)
        plt.savefig(filename)
        handle1.remove()
        handle2.remove()
        handle3.remove()

        handle1 = ax.add_patch(Rectangle((x_LB_record[index+time_stamp[1]], y_LB_record[index+time_stamp[1]]), body_width, body_length, angle=-theta_list[index+time_stamp[1]], fill=0))
        handle2 = ax.add_patch(Circle((x_LF_record[index+time_stamp[1]], y_LF_record[index+time_stamp[1]]), 1.0, fc='r'))
        handle3 = ax.add_patch(Circle((x_RF_record[index+time_stamp[1]], y_RF_record[index+time_stamp[1]]), 1.0, fc='r'))
        # print(x_LB_record[index], y_LB_record[index], theta_list[index]) # test point
        filename = f'{index+time_stamp[1]}.png'
        filenames.append(filename)
        plt.savefig(filename)
        handle1.remove()
        handle2.remove()
        handle3.remove()

        handle1 = ax.add_patch(Rectangle((x_LB_record[index+time_stamp[2]], y_LB_record[index+time_stamp[2]]), body_width, body_length, angle=-theta_list[index+time_stamp[2]], fill=0))
        handle2 = ax.add_patch(Circle((x_LB_record[index+time_stamp[2]], y_LB_record[index+time_stamp[2]]), 1.0, fc='g'))
        handle3 = ax.add_patch(Circle((x_RB_record[index+time_stamp[2]], y_RB_record[index+time_stamp[2]]), 1.0, fc='g'))
        # print(x_LB_record[index], y_LB_record[index], theta_list[index])  # test point
        filename = f'{index+time_stamp[2]}.png'
        filenames.append(filename)
        plt.savefig(filename)
        handle1.remove()
        handle2.remove()
        handle3.remove()

        handle1 = ax.add_patch(Rectangle((x_LB_record[index+time_stamp[3]], y_LB_record[index+time_stamp[3]]), body_width, body_length, angle=-theta_list[index+time_stamp[3]], fill=0))
        handle2 = ax.add_patch(Circle((x_LB_record[index+time_stamp[3]], y_LB_record[index+time_stamp[3]]), 1.0, fc='g'))
        handle3 = ax.add_patch(Circle((x_RB_record[index+time_stamp[3]], y_RB_record[index+time_stamp[3]]), 1.0, fc='g'))
        # print(x_LB_record[index], y_LB_record[index], theta_list[index])  # test point
        filename = f'{index+time_stamp[3]}.png'
        filenames.append(filename)
        plt.savefig(filename)
        handle1.remove()
        handle2.remove()
        handle3.remove()

    # plt.show()


def two_agent_trajectory_plot(theta_list, time, T, body_width, body_length, x_LB_record_1, y_LB_record_1, x_RB_record_1, y_RB_record_1, x_LF_record_1, y_LF_record_1, x_RF_record_1, y_RF_record_1,  x_LB_record_2, y_LB_record_2, x_RB_record_2, y_RB_record_2, x_LF_record_2, y_LF_record_2, x_RF_record_2, y_RF_record_2, ax, ax2, connector_length):
    ax.patches.pop()
    time_stamp = [0, int(0.2*T), int(0.5*T), int(0.7*T)]
    ax.set_xlabel("X (cm)")
    ax.set_ylabel("Y (cm)")
    ax2.set_xlabel("Time (ms)")
    ax2.set_ylabel("Robot orientation (°)")
    ax2.set_ylim([-360, 360])
    ax2.set_xlim([0, 15*T])

    for index in range(0, time, int(T)):   # LF, RB leg pair
        for time_dex in range(0, int(0.5*T), 100):
            for index_pos in range(0, 50, 10):
                handle6 = ax.add_patch(Circle((x_LB_record_2[index+time_dex+index_pos]/2 + x_RF_record_1[index+time_dex+index_pos]/2, y_LB_record_2[index+time_dex+index_pos]/2 + y_RF_record_1[index+time_dex+index_pos]/2), 0.5, fc='g'))
                handle7 = ax2.add_patch(Circle((index+time_dex+index_pos, theta_list[index+time_dex+index_pos]), 10.0, fc='k'))
        # handle1 = ax.add_patch(Rectangle((x_LB_record_2[index], y_LB_record_2[index]), body_width, body_length*2+connector_length, angle=-theta_list[index], fill=0))
        # handle2 = ax.add_patch(Circle((x_LF_record_1[index], y_LF_record_1[index]), 1.0, fc='r'))
        # handle3 = ax.add_patch(Circle((x_RF_record_1[index], y_RF_record_1[index]), 1.0, fc='r'))
        # handle4 = ax.add_patch(Circle((x_LF_record_2[index], y_LF_record_2[index]), 1.0, fc='r'))
        # handle5 = ax.add_patch(Circle((x_RF_record_2[index], y_RF_record_2[index]), 1.0, fc='r'))
        # # print(x_LB_record[index], y_LB_record[index], theta_list[index]) # test point
        # filename = f'{index}.png'
        # filenames.append(filename)
        # plt.savefig(filename)
        # handle1.remove()
        # handle2.remove()
        # handle3.remove()
        # handle4.remove()
        # handle5.remove()

            handle1 = ax.add_patch(Rectangle((x_LB_record_2[index+time_dex], y_LB_record_2[index+time_dex]), body_width, body_length*2+connector_length, angle=-theta_list[index+time_dex], fill=0))
            handle2 = ax.add_patch(Circle((x_LF_record_1[index+time_dex], y_LF_record_1[index+time_dex]), 1.0, fc='r'))
            handle3 = ax.add_patch(Circle((x_RF_record_1[index+time_dex], y_RF_record_1[index+time_dex]), 1.0, fc='r'))
            handle4 = ax.add_patch(Circle((x_LF_record_2[index+time_dex], y_LF_record_2[index+time_dex]), 1.0, fc='r'))
            handle5 = ax.add_patch(Circle((x_RF_record_2[index+time_dex], y_RF_record_2[index+time_dex]), 1.0, fc='r'))
            # print(x_LB_record[index], y_LB_record[index], theta_list[index]) # test point
            filename = f'{index+time_dex}.png'
            filenames.append(filename)
            plt.savefig(filename)
            handle1.remove()
            handle2.remove()
            handle3.remove()
            handle4.remove()
            handle5.remove()

        for time_dex in range(int(0.5 * T), int(1 * T), 100):
            for index_pos in range(0, 50, 10):
                handle6 = ax.add_patch(Circle((x_LB_record_2[index + time_dex+index_pos] / 2 + x_RF_record_1[index + time_dex+index_pos] / 2, y_LB_record_2[index + time_dex+index_pos] / 2 + y_RF_record_1[index + time_dex+index_pos] / 2), 0.5, fc='g'))
                handle7 = ax2.add_patch(Circle((index + time_dex + index_pos, theta_list[index + time_dex + index_pos]), 10.0, fc='k'))
        # handle1 = ax.add_patch(Rectangle((x_LB_record_2[index+time_stamp[2]], y_LB_record_2[index+time_stamp[2]]), body_width, body_length*2+connector_length, angle=-theta_list[index+time_stamp[2]], fill=0))
        # handle2 = ax.add_patch(Circle((x_LB_record_1[index+time_stamp[2]], y_LB_record_1[index+time_stamp[2]]), 1.0, fc='g'))
        # handle3 = ax.add_patch(Circle((x_RB_record_1[index+time_stamp[2]], y_RB_record_1[index+time_stamp[2]]), 1.0, fc='g'))
        # handle4 = ax.add_patch(Circle((x_LB_record_2[index+time_stamp[2]], y_LB_record_2[index+time_stamp[2]]), 1.0, fc='g'))
        # handle5 = ax.add_patch(Circle((x_RB_record_2[index+time_stamp[2]], y_RB_record_2[index+time_stamp[2]]), 1.0, fc='g'))
        # # print(x_LB_record[index], y_LB_record[index], theta_list[index])  # test point
        # filename = f'{index+time_stamp[2]}.png'
        # filenames.append(filename)
        # plt.savefig(filename)
        # handle1.remove()
        # handle2.remove()
        # handle3.remove()
        # handle4.remove()
        # handle5.remove()

            handle1 = ax.add_patch(Rectangle((x_LB_record_2[index+time_dex], y_LB_record_2[index+time_dex]), body_width, body_length*2+connector_length, angle=-theta_list[index+time_dex], fill=0))
            handle2 = ax.add_patch(Circle((x_LB_record_1[index+time_dex], y_LB_record_1[index+time_dex]), 1.0, fc='g'))
            handle3 = ax.add_patch(Circle((x_RB_record_1[index+time_dex], y_RB_record_1[index+time_dex]), 1.0, fc='g'))
            handle4 = ax.add_patch(Circle((x_LB_record_2[index+time_dex], y_LB_record_2[index+time_dex]), 1.0, fc='g'))
            handle5 = ax.add_patch(Circle((x_RB_record_2[index+time_dex], y_RB_record_2[index+time_dex]), 1.0, fc='g'))
            # print(x_LB_record[index], y_LB_record[index], theta_list[index])  # test point
            filename = f'{index+time_dex}.png'
            filenames.append(filename)
            plt.savefig(filename)
            handle1.remove()
            handle2.remove()
            handle3.remove()
            handle4.remove()
            handle5.remove()

    # plt.show()


############### Trajectory simulation part ###############
def single_robot(half_body_length, half_body_width, posx, posy, T, body_weight):
    global lf_theta_change_record, rf_theta_change_record, lh_theta_change_record, rh_theta_change_record

    for i in x_list:
        for j in y_list:
            plot_cicle(i, j, r, ax)

    for ori in [-6 * np.pi / 36]:
        # for ori in [-np.pi/4, -np.pi/6, -np.pi/8, 0, np.pi/8, np.pi/6, np.pi/4]:
        # for ori in np.arange(-np.pi/3, np.pi/3, np.pi/3):
        x_record, y_record, theta_record, x_LB_record, y_LB_record, x_RB_record, y_RB_record, x_LF_record, y_LF_record, x_RF_record, y_RF_record = [], [], [], [], [], [], [], [], [], [], []
        for t in range(simulation_time):
            x_record.append(posx)
            y_record.append(posy)
            theta_record.append(ori / np.pi * 180)
            posy += step_length / T * np.cos(ori)
            posx += step_length / T * np.sin(ori)
            update_x, update_y, update_theta, xlb, ylb, xrb, yrb, xlf, ylf, xrf, yrf, lf_theta_change_record, rf_theta_change_record, lh_theta_change_record, rh_theta_change_record = simulation_environment(
                x_list, y_list, r, half_body_length, half_body_width, ori, posx, posy, t, T, body_weight,
                lf_theta_change_record, rf_theta_change_record, lh_theta_change_record, rh_theta_change_record)
            ori += update_theta
            posx += update_x
            posy += update_y
            x_LB_record.append(xlb)
            y_LB_record.append(ylb)
            x_RB_record.append(xrb)
            y_RB_record.append(yrb)
            x_LF_record.append(xlf)
            y_LF_record.append(ylf)
            x_RF_record.append(xrf)
            y_RF_record.append(yrf)

        # plt.plot(range(simulation_time), theta_record)
        # print('s')
        single_trajectory_plot(theta_record, simulation_time - T, T, half_body_width * 2, half_body_length * 2, x_LB_record,
                        y_LB_record, x_RB_record, y_RB_record, x_LF_record, y_LF_record, x_RF_record, y_RF_record, ax)
        # plot_contact_position(lf_contact_x, lf_contact_y, rf_contact_x, rf_contact_y, rh_contact_x, rh_contact_y, lh_contact_x, lh_contact_y, ax)
        # test_list = theta_record[4500:]
        # test_list = theta_record[4500:]


    # ax2.plot(lf_theta_change_record, "rs")
    # ax2.plot(rf_theta_change_record, "gs")
    # ax2.plot(lh_theta_change_record, "ks")
    # ax2.plot(rh_theta_change_record, "ms")

    # plt.show()


############### Trajectory simulation part ###############
def two_robot(half_body_length, half_body_width, posx, posy, T, body_weight, connector_length, ori, ax, ax2):
    global lf_theta_change_record, rf_theta_change_record, lh_theta_change_record, rh_theta_change_record

    for i in x_list:
        for j in y_list:
            plot_cicle(i, j, r, ax)

    # for ori in [-9 * np.pi / 36, 0 * np.pi / 36, 9 * np.pi / 36]:
    # for ori in [-3 * np.pi / 8]:
    # for ori in [-3*np.pi/8, -np.pi/4, -np.pi/6, -np.pi/8, 0, np.pi/8, np.pi/6, np.pi/4, 3*np.pi/8]:
    # for ori in np.arange(-np.pi/3, np.pi/3, np.pi/3):

    test_ori=ori
    posx, posy = 0, 50
    x_record, y_record, theta_record = [], [], []
    x_LB_record_1, y_LB_record_1, x_RB_record_1, y_RB_record_1, x_LF_record_1, y_LF_record_1, x_RF_record_1, y_RF_record_1 = [], [], [], [], [], [], [], []
    x_LB_record_2, y_LB_record_2, x_RB_record_2, y_RB_record_2, x_LF_record_2, y_LF_record_2, x_RF_record_2, y_RF_record_2 = [], [], [], [], [], [], [], []
    for t in range(simulation_time):
        x_record.append(posx)
        y_record.append(posy)
        theta_record.append(ori / np.pi * 180)
        posy += step_length / T * np.cos(ori)
        posx += step_length / T * np.sin(ori)
        update_x, update_y, update_theta, xlb_2, ylb_2, xrb_2, yrb_2, xlf_2, ylf_2, xrf_2, yrf_2, xlb_1, ylb_1, xrb_1, yrb_1, xlf_1, ylf_1, xrf_1, yrf_1, lf_theta_change_record, rf_theta_change_record, lh_theta_change_record, rh_theta_change_record = two_agents_simulation(
            x_list, y_list, r, half_body_length, half_body_width, ori, posx, posy, connector_length, t, T, body_weight,
            lf_theta_change_record, rf_theta_change_record, lh_theta_change_record, rh_theta_change_record)
        ori += update_theta
        posx += update_x
        posy += update_y
        # ori += update_theta
        # posx += update_x
        # posy += update_y
        x_LB_record_1.append(xlb_1)
        y_LB_record_1.append(ylb_1)
        x_RB_record_1.append(xrb_1)
        y_RB_record_1.append(yrb_1)
        x_LF_record_1.append(xlf_1)
        y_LF_record_1.append(ylf_1)
        x_RF_record_1.append(xrf_1)
        y_RF_record_1.append(yrf_1)

        x_LB_record_2.append(xlb_2)
        y_LB_record_2.append(ylb_2)
        x_RB_record_2.append(xrb_2)
        y_RB_record_2.append(yrb_2)
        x_LF_record_2.append(xlf_2)
        y_LF_record_2.append(ylf_2)
        x_RF_record_2.append(xrf_2)
        y_RF_record_2.append(yrf_2)

    # plt.plot(range(simulation_time), theta_record)
    # print('s')

    print(posx, posy-50)
    if abs(posx) >= 18 or abs(posy - 50) >= 18:
        print(half_body_length, half_body_width, connector_length, test_ori, ori)
    # two_agent_trajectory_plot(theta_record, simulation_time - T, T, half_body_width * 2, half_body_length * 2, x_LB_record_1,
    #                 y_LB_record_1, x_RB_record_1, y_RB_record_1, x_LF_record_1, y_LF_record_1, x_RF_record_1, y_RF_record_1, x_LB_record_2,
    #                 y_LB_record_2, x_RB_record_2, y_RB_record_2, x_LF_record_2, y_LF_record_2, x_RF_record_2, y_RF_record_2, ax, ax2, connector_length)


    # plot_contact_position(lf_contact_x, lf_contact_y, rf_contact_x, rf_contact_y, rh_contact_x, rh_contact_y, lh_contact_x, lh_contact_y, ax)
    # test_list = theta_record[4500:]
    # test_list = theta_record[4500:]

    # ax2.plot(lf_theta_change_record, "rs")
    # ax2.plot(rf_theta_change_record, "gs")
    # ax2.plot(lh_theta_change_record, "ks")
    # ax2.plot(rh_theta_change_record, "ms")

    # plt.show()

###############  Main code  ############
ini_posx, ini_posy = 0, 50
posx, posy, half_body_length, half_body_width = ini_posx, ini_posy, 6.3/2, 6.3/2
step_length, T, body_weight = 0, 3000, 1
connector_length = 4.5  # bug exist for value larger than 6
simulation_time = int(15*T)

fig = plt.figure() # figure for trajectory
fig.set_figheight(15)
fig.set_figwidth(30)
ax = fig.add_subplot(121)
ax2 = fig.add_subplot(122)
# ax.set_aspect('equal', adjustable='box')

# fig2 = plt.figure() # figure for orientation vs. time
# ax2 = fig2.add_subplot(111)

# fig3= plt.figure() # figure for project obstacle
# ax3= plt.axes(projection='3d')

# single_robot(half_body_length, half_body_width, posx, posy, T, body_weight)

# for half_body_length in range(2, 8):
for connector_length in np.arange(2, 11, 0.5):
#         for half_body_width in range(2, 8):
# for ori in [-1*np.pi/16]:
    for ori in [-3*np.pi/8, -np.pi/4, -np.pi/6, -np.pi/8, 0, np.pi/8, np.pi/6, np.pi/4, 3*np.pi/8]:
        two_robot(half_body_length, half_body_width, posx, posy, T, body_weight, connector_length, ori, ax, ax2)
        # print("s")

# build gif
# with imageio.get_writer(f'body length{half_body_length * 2} body width{half_body_width * 2} obstacle x spacing {x_list[-1] - x_list[-2]} y spacing {y_list[-1] - y_list[-2]} connection length {connector_length} ori {ori/np.pi*180}.gif', mode='I') as writer:
#     for filename in filenames:
#         image = imageio.imread(filename)
#         writer.append_data(image)
#
# # Remove files
# for filename in set(filenames):
#     os.remove(filename)

################### project obstacle part ###################
# HalfBodyDiag, beta_rad = np.sqrt(half_body_length**2 + half_body_width**2), np.arctan(half_body_width / half_body_length)
# ori_proj, posx_proj, posy_proj = [], [], []
# for ori in np.arange(-np.pi/2, np.pi/2, np.pi/180):
#     for posx in np.arange(0, 7.5, 0.1):
#         for posy in np.arange(0, 7.5, 0.1):
#             y_LF_absolute = (posy+50) + HalfBodyDiag * np.cos(beta_rad - ori)
#             x_LF_absolute = posx - HalfBodyDiag * np.sin(beta_rad - ori)
#             if(contact_detect([x_LF_absolute], [y_LF_absolute], x_list, y_list, r, 1)[0] != []):
#                 ori_proj.append(ori)
#                 posx_proj.append(posx)
#                 posy_proj.append(posy)
#
# ax3.scatter3D(posx_proj, posy_proj, ori_proj, c=ori_proj, cmap='Greens')
# ax3.set_xlabel("/bar X (cm)")
# ax3.set_ylabel("'bar Y (cm)")
# ax3.set_zlabel("/theta (rads)")
#
# # for angle in range(0, 360):
# #    ax3.view_init(30, angle)
# #    plt.draw()
# #    plt.pause(.001)
#
# plt.show()



print('stop')


