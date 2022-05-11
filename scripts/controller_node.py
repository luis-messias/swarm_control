#!/usr/bin/env python3
import rospy
from swarmController import swarControllerClass
import time
import threading

import numpy as np
import matplotlib.pyplot as plt
from campos import campo_atracao, campo_repulsao, campo_repulsao2

def plot_main(controller: swarControllerClass):
    while not rospy.is_shutdown():

        x,y = np.meshgrid(np.linspace(-2,2,50),np.linspace(-2,2,50))
        u,v = np.meshgrid(np.linspace(-2,2,50),np.linspace(-2,2,50))

        robot_pos_x = controller.robotPosition_x
        robot_pos_y = controller.robotPosition_y

        for i in range(x.shape[0]):
            for j in range(x.shape[1]):
                u[i][j], v[i][j] = campo_repulsao(x[i][j], y[i][j], robot_pos_x, robot_pos_y)

        plt.clf()
        # plt.contourf(x,y,(u**2 + v**2)**0.5)
        plt.quiver(x,y,u,v)
        plt.scatter(robot_pos_x, robot_pos_y, c="green")

        plt.pause(1)


rospy.init_node("controller_name")
controller = swarControllerClass()

plot_flag = bool(rospy.get_param("/" + rospy.get_name()[1:] + '/plot_flag',0))
if plot_flag:
    plot_theread = threading.Thread(target=plot_main, args=(controller,))
    plot_theread.start()

while not rospy.is_shutdown():
    # start = time.time()

    controller.calculatePotentialField()
    # end = time.time()
    # print(format(end-start))

    # start = time.time()
    controller.publishAllVelocity()
    # end = time.time()
    # print(format(end-start))

    rospy.sleep(0.01)



