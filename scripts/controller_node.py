#!/usr/bin/env python3
import rospy
from swarmController import swarControllerClass
import time
import threading

def plot_main(controller: swarControllerClass):
    while not rospy.is_shutdown():
        #print(controller.nodeName)

        rospy.sleep(0.01)


rospy.init_node("controller_name")
controller = swarControllerClass()

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



