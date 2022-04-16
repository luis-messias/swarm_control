#!/usr/bin/env python3
import rospy

from swarmController import swarControllerClass


rospy.init_node("controller_name")
controller = swarControllerClass()

while not rospy.is_shutdown():
    controller.calculatePotentialField()
    controller.publishAllVelocity()

    rospy.sleep(0.01)