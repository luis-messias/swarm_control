#!/usr/bin/env python3

from numpy.core.defchararray import isdigit
from numpy.core.numeric import Inf, NaN
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
import rospy
import math
import numpy as np
import tf
from campos import campo_atracao, campo_repulsao, campo_repulsao2

class robot():   
    def __init__(this, swarm_name, id):
        this.x = 0
        this.y = 0
        this.yaw = 0
        this.cmd = Twist()
        this.id = id
        this.subscriber = rospy.Subscriber("/" + swarm_name + "/robot_" + str(id) + "/odom", Odometry, this.cb_odom)
        this.publisher = rospy.Publisher("/" + swarm_name + "/robot_" + str(id) + "/cmd_vel",
                                         Twist, queue_size = 10)

    def cb_odom(this, msg):
        this.x = msg.pose.pose.position.x
        this.y = msg.pose.pose.position.y
        euler = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
        this.yaw = euler[2]

    def set_vel(this,vx,wz):
        this.cmd.linear.x = vx
        this.cmd.angular.z = wz
    
    def pub_vel(this):
        this.publisher.publish(this.cmd)

x_goal = 0
y_goal = 0
flag_goal = False

def cb_goal(msg):
    global x_goal, y_goal, flag_goal

    x_goal = msg.position.x
    y_goal = msg.position.y
    flag_goal = True

rospy.init_node("controller_name")
controller_name = rospy.get_name()[1:]
swarm_name = rospy.get_param("/" + controller_name + '/swarm_name',"swarm")
number = int(rospy.get_param("/" + controller_name + '/num_robots',1))

robots = [] 
x_array = np.zeros(number)
y_array = np.zeros(number)
yaw_array = np.zeros(number)

subscriber = rospy.Subscriber("/" + swarm_name + "/goal", Pose, cb_goal)

for i in range(number):
    robots.append(robot(swarm_name, i))

while not rospy.is_shutdown():
    for i in range(number):
        x_array[i] = robots[i].x
        y_array[i] = robots[i].y
        yaw_array[i] = robots[i].yaw

    for i in range(number):
        u1, v1 = campo_repulsao(x_array, y_array, x_array[i], y_array[i])
        if flag_goal:
            u2, v2 = campo_atracao(x_array[i], y_array[i], x_goal, y_goal)
        else:
            u2, v2 = 0,0

        u = u1 + u2
        v = v1 + v2
        
        erro_angle = np.arctan2(u, v) - yaw_array[i]
        while(erro_angle > np.pi):
            erro_angle -= 2*np.pi
        while(erro_angle < -np.pi):
            erro_angle += 2*np.pi
        k_yaw = 2

        robots[i].set_vel((u**2 + v**2)**0.5,erro_angle*k_yaw) 

    for i in range(number):
        robots[i].pub_vel() 

    rospy.sleep(0.01)