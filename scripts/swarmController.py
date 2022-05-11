import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
import tf

import numpy as np

from campos import campo_atracao, campo_repulsao, campo_repulsao2

class swarControllerClass():
    def __init__(self) -> None:
        self.nodeName = rospy.get_name()[1:]
        self.swarmName = rospy.get_param("/" + self.nodeName + '/swarm_name',"swarm")
        self.numberRobots = int(rospy.get_param("/" + self.nodeName + '/num_robots',1))

        self.robotPosition_x = np.zeros(self.numberRobots)
        self.robotPosition_y = np.zeros(self.numberRobots)
        self.robotPosition_yaw = np.zeros(self.numberRobots)

        self.robotVelocity_x = np.zeros(self.numberRobots)
        self.robotVelocity_wz = np.zeros(self.numberRobots)


        self.subscriberOdometry = []
        self.subscriberIndividualGoal = []
        self.publisher = []

        self.robotGoal_x = np.zeros(self.numberRobots)
        self.robotGoal_y = np.zeros(self.numberRobots)     
        self.robotGoal_flag = np.zeros(self.numberRobots) 

        for i in range(self.numberRobots):
            self.subscriberOdometry.append(rospy.Subscriber("/" + self.swarmName + "/robot_" + str(i) + "/odom", Odometry, self.callbackOdometry, i))
            self.publisher.append(rospy.Publisher("/" + self.swarmName + "/robot_" + str(i) + "/cmd_vel", Twist, queue_size = 2))
            self.subscriberIndividualGoal.append(rospy.Subscriber("/controller_" + self.swarmName + "/goal_" + str(i), Pose, self.callbackIndividualGoal, i))

        self.masterGoalSubscriber = rospy.Subscriber("/controller_" + self.swarmName + "/goal_master", Pose, self.callbackMasterGoal)


            


    def callbackOdometry(self, msg, i):
        self.robotPosition_x[i] = msg.pose.pose.position.x
        self.robotPosition_y[i] = msg.pose.pose.position.y
        euler = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
        self.robotPosition_yaw[i] = euler[2]

    def publishAllVelocity(self):
        cmd = Twist()
        for i in range(self.numberRobots):
            cmd.linear.x = self.robotVelocity_x[i]
            cmd.angular.z = self.robotVelocity_wz[i]
 
            self.publisher[i].publish(cmd)

    def setVelocity(self, vx, wz, i):
        self.robotVelocity_x[i] = vx
        self.robotVelocity_wz[i] = wz

    def callbackMasterGoal(self, msg):
        self.robotGoal_x = np.ones(self.numberRobots) * msg.position.x
        self.robotGoal_y = np.ones(self.numberRobots) * msg.position.y
        self.robotGoal_flag = np.ones(self.numberRobots) * 1

    def callbackIndividualGoal(self, msg, i):
        self.robotGoal_x[i] = msg.position.x
        self.robotGoal_y[i] = msg.position.y
        self.robotGoal_flag[i] = 1

    def calculatePotentialField(self):
        for i in range(self.numberRobots):
            u1, v1 = campo_repulsao(self.robotPosition_x, self.robotPosition_y, self.robotPosition_x[i], self.robotPosition_y[i])
            if self.robotGoal_flag[i]:
                u2, v2 = campo_atracao(self.robotPosition_x[i], self.robotPosition_y[i], self.robotGoal_x[i], self.robotGoal_y[i])
            else:
                u2, v2 = 0,0

            u = u1 + u2
            v = v1 + v2
            
            erro_angle = np.arctan2(u, v) - self.robotPosition_yaw[i]
            while(erro_angle > np.pi):
                erro_angle -= 2*np.pi
            while(erro_angle < -np.pi):
                erro_angle += 2*np.pi
            k_yaw = 2

            self.setVelocity((u**2 + v**2)**0.5, erro_angle*k_yaw, i)
