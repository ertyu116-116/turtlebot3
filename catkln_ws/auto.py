#!/usr/bin/env python

from math import pi, sqrt, atan2, cos, sin
import numpy as np

import rospy
import tf
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D

class Turtlebot():
    def __init__(self):
        rospy.init_node("move_base_simple/goal")
        rospy.loginfo("Press Ctrl + C to terminate")
        self.vel_pub=rospy.Publisher("move_base_simple/goal",Twist,queue_size=10)
        self.rate = rospy.Rate(10)

        #reset odometry to zero
        self.reset_pub = rospy.Publisher