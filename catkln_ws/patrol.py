# #!/usr/bin/env python

# import rospy
# import actionlib
# import tf
# import numpy as np
# import matplotlib.pyplot as plt
# from geometry_msgs.msg import Twist,PoseWithCovarianceStamped
# from nav_msgs.msg import Odometry
# from math import pi,sqrt,atan2
# from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# from actionlib_msgs.msg import *
# from copy import deepcopy

# rospy.init_node('map_navigation',anonymous=False)
# ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

# current_pose = PoseWithCovarianceStamped()

# def callback(data):
#     global current_pose
#     current_pose = data.pose.pose.position



# odom_sub = rospy.Subscriber('/amcl_pose',
#                             PoseWithCovarianceStamped,
#                             poseAMCLCallback)

# print(current_pose)

#! /usr/bin/env python

from enum import auto
from pickle import STOP
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped   
import numpy as np
import rospy
import actionlib
from actionlib_msgs import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal,MoveBaseActionGoal
from math import radians, degrees, pi, sin, cos
from actionlib_msgs.msg import *
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from copy import deepcopy
import tf

WAYPOINTS = [[-1.5,1.5],[0.5,-1.5],[1.5,1.5],[1.5,0.5],[1.5,-1.5],[-1.5,-1.5],[-1.5,1.5]]


rospy.init_node('map_navigation',anonymous=False)
current_pose = None
current_goal = None
current_status = None


def callback(msg):
    global current_pose
    current_pose = msg

def goal_callback(msg):
    global current_goal
    current_goal = msg

def status_callback(msg):
    global current_status
    current_status = msg


# rospy.init_node("read_pose")
sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, callback)
goal_sub = rospy.Subscriber('/move_base/goal',MoveBaseActionGoal,goal_callback)
status_sub = rospy.Subscriber('/move_base/status',GoalStatusArray,status_callback)

pub = rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=10)


my_msg = PoseStamped()

d = [0,[0,0],'map']
e = [[0.5,0.5,0],[0,0,0,1]]

my_msg.header.seq = d[0]
my_msg.header.stamp.secs = d[1][0]
my_msg.header.stamp.nsecs = d[1][1]

my_msg.header.frame_id = d[2]
my_msg.pose.position.x= e[0][0]
my_msg.pose.position.y= e[0][1]
my_msg.pose.position.z= e[0][2]
my_msg.pose.orientation.x = e[1][0]
my_msg.pose.orientation.y = e[1][1]
my_msg.pose.orientation.z = e[1][2]
my_msg.pose.orientation.w = e[1][3]

# pub.publish(my_msg)

# if current_status is None:
#     pub.publish(my_msg)
#     if current_status.status_list[0].text=='Goal reached.':
#         if WAYPOINTS==[]:
#             pass 
        
#         my_msg.pose.position.x,my_msg.pose.position.y=WAYPOINTS[0]
#         WAYPOINTS.remove(WAYPOINTS[0])
#         pub.publish(my_msg)
    
#     rospy.spin()  


# pub.publish(my_msg)

rate = rospy.Rate(1)

def move_to_point(point_x,point_y):
    rospy.logwarn("### we are going to [{0},{1}] = ".format(point_x,point_y)+"###")
    my_msg.pose.position.x,my_msg.pose.position.y = point_x,point_y
    while not rospy.is_shutdown():
        pub.publish(my_msg)
        rate.sleep()

def stop():
    pub.publish(my_msg)
    rospy.sleep(1)



for point in WAYPOINTS:
    move_to_point(point[0],point[1])
    rospy.sleep(1)
# stop()
rospy.logwarn("Action done.")


# while not rospy.is_shutdown():
#     if current_status is None:
#         pub.publish(my_msg)


#   #Sleep at 10Hz
#         continue
#     pass

#     if current_status.status_list[0].text!='Goal reached.':
#         continue


#     if current_status.status_list[0].text=='Goal reached.':
#         if WAYPOINTS==[]:
#             break
#         my_msg.pose.position.x,my_msg.pose.position.y=WAYPOINTS[0]
#         WAYPOINTS.remove(WAYPOINTS[0])

#         pub.publish(my_msg)
#         rospy.spin()
#         rate.sleep()
#         continue


        



    
            
        



