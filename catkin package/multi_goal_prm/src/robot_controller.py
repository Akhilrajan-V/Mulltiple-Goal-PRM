#! /usr/bin/env python

"""
A simple proportional controller for the turtlebot3 robot.
Author: Akhilrajan Vethirajan (v.akhilrajan@gmail.ccom)
No LICENSE implemented.
Date: 7 May 2022.
"""
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
from PRM import run
x = 0.0
y= 0.0
theta = 0.0


def odom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])


rospy.init_node("trajectory_controller")
sub = rospy.Subscriber("/odom", Odometry, odom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
speed = Twist()
r = rospy.Rate(5)  # Transmission frequency in Hz
goal = Point()

# path = [(0, 0), (2, 0), (2, 3), (5, 5)]
path = run()

rospy.loginfo_once("Running Trajectory Controller -----")
while not rospy.is_shutdown():

    for i in range(0, len(path)):

        goal.x = path[i][0]
        goal.y = path[i][1]
        reached = False
        print(i)
        print(goal)
        while not reached:
            inc_x = goal.x - x
            inc_y = goal.y - y
            angle_to_goal = atan2(inc_y, inc_x)

            if abs(inc_x) < 0.1 and abs(inc_y) < 0.1:

                rospy.loginfo("Reached Node (%f, %f)" % (goal.x, goal.y))
                speed.linear.x = 0.0
                speed.angular.z = 0.0
                reached = True
                # print("Debug ==> 10")

            elif (angle_to_goal - theta) > 0.3:
                speed.linear.x = 0.0
                speed.angular.z = 0.3
                rospy.loginfo_throttle(35, "Orienting to goal position")
                # print("Debug ==> 20")

            elif (angle_to_goal - theta) < 0.0:
                speed.linear.x = 0.0
                speed.angular.z = -0.3
                rospy.loginfo_throttle(35, "Orienting to goal position")

            else:
                speed.linear.x = 0.5
                speed.angular.z = 0.0
                # print("Debug ==> 30")

            pub.publish(speed)
            r.sleep()
