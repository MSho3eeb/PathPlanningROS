#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np
from nav_msgs.msg import Odometry       #import msg data type "Pose" to be subscribed
import math


#Global Variables##

global yaw
global alpha
global beta

global x_desired
global y_desired
global theta_desired

#Current positions our robot is at
global x_pos
global y_pos
global theta
#parameters for contrller
global linear_v
global angular_v
global Kp
global Kalpha
global Kbeta

linear_v = 0
angular_v =0
Kyaw=0.2
Kalpha = 0.9
Kbeta = -0.2
x_pos = 0
y_pos = 0
theta = 0

def quaternion_to_euler(x,y,z,w):
    t0 = +2.0 * (w * x+y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    x_roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    y_pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    z_yaw = math.atan2(t3,t4)

    return z_yaw


def callback(Odometry):

    global x_pos
    global y_pos
    global theta

    x_pos = round(Odometry.pose.pose.position.x,3)
    y_pos = round(Odometry.pose.pose.position.y,3)
    theta = round(quaternion_to_euler(Odometry.pose.pose.orientation.x,Odometry.pose.pose.orientation.y,Odometry.pose.pose.orientation.z,Odometry.pose.pose.orientation.w),3)

def polar_coordinates():

    global yaw 
    global beta 
    global alpha 
    global x_desired
    global y_desired
    global theta_desired
    global x_pos
    global y_pos
    global theta
    global x_delta
    global y_delta


    x_delta = int(x_desired) - x_pos
    y_delta = int(y_desired) - y_pos

    yaw = np.sqrt((np.square(x_delta))+(np.square(y_delta)))

    gamma = np.arctan2(y_delta,x_delta)

    if gamma < 0 :
        gamma = gamma + (2*np.pi)

    
    
    alpha = gamma - theta
    alpha = (alpha + np.pi) % (2 * np.pi) - np.pi
    beta = - alpha -theta + ((int(theta_desired))*(np.pi/180))
    beta = (beta + np.pi) % (2 * np.pi) - np.pi



    print("alpha-----",alpha)
    print( "beta----- ",beta)
    print("gamma----- ",gamma)
    print("theta-----",theta)



def control_law():

    global yaw		
    global beta	 
    global alpha		
    global linear_v	
    global angular_v	
    global Kyaw
    global Kalpha
    global Kbeta 
    global x_delta
    global y_delta


    if np.absolute(alpha) < np.pi/2:
        linear_v = Kyaw * yaw
    else:
        linear_v = -Kyaw * yaw

    angular_v = Kalpha * alpha + Kbeta * beta

if __name__=='__main__':

    global x_desired
    global y_desired
    global theta_desired
    global yaw		
    global beta		 
    global alpha

  

    x_desired = input("Desired X goal: ")
    y_desired = input("Desired Y goal: ")
    theta_desired = input("Desired Theta goal: ")

    rospy.init_node('turtel_control', anonymous=True)
    pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)

    rate = rospy.Rate(10)

    sub = rospy.Subscriber("odom", Odometry, callback)
    go = Twist()
    while not rospy.is_shutdown():

        polar_coordinates()
        control_law()

        v = round(linear_v,2)
        w = round(angular_v,2)
        go.linear.x = v  #Linear Velocity
        go.linear.y = 0
        go.linear.z = 0
        go.angular.x = 0
        go.angular.y = 0
        go.angular.z = w #Angular Velocity

        pub.publish(go)
        rate.sleep()




