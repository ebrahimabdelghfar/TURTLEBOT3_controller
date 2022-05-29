#!/usr/bin/env python3
from time import sleep

from attr import NOTHING
import rospy
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import euler_from_quaternion
rospy.init_node("get_the_ways")

points=list()
waypoints=[[0,0,0]]
waypoints_noise=[[0,0,0]]
odom_noise=Odometry()
odom_data=Odometry()
i=0
i_noise=0
def odom_callback(odom_data):
    global i
    quant_angles=[odom_data.pose.pose.orientation.x,odom_data.pose.pose.orientation.y,odom_data.pose.pose.orientation.z,odom_data.pose.pose.orientation.w]
    _,_,yaw=euler_from_quaternion(quant_angles)
    points=[odom_data.pose.pose.position.x,odom_data.pose.pose.position.y,yaw]
    if abs(points[0]-waypoints[-1][0]) > 0.2 or abs(points[1]-waypoints[-1][1])>0.2 :
        waypoints.append(points)
        i+=1
rospy.Subscriber("/odom",Odometry,odom_callback)

def odom_noise_callback(odom_noise):
    global i_noise
    quant_angles=[odom_noise.pose.pose.orientation.x,odom_noise.pose.pose.orientation.y,odom_noise.pose.pose.orientation.z,odom_noise.pose.pose.orientation.w]
    _,_,yaw=euler_from_quaternion(quant_angles)
    points=[odom_noise.pose.pose.position.x,odom_noise.pose.pose.position.y,yaw]
    if abs(points[0]-waypoints_noise[-1][0]) > 0.2 or abs(points[1]-waypoints_noise[-1][1])> 0.2 :
        waypoints_noise.append(points)
        i_noise+=1
rospy.Subscriber("/odom_noise",Odometry,odom_noise_callback)

while not rospy.is_shutdown():
    NOTHING
    
way_array=np.array(waypoints)
np.save("/home/ebrahim/TURTLEBOT3_controller/src/path_plan/src/with_out_noise.npy",way_array)

way_noise_array=np.array(waypoints_noise)
np.save("/home/ebrahim/TURTLEBOT3_controller/src/path_plan/src/with_noise.npy",way_noise_array)
