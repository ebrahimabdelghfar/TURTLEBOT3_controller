#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import random

class noisy:
    def __init__(self):
        self.sesensor_reading_with_noise=Odometry() #the reading of the sensor that will be published with adittion of noise
        rospy.init_node("add_noise_node")
        self.publish_with_noise=rospy.Publisher("/odom_noise",Odometry, queue_size=10)
        rospy.Subscriber("/odom",Odometry,self.add_noise)
    def add_noise(self,data):
        #add a random noise on position sensor
        self.sesensor_reading_with_noise.pose.pose.position.x=data.pose.pose.position.x + random.SystemRandom().uniform(-3,3)
        self.sesensor_reading_with_noise.pose.pose.position.y=data.pose.pose.position.y + random.SystemRandom().uniform(-3,3)

        #add a random noice on orienation sensor
        self.sesensor_reading_with_noise.pose.pose.orientation.x=data.pose.pose.orientation.x + random.SystemRandom().uniform(-3,3)
        self.sesensor_reading_with_noise.pose.pose.orientation.y=data.pose.pose.orientation.y + random.SystemRandom().uniform(-3,3)
        self.sesensor_reading_with_noise.pose.pose.orientation.z=data.pose.pose.orientation.z + random.SystemRandom().uniform(-3,3)
        self.sesensor_reading_with_noise.pose.pose.orientation.w=data.pose.pose.orientation.w + random.SystemRandom().uniform(-3,3)
    def send_noisy(self):
        while not rospy.is_shutdown():
            #the noise produce of the sensor has a variane of 3 and standered diviation  
            self.publish_with_noise.publish(self.sesensor_reading_with_noise)

if __name__ == '__main__':
    try:
        what_a_noise = noisy()
        what_a_noise.send_noisy()
    except rospy.ROSInterruptException:
        pass
