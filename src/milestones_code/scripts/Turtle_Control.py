#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt
from tf.transformations import euler_from_quaternion


class TurtleBot_waffle_pi:

    def __init__(self):
        self.x_desired = float(input("Set your x goal: "))
        self.y_desired = float(input("Set your y goal: "))
        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        self.distance_tolerance = float(input("Set your tolerance: "))

        rospy.init_node('Turtlebot_Waffle_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel',Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/odom',Odometry, self.update_pose)
        self.pose = Odometry()
        self.rate = rospy.Rate(10)
        self.yaw=0

    def update_pose(self, data):
        #print(self.pose)
        self.pose.pose.pose.position.x = round(data.pose.pose.position.x , 4)
        self.pose.pose.pose.position.y = round(data.pose.pose.position.y , 4)
        quant=data.pose.pose.orientation
        quantlist=[quant.x,quant.y,quant.z,quant.w]
        _,_,self.yaw = euler_from_quaternion(quantlist)
        print("x_cordinate",data.pose.pose.position.x," y_cordinate",data.pose.pose.position.y)

    def distance(self):
        return sqrt(pow((self.x_desired - self.pose.pose.pose.position.x), 2) + pow((self.y_desired - self.pose.pose.pose.position.y), 2))

    def linear_vel(self,constant=0.1):
        return constant * self.distance()

    def steering_angle(self):
        return atan2(self.y_desired - self.pose.pose.pose.position.y, self.x_desired - self.pose.pose.pose.position.x)

    def angular_vel(self,constant=1.5):
        #as the odom only publish quantrion angle not euler so we use ready made function to euler an
        return constant * (self.steering_angle() - self.yaw )

    def move2goal(self):
        vel_msg = Twist()
        while self.distance() >= self.distance_tolerance:
            vel_msg.linear.x = self.linear_vel()
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel()
            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            # Publish at the desired rate.
            self.rate.sleep()
        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        # If we press control + C, the node will stop.
        rospy.spin()

if __name__ == '__main__':
    try:
        x = TurtleBot_waffle_pi()
        x.move2goal()
    except rospy.ROSInterruptException:
        pass