#!/usr/bin/env python3
'''
this node is resposible filtering the noise in only x position
dynamic state equation:
Xip=A(Xi-1(T))+B(U)
since:
A=1
B=deltaT

perdicition error:
Pp=(A*P(prev)*AT)+Q
since:
Pp : is the perdicted error 
P(prev) : is the previous error # intialy 1000 or biggest no #
Q : is the process error
A=1

calculate kalman gain
K=Pp*CT/(C*Pp*CT+R)
K:kalman gain
Pp: predictied error
C:correction factor
R:measurment noise

correct state equation
X(T)=(Xi)+
'''
from sympy import true
import rospy
import numpy as np
from nav_msgs.msg import Odometry
import matplotlib.pyplot as mp
from geometry_msgs.msg import Twist

class Kalman_filter:
    def __init__(self):
        #variable for state equation
        self.A_s=1.0
        self.B_s=0.1 #at sampling time 0.1 
        self.U_s=0.2 #constant velocity on X direction
        self.Xip=0.0
        self.XT_s=0.0 #initial state equal 0
        #end

        #variable for prediction error
        self.Pp=0.0 #predicted error
        self.Prev=10000000000.0
        self.Q_p= 1.0 # process variance
        #end
        
        #kalman gain
        self.Kalman_gain=0.0 
        self.C_k=1.0
        self.R_k=6.0 #variance of measurment
        #end
        rospy.init_node("kalaman_filter_node")
        rospy.Subscriber("/odom_noise",Odometry,self.sensor_reading_in_x)
        self.reading_sensor_x=0.0 #variable to store sensor reading

        #list of plotting
        self.x_predicted_list=np.array([0])
        self.x_noisy_list=np.array([0])
        self.x_filtered_list=np.array([0])
        #end
        self.velcotiy=rospy.Publisher("/cmd_vel",Twist,queue_size=10,latch=True)
        self.vel_msgs=Twist()
        #define velocity parameter
        self.vel_msgs.linear.x=0.2
        self.vel_msgs.linear.y=0.0
        self.vel_msgs.linear.z=0.0
        self.vel_msgs.angular.x=0.0
        self.vel_msgs.angular.y=0.0
        self.vel_msgs.angular.z=0.0
        #end
        self.x_axis_start = 0
        self.x_axis_end   = 0
        self.y_axis_start = 5
        self.y_axis_end   = 5
        self.dummy=0
        pass
    def predict_kalman(self): #this function predict the error and state
        self.Xip=(self.A_s*self.XT_s)+(self.B_s*self.U_s)
        self.Pp=(self.A_s*self.Prev*self.A_s)+(self.Q_p)
        self.Kalman_gain=(self.Pp*self.C_k)/((self.C_k*self.Pp*self.C_k)+self.R_k)
        print("KG =",self.Kalman_gain)

    def correct_states(self):
        self.XT_s=self.Xip+(self.Kalman_gain*(self.reading_sensor_x-self.Xip))
        self.Pp=(1-self.Kalman_gain)*self.Pp
        self.Prev=self.Pp
        #saving part
        x_predicted_length = len(self.x_predicted_list)
        x_noisy_length = len(self.x_noisy_list)
        x_filter_length = len(self.x_filtered_list)

        #this part collect point if the differnce between prev and next is 1
        if abs(self.x_predicted_list[x_predicted_length-1]-self.Xip) > 1.0 and abs(self.x_noisy_list[x_noisy_length-1]-self.reading_sensor_x)> 1.0 and abs(self.x_filtered_list[x_filter_length-1]-self.XT_s)> 1.0 :
            self.x_predicted_list=np.append(self.x_predicted_list,self.Xip)
            self.x_noisy_list=np.append(self.x_noisy_list,self.reading_sensor_x)
            self.x_filtered_list=np.append(self.x_filtered_list,self.XT_s)



    def close_loop_iteration(self):
        rospy.spin()
        mp.plot(self.x_predicted_list,label="predicted")
        mp.plot(self.x_noisy_list,label="noisy")
        mp.plot(self.x_filtered_list,label="filter")
        mp.legend()
        mp.show()


    def sensor_reading_in_x(self,data):
        '''You can include one of these two
plt.yscale( 'log' )  # lines, or both, or neither.
plt.sho
        function the get the noisy reading of the position sensor in x axis
        '''
        self.velcotiy.publish(self.vel_msgs)
        self.reading_sensor_x=data.pose.pose.position.x
        self.predict_kalman()
        self.correct_states()
        self.velcotiy


filter=Kalman_filter()
filter.close_loop_iteration()            