#!/usr/bin/env python3
from importlib.resources import path
import matplotlib
import numpy as np #Imports Numpy package full of functions and methods that can be used.
import cv2
import matplotlib.pyplot as mp
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt
from tf.transformations import euler_from_quaternion


class TurtleBot_waffle_pi:

    def __init__(self):
        global path
        self.no_of_pointx,_=np.shape(path,)
        print(self.no_of_pointx)
        rospy.init_node('Turtlebot_Waffle_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel',Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/odom',Odometry, self.update_pose)
        self.pose = Odometry()
        self.rate = rospy.Rate(10)
        self.yaw=0
        self.point_no=1
        self.x_desired = path[self.point_no][0]
        self.y_desired = path[self.point_no][1]
        print("current x= ",self.x_desired,"current y= ",self.y_desired)
        print(path)
        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        self.distance_tolerance = 0.5

    def update_pose(self, data):
        #print(self.pose)
        self.pose.pose.pose.position.x = round(data.pose.pose.position.x , 2)
        self.pose.pose.pose.position.y = round(data.pose.pose.position.y , 2)
        quant=data.pose.pose.orientation
        quantlist=[quant.x,quant.y,quant.z,quant.w]
        _,_,self.yaw = euler_from_quaternion(quantlist)
        # print("x_cordinate :",data.pose.pose.position.x,"\ny_cordinate :",data.pose.pose.position.y,"\n")

    def distance(self):
        print("next point to go x= ",self.x_desired,"next point to go y= ",self.y_desired)
        return sqrt(pow((self.x_desired - self.pose.pose.pose.position.x), 2) + pow((self.y_desired - self.pose.pose.pose.position.y), 2))

    def linear_vel(self,constant=0.1):
        return constant * self.distance()

    def steering_angle(self):
        return atan2(self.y_desired - self.pose.pose.pose.position.y, self.x_desired - self.pose.pose.pose.position.x)

    def angular_vel(self,constant=0.2):
        #as the odom only publish quantrion angle not euler so we use ready made function to euler an
        return constant * (self.steering_angle() - self.yaw )

    def move2goal(self):
        vel_msg = Twist()
        while not rospy.is_shutdown():
            vel_msg.linear.x = self.linear_vel()
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0
            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = self.angular_vel()
            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            if(((self.y_desired < (self.pose.pose.pose.position.y+0.1))) and ((self.x_desired < (self.pose.pose.pose.position.x+0.1)))):
                self.point_no+=1
                self.x_desired = path[self.point_no][0]
                self.y_desired = path[self.point_no][1]
            if((self.no_of_pointx-1)==self.point_no):
                break
            # Publish at the desired rate.
            self.rate.sleep()
        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.velocity_publisher.publish(vel_msg)


def BFS(Map,Start,Goal,Direction):  # Direction for CCW -1  for CW 1

    Flag = False # Flag indicating if goal node is found
    
    # Initializing a queue 
    queue = []
    queue.append(Start)

    # Visited Nodes
    Visited = []

    # Parent Node Mapping
    Parents = np.zeros([15,15,2],dtype = int)


    while queue:   # If queue is not empty 

        Node = queue.pop(0)  # Pop first element in the queue

        if Node==Goal:   # Did we reach the goal?
            Flag = True
            break

        Neighbours = get_Neighbours(Node,Direction) # Get neighbours of this node

        for N in Neighbours: # For each neighbour of this node
            if not N in Visited and Map[N[0],N[1]]!=0:   # if this neighbour wasnt visited before and doesnt equal 0 (ie not an obstacle)
                queue.append(N)      # add to the queue
                Visited.append(N)     # add it to the visited
                Parents[N[0],N[1]] = Node  # add its parent node 

    if Flag == False:
        print('no path_found')

    if Flag == True:
        path = get_Path(Parents,Goal,Start) # this function returns the path to the goal 
        return path
def DFS(Map,Start,Goal,Direction):  # Direction for CCW -1  for CW 1

    Flag = False # Flag indicating if goal node is found
    
    # Initializing a queue 
    stack = []
    stack.append(Start)

    # Visited Nodes
    Visited = []

    # Parent Node Mapping
    Parents = np.zeros([15,15,2],dtype = int)


    while stack:   # If queue is not empty 

        Node = stack.pop(-1)  # Pop first element in the queue

        if Node==Goal:   # Did we reach the goal?
            Flag = True
            break

        Neighbours = get_Neighbours(Node,Direction) # Get neighbours of this node

        for N in Neighbours: # For each neighbour of this node
            if not N in Visited and Map[N[0],N[1]]!=0:   # if this neighbour wasnt visited before and doesnt equal 0 (ie not an obstacle)
                stack.append(N)      # add to the queue
                Visited.append(N)     # add it to the visited
                Parents[N[0],N[1]] = Node  # add its parent node 

    if Flag == False:
        print('no path found')

    if Flag == True:
        path = get_Path(Parents,Goal,Start) # this function returns the path to the goal 
        return path
def get_Path(Parents,Goal,Start):

    path = []
    Node = Goal  # start backward 
    path.append(Goal)
   
    while not np.array_equal(Node, Start):

       path.append(Parents[Node[0],Node[1]])  # add the parent node 
       Node = Parents[Node[0],Node[1]]        # search for the parent of that node 
     
    path.reverse()
    return path
def get_Neighbours(Node,Direction):    # Returns neighbours of the parent node   Direction = 1 for Clockwise/ Direction = -1 for CCW 

    Neighbours = []
    
    if Direction==1:
        Neighbours.append([Node[0],Node[1]+1])
        Neighbours.append([Node[0]+1,Node[1]+1])
        Neighbours.append([Node[0]+1,Node[1]])
        Neighbours.append([Node[0]+1,Node[1]-1])
        Neighbours.append([Node[0],Node[1]-1])
        Neighbours.append([Node[0]-1,Node[1]-1])
        Neighbours.append([Node[0]-1,Node[1]])
        Neighbours.append([Node[0]-1,Node[1]+1])
                        
    if Direction==-1:
        Neighbours.append([Node[0],Node[1]+1])
        Neighbours.append([Node[0]-1,Node[1]+1])
        Neighbours.append([Node[0]-1,Node[1]])
        Neighbours.append([Node[0]-1,Node[1]-1])
        Neighbours.append([Node[0],Node[1]-1])
        Neighbours.append([Node[0]+1,Node[1]-1])
        Neighbours.append([Node[0]+1,Node[1]])
        Neighbours.append([Node[0]+1,Node[1]+1])

    return Neighbours

if __name__ == '__main__':     # Main function that is executed 

    desired_x_cordinate=int(input("please enter the required X-cordinate: "))
    desired_y_cordinate=int(input("please enter the required y-cordinate: "))
    img = cv2.imread('/home/ebrahim/TURTLEBOT3_controller/src/path_plan/map_new/maze.png')
    grayImage = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, bw_img = cv2.threshold(grayImage,30,255,cv2.THRESH_BINARY)
    bw_img=np.rot90(bw_img,k=2)
    bw_img=np.fliplr(bw_img)
    path = BFS(bw_img, [1,1],[desired_y_cordinate,desired_x_cordinate],-1)
    path=np.array(path)
    path=np.rot90(path,2)
    mp.imshow(bw_img) 
    mp.plot(path[:,0],path[:,1])
    mp.gca().invert_yaxis()
    mp.show(block=False)
    x=TurtleBot_waffle_pi()
    x.move2goal()
    cv2.waitKey(0)
    cv2.destroyAllWindows()


