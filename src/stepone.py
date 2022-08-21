#!/usr/bin/python3

from dis import dis
from operator import index
from matplotlib import pyplot as plt
import numpy as np
import rospy
import tf

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point

from math import radians, sqrt

class Controller:
    
    def __init__(self) -> None:
        
        rospy.init_node("controller" , anonymous=False)
        
        ## self.laser_subscriber = rospy.Subscriber("/scan" , LaserScan , callback=self.laser_callback)
        sub = rospy.Subscriber("/odometry/filtered", Odometry, self.get_heading)
        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=10)

        # getting specified parameters
        self.linear_speed = rospy.get_param("/controller/linear_speed") # m/s
        self.angular_speed = rospy.get_param("/controller/angular_speed") # rad/s
        self.goal_angle = radians(rospy.get_param("/controller/goal_angle")) # rad
        self.stop_distance = rospy.get_param("/controller/stop_distance") # m
        self.epsilon = rospy.get_param("/controller/epsilon")
        self.distanceThreshold = 0.07
        
        # defining the states of our robot
        self.GO, self.ROTATE = 0, 1
        self.state = self.GO 

        self.errors =[]
        self.rectangle = []

        origin = Point()
        origin.x = 2
        origin.y = 0

        X1 = np.linspace(-2, 2 , 100)

        for x in X1:
            self.rectangle.append([x,3.0])

        Y2 = np.linspace(-3, 3 , 100)

        for y in Y2:
            self.rectangle.append([2.0,y])

        X3 = np.linspace(-2, 2 , 100)

        for x in X3:
            self.rectangle.append([x,-3.0])

        Y4 = np.linspace(-3, 3 , 100)

        for y in Y4:
            self.rectangle.append([-2.0,y])
        
    # checks whether there is an obstacle in front of the robot
    # or not
    # def laser_callback(self, msg: LaserScan):
    #     if msg.ranges[0] <= self.stop_distance:
    #         self.state = self.ROTATE
    
    # heading of the robot 
    def get_heading(self, euler = True):
        
        # waiting for the most recent message from topic /odom
        msg = rospy.wait_for_message("/odom" , Odometry)
        
        orientation = msg.pose.pose.orientation
        
        # convert quaternion to odom
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        if euler:
            return yaw
        return  msg.pose.pose.position

    def rotate(self):
        twist = Twist()
        self.cmd_publisher.publish(Twist())
        rospy.sleep(1)

        remaining = self.goal_angle
        prev_angle = self.get_heading()
        
        twist = Twist()
        twist.angular.z = self.angular_speed
        self.cmd_publisher.publish(twist)
        
        # rotation loop
        while remaining >= self.epsilon:
            current_angle = self.get_heading()
            delta = abs(prev_angle - current_angle)
            remaining -= delta
            prev_angle = current_angle
        
        self.cmd_publisher.publish(Twist())
        

    def move_straight(self):
        twist = Twist()
        self.cmd_publisher.publish(twist)
        twist.linear.x = self.linear_speed
        self.cmd_publisher.publish(twist)

    def plan_move(self,p2, threshold):
        while True:
            current_pose = self.get_heading(False)
            rospy.loginfo(get_error(self.rectangle, current_pose))
            self.errors.append(get_error(self.rectangle, current_pose))
            if check_distance(current_pose, p2, threshold=threshold):
                rospy.loginfo("Moving Straight")
                self.move_straight()
            else:
                rospy.loginfo("---Roatating---")
                self.rotate()
                return            

    def run(self):
        origin = Point()
        origin.x = 2
        origin.y = 0

        while True:
            current_pose = self.get_heading(False)
            rospy.loginfo(get_error(self.rectangle, current_pose))
            self.errors.append(get_error(self.rectangle, current_pose))
            if sqrt((float(current_pose.x) - origin.x) ** 2 + (float(current_pose.y) - origin.y) ** 2) >= self.distanceThreshold:
                rospy.loginfo("Moving Straight")
                twist = Twist()
                twist.linear.x = self.linear_speed
                self.cmd_publisher.publish(twist)
            else:
                rospy.loginfo("---Roatating---")
                self.rotate()
                break

        while not rospy.is_shutdown():

            origin.x = 2
            origin.y = 3
            self.plan_move(origin, 6)

            origin.x = 2
            origin.y = -3
            self.plan_move(origin, 4)

            origin.x = -2
            origin.y = -3
            self.plan_move(origin, 6)

            origin.x = -2
            origin.y = 3
            self.plan_move(origin, 4)
            
            self.state = self.GO


def get_distance(p1, p2):
    return sqrt((float(p1.x) - float(p2.x)) ** 2 + (float(p1.y) - float(p2.y)) ** 2)

def check_distance(p1,p2, threshold):
    if get_distance(p1,p2) <= threshold:
        return True

def get_error(rectangle,p2):
    distances = []
    print(p2)
    for p in rectangle:
        distances.append((pow((p[0] - float(p2.x)), 2) + pow((p[1] - float(p2.y)),2))**.5)
    print(distances.index(min(distances)))
    return min(distances)

def show_plot(error):
    # plt.plot(robot_x_vector, robot_y_vector,linewidth = '1.4',color='blue', label='robot path')
    # plt.xlabel("x - axis")
    # plt.ylabel("y - axis")
    # plt.xlim((-4,4))
    # plt.ylim((-4,4))
    # plt.legend()
    # plt.title('Mobile Robot Trajectory')

    plt.figure()
    plt.plot(range(len(error)), error)
    plt.xlabel('Second')
    plt.ylabel('Error')
    plt.legend()
    plt.title('Path Error')
    plt.show()

if __name__ == "__main__":

    try:
        controller = Controller()
        controller.run()
    except rospy.ROSInterruptException:
        show_plot(controller.errors)
