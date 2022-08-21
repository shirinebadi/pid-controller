#!/usr/bin/python3

from cmath import rect
from matplotlib import pyplot as plt
import numpy as np
import rospy
import tf
import math

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point

from math import atan2, pi, radians, sqrt

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
        self.distanceThreshold = 0.3
        self.twist = Twist()
        
        # defining the states of our robot
        self.GO, self.ROTATE = 0, 1
        self.state = self.GO 

        self.kp_distance = 10
        self.ki_distance = 20
        self.kd_distance = 1

        self.kp_angle = 1
        self.ki_angle = 0.03
        self.kd_angle = 0.05

        self.errors =[]
        self.shape = []
        self.archimed = []
        self.eighteeth = []
        self.logarithmic = []
        self.circle = []
        
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
        twist.linear.x = self.linear_speed
        self.cmd_publisher.publish(twist)

    def plan_move(self,p2, threshold):
        while True:
            current_pose = self.get_heading(False)
            self.errors.append(get_error(self.rectangle, current_pose))
            if check_distance(current_pose, p2, threshold=threshold):
                rospy.loginfo("Moving Straight")
                self.move_straight()
            else:
                rospy.loginfo("---Roatating---")
                self.rotate()
                return   

    def go_to_point(self,point):
        pass
                 

    def run(self):

        ## Circle

        X1 = np.linspace(-6., -2 , 50)
        Y1 = np.zeros((50,))
        for i,x in enumerate(X1):
            self.circle.append([x,Y1[i]])

        x_dim, y_dim = 2,2
        t = np.linspace(np.pi, 0, 100)
        X2 = x_dim * np.cos(t) 
        Y2 = y_dim * np.sin(t)
        for i,x in enumerate(X2):
            self.circle.append([x,Y2[i]])

        X3 = np.linspace(2, 6 , 50)
        Y3 = np.zeros((50,))
        for i,x in enumerate(X3):
            self.circle.append([x,Y3[i]])

        x_dim, y_dim = 6,6
        t = np.linspace(np.pi*2, np.pi, 200)
        X4 = x_dim * np.cos(t) 
        Y4 = y_dim * np.sin(t)
        for i,x in enumerate(X4):
            self.circle.append([x,Y4[i]])
        self.shape.append(self.circle)

        ## Archimedean spiral
        growth_factor = 0.1
        for i in range(400):
            t = i / 20 * math.pi
            dx = (1 + growth_factor * t) * math.cos(t)
            dy = (1 + growth_factor * t) * math.sin(t)
            self.archimed.append([dx,dy])
        self.shape.append(self.archimed)

        ## Logarithmic
        a = 0.17
        k = math.tan(a)
        for i in range(150):
            t = i / 20 * math.pi
            dx = a * math.exp(k * t) * math.cos(t)
            dy = a * math.exp(k * t) * math.sin(t)
            self.logarithmic.append([dx,dy])
        self.shape.append(self.logarithmic)


        ## Eightees
        X2 = np.linspace(1, 1 + 2**(1/2) , 10)
        Y2 = - (2**(1/2)) * (X2 - 1) + 3
        for i,x in enumerate(X2):
            self.eighteeth.append([x,Y2[i]])

        Y3 = np.linspace(1, -1 , 10)
        X3 = np.array([1 + 2**(1/2)]*10)
        for i,x in enumerate(X3):
            self.eighteeth.append([x,Y3[i]])

        X4 = np.linspace(1 + 2**(1/2), 1, 10)
        Y4 = (2**(1/2)) * (X4 - 1 - 2**(1/2)) -1 
        for i,x in enumerate(X4):
            self.eighteeth.append([x,Y4[i]])

        X5 = np.linspace(1, -1 , 10)
        for i,x in enumerate(X5):
            self.eighteeth.append([x,-3])

        X6 = np.linspace(-1, -1 - 2**(1/2) , 10)
        Y6 = - (2**(1/2)) * (X6 + 1) - 3 
        for i,x in enumerate(X6):
            self.eighteeth.append([x,Y6[i]])


        Y7 = np.linspace(-1, 1 , 10)
        X7 = np.array([- 1 - 2**(1/2)]*10)
        for i,x in enumerate(X7):
            self.eighteeth.append([x,Y7[i]])


        X8 = np.linspace(-1 - 2**(1/2), -1, 10)
        Y8 = (2**(1/2)) * (X8 + 1 + 2**(1/2)) + 1
        for i,x in enumerate(X8):
            self.eighteeth.append([x,Y8[i]])

        
        X1 = np.linspace(-1, 1 , 10)
        for x in X1:
            self.eighteeth.append([x,3])
        self.shape.append(self.eighteeth)

        while not rospy.is_shutdown():
            last_rotation = 0
            for i,goal in enumerate(self.circle):
                self.cmd_publisher.publish(Twist())
                ##rospy.sleep(2)
                            
                previous_distance = 0
                total_distance = 0
                
                rospy.loginfo('--New Goal--')
                rospy.loginfo(goal)

                current_pose = self.get_heading(False)
                distance = sqrt(pow((goal[0] - current_pose.x), 2) + pow((goal[1] - current_pose.y), 2))
                rospy.loginfo(distance)

                while distance > 0.25:
                    print('im again hereeeeee')
                    rotation = self.get_heading(True)
                    current_pose = self.get_heading(False)
                    path_angle = atan2(goal[1]-current_pose.y , goal[0] - current_pose.x) 
                    if path_angle < -pi/4 or path_angle > pi/4:
                        if goal[1] < 0 and current_pose.y < goal[1]:
                            path_angle = -2*pi + path_angle
                        elif goal[1] >= 0 and current_pose.y > goal[1]:
                            path_angle = 2*pi + path_angle
                    if last_rotation > pi-0.1 and rotation <= 0:
                        rotation = 2*pi + rotation
                    elif last_rotation < -pi+0.1 and rotation > 0:
                        rotation = -2*pi + rotation

                    diff_distance = distance - previous_distance
                    distance = sqrt(pow((goal[0] - current_pose.x), 2) + pow((goal[1] - current_pose.y), 2))


                    control_signal_distance = self.kp_distance*distance + self.ki_distance*total_distance + self.kd_distance*diff_distance

                    control_signal_angle = self.kp_angle*(path_angle - rotation)

                    self.twist.angular.z = (control_signal_angle)

                    self.twist.linear.x = min(control_signal_distance,0.1)
                    rospy.loginfo('Linear')
                    rospy.loginfo(self.twist.linear.x)

                    if self.twist.angular.z > 0:
                        self.twist.angular.z = min(self.twist.angular.z, 1.5)
                    else:
                        self.twist.angular.z = max(self.twist.angular.z, -1.5)
                    
                    last_rotation = rotation
                    self.cmd_publisher.publish(self.twist)

                    rospy.sleep(1)
                    previous_distance = distance
                    total_distance = total_distance + distance
                    print('distance: ', distance)

                


def get_distance(p1, p2):
    return sqrt((int(p1.x) - int(p2.x)) ** 2 + (int(p1.y) - int(p2.y)) ** 2)

def check_distance(p1,p2, threshold):
    if get_distance(p1,p2) <= threshold:
        return True

def get_error(rectangle,p2):
    distances = []
    for p in rectangle:
        distances.append(sqrt((int(p[0]) - int(p2.x)) ** 2 + (int(p[1]) - int(p2.y)) ** 2))

    return np.argmin(np.asarray(distances))

def show_plot(error):
    # plt.plot(robot_x_vector, robot_y_vector,linewidth = '1.4',color='blue', label='robot path')
    # plt.xlabel("x - axis")
    # plt.ylabel("y - axis")
    # plt.xlim((-4,4))
    # plt.ylim((-4,4))
    # plt.legend()
    # plt.title('Mobile Robot Trajectory')

    plt.figure()
    plt.plot(error,range(len(error)))
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
