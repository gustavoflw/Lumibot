#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from turtlesim.msg import Pose
from math import pow,atan2,sqrt,pi
PI = 3.1415926535897

class Robot():

    def __init__(self):
        rospy.init_node('path', anonymous=True)
        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('pose', Pose, self.callback)
        self.rate = rospy.Rate(10)
        self.pose = Pose()

    #Callback function implementing the pose value received
    def callback(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def rotate(self, speed, angle, clockwise):           
        vel_msg = Twist()

        #Converting angles from degrees to radians
        angular_speed = speed*2*PI/360
        relative_angle = angle*2*PI/360

        #We wont use linear components
        vel_msg.linear.x=0
        vel_msg.linear.y=0
        vel_msg.linear.z=0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0

        # Checking if our movement is CW or CCW
        if clockwise:
            vel_msg.angular.z = -abs(angular_speed)
        else:
            vel_msg.angular.z = abs(angular_speed)
        # Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        while(current_angle < relative_angle):
            self.velocity_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed*(t1-t0)

        #Forcing our robot to stop
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
    
    def move(self, speed, distance, isForward):
        vel_msg = Twist()
        
        #Checking if the movement is forward or backwards
        if(isForward):
            vel_msg.linear.x = abs(speed)
        else:
            vel_msg.linear.x = -abs(speed)
        #Since we are moving just in x-axis
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        
        #while not rospy.is_shutdown():

        #Setting the current time for distance calculus
        t0 = float(rospy.Time.now().to_sec())
        current_distance = 0

        #Loop to move the turtle in an specified distance
        while(current_distance < distance):
            #Publish the velocity
            self.velocity_publisher.publish(vel_msg)
            #Takes actual time to velocity calculus
            t1=float(rospy.Time.now().to_sec())
            #Calculates distancePoseStamped
            current_distance= speed*(t1-t0)
        #After the loop, stops the robot
        vel_msg.linear.x = 0
        #Force the robot to stop
        self.velocity_publisher.publish(vel_msg)
   
    def path(self):
        move = [((0.5, 2, 1), (50, 90, 1)), ((0.5, 0.5, 1), (50, 90, 1)),
                ((0.5, 1, 1), (50, 90, 1)), ((0.5, 0.5, 1), (50, 90, 1))]
        
        for m, r in move:
            self.move(m[0], m[1], m[2])
            self.rotate(r[0], r[1], r[2])

    def get_distance(self, goal_x, goal_y):
        return sqrt(pow((goal_x - self.pose.x), 2) + pow((goal_y - self.pose.y), 2))
    
    def get_angle_to_goal(self, goal_x, goal_y):
        return atan2(goal_y - self.pose.y, goal_x - self.pose.x)
    
    def move2goal(self):
        goal_pose = Pose()
        goal_pose.x = float(input("Set your x goal:"))
        goal_pose.y = float(input("Set your y goal:"))
        distance_tolerance = float(input("Set your tolerance:"))

        vel_msg = Twist()
        
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        
        dist = self.get_distance(goal_pose.x, goal_pose.y)
        
        while dist >= distance_tolerance:

            #Proportional Controller
            #linear velocity in the x-axis:
            vel_msg.linear.x = 1.5 * dist

            #angular velocity in the z-axis:
            ang_to_goal = self.get_angle_to_goal(goal_pose.x, goal_pose.y)
            ang_dif = (ang_to_goal - self.pose.theta)            
            if( (ang_dif) >= (pi) ):
                 ang_dif = (2*pi)-ang_dif
            if( (ang_dif) < -(pi) ):
                 ang_dif = ang_dif+(2*pi)             
            vel_msg.angular.z = 4 * (ang_dif)

            #Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
            
            dist = self.get_distance(goal_pose.x, goal_pose.y)
            
        #Stopping our robot after the movement is over
        vel_msg.linear.x = 0
        vel_msg.angular.z =0
        self.velocity_publisher.publish(vel_msg)
        
if __name__ == '__main__':
    try:
        #Testing our function
        x = Robot()
        x.move2goal()

    except rospy.ROSInterruptException: 
        pass
