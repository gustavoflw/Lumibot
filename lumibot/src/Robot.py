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
        self.rate = rospy.Rate(10)

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
        
if __name__ == '__main__':
    try:
        #Testing our function
        x = Robot()
        x.path()

    except rospy.ROSInterruptException: 
        pass
