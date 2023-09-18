#!/usr/bin/env python3
'''
    This node publishes the camera information to the topic /camera_info. 
    The camera information is required for the AR Tag detection node to work. 
    The AR Tag detection node uses the camera information to calculate the pose of the AR Tag in the camera frame. 
    The AR Tag detection node subscribes to the topic /camera_info to get the camera information.
'''

import rospy
from sensor_msgs.msg import CameraInfo

from aux import Aux

if __name__ == '__main__':
    rospy.init_node('camera_info_node')
    Aux.print("Camera Info Node Started")

    # Create the camera info message
    camera_info_msg = CameraInfo()
    camera_info_msg.header.frame_id = 'kinect_rgb'

    # Set the camera info
    camera_info_msg.height = 480
    camera_info_msg.width = 640
    camera_info_msg.distortion_model = 'plumb_bob'
    camera_info_msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]
    camera_info_msg.K = [554.382712, 0.0, 320.5, 0.0, 554.382712, 240.5, 0.0, 0.0, 1.0]
    camera_info_msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    camera_info_msg.P = [554.382712, 0.0, 320.5, 0.0, 0.0, 554.382712, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]

    # Publish the camera info
    camera_info_pub = rospy.Publisher('/camera/info', CameraInfo, queue_size=10)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        camera_info_msg.header.stamp = rospy.Time.now()
        camera_info_pub.publish(camera_info_msg)
        rate.sleep()