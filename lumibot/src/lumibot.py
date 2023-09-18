#!/usr/bin/env python3

import time
import math
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

from aux import Aux

'''
    The program will have the following behaviour:
    If the robot has just started moving, it will look for the first tag (ID = 0).
    Then it will run a PID control to move towards the tag.
    Once the robot is close enough to the tag, it will stop and look for the next tag.
    It will turn to the left first and look for tag with ID = 1.
    If it finds the tag, it will move towards it.
    Else, it will turn to the right and look for tag with ID = 1.
    If it finds the tag, it will move towards it.
    Else, it will raise an error and stop.
    Once it reaches the tag, it will stop and look for the next tag.
    The maximum number of tags is 4 (ID = 0, 1, 2, 3).
    The robot will stop once it reaches the last tag.
    '''

class RobotControllerNode:
    def __init__(self):
        self.current_markers = {}
        self.target_tag_id = None
        self.arrived_in_target = bool(False)

        self.kp_linear = {
            0: 0.9,
            1: 0.6,
            2: 0.4,
            3: 0.3,
        }
        self.kp_angular = {
            0: 0.7,
            1: 0.01,
            2: 0.01,
            3: 0.1,
        }
        #0.4

        # The robot will not get closer than this distance to the tag
        self.min_distance = {
            0: 0.4,
            1: 0.4,
            2: 0.4,
            3: 0.4,
        }

        self.sub_marker = rospy.Subscriber('/visualization_marker', Marker, self.marker_callback)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        rospy.init_node('ar_tag_detection_node')
        Aux.print("AR Tag Detection Node Started", color='green')

        self.rate = rospy.Rate(10)
        self.main_loop()        

    def angle_between_positions_in_degrees(self, x1, y1, x2, y2):
        dx = x2 - x1
        dy = y2 - y1
        Aux.print(f"dx: {dx}, dy: {dy}", color='white')
        angle_rad = math.atan2(dx, dy)
        angle_deg = math.degrees(angle_rad)
        Aux.print(f"Angle between positions: {angle_rad} rad // {angle_deg} deg", color='yellow')
        return angle_deg
    
    def get_marker_info(self, marker):
        marker_position = [marker.pose.position.x, marker.pose.position.y, marker.pose.position.z]
        marker_orientation = [marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w]
        marker_scale = [marker.scale.x, marker.scale.y, marker.scale.z]
        Aux.print(f"Marker position: {marker_position}", color='white')
        Aux.print(f"Marker orientation: {marker_orientation}", color='white')
        Aux.print(f"Marker scale: {marker_scale}", color='white')
        return marker_position, marker_orientation, marker_scale
    
    def compute_marker_distance(self, marker_position):
        marker_distance = marker_position[2]
        Aux.print(f"Marker distance: {marker_distance}", color='yellow')
        return marker_distance

    def compute_twist(self, distance, angle, kp_linear, kp_angular):
        twist = Twist()
        twist.linear.x = min(distance * kp_linear, 0.9)
        twist.angular.z = min(angle * kp_angular, 0.7)
        Aux.print(f"twist.linear.x: {twist.linear.x}, twist.angular.z: {twist.angular.z}", color='yellow')
        return twist

    def move_to_tag(self, tag_id):
        
        marker = self.current_markers[str(tag_id)]['msg']
        marker_position, marker_orientation, marker_scale = self.get_marker_info(marker)
        marker_distance = self.compute_marker_distance(marker_position)
        angle = self.angle_between_positions_in_degrees(
            0,
            0,
            marker_position[0],
            marker_position[2],)
        if marker_distance > self.min_distance[tag_id]:
            Aux.print(f"Moving to tag {tag_id}!", color='green')
            twist = self.compute_twist(
                marker_distance,
                -angle,
                self.kp_linear[tag_id],
                self.kp_angular[tag_id])
            self.pub_cmd_vel.publish(twist)
        else:
            Aux.print(f"Arrived in target {tag_id}!", color='green')
            time.sleep(1)
            self.arrived_in_target = True

    def decide_next_tag_id(self, current_target_id):
        if current_target_id == None:
            next_target_tag_id = 0
        else:
            if current_target_id >= 3:
                next_target_tag_id = 0
            else:
                next_target_tag_id = current_target_id + 1
        Aux.print(f"Next tag: {next_target_tag_id}", color='cyan')
        return next_target_tag_id
    
    def currently_detecting_tag(self, current_ids, tag_id):
        if int(tag_id) in [int(id) for id in current_ids]:
            return True
        else:
            return False
    
    def marker_callback(self, msg):
        self.current_markers[str(msg.id)] = {
            'msg': msg,
            'time': time.time()}
        
    def update_markers(self):
        # If the marker is not detected for more than 1 second,
        # we consider that the marker is not detected anymore.
        new_current_markers = {}
        for key, value in self.current_markers.items():
            if value != None and value['msg'] != None and value['time'] != None and value['msg'].id != None:
                if time.time() - value['time'] < 1:
                    new_current_markers[key] = value
        self.current_markers = new_current_markers
        Aux.print(f"\n*** Current marker IDs: {list(self.current_markers.keys())}", color='yellow')

    def main_loop(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            self.update_markers()
            if len(self.current_markers) > 0:
                next_tag = self.decide_next_tag_id(self.target_tag_id)
                if self.currently_detecting_tag(list(self.current_markers.keys()), next_tag):
                    self.move_to_tag(next_tag)
                    if self.arrived_in_target:
                        self.target_tag_id = next_tag
                        self.arrived_in_target = False
                else:
                    Aux.print(f"Tag {next_tag} not detected. Turning around", color='red')
                    twist = Twist()
                    twist.angular.z = -0.4
                    self.pub_cmd_vel.publish(twist)


if __name__ == '__main__':
    RobotControllerNode()

