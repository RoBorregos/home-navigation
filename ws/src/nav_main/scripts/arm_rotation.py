#!/usr/bin/env python3

"""
This node is responsible for the arm rotation towards the person detected. It listens the topic /person_detection 
and publishes the angle to rotate the arm to the topic /arm_rotation.

This node runs on the jetson xavier.
"""

import math
from math import fabs
import numpy as np
import time
import rospy
from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import Point
from frida_manipulation_interfaces.srv import MoveJointSDK, MoveVeloJointSDK

# MoveJointSDS_req = rospy.ServiceProxy("/move_joint_sdk", MoveJointSDK)
# MoveJointSDS_req(0, math.degrees(angle-math.radians(90)))

class ArmRotation:
    def __init__(self) -> None:
        rospy.loginfo("Arm Rotation Node Started")
        self.person_angle = 0
        self.arm_angle = 0
        self.rotation_service = rospy.ServiceProxy('/move_joint_sdk', MoveJointSDK)
        self.rotation_velocity_service = rospy.ServiceProxy('/move_joint_velocity_sdk', MoveVeloJointSDK)
        self.person_position_sub = rospy.Subscriber('/test_person_pose_base', PointStamped, self.person_position_callback)
        self.person_point = PointStamped()
        self.angle_buffer = [0,0]

    def person_position_callback(self, msg: PointStamped):
        self.person_point = msg
        self.person_angle = self.get_angle(msg.point.x, msg.point.y)
        self.angle_buffer.append(self.person_angle)
        angular_velocity = self.get_angular_velocity(self.person_angle, self.arm_angle)
        if fabs(angular_velocity) > 0.1:
            angular_velocity = 0.1 if angular_velocity > 0 else -0.1
        self.angle_buffer.pop(0)
        rospy.loginfo(f"Angular Velocity: {angular_velocity}")
        try:
            pass
            #rospy.loginfo(f"Rotating arm to {self.person_angle}")
            self.rotation_velocity_service(0, angular_velocity)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def get_angle(self, x, y):
        angle = math.atan2(y, x)
        if angle < 0:
            angle += 2 * math.pi
        #rospy.loginfo(f"Angle: {angle}")
        corrected_angle = math.degrees(angle - math.radians(90))
        #rospy.loginfo(f"Degrees: {corrected_angle}")
        return corrected_angle

    def get_angular_velocity(self, angle, angle2):
        angular_velocity = (angle2 - angle) / 0.1
        rospy.sleep(0.1)      
        return math.radians(angular_velocity)
    


    # def move_arm(self, angle):
    #     rospy.loginfo(f"Moving arm to {angle}")
    #     try:
    #         self.rotation_service(0, angle)
    #     except rospy.ServiceException as e:
    #         rospy.logerr(f"Service call failed: {e}")

    # def get_person_angular_velocity(self):
    #     if len(self.angle_buffer) < 2:
    #         return 0
    #     angular_velocity = (self.angle_buffer[1] - self.angle_buffer[0]) / 0.1
    #     return angular_velocity

if __name__ == "__main__":
    # rospy.logwarn("Starting depth")
    rospy.init_node("human_position_publisher", anonymous=False)
    # rate = rospy.Rate(10)  # 10hz
    human_position_pub = ArmRotation()
    rospy.spin()
    
    
        

