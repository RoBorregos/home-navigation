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
from threading import Lock
from tf2_geometry_msgs import PointStamped, PoseStamped
from geometry_msgs.msg import Point
from frida_manipulation_interfaces.srv import MoveJointSDK, MoveVeloJointSDK
from sensor_msgs.msg import JointState

# MoveJointSDS_req = rospy.ServiceProxy("/move_joint_sdk", MoveJointSDK)
# MoveJointSDS_req(0, math.degrees(angle-math.radians(90)))

class ArmRotation:
    def __init__(self) -> None:
        rospy.loginfo("Arm Rotation Node Started")
        self.person_angle = 0
        self.arm_angle = 0
        self.rotation_service = rospy.ServiceProxy('/move_joint_sdk', MoveJointSDK)
        self.rotation_velocity_service = rospy.ServiceProxy('/move_joint_velocity_sdk', MoveVeloJointSDK)
        self.person_position_sub = rospy.Subscriber('/person_pose_base', PointStamped, self.person_position_callback, queue_size=1)
        self.arm_joint_angle_sub = rospy.Subscriber('/xarm/joint_states', JointState, self.arm_joint_angle_callback)
        self.person_point = PointStamped()
        self.angle_buffer = []

    def arm_joint_angle_callback(self, msg: JointState):
        self.arm_angle = msg.position[0]

    def person_position_callback(self, msg: PointStamped):
        self.person_point = msg
        self.person_angle = self.get_angle(msg.point.x, msg.point.y)
        angular_velocity = self.get_angular_velocity(self.arm_angle, self.person_angle)
        #rospy.loginfo(f"Person Angle: {self.from_minus_pi_to_pi(self.person_angle)}")
        #rospy.loginfo(f"Arm Angle: {self.from_minus_pi_to_pi(self.arm_angle)}")
        #rospy.loginfo(f"Angular Velocity: {angular_velocity}")
        #self.move_arm_with_velocity(angular_velocity)
        angle = self.moving_median(self.person_angle)
        #rospy.loginfo(angle)
        if angle is not None:
            self.move_arm(angle)
            

    def moving_average(self, angle):
        self.angle_buffer.append(angle)
        if len(self.angle_buffer) >5:
            self.angle_buffer.pop(0)
            return np.mean(self.angle_buffer)
        else:
            return None

    def moving_median(self, angle):
        self.angle_buffer.append(angle)
        if len(self.angle_buffer) >5:
            self.angle_buffer.pop(0)
            return np.median(self.angle_buffer)
        else:
            return None   

    def move_arm(self, angle):
        if angle is not None:
            try:
                # Normalize the angle to -pi to pi range
                normalized_angle = self.from_minus_pi_to_pi(angle)
                
                # Calculate the shortest angular distance considering wrap-around
                diff = normalized_angle - self.arm_angle
                
                # Ensure diff is within -pi to pi range
                if(normalized_angle%math.pi == 0):
                    normalized_angle = normalized_angle + 0.001
                if diff > math.pi:
                    diff -= 2 * math.pi
                elif diff < -math.pi:
                    diff += 2 * math.pi
                
                # Calculate the target angle to move to
                target_angle = self.arm_angle + diff
                #rospy.loginfo(target_angle)
                rospy.loginfo(target_angle)
                # Move the arm to the target angle
                if(abs(self.arm_angle - target_angle) > 0.4):
                    self.rotation_service(0, math.degrees(target_angle),0.8,0.4)
                    #rospy.loginfo(abs(self.arm_angle - target_angle))
                    if(abs(target_angle)):        
                        rospy.sleep(0.1)
                else:
                    pass
                    #rospy.loginfo("Arm is already at the target angle")
                #rospy.loginfo(target_angle)   

            except rospy.ServiceException as e:
                pass            

    def limit_angular_velocity(self, angular_velocity):
        max_angular_velocity = 0.2
        if angular_velocity > max_angular_velocity:
            angular_velocity = max_angular_velocity
        elif angular_velocity < -max_angular_velocity:
            angular_velocity = -max_angular_velocity
        return angular_velocity

    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.degrees(math.atan2(t3, t4))-18

        return X, Y, Z

    def from_minus_pi_to_pi(self, angle):
        if angle > math.pi:
            angle -= 2 * math.pi
        if angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def angle_conversor(self, zed_angle):
        if(self.arm_angle > 0 and self.arm_angle < math.radians(90)):
            return (zed_angle - math.radians(360))

    def get_angle(self, x, y):
        angle = math.atan2(y, x)    
        corrected_angle = math.degrees(angle - math.radians(90))
        return math.radians(corrected_angle)

    def get_angular_velocity(self, current_angle, target_angle):
        # Calculate the shortest angular distance considering wrap-around
        diff = self.from_minus_pi_to_pi(target_angle - current_angle)

        # Ensure angular velocity is in the range [-pi, pi]
        if diff > math.pi:
            diff -= 2 * math.pi
        elif diff < -math.pi:
            diff += 2 * math.pi

        # Calculate angular velocity to achieve the target angle in 0.1 seconds
        angular_velocity = diff / 0.1
        rospy.sleep(0.1)
        return angular_velocity

    def move_arm_with_velocity(self):
        try:
            self.rotation_service(0, math.degrees(self.from_minus_pi_to_pi(self.person_angle)))
            #self.rotation_velocity_service(0, angular_velocity)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")


if __name__ == "__main__":
    # rospy.logwarn("Starting depth")
    rospy.init_node("human_position_publisher", anonymous=False)
    # rate = rospy.Rate(10)  # 10hz
    human_position_pub = ArmRotation()
    rospy.spin()
    
    
        

