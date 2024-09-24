#!/usr/bin/env python3

"""
This node receives the image from the zed2 camera and detects the position of the person in the image.
It then publishes the position to the /person_pose_odom topic in the base_footprint frame.

This node runs on the jetson xavier.
"""

import sys
import math
from math import fabs
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

import rospy
import tf2_ros
from sensor_msgs.msg import Image, CameraInfo
from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import Point
from std_srvs.srv import SetBool

FLT_EPSILON = sys.float_info.epsilon


class HumanPositionPublisher:
    def __init__(self):
        self.bridge = CvBridge()
        self.pose_publisher = rospy.Publisher(
            "/person_pose_base", PointStamped, queue_size=5
        )
        self.test_pose_publisher = rospy.Publisher(
            "/test_person_pose_base", PointStamped, queue_size=5
        )
        self.image_subscriber = rospy.Subscriber(
            "/zed2/zed_node/rgb/image_rect_color", Image, self.image_callback
        )
        self.depth_subscriber = rospy.Subscriber(
            "/zed2/zed_node/depth/depth_registered", Image, self.depth_image_callback
        )
        self.info_subscriber = rospy.Subscriber(
            "/zed2/zed_node/depth/camera_info", CameraInfo, self.camera_info_callback
        )

        self.follow_person = False
        
        self.change_follow_state = rospy.Service("/change_follow_person_state", SetBool, self.change_follow_person_state)
        self.change_person_tracker_state = rospy.ServiceProxy("/change_person_tracker_state", SetBool)
        self.person_tracker_subscriber = rospy.Subscriber("/person_detection", Point, self.person_tracker_callback)


        self.cv_image = None
        self.camera_info = None
        rospy.loginfo("Subscribed to image")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.x, self.y = 0, 0

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.

        Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.

        Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(
            roll / 2
        ) * np.sin(pitch / 2) * np.sin(yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(
            roll / 2
        ) * np.cos(pitch / 2) * np.sin(yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(
            roll / 2
        ) * np.sin(pitch / 2) * np.cos(yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(
            roll / 2
        ) * np.sin(pitch / 2) * np.sin(yaw / 2)

        return [qx, qy, qz, qw]
    
    def change_follow_person_state(self, req: SetBool):
        print(f"Received: {req.data}")

        # change person tracker state
        self.change_person_tracker_state(req.data)

        self.follow_person = req.data
        return True, "Success"

    def depth_image_callback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)

    # Function to handle a ROS camera info input.
    def camera_info_callback(self, data):
        self.camera_info = data
        self.info_subscriber.unregister()

    def image_callback(self, data):
        # self.bridge.imgmsg_to_cv2(data,desired_encoding="rgb8")
        self.cv_image = data

    def person_tracker_callback(self, detection: Point):
        if self.cv_image is None or self.camera_info is None:
            print(f'Follow person: {self.follow_person}, cv_image: {self.cv_image is None}, camera_info: {self.camera_info is None}')
            return
        
        self.x, self.y = int(detection.x), int(detection.y)

        point3D = Point()
        point2D = [self.x, self.y]
        if len(self.depth_image) != 0:
            depth = self.get_depth(self.depth_image, point2D)
            point3D_ = self.deproject_pixel_to_point(
                self.camera_info, point2D, depth
            )
            point3D.x = point3D_[0]
            point3D.y = point3D_[1]
            point3D.z = point3D_[2]

            point_x = PointStamped()
            point_x.header.frame_id = "zed2_left_camera_optical_frame"
            point_x.point.x = point3D.x
            point_x.point.y = point3D.y
            point_x.point.z = point3D.z

            # print(point_x)

            try:
                target_pt = self.tf_buffer.transform(point_x, "base_footprint")
                # print(f"Transformed point: ({target_pt.point.x}, {target_pt.point.y}, {target_pt.point.z})")
                print(f"Transformed point: ({target_pt})")
            except tf2_ros.LookupException:
                print("Transform lookup failed.")
                return
            
            if target_pt.point.x > 4.5:
                print("Too far from the robot")
                return

            if target_pt.point.x < 0.3:
                print("Too close to the robot")
                # return
            self.pose_publisher.publish(target_pt)
            self.test_pose_publisher.publish(point_x)

    def get_depth(self, depthframe_, pixel):
        '''
            Given pixel coordinates in an image, the actual image and its depth frame, compute the corresponding depth.
        '''
        heightDEPTH, widthDEPTH = (depthframe_.shape[0], depthframe_.shape[1])

        x = pixel[0]
        y = pixel[1]

        def medianCalculation(x, y, width, height, depthframe_):
            medianArray = []
            requiredValidValues = 20
            def spiral(medianArray, depthframe_, requiredValidValues, startX, startY, endX, endY, width, height):
                if startX <  0 and startY < 0 and endX > width and endY > height:
                    return
                # Check first and last row of the square spiral.
                for i in range(startX, endX + 1):
                    if i >= width:
                        break
                    if startY >= 0 and math.isfinite(depthframe_[startY][i]):
                        medianArray.append(depthframe_[startY][i])
                    if startY != endY and endY < height and math.isfinite(depthframe_[endY][i]):
                        medianArray.append(depthframe_[endY][i])
                    if len(medianArray) > requiredValidValues:
                        return
                # Check first and last column of the square spiral.
                for i in range(startY + 1, endY):
                    if i >= height:
                        break
                    if startX >= 0 and math.isfinite(depthframe_[i][startX]):
                        medianArray.append(depthframe_[i][startX])
                    if startX != endX and endX < width and math.isfinite(depthframe_[i][endX]):
                        medianArray.append(depthframe_[i][endX])
                    if len(medianArray) > requiredValidValues:
                        return
                # Go to the next outer square spiral of the depth pixel.
                spiral(medianArray, depthframe_, requiredValidValues, startX - 1, startY - 1, endX + 1, endY + 1, width, height)
            
            # Check square spirals around the depth pixel till requiredValidValues found.
            spiral(medianArray, depthframe_, requiredValidValues, x, y, x, y, width, height)
            if len(medianArray) == 0:
                return float("NaN")

            # Calculate Median
            medianArray.sort()
            return medianArray[len(medianArray) // 2]
        
        # Get the median of the values around the depth pixel to avoid incorrect readings.
        return medianCalculation(x, y, widthDEPTH, heightDEPTH, depthframe_)

    def deproject_pixel_to_point(self, cv_image_rgb_info, pixel, depth):
        '''
            Given pixel coordinates and depth in an image with no distortion or inverse distortion coefficients, 
            compute the corresponding point in 3D space relative to the same camera
            Reference: https://github.com/IntelRealSense/librealsense/blob/e9f05c55f88f6876633bd59fd1cb3848da64b699/src/rs.cpp#L3505
        '''
        def CameraInfoToIntrinsics(cameraInfo):
            intrinsics = {}
            intrinsics["width"] = cameraInfo.width
            intrinsics["height"] = cameraInfo.height
            intrinsics["ppx"] = cameraInfo.K[2]
            intrinsics["ppy"] = cameraInfo.K[5]
            intrinsics["fx"] = cameraInfo.K[0]
            intrinsics["fy"] = cameraInfo.K[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                intrinsics["model"] = "RS2_DISTORTION_BROWN_CONRADY"
            elif cameraInfo.distortion_model == 'equidistant':
                intrinsics["model"] = "RS2_DISTORTION_KANNALA_BRANDT4"
            intrinsics["coeffs"] = [i for i in cameraInfo.D]
            return intrinsics
        
        # Parse ROS CameraInfo msg to intrinsics dictionary.
        intrinsics = CameraInfoToIntrinsics(cv_image_rgb_info)

        if(intrinsics["model"] == "RS2_DISTORTION_MODIFIED_BROWN_CONRADY"): # Cannot deproject from a forward-distorted image
            return

        x = (pixel[0] - intrinsics["ppx"]) / intrinsics["fx"]
        y = (pixel[1] - intrinsics["ppy"]) / intrinsics["fy"]

        xo = x
        yo = y

        if (intrinsics["model"] == "RS2_DISTORTION_INVERSE_BROWN_CONRADY"):
            # need to loop until convergence 
            # 10 iterations determined empirically
            for i in range(10):
                r2 = float(x * x + y * y)
                icdist = float(1) / float(1 + ((intrinsics["coeffs"][4] * r2 + intrinsics["coeffs"][1]) * r2 + intrinsics["coeffs"][0]) * r2)
                xq = float(x / icdist)
                yq = float(y / icdist)
                delta_x = float(2 * intrinsics["coeffs"][2] * xq * yq + intrinsics["coeffs"][3] * (r2 + 2 * xq * xq))
                delta_y = float(2 * intrinsics["coeffs"][3] * xq * yq + intrinsics["coeffs"][2] * (r2 + 2 * yq * yq))
                x = (xo - delta_x) * icdist
                y = (yo - delta_y) * icdist

        if intrinsics["model"] == "RS2_DISTORTION_BROWN_CONRADY":
            # need to loop until convergence 
            # 10 iterations determined empirically
            for i in range(10):
                r2 = float(x * x + y * y)
                icdist = float(1) / float(1 + ((intrinsics["coeffs"][4] * r2 + intrinsics["coeffs"][1]) * r2 + intrinsics["coeffs"][0]) * r2)
                delta_x = float(2 * intrinsics["coeffs"][2] * x * y + intrinsics["coeffs"][3] * (r2 + 2 * x * x))
                delta_y = float(2 * intrinsics["coeffs"][3] * x * y + intrinsics["coeffs"][2] * (r2 + 2 * y * y))
                x = (xo - delta_x) * icdist
                y = (yo - delta_y) * icdist

        if intrinsics["model"] == "RS2_DISTORTION_KANNALA_BRANDT4":
            rd = float(math.sqrt(x * x + y * y))
            if rd < FLT_EPSILON:
                rd = FLT_EPSILON

            theta = float(rd)
            theta2 = float(rd * rd)
            for i in range(4):
                f = float(theta * (1 + theta2 * (intrinsics["coeffs"][0] + theta2 * (intrinsics["coeffs"][1] + theta2 * (intrinsics["coeffs"][2] + theta2 * intrinsics["coeffs"][3])))) - rd)
                if fabs(f) < FLT_EPSILON:
                    break
                df = float(1 + theta2 * (3 * intrinsics["coeffs"][0] + theta2 * (5 * intrinsics["coeffs"][1] + theta2 * (7 * intrinsics["coeffs"][2] + 9 * theta2 * intrinsics["coeffs"][3]))))
                theta -= f / df
                theta2 = theta * theta
            r = float(math.tan(theta))
            x *= r / rd
            y *= r / rd

        if intrinsics["model"] == "RS2_DISTORTION_FTHETA":
            rd = float(math.sqrt(x * x + y * y))
            if rd < FLT_EPSILON:
                rd = FLT_EPSILON
            r = (float)(math.tan(intrinsics["coeffs"][0] * rd) / math.atan(2 * math.tan(intrinsics["coeffs"][0] / float(2.0))))
            x *= r / rd
            y *= r / rd

        return (depth * x, depth * y, depth)



if __name__ == "__main__":
    rospy.logwarn("Starting depth")
    rospy.init_node("detector_humano", anonymous=True)
    # rate = rospy.Rate(10)  # 10hz
    human_position_pub = HumanPositionPublisher()
    rospy.spin()
    # try:
    #     while not rospy.is_shutdown():
    #         human_position_pub.get_goal_pose()
    #         rate.sleep()
    # except KeyboardInterrupt:
    #     rospy.logwarn("Keyboard interrupt detected, stopping listener")