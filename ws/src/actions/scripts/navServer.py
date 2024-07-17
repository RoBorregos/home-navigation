#! /usr/bin/env python3

import json
import math
import tf
import time
import numpy
from tf.transformations import quaternion_from_euler
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
import pathlib
import actionlib
import rospy
from sensor_msgs.msg import LaserScan
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseStamped, Twist
import tf2_geometry_msgs
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from frida_navigation_interfaces.msg import navServAction, navServGoal, navServResult
import tf2_ros
from sklearn.linear_model import RANSACRegressor

BASE_PATH = str(pathlib.Path(__file__).parent) + "/../../map_contextualizer/scripts"
DEBUG = False

TARGET_APPROACH = 0.3
TARGET_DEPARTURE = 0.5

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z

class navigationServer(object):

    def __init__(self, name):
        self._action_name = name
        rospy.loginfo(name)

        self.ransac_model = RANSACRegressor()

        self.visual_scan_data = None
        self.lidar_scan_data = None
        self.robot_pose = None
        self.x_vel = 0.2
        self.angular_vel = 0.05
        self.success = True
        self.visual_scanner = rospy.get_param('~visual_scanner', '/zed2/scan')
        self.lidar_scanner = rospy.get_param('~lidar_scanner', '/scan')
        rospy.loginfo("Approach scanner topic: " + self.visual_scanner)
        rospy.loginfo("Waiting for MoveBase AS...")
        
        self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.robot_pose_sub = rospy.Subscriber('/robot_pose', Pose, self.robot_pose_callback, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.visual_scan_sub = rospy.Subscriber(self.visual_scanner, LaserScan, self.visual_scan_callback, queue_size=1)
        self.lidar_scan_sub = rospy.Subscriber(self.lidar_scanner, LaserScan, self.lidar_scan_callback, queue_size=1)

        self.move_client.wait_for_server()
        rospy.loginfo("MoveBase AS Loaded ...")

        self.r = rospy.Rate(50)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.initPlaces()

        # Initialize Navigation Action Server
        self._as = actionlib.SimpleActionServer(self._action_name, navServAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
    

    def initPlaces(self):
        # create a dictionary of places and their poses using the json file which has the following format
        # key : {subkey_1 : pose1, subkey_2 : pose2, ...}
        # where pose is a list of 6 elements [x, y, z, qx, qy, qz, qw]

        self.placesPoses = {}
        with open(BASE_PATH + '/areas.json') as json_file:
            data = json.load(json_file)
            for key in data:
                self.placesPoses[key] = {}
                for subkey in data[key]:
                    if len(data[key][subkey]) == 0:
                        continue
                    self.placesPoses[key][subkey] = PoseStamped()
                    self.placesPoses[key][subkey].header.frame_id = "map"
                    self.placesPoses[key][subkey].pose.position.x = data[key][subkey][0]
                    self.placesPoses[key][subkey].pose.position.y = data[key][subkey][1]
                    self.placesPoses[key][subkey].pose.position.z = data[key][subkey][2]
                    self.placesPoses[key][subkey].pose.orientation.x = data[key][subkey][3]
                    self.placesPoses[key][subkey].pose.orientation.y = data[key][subkey][4]
                    self.placesPoses[key][subkey].pose.orientation.z = data[key][subkey][5]
                    self.placesPoses[key][subkey].pose.orientation.w = data[key][subkey][6]
    
    def execute_cb(self, goal):
        rospy.loginfo("[INFO] Executing goal")
        print (goal)

        target = str(goal.target_location)
        goal_type = goal.goal_type
        goal_pose = self.get_goal(target) if target != "" else goal.target_pose
        if goal_pose is None:
            rospy.loginfo("[ERROR] Invalid target")
            self._as.set_succeeded(navServResult(result=False))
            return
        
        cmd_vel = Twist()
        if (goal_type == goal.NAV_MODE):
            if goal_pose is None:
                rospy.loginfo("[ERROR] Invalid target")
                self._as.set_succeeded(navServResult(result=False))
            
            self.send_goal(goal_pose)
            rospy.loginfo("[INFO] Robot Moving Towards " + target if target != "" else goal.target_pose)
            self._as.set_succeeded(navServResult(result=True))
        elif (goal_type == goal.FORWARD):
            if goal_pose is None:
                rospy.loginfo("[ERROR] Invalid target")
                self._as.set_succeeded(navServResult(result=False))

            self.move_forward(cmd_vel, goal_pose, target_approach=TARGET_APPROACH if goal.target_approach == 0.0 else goal.target_approach)
            rospy.loginfo("[INFO] Robot approached " + target)
            self._as.set_succeeded(navServResult(result=self.success))
        elif (goal_type == goal.BACKWARD):
            self.move_backward(cmd_vel, target_departure=TARGET_DEPARTURE if goal.target_departure == 0.0 else goal.target_departure)
            rospy.loginfo("[INFO] Robot moved backward")
            self._as.set_succeeded(navServResult(result=self.success))
        elif (goal_type == goal.DOOR_SIGNAL):
            self.door_signal()
            rospy.loginfo("[INFO] Detected door signal")
            self._as.set_succeeded(navServResult(result=self.success))
        else:
            rospy.loginfo("[ERROR] Invalid goal type")
            self._as.set_succeeded(navServResult(result=False))
         

    def get_goal(self, target : str):
        keys = target.split(" ")

        if (len(keys) <= 1):
            if (keys[0] in self.placesPoses):
                rospy.loginfo(f"Robot Moving Towards Safe Place POSE: {self.placesPoses[keys[0]]['safe_place']}")
                return self.placesPoses[keys[0]]['safe_place']
            else: 
                for key in self.placesPoses:
                    if keys[0] in self.placesPoses[key]:
                        rospy.loginfo(f"Robot Moving Towards Safe Place POSE: {self.placesPoses[keys[0]]['safe_place']}")
                        return self.placesPoses[keys[0]]['safe_place']
        elif (len(keys) <= 2 and keys[0] in self.placesPoses and keys[1] in self.placesPoses[keys[0]]):
            rospy.loginfo(f"Robot Moving Towards Safe Place POSE: {self.placesPoses[keys[0]][keys[1]]}")
            return self.placesPoses[keys[0]][keys[1]]
        
        rospy.loginfo("Invalid target")
        return None

    def send_goal(self, target_pose):
        goal = MoveBaseGoal()
        goal.target_pose = target_pose
        goal.target_pose.header.stamp = rospy.Time.now()
        self.move_client.send_goal(goal)
        self.move_client.wait_for_result()
    
    def rotate(self):
        velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        vel_msg = Twist()
        
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0.16

        t0 = time.time()

        while not rospy.is_shutdown() and time.time() - t0 < 15:
            velocity_publisher.publish(vel_msg)
            rospy.sleep(1/50)
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)

    def move_forward(self, cmd_vel : Twist, goal : PoseStamped, target_approach : float):
        if self.robot_pose is None or self.visual_scan_data is None:
            if self.robot_pose is None:
                rospy.loginfo("Failed to get robot pose")
            if self.visual_scan_data is None:
                rospy.loginfo("Failed to get scan data")
            self.success = False
            return
        # conver goal to a transfor
        goal_roll, goal_pitch, goal_yaw = euler_from_quaternion(goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w)
       
        try:
            transform = self.tf_buffer.lookup_transform('base_footprint', 'map', rospy.Time())
        except:
            rospy.loginfo("Failed to get transform")
            return

        
        min_scanned_distance = float('inf')
         
        while min_scanned_distance > target_approach:
            try:
                goal_pose = self.tf_buffer.transform(goal, 'base_footprint', rospy.Duration(2.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.loginfo(e)
                continue

            if self._as.is_preempt_requested() or rospy.is_shutdown():
                rospy.loginfo('Preempted')
                self.success = False
                return
            
            x_distance = []
            y_distance = []
            for inx, data in enumerate(self.visual_scan_data.ranges):
                if data == float('inf'):
                    continue

                ray_angle = self.visual_scan_data.angle_min + inx * self.visual_scan_data.angle_increment
                if ray_angle < -(3 * math.pi) / 4 or ray_angle > (3 * math.pi) / 4:
                    continue
                x_distance.append(data * math.sin(ray_angle))
                y_distance.append(data * math.cos(ray_angle))

            # Fit the RANSAC model
            x_distance = numpy.array(x_distance).reshape(-1, 1)
            y_distance = numpy.array(y_distance).reshape(-1, 1)
            self.ransac_model.fit(x_distance, y_distance)

            curr_scanned_distance = self.ransac_model.predict(numpy.array([[0]]))[0][0]
                        
            angle = math.atan2(goal_pose.pose.position.y, goal_pose.pose.position.x)
            #robot_roll, robot_pitch, robot_yaw = euler_from_quaternion(self.robot_pose.orientation.x, self.robot_pose.orientation.y, self.robot_pose.orientation.z, self.robot_pose.orientation.w)
            
            error_angle_left = angle
            error_angle_right = error_angle_left - 2 * math.pi if error_angle_left > 0 else error_angle_left + 2 * math.pi

            error_angle = error_angle_left if abs(error_angle_left) < abs(error_angle_right) else error_angle_right 

            if abs(error_angle) > 0.1:
                cmd_vel.angular.z = self.angular_vel * (error_angle / abs(error_angle))
                cmd_vel.linear.x = 0
            else:
                cmd_vel.linear.x = self.x_vel
                cmd_vel.angular.z = 0                

            if not DEBUG:
                self.cmd_vel_pub.publish(cmd_vel)

            rospy.loginfo('Scanned distance: {} Error angle: {}'.format(curr_scanned_distance, error_angle))

            min_scanned_distance = curr_scanned_distance
            # if min_scanned_distance <= curr_scanned_distance:
            #     min_scanned_distance = curr_scanned_distance
            # else:
            #     rospy.loginfo('Blocked')
            #     self.success = False
            #     return
            #self._as.publish_feedback('Distance to goal: {}'.format(distance))
            self.r.sleep()

        # Adjust to the goal's orientation
        robot_roll, robot_pitch, robot_yaw = euler_from_quaternion(self.robot_pose.orientation.x, self.robot_pose.orientation.y, self.robot_pose.orientation.z, self.robot_pose.orientation.w)
        error_angle_left = goal_yaw - robot_yaw
        error_angle_right = error_angle_left - 2 * math.pi if error_angle_left > 0 else error_angle_left + 2 * math.pi
        error_angle = error_angle_left if abs(error_angle_left) < abs(error_angle_right) else error_angle_right 

        while abs(error_angle) > 0.1:
            if self._as.is_preempt_requested() or rospy.is_shutdown():
                rospy.loginfo('Preempted')
                self._as.set_preeumpted()
                self.success = False
                self._as.set_aborted()
                return
            
            cmd_vel.angular.z = self.angular_vel * (error_angle / abs(error_angle))
            cmd_vel.linear.x = 0
            rospy.loginfo('Error angle: {}'.format(error_angle))
            robot_roll, robot_pitch, robot_yaw = euler_from_quaternion(self.robot_pose.orientation.x, self.robot_pose.orientation.y, self.robot_pose.orientation.z, self.robot_pose.orientation.w)
            error_angle_left = goal_yaw - robot_yaw
            error_angle_right = error_angle_left - 2 * math.pi if error_angle_left > 0 else error_angle_left + 2 * math.pi
            error_angle = error_angle_left if abs(error_angle_left) < abs(error_angle_right) else error_angle_right 
            
            if not DEBUG:
                self.cmd_vel_pub.publish(cmd_vel)
            
            self.r.sleep()

        self.success = True


    
    # REDO THIS FUNCTION
    def move_backward(self, cmd_vel, target_departure):
        if self.visual_scan_data is None:
            if self.visual_scan_data is None:
                rospy.loginfo("Failed to get scan data")
            self.success = False
            return
        
        min_scanned_distance = 0.0

        while min_scanned_distance < target_departure:
            if self._as.is_preempt_requested() or rospy.is_shutdown():
                rospy.loginfo('Preempted')
                self.success = False
                return
            x_distance = []
            y_distance = []
            for inx, data in enumerate(self.visual_scan_data.ranges):
                if data == float('inf'):
                    continue
                
                ray_angle = self.visual_scan_data.angle_min + inx * self.visual_scan_data.angle_increment
                if ray_angle < -(3 * math.pi) / 4 or ray_angle > (3 * math.pi) / 4:
                    continue
                x_distance.append(data * math.sin(ray_angle))
                y_distance.append(data * math.cos(ray_angle))

            # Fit the RANSAC model
            x_distance = numpy.array(x_distance).reshape(-1, 1)
            y_distance = numpy.array(y_distance).reshape(-1, 1)
            self.ransac_model.fit(x_distance, y_distance)


            curr_scanned_distance = self.ransac_model.predict(numpy.array([[0]]))[0][0]

            rospy.loginfo('Scanned distance: {}'.format(curr_scanned_distance))
              
            cmd_vel.linear.x = -self.x_vel
            cmd_vel.angular.z = 0                

            if not DEBUG:
                self.cmd_vel_pub.publish(cmd_vel)
        
            min_scanned_distance = curr_scanned_distance
        
        self.success = True
        #self._as.publish_feedback("Achieved max distance")

    def door_signal(self):
        if self.lidar_scan_data is None:
            if self.lidar_scan_data is None:
                rospy.loginfo("Failed to get scan data")
            self.success = False
            return
        
        count = 1
        average = 0

        while True:
            if self._as.is_preempt_requested() or rospy.is_shutdown():
                rospy.loginfo('Preempted')
                self.success = False
                return
            
            
            x_distance = []
            y_distance = []
            cnt = 1
            for inx, data in enumerate(self.lidar_scan_data.ranges):
                if data == float('inf') or data == 0.0:
                    continue
                
                ray_angle = self.lidar_scan_data.angle_min + inx * self.lidar_scan_data.angle_increment
                if ray_angle < -(3 * math.pi) / 4 or ray_angle > (3 * math.pi) / 4:
                    continue
                # print (data, ray_angle)
                y_distance.append(data * math.cos(ray_angle))

            # Fit the RANSAC model
            y_distance = numpy.array(y_distance).reshape(-1, 1)
            mean = numpy.mean(y_distance)

            curr_scanned_distance = mean
            rospy.loginfo('Scanned distance: {}'.format(curr_scanned_distance))

            if count > 1 and abs(curr_scanned_distance - average) > 1:
                rospy.loginfo('Door signal detected')
                self.success = True
                break

            average = (curr_scanned_distance + average) / count
            count += 1
            self.r.sleep()

    def visual_scan_callback(self, data):
        self.visual_scan_data = data

    def lidar_scan_callback(self, data):
        self.lidar_scan_data = data

    def robot_pose_callback(self, data):
        self.robot_pose = data

if __name__ == '__main__':
    rospy.init_node('navServer')
    server = navigationServer(rospy.get_name())
    rospy.spin()