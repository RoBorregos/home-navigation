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
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actions.msg
from actions.msg import navServAction, navServGoal, navServResult
import tf2_ros

BASE_PATH = str(pathlib.Path(__file__).parent) + "/../../map_contextualizer/scripts"
DEBUG = False
class navigationServer(object):

    def __init__(self, name):
        self._action_name = name
        rospy.loginfo(name)

        self.scan_data = None
        self.robot_pose = None
        self.target_departure = 0.6
        self.target_approach = 0.25
        self.x_vel = 0.05
        self.angular_vel = 0.05

        rospy.loginfo("Waiting for MoveBase AS...")
        self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.robot_pose_sub = rospy.Subscriber('/robot_pose', PoseStamped, self.robot_pose_callback, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.move_client.wait_for_server()
        rospy.loginfo("MoveBase AS Loaded ...")

        self.r = rospy.Rate(50)
        self.tf_buffer = tf2_ros.Buffer()
        self.scan_sub = rospy.Subscriber('/oakd/scan', LaserScan, self.scan_callback, queue_size=1)

        self.initPlaces()

        # Initialize Navigation Action Server
        self._as = actionlib.SimpleActionServer(self._action_name, actions.msg.navServAction, execute_cb=self.execute_cb, auto_start = False)
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
                    self.placesPoses[key][subkey] = Pose(
                                                        Point(
                                                            x=data[key][subkey][0], 
                                                            y=data[key][subkey][1], 
                                                            z=data[key][subkey][2]), 
                                                        Quaternion(
                                                            x=data[key][subkey][3], 
                                                            y=data[key][subkey][4], 
                                                            z=data[key][subkey][5], 
                                                            w=data[key][subkey][6]))

    
    def execute_cb(self, goal):
        target = str(goal.target_location)
        goal_type = str(goal.goal_type)
        goal_pose = self.get_goal(target)
        if (goal_type == goal.NAV_MODE):
            if goal_pose is None:
                rospy.loginfo("Invalid target")
                self._as.set_succeeded(navServResult(result=False))
            
            self.send_goal(goal_pose)
            rospy.loginfo("Robot Moving Towards " + target)
            self._as.set_succeeded(navServResult(result=True))
        elif (goal_type == goal.FORWARD):
            if goal_pose is None:
                rospy.loginfo("Invalid target")
                self._as.set_succeeded(navServResult(result=False))

            self.move_forward(Twist(), goal_pose)
            rospy.loginfo("Robot approached " + target)
            self._as.set_succeeded(navServResult(result=True))
        elif (goal_type == goal.BACKWARD):
            self.move_backward(Twist())
            rospy.loginfo("Robot moved backward")
            self._as.set_succeeded(navServResult(result=True))
        elif (goal_type == goal.DOOR_SIGNAL):
            self.door_signal()
            rospy.loginfo("Detected door signal")
            self._as.set_succeeded(navServResult(result=True))
         

    def get_goal(self, target : str):
        if (target == ""):
            return None

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
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose = target_pose
        goal.target_pose = pose_stamped
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

    def move_forward(self, cmd_vel : Twist, goal : PoseStamped):
        goal_pose = self.tf_buffer.transform(goal, 'base_footprint', rospy.Duration(0.1))
        min_scanned_distance = float('inf')
        solve_angle = False
        solve_approach = False
        while min_scanned_distance > self.target_approach:
            if self._as.is_preempt_requested() or rospy.is_shutdown():
                rospy.loginfo('Preempted')
                self._as.set_preeumpted()
                self.success = False
                self._as.set_aborted()
                return
            
            curr_scanned_distance = float('inf') 

            for i in range(self.init_index, self.end_index):
                if self.scan_data.ranges[i] == float('inf'):
                    continue
                curr_scanned_distance = min(curr_scanned_distance, self.scan_data.ranges[i])
            
            rospy.loginfo('Scanned distance: {}'.format(curr_scanned_distance))

            if not solve_angle:
                angle = math.atan2(goal_pose.pose.position.y - self.robot_pose.pose.position.y, goal_pose.pose.position.x - self.robot_pose.pose.position.x)
                
                if abs(angle - self.robot_pose.pose.orientation.z) > 5:
                    cmd_vel.angular.z = self.angular_vel if angle > self.robot_pose.pose.orientation.z else -self.angular_vel
                else:
                    cmd_vel.angular.z = 0
                    solve_angle = True
            elif not solve_approach:
                distance = math.sqrt((goal_pose.pose.position.x - self.robot_pose.pose.position.x) ** 2 + (goal_pose.pose.position.y - self.robot_pose.pose.position.y) ** 2)
                if distance > 0.05:
                    cmd_vel.linear.x = self.x_vel
                else:
                    cmd_vel.linear.x = 0
                    solve_approach = True


            if not DEBUG:
                self.cmd_vel_pub.publish(cmd_vel)


            if min_scanned_distance <= curr_scanned_distance:
                min_scanned_distance = curr_scanned_distance
            else:
                rospy.loginfo('Blocked')
                self._as.set_aborted()
                self.success = False
                return
            #self._as.publish_feedback('Distance to goal: {}'.format(distance))
            self.r.sleep()
    
    # REDO THIS FUNCTION
    def move_backward(self, cmd_vel):
        min_scanned_distance = self.min_range

        while True:
            min_scanned_distance = float('inf')
            
            if self._as.is_preempt_requested() or rospy.is_shutdown():
                rospy.loginfo('Preempted')
                self._as.set_preempted()
                self.success = False
                self._as.set_aborted()
                return
            
            if not DEBUG:
                cmd_vel.linear.x = -self.x_vel
                self.cmd_vel_pub.publish(cmd_vel)

            for i in range(self.init_index, self.end_index):
                if self.scan_data.ranges[i] == float('inf'):
                    continue

                min_scanned_distance = min(min_scanned_distance, self.scan_data.ranges[i])
            
            rospy.loginfo('Max distance: {}'.format(min_scanned_distance))
            #self._as.publish_feedback('Max distance: {}'.format(max_distance))
            self.r.sleep()

            if min_scanned_distance >= self.max_range:
                break
        
        #self._as.publish_feedback("Achieved max distance")

    def door_signal(self):
        mean = 0
        while True:
            if self._as.is_preempt_requested() or rospy.is_shutdown():
                rospy.loginfo('Preempted')
                self._as.set_preempted()
                self.success = False
                self.sever.set_aborted()
                break

            count = 0

            for i in range(self.init_index, self.end_index):
                if self.scan_data.ranges[i] == float('inf'):
                    continue
                count += 1
                ray_angle = self.scan_data.angle_min + i * self.scan_data.angle_increment
                mean += self.scan_data.ranges[i] * math.cos(ray_angle)
                rospy.loginfo('Mean distance: {}'.format(mean))
            
            mean /= count

            
            if mean >= 1:
                break

            #self._as.publish_feedback('Mean distance: {}'.format(mean))
            self.r.sleep()

    def scan_callback(self, data):
        rospy.loginfo('Scan data received')
        self.scan_data = data

    def robot_pose_callback(self, data):
        rospy.loginfo('Robot pose received')
        self.robot_pose = data

if __name__ == '__main__':
    rospy.init_node('navServer')
    server = navigationServer(rospy.get_name())
    rospy.spin()