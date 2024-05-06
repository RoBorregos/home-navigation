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
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actions.msg
from actions.msg import navServAction, navServGoal, navServResult

BASE_PATH = str(pathlib.Path(__file__).parent) + "/../../map_contextualizer/scripts"

class navigationServer(object):

    def __init__(self, name):
        self._action_name = name
        rospy.loginfo(name)

        rospy.loginfo("Waiting for MoveBase AS...")
        self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_client.wait_for_server()
        rospy.loginfo("MoveBase AS Loaded ...")

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

        if target == "":
            return

        keys = target.split(" ")

        if (len(keys) <= 1):
            if (keys[0] in self.placesPoses):
                rospy.loginfo(f"Robot Moving Towards Safe Place POSE: {self.placesPoses[keys[0]]['safe_place']}")
                self.send_goal(self.placesPoses[keys[0]]['safe_place'])
            else: 
                flag = False
                for key in self.placesPoses:
                    if keys[0] in self.placesPoses[key]:
                        rospy.loginfo(f"Robot Moving Towards Safe Place POSE: {self.placesPoses[keys[0]]['safe_place']}")
                        self.send_goal(self.placesPoses[keys[0]]['safe_place'])
                        flag = True
                        break
                
                if not flag:
                    rospy.loginfo("Invalid target")
                    self._as.set_succeeded(navServResult(result=False))
                    return
        elif (len(keys) <= 2 and keys[0] in self.placesPoses and keys[1] in self.placesPoses[keys[0]]):
            rospy.loginfo(f"Robot Moving Towards Safe Place POSE: {self.placesPoses[keys[0]][keys[1]]}")
            self.send_goal(self.placesPoses[keys[0]][keys[1]])    
        else: 
            rospy.loginfo("Invalid target")
            self._as.set_succeeded(navServResult(result=False))
            return
            

        rospy.loginfo("Robot Moving Towards " + target)
        self._as.set_succeeded(navServResult(result=True))

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

if __name__ == '__main__':
    rospy.init_node('navServer')
    server = navigationServer(rospy.get_name())
    rospy.spin()