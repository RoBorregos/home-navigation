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

BASE_PATH = str(pathlib.Path(__file__).parent) + '/../../../../'

placesPoses = {
 "BATHROOM": Pose(Point(x=-0.3963, y=3.802, z=0.0), Quaternion(x=0.0, y=0.0, z=0.4664, w=0.8845)),
 "CLOSET": Pose(Point(x=-1.9308, y=2.7578, z=0.0), Quaternion(x=0.0, y=0.0, z=-0.6844, w=0.729)),
 "COUCH": Pose(Point(x=0.0496, y=0.1873, z=0.0), Quaternion(x=0.0, y=0.0, z=0.0299, w=0.999)),
 "KITCHEN": Pose(Point(x=-1.256, y=1.5809, z=0.0), Quaternion(x=0.0, y=0.0, z=-0.9992, w=0.0392)),
 "HOME": Pose(Point(x=-4.344, y=0.579, z=0.0), Quaternion(x=0.0, y=0.0, z=0.679, w=0.733)),
}

class navigationServer(object):

    def __init__(self, name):
        self._action_name = name
        rospy.loginfo(name)

        rospy.loginfo("Waiting for MoveBase AS...")
        self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_client.wait_for_server()
        rospy.loginfo("MoveBase AS Loaded ...")

        # Localization
        #rospy.loginfo("Global Localization")
        #rospy.wait_for_service('/global_localization')
        #global_localization = rospy.ServiceProxy('/global_localization', Empty)
        #global_localization()
        #self.rotate()

        # Initialize Navigation Action Server
        self._as = actionlib.SimpleActionServer(self._action_name, actions.msg.navServAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
    
    def execute_cb(self, goal):
        target = goal.target_location
        rospy.loginfo("Robot Moving Towards " + target)
        self.send_goal(placesPoses[target])
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