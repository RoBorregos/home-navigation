#! /usr/bin/env python3
import json
import rospy
import actionlib
from frida_navigation_interfaces.msg import navServAction, navServGoal
from geometry_msgs.msg import PoseStamped
from enum import Enum
import pathlib

BASE_PATH = str(pathlib.Path(__file__).parent) + "/../../map_contextualizer/scripts"

def handleIntInput(msg_ = "", range=(0, 10)):
    x = -1
    while x < range[0] or x > range[1]:
        print(msg_)
        while True:
            x = input()

            if x and x.isnumeric():
                break
        x = int(x)
    return 

class NavClient(object):
    
    def __init__(self):
        self.client = actionlib.SimpleActionClient('navServer', navServAction)
        self.client.wait_for_server()
        msg, count = self.createMsg()
        while True:
            x = int(input(msg + "0. Exit\n"))
            if x == 0 or x > count:
                break

            #rospy.loginfo(f"Moving to {self.MoveGoals[x]}")
            self.nav_goal(self.MoveGoals[x] if x in self.MoveGoals else "")

    def createMsg(self):
        self.MoveGoals = {}
        count = 1
        msg = ""
        with open(BASE_PATH + '/areas.json') as json_file:
            data = json.load(json_file)
            for key in data:
                for subkey in data[key]:
                    if len (data[key][subkey]) == 0:
                        continue
                    msg += f"{count}. {key} {subkey}\n"
                    self.MoveGoals[count] = key + " " + subkey
                    count += 1
        
        return msg, count

    def nav_goal(self, target):
        class NavGoalScope:
            target_location = target
            result = False
            pose = PoseStamped()
            
            result_received = False
        
        def nav_goal_feedback(feedback_msg):
            NavGoalScope.pose = feedback_msg.pose
        
        def get_result_callback(state, result):
            NavGoalScope.result = result.result

            NavGoalScope.result_received = True
            rospy.loginfo("Nav Goal Finished")

        rospy.loginfo("Sending Nav Goal")
        self.client.send_goal(
                    navServGoal(target_location = NavGoalScope.target_location, goal_type = navServGoal.BACKWARD,  target_pose = NavGoalScope.pose),
                    feedback_cb=nav_goal_feedback,
                    done_cb=get_result_callback)
        
        while not NavGoalScope.result_received:
            pass
        
        return NavGoalScope.result

if __name__ == '__main__':
    try:
        rospy.init_node('NavGoalClient', anonymous=True)
        rospy.loginfo("NavGoalClient initialized.")
        rospy.loginfo(BASE_PATH)
        NavClient()

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
