#! /usr/bin/env python3
import rospy 
import actionlib
from actions.msg import moveActionAction, moveActionGoal

class MoveClient(object):
    def __init__(self):
        self.client = actionlib.SimpleActionClient('moveServer', moveActionAction)
        self.client.wait_for_server()
        self.goal = moveActionGoal()
        self.goal.goal_type = moveActionGoal.BACKWARD
        self.client.send_goal(self.goal)
        self.client.wait_for_result()
        rospy.loginfo('Goal reached')

if __name__ == '__main__':
    rospy.init_node('moveClient')
    MoveClient()
    rospy.spin()