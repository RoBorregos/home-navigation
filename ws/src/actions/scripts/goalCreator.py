#!/usr/bin/python3

import rospy

from tf2_ros import Buffer, TransformListener
from actions.srv import CreateGoal
from geometry_msgs.msg import PoseStamped


class GoalCreator:
    def __init__(self) -> None:
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        self.goal_creator_service = rospy.Service(
            "/create_goal", CreateGoal, self.create_goal
        )

        rospy.loginfo("Goal Creator Service is ready")

    # Receive target pose and offset to goal
    def create_goal(self, req: CreateGoal):
        target: PoseStamped = req.target
        x_offset: float = req.x_offset
        y_offset: float = req.y_offset

        try:
            self.tf_buffer.transform(target, "base_footprint")
        except:
            print("Error on transform")
            return
        
        target.pose.position.x += x_offset
        target.pose.position.y += y_offset

        try:
            self.tf_buffer.transform(target, "map")
        except:
            print("Error on transform")
            return

        return target.pose


if __name__ == "__main__":
    rospy.init_node("goal_creator", anonymous=False)
    goalCreator = GoalCreator()

    rospy.spin()
