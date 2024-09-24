#!/usr/bin/python3

import rospy
from frida_navigation_interfaces.srv import CreateGoal
from geometry_msgs.msg import PoseStamped


class GoalCreator:
    def __init__(self) -> None:
        self.goal_creator_service = rospy.ServiceProxy(
            "/create_goal", CreateGoal
        )
        self.goal_creator_service.wait_for_service()

        self.og_pub = rospy.Publisher("/og_goal", PoseStamped, queue_size=10)
        self.pub = rospy.Publisher("/test_goal", PoseStamped, queue_size=10)

        rospy.loginfo("Goal Creator Service is ready")

    # Receive target pose and offset to goal
    def call_service(self):
        target = PoseStamped()
        target.header.frame_id = "laser_frame"
        target.pose.position.x = 0.0
        target.pose.position.y = 0.0
        target.pose.position.z = 0.0
        target.pose.orientation.x = 0.0
        target.pose.orientation.y = 0.0
        target.pose.orientation.z = 0.0
        target.pose.orientation.w = 1.0

        self.og_pub.publish(target)

        x_offset = 1.0
        y_offset = 0.0

        try:
            response = self.goal_creator_service(target, x_offset, y_offset)
            print(response)
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return

        self.pub.publish(response.goal)

if __name__ == "__main__":
    rospy.init_node("goal_caller", anonymous=False)
    goalCreator = GoalCreator()

    while not rospy.is_shutdown():
        goalCreator.call_service()
        rospy.sleep(1)


