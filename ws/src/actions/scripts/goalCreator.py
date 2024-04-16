#!/usr/bin/python3

import rospy

import tf2_ros
from actions.srv import CreateGoal
from tf2_geometry_msgs import PoseStamped


class GoalCreator:
    def __init__(self) -> None:
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.goal_creator_service = rospy.Service(
            "/create_goal", CreateGoal, self.create_goal
        )

        self.test_pub = rospy.Publisher("/test_base", PoseStamped, queue_size=10)

        rospy.loginfo("Goal Creator Service is ready")

    def create_goal(self, req: CreateGoal):
        x_offset: float = req.x_offset
        y_offset: float = req.y_offset
    
        pose = PoseStamped()
        pose.header.frame_id = req.target.header.frame_id
        pose.pose.position.x = req.target.pose.position.x
        pose.pose.position.y = req.target.pose.position.y
        pose.pose.position.z = req.target.pose.position.z
        pose.pose.orientation.x = req.target.pose.orientation.x
        pose.pose.orientation.y = req.target.pose.orientation.y
        pose.pose.orientation.z = req.target.pose.orientation.z
        pose.pose.orientation.w = req.target.pose.orientation.w

        try:
            base_pose = self.tf_buffer.transform(pose, "base_footprint")
            print(f"Transformed point: ({base_pose})")
        except tf2_ros.LookupException:
            print("Transform lookup failed.")
            return
        
        base_footprint_target = PoseStamped()
        base_footprint_target.header.frame_id = "base_footprint"
        base_footprint_target.pose.position.x = base_pose.pose.position.x + x_offset
        base_footprint_target.pose.position.y = base_pose.pose.position.y + y_offset
        base_footprint_target.pose.position.z = base_pose.pose.position.z
        base_footprint_target.pose.orientation.x = base_pose.pose.orientation.x
        base_footprint_target.pose.orientation.y = base_pose.pose.orientation.y
        base_footprint_target.pose.orientation.z = base_pose.pose.orientation.z
        base_footprint_target.pose.orientation.w = base_pose.pose.orientation.w

        try:
            target_pose = self.tf_buffer.transform(base_footprint_target, "map")
        except tf2_ros.LookupException:
            print("Transform lookup failed.")
            return

        self.test_pub.publish(target_pose)            
        print(f"Final target: ({target_pose})")
        return target_pose


if __name__ == "__main__":
    rospy.init_node("goal_creator", anonymous=False)
    goalCreator = GoalCreator()

    rospy.spin()
