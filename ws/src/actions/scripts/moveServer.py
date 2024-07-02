#! /usr/bin/env python3
import rospy    
import actionlib
import math
from actions.msg import moveActionAction, moveActionGoal, moveActionResult
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped, Pose, TransformStamped
import tf2_ros

DEBUG = True

class MoveServer(object):
    def __init__(self):
        self.scan_data = None
        self.robot_pose = None
        self.max_range = 0.8
        self.min_range = 0.40
        self.x_vel = 0.05
        self.angular_vel = 0.05
        self.success = True
        self.server = actionlib.SimpleActionServer('moveServer', moveActionAction, self.execute, False)
        self.server.start()
        self.r = rospy.Rate(50)
        self.tf_buffer = tf2_ros.Buffer()
        self.scan_sub = rospy.Subscriber('/oakd/scan', LaserScan, self.scan_callback, queue_size=1)
        self.robot_pose_sub = rospy.Subscriber('/robot_pose', PoseStamped, self.robot_pose_callback, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.loginfo('Move server initialized')

    def execute(self, goal):
        while self.scan_data is None:
            rospy.loginfo('Waiting for scan data')
            self.r.sleep()
            if rospy.is_shutdown():
                return
        
        ##self.server.publish_feedback(str("Analizing LaserScan"))

        self.init_index = int((-(math.pi / 4) - self.scan_data.angle_min) / self.scan_data.angle_increment)
        self.end_index = int(((math.pi / 4) - self.scan_data.angle_min) / self.scan_data.angle_increment) + 1
        cmd_vel = Twist()

        if goal.goal_type == moveActionGoal.APROACH:
            self.move_approach(cmd_vel=cmd_vel, goal=goal)
        elif goal.goal_type == moveActionGoal.BACKWARD:
            self.move_backward(cmd_vel)
        elif goal.goal_type == moveActionGoal.DOOR_SIGNAL:
            self.door_signal()
        else:
            rospy.loginfo('Invalid goal type')
            self.server.set_aborted()
            return
        
        if not DEBUG:
            cmd_vel.linear.x = self.x_vel
            self.cmd_vel_pub.publish(cmd_vel)
            
        if self.success:
            rospy.loginfo('Goal reached')
            self.server.set_succeeded(moveActionResult(result=True))
        else:
            rospy.loginfo('Goal not reached')
            self.server.set_aborted()

        return

    def move_approach(self, cmd_vel : Twist, goal : PoseStamped):
        tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        tf = TransformStamped()
        tf.header.stamp = rospy.Time.now()
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'goal'
        tf.transform.translation.x = goal.pose.position.x
        tf.transform.translation.y = goal.pose.position.y
        tf.transform.translation.z = 0
        tf.transform.rotation.x = goal.pose.orientation.x
        tf.transform.rotation.y = goal.pose.orientation.y
        tf.transform.rotation.z = goal.pose.orientation.z
        tf.transform.rotation.w = goal.pose.orientation.w

        tf_broadcaster.sendTransform(tf)

        try: 
            self.tf_buffer.lookup_transform('map', 'goal', rospy.Time())
        except Exception as e:
            print(e)
            rospy.logerr("error on transformation lookup")
            self.r.sleep()
            return
        
        try:
            robot_from_table = self.tf_buffer.transform(self.robot_pose, 'goal', rospy.Duration(1))
        except Exception as e:
            print(e)
            rospy.logerr("Failed to transform object pose from body_frame to map_frame")
            self.r.sleep()
            return

        # Get transformation matrix between goal poseStamped and robot poseStamped
        medium_pose_table = PoseStamped()
        medium_pose_table.header.stamp = rospy.Time.now()
        medium_pose_table.header.frame_id = 'map'
        medium_pose_table.pose.position.x = robot_from_table.pose.position.x
        medium_pose_table.pose.position.y = robot_from_table.pose.position.y * 0.5
        medium_pose_table.pose.position.z = 0
        medium_pose_table.pose.orientation.x = 0
        medium_pose_table.pose.orientation.y = 0
        medium_pose_table.pose.orientation.z = 0
        medium_pose_table.pose.orientation.w = 1

        try:
            medium_pose = self.tf_buffer.transform(medium_pose_table, 'map', rospy.Duration(1))
        except Exception as e:
            print(e)
            rospy.logerr("Failed to transform object pose from body_frame to map_frame")
            self.r.sleep()
            return

        path = []
        path.append(medium_pose)
        path.append(goal)

        solve_angle = True
        solve_approach = True
        distance = float('inf')
        goal_map = PoseStamped()

        while len(path) > 0 and solve_angle and solve_approach:
            if solve_angle and solve_approach:
                goal_map = path.pop(0)
                solve_angle = False
                solve_approach = False

            if self.server.is_preempt_requested() or rospy.is_shutdown():
                rospy.loginfo('Preempted')
                self.server.set_preeumpted()
                self.success = False
                self.server.set_aborted()
                return
            
            if not solve_angle:
                angle = math.atan2(goal_map.pose.position.y - self.robot_pose.pose.position.y, goal_map.pose.position.x - self.robot_pose.pose.position.x)
                
                if abs(angle - self.robot_pose.pose.orientation.z) > 0.1:
                    cmd_vel.angular.z = self.angular_vel if angle > self.robot_pose.pose.orientation.z else -self.angular_vel
                else:
                    cmd_vel.angular.z = 0
                    solve_angle = True
            elif not solve_approach:
                distance = math.sqrt((goal_map.pose.position.x - self.robot_pose.pose.position.x) ** 2 + (goal_map.pose.position.y - self.robot_pose.pose.position.y) ** 2)
                if distance > 0.05:
                    cmd_vel.linear.x = self.x_vel
                else:
                    cmd_vel.linear.x = 0
                    solve_approach = True

            if not DEBUG:
                self.cmd_vel_pub.publish(cmd_vel)
            #self.server.publish_feedback('Distance to goal: {}'.format(distance))
            self.r.sleep()
    
    # REDO THIS FUNCTION
    def move_backward(self, cmd_vel):
        min_distance = self.min_range

        while True:
            min_distance = float('inf')
            
            if self.server.is_preempt_requested() or rospy.is_shutdown():
                rospy.loginfo('Preempted')
                self.server.set_preempted()
                self.success = False
                self.server.set_aborted()
                return
            
            if not DEBUG:
                cmd_vel.linear.x = -self.x_vel
                self.cmd_vel_pub.publish(cmd_vel)

            for i in range(self.init_index, self.end_index):
                if self.scan_data.ranges[i] == float('inf'):
                    continue

                min_distance = min(min_distance, self.scan_data.ranges[i])
            
            rospy.loginfo('Max distance: {}'.format(min_distance))
            #self.server.publish_feedback('Max distance: {}'.format(max_distance))
            self.r.sleep()

            if min_distance >= self.max_range:
                break
        
        #self.server.publish_feedback("Achieved max distance")

    def door_signal(self):
        mean = 0
        while True:
            if self.server.is_preempt_requested() or rospy.is_shutdown():
                rospy.loginfo('Preempted')
                self.server.set_preempted()
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

            #self.server.publish_feedback('Mean distance: {}'.format(mean))
            self.r.sleep()


    def scan_callback(self, data):
        rospy.loginfo('Scan data received')
        self.scan_data = data

    def robot_pose_callback(self, data):
        rospy.loginfo('Robot pose received')
        self.robot_pose = data

    
if __name__ == '__main__':
    rospy.init_node('moveServer')
    server = MoveServer()
    rospy.spin()