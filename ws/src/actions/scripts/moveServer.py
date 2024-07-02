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
        self.target_departure = 0.6
        self.target_approach = 0.25
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

        if goal.goal_type == moveActionGoal.FORWARD:
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
        goal_pose = self.tf_buffer.transform(goal, 'base_footprint', rospy.Duration(0.1))
        min_scanned_distance = float('inf')
        solve_angle = False
        solve_approach = False
        while min_scanned_distance > self.target_approach:
            if self.server.is_preempt_requested() or rospy.is_shutdown():
                rospy.loginfo('Preempted')
                self.server.set_preeumpted()
                self.success = False
                self.server.set_aborted()
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
            #self.server.publish_feedback('Distance to goal: {}'.format(distance))
            self.r.sleep()
    
    # REDO THIS FUNCTION
    def move_backward(self, cmd_vel):
        min_scanned_distance = self.min_range

        while True:
            min_scanned_distance = float('inf')
            
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

                min_scanned_distance = min(min_scanned_distance, self.scan_data.ranges[i])
            
            rospy.loginfo('Max distance: {}'.format(min_scanned_distance))
            #self.server.publish_feedback('Max distance: {}'.format(max_distance))
            self.r.sleep()

            if min_scanned_distance >= self.max_range:
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