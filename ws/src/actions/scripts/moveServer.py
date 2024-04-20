#! /usr/bin/env python3
import rospy    
import actionlib
import math
from actions.msg import moveActionAction, moveActionGoal, moveActionResult
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

DEBUG = False

class MoveServer(object):
    def __init__(self):
        self.scan_data = None
        self.max_range = 0.8
        self.min_range = 0.40
        self.x_vel = 0.05
        self.success = True
        self.server = actionlib.SimpleActionServer('moveServer', moveActionAction, self.execute, False)
        self.server.start()
        self.r = rospy.Rate(50)
        self.scan_sub = rospy.Subscriber('/dṕ2ls/scan', LaserScan, self.scan_callback, queue_size=1)
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
            self.move_forward(cmd_vel)
        elif goal.goal_type == moveActionGoal.BACKWARD:
            self.move_backward(cmd_vel)
        elif goal.goal_type == moveActionGoal.WAIT_FOR_DOOR:
            self.wait_for_door()
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

    def move_forward(self, cmd_vel):
        min_distance = self.max_range
        
        while True:
            min_distance = float('inf')
            if self.server.is_preempt_requested() or rospy.is_shutdown():
                rospy.loginfo('Preempted')
                self.server.set_preempted() 
                self.success = False
                self.server.set_aborted()
                return
            
            if not DEBUG:
                cmd_vel.linear.x = self.x_vel
                self.cmd_vel_pub.publish(cmd_vel)

            for i in range(self.init_index, self.end_index):
                if self.scan_data.ranges[i] == float('inf') or self.scan_data.ranges[i] == 0:
                    continue

                min_distance = min(min_distance, self.scan_data.ranges[i])
            
            rospy.loginfo('Min distance: {}'.format(min_distance))
            #self.server.publish_feedback('Min distance: {}'.format(min_distance))
            self.r.sleep()

            if min_distance <= self.min_range:
                break
        
        #self.server.publish_feedback("Achieved min distance")
    
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

    def wait_for_door(self):
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

    
if __name__ == '__main__':
    rospy.init_node('moveServer')
    server = MoveServer()
    rospy.spin()