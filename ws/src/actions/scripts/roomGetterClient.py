#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped
from frida_navigation_interfaces.srv import RoomGetter


def main():
    rospy.init_node("room_getter_client")
    rospy.loginfo("Room Getter client is ready")
    rospy.wait_for_service('room_getter')
    try:
        room_getter = rospy.ServiceProxy('room_getter', RoomGetter)
        goal = PointStamped()
        x= -6.475536823272705
        y= 4.268789291381836
        z= 0.0057659149169921875
        goal.point.x = x
        goal.point.y = y
        goal.point.z = 0
        goal.header.frame_id = "map"
        resp1 = room_getter(goal)
        print(f"Room: {resp1.room}")
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    rospy.spin()

if __name__ == "__main__":
    main()