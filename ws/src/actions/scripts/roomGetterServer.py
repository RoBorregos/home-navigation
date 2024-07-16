#! /usr/bin/env python3

import rospy
import json
from geometry_msgs.msg import PointStamped
import numpy as np
import cv2 as cv
from frida_navigation_interfaces.srv import RoomGetter
import pathlib
import copy
import tf2_ros
import tf2_geometry_msgs
import os

BASE_PATH = str(pathlib.Path(__file__).parent)


class RoomGetterServer:
    def __init__(self) -> None:
        self.room_getter_service = rospy.Service(
            "/room_getter", RoomGetter, self.get_room_from_map
        )

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.map_file = os.path.join(BASE_PATH ,"map_dimension.json")
        self.mask = None
        self.scale = 100
        self.room = dict()
        self.room[0] = "Unknown"
        
        self.load_data()
        self.load_map()
        self.load_rooms()
        # self.see_mask_cv()

        rospy.loginfo("Room Getter Service is ready")
    
    def load_data(self):
        with open(self.map_file, "r") as f:
            self.data = json.load(f)
    
    def load_map(self):
        map_data = self.data["map"]
        x_min, x_max, y_min, y_max = float("inf"), -float("inf"), float("inf"), -float("inf")

        for point in map_data:
            x, y = point[0], point[1]
            x_min = min(x_min, x)
            x_max = max(x_max, x)
            y_min = min(y_min, y)
            y_max = max(y_max, y)
        
        x_min*=self.scale
        x_max*=self.scale
        y_min*=self.scale
        y_max*=self.scale

        self.mask = np.zeros((int(y_max-y_min), int(x_max-x_min)), dtype=np.uint8)
        self.room_mask = np.zeros((int(y_max-y_min), int(x_max-x_min)), dtype=np.uint8)

        rospy.loginfo(f"Map dimensions: {x_min, x_max, y_min, y_max}")

    def load_rooms(self):
        current_room_id = 1
        original_mask = copy.deepcopy(self.mask)
        for room in self.data:
            if room == "map":
                continue

            current_points = []
            copy_mask = copy.deepcopy(original_mask)
            self.room[current_room_id] = room
            for point in self.data[room]:
                x, y = point[0], point[1]
                x*=self.scale
                y*=self.scale
                # Denormalize the points
                x-=self.scale*min(self.data["map"], key=lambda x: x[0])[0]
                y-=self.scale*min(self.data["map"], key=lambda x: x[1])[1]
                current_points.append([int(x),int(y)])

            cv.fillConvexPoly(copy_mask, np.array([current_points]), 255)
            # FLip the mask
            copy_mask = cv.flip(copy_mask, 0)
            copy_mask = cv.threshold(copy_mask, 0, 255, cv.THRESH_BINARY)[1]

            self.see_mask_cv(copy_mask)    

            indexes = np.where(copy_mask == 255)
            self.mask[indexes] = 255
            self.room_mask[indexes] = current_room_id
            current_room_id+=1
        cv.destroyAllWindows()

    def see_mask_cv(self, IMG):
        cv.imshow("mask", IMG)
        cv.waitKey(0)
        cv.destroyAllWindows
        

    def get_room_from_map(self, goal) -> str:
        try:
            self.tfBuffer.lookup_transform("map", goal.header.frame_id, rospy.Time())
        except Exception as e:
            rospy.logerr(e)
            return "Unknown"

        try:
            goal = self.tfBuffer.transform(goal, "map")
        except Exception as e:
            rospy.logerr(e)
            return "Unknown"
        
        # Denormalize the points
        x = goal.object_point.point.x * self.scale
        y = goal.object_point.point.y * self.scale

        x-=self.scale*min(self.data["map"], key=lambda x: x[0])[0]
        y-=self.scale*min(self.data["map"], key=lambda x: x[1])[1]
        # Invert the y coordinate
        y = self.mask.shape[0] - y

        # Ensure coordinates are within bounds
        x = int(max(0, min(self.mask.shape[1] - 1, x)))
        y = int(max(0, min(self.mask.shape[0] - 1, y)))

        room_id = self.room_mask[int(y), int(x)]

        room = self.room[room_id]

        return str(room)
    
if __name__ == "__main__":
    rospy.init_node("room_getter")
    RoomGetterServer()
    rospy.spin()