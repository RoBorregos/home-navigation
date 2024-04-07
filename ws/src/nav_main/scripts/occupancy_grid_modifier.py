#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid

class OccupancyGridModifier:
    def __init__(self) -> None:
        self.rate = rospy.Rate(10)

        self.map_subscriber = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        
        self.modified_map_publisher = rospy.Publisher('/modified_map', OccupancyGrid, queue_size=10)

        self.map_width = 0
        self.map_height = 0


    def map_callback(self, map: OccupancyGrid):
        print(map.info)
        print(type(map.data))
        self.map_width = map.info.width
        self.map_height = map.info.height

        self.modify_map(map, 400, 400, 100)
        self.modify_map(map, 10, 10, 100)

        self.set_area_occupied(map, 50, 70, 380, 400)

        self.publish_map(map)

    
    def set_area_occupied(self, map: OccupancyGrid, start_row, end_row, start_col, end_col):
        for row in range(start_row, end_row):
            for col in range(start_col, end_col):
                self.modify_map(map, row, col, 100)


    def modify_map(self, map: OccupancyGrid, row, col, value):
        index = self.map_width * row + col

        data = list(map.data)
        data[index] = value

        map.data = tuple(data)


    def publish_map(self, map: OccupancyGrid):
        while True:
            self.modified_map_publisher.publish(map)
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('occupancy_grid_modifier', anonymous=True)

    occupancy_grip_modifier = OccupancyGridModifier()

    rospy.spin()