#!/usr/bin/env python3

from __future__ import print_function

import rospy
import copy
import json
import os
import time

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import PointStamped, Pose, Point
from nav_msgs.msg import Odometry
import pathlib

from math import sin
BASE_PATH = str(pathlib.Path(__file__).parent)


MODES = ["PoseSaver", "DimensionSaver"]

contextualizer_mode = MODES[0]

node_name = "map_roi"
class PoseSaver:
    def __init__(self) -> None:
        self.server = InteractiveMarkerServer(node_name)
        self.marker_z = 0.5

        self.using_pose = False # True if using robot pose, False for free movement
        
        if self.using_pose:
            self.robot_pose = None
        
        self.menu_handler = MenuHandler()
        self.not_moving_marker = InteractiveMarker()
        self.arrow_marker = InteractiveMarker()

        self.roi_dict = dict()
        self.last_key1, self.last_key2 = None, None

        self.initDict(BASE_PATH + '/areas.json')
        self.initMenu()
        self.initMarker()

    def pose_callback(self, data):
        self.robot_pose = data
        self.robot_pose.position.z += self.marker_z
        self.server.setPose( self.not_moving_marker.name, self.robot_pose)
        self.server.applyChanges()

    def makeMarker(self):
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = 0.20
        marker.scale.y = 0.20
        marker.scale.z = 0.20
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0 
        return marker

    def makeBox(self):
        marker = Marker()

        marker.type = Marker.CUBE
        marker.scale.x = 0.20
        marker.scale.y = 0.20
        marker.scale.z = 0.20
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 1.0
        # marker.pose.position.z = 0.2

        return marker

    def makeArrow(self):
        marker = Marker()

        marker.type = Marker.ARROW
        marker.scale.x = 0.4
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 1.0
        # marker.pose.position.z = 0.2

        return marker

    def normalizeQuaternion(self, quaternion_msg ):
        norm = quaternion_msg.x**2 + quaternion_msg.y**2 + quaternion_msg.z**2 + quaternion_msg.w**2
        s = norm**(-0.5)
        quaternion_msg.x *= s
        quaternion_msg.y *= s
        quaternion_msg.z *= s
        quaternion_msg.w *= s

    def processFeedback(self, feedback ):
        self.server.setPose(self.arrow_marker.name, feedback.pose)
        self.server.applyChanges()

    def makeMovingMarker(self, position):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.pose.position = position
        int_marker.pose.position.z = 0.1
        int_marker.scale = 0.5

        int_marker.name = "robot_context_menu"
        int_marker.description = "Marker Attached to a\nMoving Frame"

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        self.normalizeQuaternion(control.orientation)
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(copy.deepcopy(control))


        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        self.normalizeQuaternion(control.orientation)
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        control.always_visible = True
        control.markers.append( self.makeBox())
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.MENU
        control.description="Options"
        control.name = "menu_only_control"
        int_marker.controls.append(copy.deepcopy(control))

        self.server.insert(int_marker, self.processFeedback)



    def makeArrowMarker(self, position ):
        self.arrow_marker.header.frame_id = "map"
        self.arrow_marker.pose.position = position
        self.arrow_marker.scale = 0.5

        self.arrow_marker.name = "ARROW"

        # make one control using default visuals
        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.NONE
        self.arrow_marker.controls.append(copy.deepcopy(control))
        
        control.markers.append( self.makeArrow() )
        control.always_visible = True
        self.arrow_marker.controls.append(control)

        self.server.insert( self.arrow_marker )    

    def makeMenuMarker(self, position ):
        self.not_moving_marker.header.frame_id = "base_footprint" # pose
        self.not_moving_marker.pose.position = position
        self.not_moving_marker.scale = 0.5

        self.not_moving_marker.name = "robot_context_menu"

        # make one control using default visuals
        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.MENU
        control.description="Options"
        control.name = "menu_only_control"
        self.not_moving_marker.controls.append(copy.deepcopy(control))
        
        control.markers.append( self.makeMarker() )
        control.always_visible = True
        self.not_moving_marker.controls.append(control)

        self.server.insert( self.not_moving_marker )
    
    def savePose(self, key1, key2, p_x, p_y, p_z, o_x, o_y, o_z, o_w):
        self.roi_dict[key1][key2] = [p_x, p_y, p_z, o_x, o_y, o_z, o_w]

    def deleteLastPoint(self):
        self.roi_dict[self.last_key1][self.last_key2] = []


    def poseFeedback(self, feedback ):
        last_entry = feedback.menu_entry_id
        #rospy.loginfo('%s \n', last_entry)
        context = self.menu_handler.entry_contexts_[last_entry].title
        context = context.split('-')
        self.last_key1 = context[0]
        self.last_key2 = context[1]
        rospy.loginfo("Insert ROI => ROI Room: " + context[0] + " ROI Place: " + context[1] + "\nPOSE: x = {} y = {} z = {}\nORIENTATION: x = {} y = {} z = {} w = {}".format(feedback.pose.position.x, feedback.pose.position.y, 0, feedback.pose.orientation.x, feedback.pose.orientation.y, feedback.pose.orientation.z, feedback.pose.orientation.w))   
        self.savePose(context[0], context[1] , feedback.pose.position.x, feedback.pose.position.y, 0, feedback.pose.orientation.x, feedback.pose.orientation.y, feedback.pose.orientation.z, feedback.pose.orientation.w)

    def initDict(self, config_file):
        with open(config_file) as infile:
            config = json.load(infile)

        self.roi_dict = dict()
        self.roi_dict = config
        rospy.loginfo("Loaded points from file")

    def save_points(self, save):
        # Abre el archivo JSON y escribe el diccionario nuevo en él 
        #file = "/home/jetson/robocup-home/catkin_home/src/navigation/map_contextualizer/scripts/areas.json"
        with open(BASE_PATH + '/areas.json', "w") as outfile:
            json.dump(self.roi_dict, outfile, indent=4)

        rospy.loginfo("Saved points to file")

    def initMenu(self):
        h_first_entry = self.menu_handler.insert( "Insert ROI Entry")
        del_point_entry = self.menu_handler.insert( "Delete last Entry", callback=self.deleteLastPoint)
        for roi_name, roi_id in self.roi_dict.items():
            entry = self.menu_handler.insert( str(roi_name), parent=h_first_entry) 
            for roi_room, roi_id2 in self.roi_dict[roi_name].items():
                h_entry = self.menu_handler.insert( str(roi_name) + '-' + str(roi_room), parent=entry , callback=self.poseFeedback)
        
        save_map = self.menu_handler.insert("Save Map", callback=self.save_points)
        rospy.loginfo("Menu initialized")
    

    def initMarker(self):
        if self.using_pose:
            rospy.Subscriber("/robot_pose", Pose, self.pose_callback)
            rospy.loginfo("Subscribed to /robot_pose topic")
            position = Point(0, 0, 0.5)
            self.makeMenuMarker( position )
        else:
            position = Point(0, 0, 0)
            self.makeMovingMarker( position )
            self.makeArrowMarker( position )

        self.menu_handler.apply( self.server, "robot_context_menu" )
        self.server.applyChanges()
    
class DimensionSaver():
    def __init__(self) -> None:
        self.server = InteractiveMarkerServer(node_name)
        self.menu_handler = MenuHandler()
        self.not_moving_marker = InteractiveMarker()

        self.roi_dict = dict()
        self.last_key1, self.last_key2 = None, None

        self.current_map_area = None

        self.sub = rospy.Subscriber("/clicked_point", PointStamped, self.click_callback)
        self.initDict(BASE_PATH + '/map_dimension.json')
        self.initMenu()
        self.initMarker()

    def click_callback(self, data):
        if self.current_map_area != None:
            self.roi_dict[self.current_map_area].append([data.point.x, data.point.y])
            rospy.loginfo("Insert ROI => ROI Room: " + self.current_map_area + " ROI Place: " + str(len(self.roi_dict[self.current_map_area])) + "\nPOINT: x = {} y = {}".format(data.point.x, data.point.y))

    def clear(self, data):
        for key in self.roi_dict.keys():
            self.roi_dict[key] = []
        rospy.loginfo("Cleared all entries")
        
    def updateMapArea(self, feedback ):
        last_entry = feedback.menu_entry_id
        #rospy.loginfo('%s \n', last_entry)
        room = self.menu_handler.entry_contexts_[last_entry].title
        self.current_map_area = room
        rospy.loginfo("Room updated to: " + room)


    def initDict(self, config_file):
        with open(config_file) as infile:
            config = json.load(infile)

        self.roi_dict = config
        rospy.loginfo("Loaded rooms from file")

    def deleteLastPoint(self):
        if self.current_map_area != None and len(self.roi_dict[self.current_map_area]) > 0:
            self.roi_dict[self.current_map_area].pop()

    def save_points(self, save):
        # Abre el archivo JSON y escribe el diccionario nuevo en él 
        #file = "/home/jetson/robocup-home/catkin_home/src/navigation/map_contextualizer/scripts/areas.json"
        with open(BASE_PATH + '/map_dimension.json', "w") as outfile:
            json.dump(self.roi_dict, outfile, indent=4)

        rospy.loginfo("Saved points to file")

    def initMenu(self):
        h_first_entry = self.menu_handler.insert( "Select Room for Entries")
        del_point_entry = self.menu_handler.insert( "Delete last Entry", callback=self.deleteLastPoint)
        clear_entries = self.menu_handler.insert( "Clear Entries", callback=self.clear)
        for roi_name in self.roi_dict.keys():
            entry = self.menu_handler.insert( str(roi_name), parent=h_first_entry, callback=self.updateMapArea) 
        
        save_map = self.menu_handler.insert("Save Map", callback=self.save_points)
        rospy.loginfo("Menu initialized")

    def makeMenuMarker(self, position ):
        self.not_moving_marker.header.frame_id = "map" # pose
        self.not_moving_marker.pose.position = position
        self.not_moving_marker.scale = 1

        self.not_moving_marker.name = "map_dimensions_menu"

        # make one control using default visuals
        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.MENU
        control.description="Options"
        control.name = "menu_only_control"
        self.not_moving_marker.controls.append(copy.deepcopy(control))
        
        control.markers.append( self.makeMarker() )
        control.always_visible = True
        self.not_moving_marker.controls.append(control)
        rospy.loginfo("Menu Marker created")

        self.server.insert( self.not_moving_marker )


    def makeMarker(self):
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = 0.20
        marker.scale.y = 0.20
        marker.scale.z = 0.20
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0 
        return marker
    
    def initMarker(self):
        position = Point(0, 0, 1)
        self.makeMenuMarker( position )
        self.menu_handler.apply( self.server, "map_dimensions_menu" )
        self.server.applyChanges()

if __name__=="__main__":
    try: 
        rospy.init_node(node_name)
        rospy.loginfo("Map Contextualizer Mode: " + contextualizer_mode)
        if contextualizer_mode == MODES[0]:
            try:   
                PoseSaver()
                rospy.loginfo("Service call Success")
            except rospy.ServiceException as e:
                rospy.loginfo("Service call failed: %s" % (e,))
        elif contextualizer_mode == MODES[1]:
            try:   
                DimensionSaver()
                rospy.loginfo("Service call Success")
            except rospy.ServiceException as e:
                rospy.loginfo("Service call failed: %s" % (e,))
        rospy.spin()

    except rospy.ROSException as e:
        rospy.loginfo("Service call failed: %s" % (e,))