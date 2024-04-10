#!/usr/bin/env python

from __future__ import print_function

import rospy
import copy
import json
import os

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point, Pose
from nav_msgs.msg import Odometry

from math import sin


server = None
odom_pose = None
marker_z = 0.5

using_pose = False # True if using robot pose, False for free movement
  
menu_handler = MenuHandler()
not_moving_marker = InteractiveMarker()
arrow_marker = InteractiveMarker()

#roi_dict = {"Test 1" : {"test1.1": "1.1", "Test1.2":"1.2"}, "Test 2" : {"test2.1": "2.1", "Test2.2":"2.2"}, "Test 3" : {"test3.1": "3.1", "Test3.2":"3.2"}}

def pose_callback(data):
    global robot_pose, not_moving_marker
    robot_pose = data
    robot_pose.position.z += marker_z
    server.setPose( not_moving_marker.name, robot_pose)
    server.applyChanges()

def makeMarker():
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

def makeBox(  ):
    marker = Marker()

    marker.type = Marker.CUBE
    marker.scale.x = 0.20
    marker.scale.y = 0.20
    marker.scale.z = 0.20
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0

    return marker

def makeArrow(  ):
    marker = Marker()

    marker.type = Marker.ARROW
    marker.scale.x = 0.4
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0

    return marker

def normalizeQuaternion( quaternion_msg ):
    norm = quaternion_msg.x**2 + quaternion_msg.y**2 + quaternion_msg.z**2 + quaternion_msg.w**2
    s = norm**(-0.5)
    quaternion_msg.x *= s
    quaternion_msg.y *= s
    quaternion_msg.z *= s
    quaternion_msg.w *= s


def processFeedback( feedback ):
    global arrow_marker
    server.setPose(arrow_marker.name, feedback.pose)
    server.applyChanges()

def makeMovingMarker(position):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_footprint"
    int_marker.pose.position = position
    int_marker.scale = 0.5

    int_marker.name = "robot_context_menu"
    int_marker.description = "Marker Attached to a\nMoving Frame"

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    normalizeQuaternion(control.orientation)
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(copy.deepcopy(control))


    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    normalizeQuaternion(control.orientation)
    control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    control.always_visible = True
    control.markers.append( makeBox())
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.interaction_mode = InteractiveMarkerControl.MENU
    control.description="Options"
    control.name = "menu_only_control"
    int_marker.controls.append(copy.deepcopy(control))

    server.insert(int_marker, processFeedback)


def makeArrowMarker( position ):
    global arrow_marker
    arrow_marker.header.frame_id = "base_footprint"
    arrow_marker.pose.position = position
    arrow_marker.scale = 0.5

    arrow_marker.name = "ARROW"

    # make one control using default visuals
    control = InteractiveMarkerControl()
    control.interaction_mode = InteractiveMarkerControl.NONE
    arrow_marker.controls.append(copy.deepcopy(control))
    
    control.markers.append( makeArrow() )
    control.always_visible = True
    arrow_marker.controls.append(control)

    server.insert( arrow_marker )    

def makeMenuMarker( position ):
    global not_moving_marker
    not_moving_marker.header.frame_id = "odom" # pose
    not_moving_marker.pose.position = position
    not_moving_marker.scale = 0.5

    not_moving_marker.name = "robot_context_menu"

    # make one control using default visuals
    control = InteractiveMarkerControl()
    control.interaction_mode = InteractiveMarkerControl.MENU
    control.description="Options"
    control.name = "menu_only_control"
    not_moving_marker.controls.append(copy.deepcopy(control))
    
    control.markers.append( makeMarker() )
    control.always_visible = True
    not_moving_marker.controls.append(control)

    server.insert( not_moving_marker )
    
def savePose(key1, key2, p_x, p_y, p_z, o_x, o_y, o_z, o_w):
    roi_dict[key1][key2] = [p_x, p_y, p_z, o_x, o_y, o_z, o_w]
    return

def deleteLastPoint(delete):
    roi_dict[last_key1][last_key2] = []

    return

def poseFeedback( feedback ):
    global last_key1, last_key2
    last_entry = feedback.menu_entry_id
    #rospy.loginfo('%s \n', last_entry)
    context = menu_handler.entry_contexts_[last_entry].title
    context = context.split('-')
    last_key1 = context[0]
    last_key2 = context[1]
    rospy.loginfo("Insert ROI => ROI Room: " + context[0] + " ROI Place" + context[1] + "\nPOSE: x = {} y = {} z = {}\nORIENTATION: x = {} y = {} z = {} w = {}".format(feedback.pose.position.x, feedback.pose.position.y, feedback.pose.position.z - marker_z, feedback.pose.orientation.x, feedback.pose.orientation.y, feedback.pose.orientation.z, feedback.pose.orientation.w))   
    savePose(context[0], context[1] , feedback.pose.position.x, feedback.pose.position.y, feedback.pose.position.z - marker_z, feedback.pose.orientation.x, feedback.pose.orientation.y, feedback.pose.orientation.z, feedback.pose.orientation.w)

def initDict(config_file):
    global roi_dict
    with open(config_file) as infile:
        config = json.load(infile)

    roi_dict = dict()
    roi_dict = config

def save_points(save):
    # Abre el archivo JSON y escribe el diccionario nuevo en él 
    file = "/home/jetson/robocup-home/catkin_home/src/navigation/map_contextualizer/scripts/areas.json"
    with open(file, "w") as outfile:
        json.dump(roi_dict, outfile, indent=4)

    rospy.loginfo("Guardado")

def initMenu():
    h_first_entry = menu_handler.insert( "Insert ROI Entry")
    del_point_entry = menu_handler.insert( "Delete last Entry", callback=deleteLastPoint)
    for roi_name, roi_id in roi_dict.items():
        entry = menu_handler.insert( str(roi_name), parent=h_first_entry) 
        for roi_room, roi_id2 in roi_dict[roi_name].items():
            h_entry = menu_handler.insert( str(roi_name) + '-' + str(roi_room), parent=entry , callback=poseFeedback)

    save_map = menu_handler.insert("Save Map", callback=save_points)

if __name__=="__main__":
    try: 
      rospy.init_node("map_roi")
      rospy.loginfo("Init ROS Node: map_roi")

      try:   
        server = InteractiveMarkerServer("map_roi")
        rospy.loginfo("Service call Success")
      except rospy.ServiceException as e:
          rospy.loginfo("Service call failed: %s" % (e,))

      

      initDict("/home/jetson/robocup-home/catkin_home/src/navigation/map_contextualizer/scripts/areas.json")
      initMenu()

      rospy.loginfo("inicializado")
      
      if using_pose:
        rospy.Subscriber("/robot_pose", Pose, pose_callback)
        rospy.loginfo("Subscribed to /robot_pose topic")
        position = Point(0, 0, 0.5)
        makeMenuMarker( position )
      else:
        position = Point(0, 0, 0)
        makeMovingMarker( position )
        makeArrowMarker( position )

      menu_handler.apply( server, "robot_context_menu" )
      server.applyChanges()

      rospy.spin()

    except rospy.ROSException as e:
        rospy.loginfo("Service call failed: %s" % (e,))

    # Existen dos tipos de implementaicon, la priemra es utilizar el menu_handler con el parametro feedback y guardar las poses en un csv, 
    # La segunda es hacer un menu handler que al momento de hacer un callback, se saca el ultimo nav goal de un queue y se guarda esa pose en un csv