#!/usr/bin/env python
import rospy
import json
import pathlib

from geometry_msgs import Pose

BASE_PATH = str(pathlib.Path(__file__).parent) + "/areas.json"
count = 0

def createMsg():
    poses = []
    with open(BASE_PATH + '/areas.json') as json_file:
        data = json.load(json_file)
        for key in data:
            for subkey in data[key]:
                newPose = Pose()
                newPose.position.x = data[key][subkey][0]
                newPose.position.y = data[key][subkey][1]
                newPose.position.z = data[key][subkey][2]
                newPose.orientation.x = data[key][subkey][3]
                newPose.orientation.y = data[key][subkey][4]
                newPose.orientation.z = data[key][subkey][5]
                newPose.orientation.w = data[key][subkey][6]

                poses.append(newPose)

    return poses

def nodePublisher():
    publsiher = rospy.Publisher('poses', Pose, queue_size=10)
    rospy.init_node('pose_publisher')

    rate = rospy.Rate(1)
    poses = createMsg()

    while not rospy.is_shutdown():
        count = (count + 1) % count
        publsiher.publish(poses[count])
        rate.sleep()

if __name__ == '__main__':
    try:
        nodePublisher()
    except rospy.ROSInterruptException:
        pass