#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion

def send_goal(x, y, yaw=1.0):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "odom"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position = Point(x, y, 0)
    goal.target_pose.pose.orientation = Quaternion(0, 0, yaw, 1)

    client.send_goal(goal)
    client.wait_for_result()

    return client.get_result()

if __name__ == '__main__':
    rospy.init_node('waypoint_sender')

    waypoints = [
        (1.0, 0.0),
        (1.0, 1.0),
        (0.0, 1.0),
        (0.0, 0.0)
    ]

    for x, y in waypoints:
        result = send_goal(x, y)
        rospy.loginfo("Goal reached: %s", result)
