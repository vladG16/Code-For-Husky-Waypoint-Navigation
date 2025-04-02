#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

waypoints = [
    [1.0, 0.0, 0.0],
    [1.0, 1.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0]
]

def movebase_client():
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base action server.")

    for point in waypoints:
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = point[0]
        goal.target_pose.pose.position.y = point[1]
        goal.target_pose.pose.orientation.w = 1.0  # No rotation for now

        rospy.loginfo(f"Sending goal: {point}")
        client.send_goal(goal)
        client.wait_for_result()
        rospy.loginfo("Goal reached!")

if __name__ == '__main__':
    rospy.init_node('waypoint_follower')
    try:
        movebase_client()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted.")

