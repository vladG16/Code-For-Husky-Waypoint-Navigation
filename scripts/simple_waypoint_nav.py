#!/usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

positions = []
cumulative_distance = 0.0
last_position = None

def odom_callback(msg):
    global cumulative_distance, last_position

    # Get current position
    pos = msg.pose.pose.position
    current = (pos.x, pos.y)

    if last_position is not None:
        dx = current[0] - last_position[0]
        dy = current[1] - last_position[1]
        dist = math.sqrt(dx**2 + dy**2)
        cumulative_distance += dist

    last_position = current
    positions.append((rospy.Time.now().to_sec(), cumulative_distance))

def publish_goal(pub, x, y):
    goal = PoseStamped()
    goal.header.frame_id = "odom"
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.orientation.w = 1.0
    pub.publish(goal)
    rospy.loginfo(f"Published goal: x={x}, y={y}")
    rospy.sleep(10)

if __name__ == "__main__":
    rospy.init_node("simple_waypoint_nav")
    rospy.Subscriber("/odom", Odometry, odom_callback)
    pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
    rospy.sleep(1)

    waypoints = [(1.0, 0.0), (2.0, 0.0), (2.0, 1.0), (1.0, 1.0), (0.0, 0.0)]
    for x, y in waypoints:
        publish_goal(pub, x, y)

    rospy.loginfo(f"Final distance traveled: {cumulative_distance:.2f} meters")

    # Save to file
    with open("/tmp/husky_distance.csv", "w") as f:
        f.write("time,distance\n")
        for t, d in positions:
            f.write(f"{t},{d}\n")

    rospy.loginfo("Saved distance data to /tmp/husky_distance.csv")

