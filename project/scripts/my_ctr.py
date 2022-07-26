#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import sys
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import numpy as np


def goal_pose(area_pos):  # <2>
    goal_point = MoveBaseGoal()
    goal_point.target_pose.header.frame_id = 'map'
    goal_point.target_pose.pose.position.x = area_pos[0]
    goal_point.target_pose.pose.position.y = area_pos[1]
    goal_point.target_pose.pose.position.z = area_pos[2]
    goal_point.target_pose.pose.orientation.x = area_pos[3]
    goal_point.target_pose.pose.orientation.y = area_pos[4]
    goal_point.target_pose.pose.orientation.z = area_pos[5]
    goal_point.target_pose.pose.orientation.w = area_pos[6]
    print(goal_point)
    return goal_point


def get_data():
    f = open(sys.path[0] + "/mapinfo.txt", mode="r")
    read_poses = np.loadtxt(f, dtype=float, delimiter=",")
    return read_poses


if __name__ == '__main__':
    rospy.init_node('control')
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)  # <3>
    client.wait_for_server()
    area_pos = get_data()
    print(area_pos)
    while not rospy.is_shutdown():
        for i in range(area_pos.shape[0]):  # <4>
            rospy.loginfo("goal_point = "+str(area_pos[i, ...]))
            goal = goal_pose(area_pos[i, ...])
            client.send_goal(goal)
            client.wait_for_result()
