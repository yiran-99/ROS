#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
store co-ordinate file
"""
import rospy
import sys
import io
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, PoseStamped
import time
import math
import numpy as np

area_pos = None
area_num = 0
area_n = 0


def click_point_callback(data):
    global area_pos
    global area_num
    global area_n
    area_pos[area_n, ...] = [data.pose.position.x, data.pose.position.y, data.pose.position.z,
                             data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z,
                             data.pose.orientation.w]
    area_n += 1
    print(" Set area " + str(area_n) + " success !\n")
    if area_n == area_num:
        f = open(sys.path[0]+"/mapinfo.txt", mode="w+")
        np.savetxt(f, area_pos, fmt="%f", delimiter=",")
        f.flush()
        f.close()
        print("Input over !")


if __name__ == '__main__':
    try:
        rospy.init_node('save_area', anonymous=True)  # initialize the node
        if area_num == 0:
            area_num = input("input the number of  navigation_points ：\n")
            area_pos = np.zeros([area_num, 7], float)
        rospy.Subscriber("/move_base_simple/goal",
                         PoseStamped,
                         click_point_callback)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
