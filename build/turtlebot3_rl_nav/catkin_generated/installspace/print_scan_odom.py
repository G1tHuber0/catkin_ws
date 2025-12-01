#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

last_scan = None
last_odom = None


def scan_callback(msg):
    global last_scan
    last_scan = msg


def odom_callback(msg):
    global last_odom
    last_odom = msg


def main():
    rospy.init_node("print_scan_odom")

    # 订阅雷达和里程计
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rospy.Subscriber("/odom", Odometry, odom_callback)

    rospy.loginfo("Waiting for first /scan and /odom...")
    # 阻塞等待第一次消息（避免一开始 last_xxx 是 None）
    scan_msg = rospy.wait_for_message("/scan", LaserScan)
    odom_msg = rospy.wait_for_message("/odom", Odometry)

    rospy.loginfo("Got first /scan and /odom, start printing...")

    rate = rospy.Rate(1.0)  # 1 Hz

    while not rospy.is_shutdown():
        if last_scan is not None:
            # 计算激光最小距离
            ranges = np.array(last_scan.ranges, dtype=np.float32)
            # 替换 inf / nan
            ranges = np.where(np.isinf(ranges), last_scan.range_max, ranges)
            ranges = np.where(np.isnan(ranges), last_scan.range_max, ranges)
            min_range = float(np.min(ranges))
        else:
            min_range = float("nan")

        if last_odom is not None:
            pose = last_odom.pose.pose
            twist = last_odom.twist.twist
            x = pose.position.x
            y = pose.position.y
            v = twist.linear.x
            w = twist.angular.z
        else:
            x = y = v = w = float("nan")

        rospy.loginfo(
            "min_scan = %.3f m | pose = (%.3f, %.3f) | v = %.3f m/s, w = %.3f rad/s",
            min_range, x, y, v, w
        )

        rate.sleep()


if __name__ == "__main__":
    main()
