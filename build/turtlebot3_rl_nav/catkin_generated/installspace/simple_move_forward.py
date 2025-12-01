#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist


def main():
    rospy.init_node("simple_move_forward")

    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    rate = rospy.Rate(10.0)  # 10 Hz

    rospy.loginfo("Start moving forward for 5 seconds...")

    start_time = rospy.Time.now()

    while not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - start_time).to_sec()

        cmd = Twist()

        if elapsed < 5.0:
            # 前 5 秒，向前走
            cmd.linear.x = 0.1   # m/s
            cmd.angular.z = 0.0
        else:
            # 后面停住
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        cmd_pub.publish(cmd)

        if elapsed > 7.0:
            rospy.loginfo("Done, exit node.")
            break

        rate.sleep()


if __name__ == "__main__":
    main()
