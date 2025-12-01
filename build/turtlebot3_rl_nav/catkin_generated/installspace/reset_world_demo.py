#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_srvs.srv import Empty

def main():
    rospy.init_node("reset_world_demo")

    rospy.loginfo("Waiting for service /gazebo/reset_world ...")
    rospy.wait_for_service("/gazebo/reset_world")

    reset_world = rospy.ServiceProxy("/gazebo/reset_world", Empty)

    rate = rospy.Rate(0.2)  # 每 5 秒 reset 一次

    count = 0
    while not rospy.is_shutdown() and count < 5:
        try:
            rospy.loginfo("Calling /gazebo/reset_world ...")
            reset_world()  # 调用一次
            rospy.loginfo("Reset done.")
        except Exception as e:
            rospy.logwarn("reset_world failed: %s", str(e))

        count += 1
        rate.sleep()

    rospy.loginfo("Exit reset_world_demo node.")

if __name__ == "__main__":
    main()
