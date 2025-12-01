#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from env_turtlebot3_rl import GazeboTurtlebot3Env

if __name__ == "__main__":
    rospy.init_node("test_env_goal_sampling")

    env = GazeboTurtlebot3Env()

    for i in range(5):
        obs, info = env.reset()
        goal = info["goal"]
        rospy.loginfo("Episode %d: goal at (%.2f, %.2f)", i, goal[0], goal[1])
        rospy.sleep(2.0)  # 给你 2 秒钟时间在 Gazebo/RViz 看一下

    env.close()
