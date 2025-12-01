#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from env_turtlebot3_rl import GazeboTurtlebot3Env

if __name__ == "__main__":
    rospy.init_node("test_env_goal_reach")

    env = GazeboTurtlebot3Env()

    obs, info = env.reset()
    goal = info["goal"]
    rospy.loginfo("Test episode: goal at (%.2f, %.2f)", goal[0], goal[1])

    terminated = False
    truncated = False
    step_count = 0

    # 简单 P 控制参数
    K_w = 1.5   # 朝向 P 增益
    v_forward = 0.2  # 固定小线速度 (m/s)

    while not rospy.is_shutdown():
        step_count += 1

        # 从 env 内部拿一下当前 dist 和 heading
        dist, heading = env._compute_dist_and_heading()

        # 如果离目标很近，就不再往前走，避免 overshoot
        if dist < env.goal_tolerance:
            v = 0.0
            w = 0.0
        else:
            v = v_forward
            w = np.clip(K_w * heading, -env.max_ang_vel, env.max_ang_vel)

        # 把 (v, w) 映射回 action in [-1, 1]^2
        a_v = (v / env.max_lin_vel) * 2.0 - 1.0
        a_w = w / env.max_ang_vel
        action = np.array([a_v, a_w], dtype=np.float32)

        obs, reward, terminated, truncated, info = env.step(action)

        if step_count % 20 == 0:
            rospy.loginfo(
                "step=%d, dist=%.3f, heading=%.3f rad, reward=%.3f",
                step_count, dist, heading, reward,
            )

        if terminated or truncated:
            rospy.loginfo("Episode finished with done_reason=%s", info.get("done_reason", ""))
            break

    env.close()
