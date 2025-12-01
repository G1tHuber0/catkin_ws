#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from my_env import myEnv

if __name__ == "__main__":
    rospy.init_node("test_steps")

    env = myEnv()
    print("Action space:", env.action_space)
    print("Observation space:", env.observation_space)

    obs, info = env.reset()
    print("Initial obs shape:", obs.shape)

    episode = 0
    step_in_ep = 0

    while not rospy.is_shutdown():
        # 随机采样一个动作
        action = env.action_space.sample()

        obs, reward, terminated, truncated, info = env.step(action)
        step_in_ep += 1

        print(
            "ep %d | step %d | reward %.3f | min_range=%.3f | done=%s" %
            (
                episode,
                step_in_ep,
                reward,
                info.get("min_range", -1.0),
                info.get("done_reason", None),
            )
        )

        if terminated or truncated:
            print("=== Episode %d ended: %s ===" % (episode, info.get("done_reason")))
            episode += 1
            step_in_ep = 0
            obs, info = env.reset()
