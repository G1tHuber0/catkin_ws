#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.monitor import Monitor
from turtlebot3_rl_nav.scripts.env_turtlebot3_rl import GazeboTurtlebot3Env


def make_env():
    # 这里返回被 Monitor 包装过的 env
    env = GazeboTurtlebot3Env()
    env = Monitor(env)   # 关键：让 SB3 能统计 episode reward/length
    return env


def main():
    rospy.init_node('turtlebot3_ppo_train', anonymous=True)

    # VecEnv 封装
    env = DummyVecEnv([make_env])

    # 也可以加 tensorboard_log，自行设定目录
    model = PPO(
        policy="MlpPolicy",
        env=env,
        verbose=1,
        tensorboard_log=None,  # 比如 "~/tb3_ppo_logs"
    )

    # 开始训练
    total_timesteps = 500_000  # 先跑个 20 万步看看效果，再慢慢加
    model.learn(total_timesteps=total_timesteps,progress_bar=True)

    # 保存模型
    model.save("ppo_turtlebot3_rl")

    env.close()


if __name__ == "__main__":
    main()
