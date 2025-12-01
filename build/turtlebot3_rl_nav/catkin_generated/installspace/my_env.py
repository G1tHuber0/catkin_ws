#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import rospy
import gymnasium as gym
from gymnasium import spaces

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class myEnv(gym.Env):

    metadata = {"render_modes": ["human"]}

    def __init__(self):
        super(myEnv, self).__init__()

        # ---------- 1. 一些超参数 ----------
        self.scan_topic = "/scan"              # 订阅激光雷达的话题
        self.cmd_vel_topic = "/cmd_vel"     # 发布控制指令的话题

        self.max_lin_vel = 0.22   # 先用 TurtleBot3 burger 官方线速度上限
        self.max_ang_vel = 2.84   # 官方角速度上限
        self.collision_distance = 0.18   # 判定碰撞的最小距离
        self.safe_distance = self.collision_distance + 0.1  # 安全距离

        self.step_time = 0.1          # 每一步控制间隔 [s]，先别太小，方便看效果
        self.max_episode_steps = 500  # 一局最多 500 步

        # ---------- 2. ROS 通信 ----------
        # 发布 /cmd_vel，保证发布消息是当前最新的
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)         

        # 等待第一次 /scan，目的：
        #  - 确认话题存在
        #  - 得到 num_beams 和 range_max
        rospy.loginfo("Waiting for first /scan ...")
        first_scan = rospy.wait_for_message(self.scan_topic, LaserScan)
        self.scan_range_max = first_scan.range_max          # 记录雷达最大量程
        self.num_beams = len(first_scan.ranges)     # 记录激光束数量
        self.last_scan = first_scan                 # 记录最新的激光雷达数据

        # 持续订阅 /scan，实时更新 self.last_scan
        self.scan_sub = rospy.Subscriber(
            self.scan_topic, LaserScan, 
            self._scan_callback
        )

        rospy.loginfo("myEnv: got %d laser beams.", self.num_beams)

        # ---------- 3. 定义 Gymnasium 的 action_space / observation_space ----------

        # 动作空间：两个连续动作 [a_v, a_w] ∈ [-1, 1]^2
        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0], dtype=np.float32),
            high=np.array([1.0, 1.0], dtype=np.float32),
            shape=(2,),
            dtype=np.float32,
        )

        # 观测空间：num_beams 个 [0, 1] 浮点数
        obs_dim = self.num_beams
        self.observation_space = spaces.Box(
            low=0.0,
            high=1.0,                     #数据归一化
            shape=(obs_dim,),           # 观测维度，与激光雷达束数相同
            dtype=np.float32,
        )

        # ---------- 4. 一些内部状态 ----------
        self.step_count = 0             # 记录当前回合步数

    # ========= ROS 回调 =========
    def _scan_callback(self, msg):
        self.last_scan = msg

    # ========= 工具函数：把 LaserScan -> np.array =========
    def _get_scan_obs(self):
        """
        将最新的 LaserScan 转成 0~1 的 numpy 数组
        """
        scan = np.array(self.last_scan.ranges, dtype=np.float32)        # 转成 numpy 数组
        scan = np.where(np.isinf(scan), self.scan_range_max, scan)      # 替换 inf为最大量程
        scan = np.where(np.isnan(scan), self.scan_range_max, scan)      # 替换 nan为最大量程
        scan = np.clip(scan, 0.0, self.scan_range_max)                 # 裁剪到 [0, range_max]
        scan_norm = scan / self.scan_range_max                      # 归一化到 [0, 1]
        return scan_norm.astype(np.float32)

    # ========= Gymnasium 必需接口：先留空，下一步再实现 =========
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)        # 调用父类方法

        # 停车
        cmd = Twist()                  
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd) 
        rospy.sleep(0.1)

        try:
            self.last_scan = rospy.wait_for_message(
                self.scan_topic, LaserScan, timeout=2.0
            )
        except rospy.ROSException:
            rospy.logwarn("reset(): timeout waiting for /scan, use last_scan as fallback")

        self.step_count = 0         # 重置步数

        obs = self._get_scan_obs()      # 获取初始观测
        
        return obs, {}
    

    def step(self, action):
        raise NotImplementedError

    def render(self):
        # Gazebo 自带可视化，这里不额外实现
        pass

    def close(self):
        # 停车
        cmd = Twist()
        self.cmd_pub.publish(cmd)
