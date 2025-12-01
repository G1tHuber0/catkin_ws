#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import rospy
import gymnasium as gym
from gymnasium import spaces

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
import tf

class myEnv(gym.Env):

    metadata = {"render_modes": ["human"]}

    def __init__(self):
        super(myEnv, self).__init__()

        # 机器人模型名字，以后可能改成 rosparam 读取
        self.robot_model_name = "turtlebot3"
        self.goal_model_name = "rl_goal_marker"  # 如果你有目标模型的话

        # set_model_state 服务
        try:
            rospy.wait_for_service("/gazebo/set_model_state", timeout=5.0)
            self.set_model_state_srv = rospy.ServiceProxy(
                "/gazebo/set_model_state", SetModelState
            )
            rospy.loginfo("Connected to /gazebo/set_model_state")
        except rospy.ROSException:
            rospy.logwarn("Service /gazebo/set_model_state not available")
            self.set_model_state_srv = None

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
        self._stop_robot()
        # 随机移动机器人位置
        mx,my,r=-2,0,0.25
        while True:
            # 1. 采样相对坐标 (dx, dy)
            dx = np.random.uniform(-r, r)
            dy = np.random.uniform(-r, r)
            # 2. 检查是否在圆内 (避免引入采样偏差的关键)
            if dx*dx + dy*dy <= r*r:
                rx = mx + dx
                ry = my + dy
                break
        rx = np.random.uniform(-3.0, -2.0)
        ry = np.random.uniform(-1.0, 1.0)
        ryaw = np.random.uniform(-np.pi, np.pi)

        self._set_model_pose(self.robot_model_name, rx, ry, ryaw, z=0.0)

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
        self.step_count += 1
        #归一数据映射实际速度角度
        a = np.clip(np.array(action, dtype=np.float32), -1.0, 1.0)
        v_cmd = (a[0]+1)/2 * self.max_lin_vel
        w_cmd = a[1]/2 *self.max_ang_vel        

        cmd=Twist()
        cmd.linear.x = v_cmd
        cmd.angular.z = w_cmd
        self.cmd_pub.publish(cmd)
        rospy.sleep(self.step_time)

        obs = self._get_scan_obs()
        scan_norm = obs  # [0,1]
        min_range = float(np.min(scan_norm) * self.scan_range_max)

        #奖励设计
        reward = 0.0
        terminated = False
        truncated = False
        info = {}
        # (1) 时间惩罚：每走一步都付一点“时间成本”
        reward -= 0.01
        # (2) 靠近障碍物惩罚
        if min_range < self.safe_distance:
            # [0,1] 之间的比例：越接近越大
            proximity = (self.safe_distance - min_range) / self.safe_distance
            proximity = np.clip(proximity, 0.0, 1.0)
            reward -= 0.5 * proximity  # 系数

        # (3) 安全前进奖励：前面安全 + 有一定前进速度
        if v_cmd > 0.05 and min_range > self.safe_distance:
            reward += 0.02 * (v_cmd / self.max_lin_vel)

        #终止 / 截断 条件 ----------
        # (a) 碰撞：最小距离小于碰撞阈值
        if min_range < self.collision_distance:
            reward -= 10.0
            terminated = True
            info["done_reason"] = "collision"

        # (b) 超过最大步数：截断
        if self.step_count >= self.max_episode_steps and not terminated:
            truncated = True
            info["done_reason"] = "timeout"

        # 如果 episode 结束了，顺便停车
        if terminated or truncated:
            self._stop_robot()

        # 你也可以在 info 里放一些调试信息
        info["min_range"] = min_range

        return obs, float(reward), terminated, truncated, info

    def render(self):
        # Gazebo 自带可视化，这里不额外实现
        pass

    def _stop_robot(self):
        # 停车
        cmd = Twist()
        cmd.linear.x = 0
        cmd.angular.z = 0
        self.cmd_pub.publish(cmd)

    def _set_model_pose(self, model_name, x, y, yaw, z=0.0):
        """
        通用的 '传送' Gazebo 模型到 (x, y, z, yaw)。
        可用于：
        - 重置机器人起点
        - 移动目标点模型
        - 将来移动动态障碍
        """
        if self.set_model_state_srv is None:
            rospy.logwarn("_set_model_pose(): set_model_state_srv is None")
            return False

        # yaw -> quaternion
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)

        state = ModelState()
        state.model_name = model_name
        state.pose.position.x = float(x)
        state.pose.position.y = float(y)
        state.pose.position.z = float(z)
        state.pose.orientation.x = q[0]
        state.pose.orientation.y = q[1]
        state.pose.orientation.z = q[2]
        state.pose.orientation.w = q[3]

        # 一般 reset/teleport 时都清零速度
        state.twist.linear.x = 0.0
        state.twist.linear.y = 0.0
        state.twist.linear.z = 0.0
        state.twist.angular.x = 0.0
        state.twist.angular.y = 0.0
        state.twist.angular.z = 0.0

        state.reference_frame = "world"

        try:
            resp = self.set_model_state_srv(state)
            if resp.success:
                return True
            else:
                rospy.logwarn("_set_model_pose(): Gazebo failed to set %s: %s", model_name, resp.status_message)
                return False
        except Exception as e:
            rospy.logwarn("_set_model_pose() failed for %s: %s", model_name, str(e))
            return False
