#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import rospy
import gymnasium as gym
from gymnasium import spaces

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SetModelState
from gazebo_msgs.msg import ModelState
from visualization_msgs.msg import Marker
import tf


# 在 Gazebo 里用的目标模型（一个绿色小圆柱）
GOAL_MODEL_NAME = "rl_goal_marker"
GOAL_MODEL_SDF = """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>0</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.15</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0 1 0 1</ambient>
          <diffuse>0 1 0 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.15</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
""".format(name=GOAL_MODEL_NAME)


class GazeboTurtlebot3Env(gym.Env):
    """
    TurtleBot3 + Gazebo + ROS 封装成 Gymnasium 环境

    任务：到达随机目标点 + 避障
    """

    metadata = {"render_modes": ["human"]}

    def __init__(self):
        super(GazeboTurtlebot3Env, self).__init__()

        self.goal_sample_region = (-4.5, 4.5, -4.5, 4.5)  # xmin, xmax, ymin, ymax
        self.static_obstacles = [
            # 书架 bookshelf，尺寸大约 0.9 x 0.4
            {"name": "bookshelf",       "x":  4.78412,  "y": -3.73836,  "r": 0.8},
            # 桌子 table，尺寸大约 1.5 x 0.8
            {"name": "table",           "x": -4.34248,  "y": -0.532954, "r": 1.0},
            # 消防栓 fire_hydrant
            {"name": "fire_hydrant",    "x": -4.57173,  "y":  4.55086,  "r": 0.5},
            # 四个纸箱 cardboard_box_0~3，尺寸 0.5 x 0.4
            {"name": "cardboard_box_0", "x":  3.96847,  "y":  0.977565, "r": 0.6},
            {"name": "cardboard_box_1", "x":  0.006438, "y": -3.96264,  "r": 0.6},
            {"name": "cardboard_box_2", "x": -4.88373,  "y":  2.96722,  "r": 0.6},
            {"name": "cardboard_box_3", "x": -0.0158,   "y":  3.98725,  "r": 0.6},
        ]
        # 距离墙的安全边界（墙大约在 x,y=±5.425）
        self.wall_margin = 0.3

        # ---- 参数设置 ----
        self.scan_topic = "/scan"
        self.odom_topic = "/odom"
        self.cmd_vel_topic = "/cmd_vel"

        self.step_time = 0.001  # 每步控制间隔 [s]
        self.max_episode_steps = 100000000

        # TurtleBot3 物理限制（保守一点）
        self.max_lin_vel = 1.5   # m/s
        self.max_ang_vel = 2.84  # rad/s

        self.collision_distance = 0.18  # 判定碰撞的最小距离 [m]
        self.goal_tolerance = 0.1       # 到目标的判定阈值 [m]

        # ---- ROS 通信 ----
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)

        rospy.loginfo("Waiting for first /scan and /odom ...")
        first_scan = rospy.wait_for_message(self.scan_topic, LaserScan)
        first_odom = rospy.wait_for_message(self.odom_topic, Odometry)

        self.scan_range_max = first_scan.range_max
        self.num_beams = len(first_scan.ranges)
        self.last_scan = first_scan
        self.last_odom = first_odom

        # 订阅持续更新
        self.scan_sub = rospy.Subscriber(
            self.scan_topic, LaserScan, self._scan_callback
        )
        self.odom_sub = rospy.Subscriber(
            self.odom_topic, Odometry, self._odom_callback
        )

        # Gazebo reset 服务
        self.reset_world_srv = None
        try:
            rospy.wait_for_service("/gazebo/reset_world", timeout=5.0)
            self.reset_world_srv = rospy.ServiceProxy("/gazebo/reset_world", Empty)
        except rospy.ROSException:
            rospy.logwarn(
                "Service /gazebo/reset_world not available, reset() will not fully reset world!"
            )

        # Gazebo 目标可视化相关服务
        self.spawn_model_srv = None
        self.set_model_state_srv = None
        self.goal_model_spawned = False

        try:
            rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=5.0)
            self.spawn_model_srv = rospy.ServiceProxy(
                "/gazebo/spawn_sdf_model", SpawnModel
            )
        except rospy.ROSException:
            rospy.logwarn(
                "Service /gazebo/spawn_sdf_model not available, goal will not be shown in Gazebo."
            )

        try:
            rospy.wait_for_service("/gazebo/set_model_state", timeout=5.0)
            self.set_model_state_srv = rospy.ServiceProxy(
                "/gazebo/set_model_state", SetModelState
            )
        except rospy.ROSException:
            rospy.logwarn(
                "Service /gazebo/set_model_state not available, goal cannot be moved in Gazebo."
            )

        # RViz 目标 Marker 发布
        self.goal_marker_pub = rospy.Publisher(
            "/rl_goal_marker", Marker, queue_size=1
        )

        # ---- Gymnasium 空间定义 ----
        # 动作空间：[-1, 1] -> [0, max_lin_vel], [-max_ang_vel, max_ang_vel]
        self.action_space = spaces.Box(
            low=np.array([-1.0, -1.0], dtype=np.float32),
            high=np.array([1.0, 1.0], dtype=np.float32),
            shape=(2,),
            dtype=np.float32,
        )

        # 观测：laser (0~1) + [dist_norm, heading_norm, v_norm, w_norm]
        obs_dim = self.num_beams + 4
        self.observation_space = spaces.Box(
            low=0.0,
            high=1.0,
            shape=(obs_dim,),
            dtype=np.float32,
        )

        # 内部状态
        self.goal = np.array([0.0, 0.0], dtype=np.float32)
        self.prev_dist_to_goal = None
        self.step_count = 0

        rospy.loginfo(
            "GazeboTurtlebot3Env initialized with %d laser beams.", self.num_beams
        )

    # ---------- ROS 回调 ----------
    def _scan_callback(self, msg):
        self.last_scan = msg

    def _odom_callback(self, msg):
        self.last_odom = msg

    # ---------- 工具函数 ----------
    def _get_robot_pose(self):
        """
        从 odom 中取出 (x, y, yaw)，坐标系默认用 odom
        """
        pose = self.last_odom.pose.pose
        x = pose.position.x
        y = pose.position.y

        q = pose.orientation
        quaternion = [q.x, q.y, q.z, q.w]
        (_, _, yaw) = tf.transformations.euler_from_quaternion(quaternion)
        return x, y, yaw

    def _get_robot_velocity(self):
        twist = self.last_odom.twist.twist
        v = twist.linear.x
        w = twist.angular.z
        return v, w

    def _sample_goal(self):
        """
        在地图内随机采样一个目标点，保证不会落在障碍物内部/太靠近墙。
        """
        xmin, xmax, ymin, ymax = self.goal_sample_region
        max_tries = 100

        for _ in range(max_tries):
            gx = np.random.uniform(xmin, xmax)
            gy = np.random.uniform(ymin, ymax)

            if not self._is_in_obstacle(gx, gy):
                self.goal = np.array([gx, gy], dtype=np.float32)
                return

        rospy.logwarn("Failed to sample free goal after %d tries, using (0,0)", max_tries)
        self.goal = np.array([0.0, 0.0], dtype=np.float32)

    def _compute_dist_and_heading(self):
        x, y, yaw = self._get_robot_pose()
        dx = self.goal[0] - x
        dy = self.goal[1] - y
        dist = np.hypot(dx, dy)
        goal_angle = np.arctan2(dy, dx)
        heading = goal_angle - yaw
        # Wrap to [-pi, pi]
        heading = np.arctan2(np.sin(heading), np.cos(heading))
        return dist, heading

    def _get_scan_array(self):
        """
        将 LaserScan 转成 0~1 的 numpy 数组
        """
        scan = np.array(self.last_scan.ranges, dtype=np.float32)
        scan = np.where(np.isinf(scan), self.scan_range_max, scan)
        scan = np.where(np.isnan(scan), self.scan_range_max, scan)
        scan = np.clip(scan, 0.0, self.scan_range_max)
        scan_norm = scan / self.scan_range_max
        return scan_norm

    def _is_in_obstacle(self, x, y):
        """
        判断 (x, y) 是否落在障碍物内部 / 太靠近墙。
        """
        for obs in self.static_obstacles:
            dx = x - obs["x"]
            dy = y - obs["y"]
            if np.hypot(dx, dy) <= obs["r"]:
                return True

        # 靠近四周墙体（墙在 +-5.425 左右），留一点安全 margin
        if abs(x) > 5.425 - self.wall_margin:
            return True
        if abs(y) > 5.425 - self.wall_margin:
            return True

        return False

    def _get_obs(self):
        scan = self._get_scan_array()
        dist, heading = self._compute_dist_and_heading()
        v, w = self._get_robot_velocity()

        # 简单归一化
        dist_norm = np.clip(dist / 5.0, 0.0, 1.0)  # 假设 5m 视为“远”
        heading_norm = (heading + np.pi) / (2.0 * np.pi)  # [-pi,pi] -> [0,1]

        v_norm = np.clip((v + self.max_lin_vel) / (2.0 * self.max_lin_vel), 0.0, 1.0)
        w_norm = np.clip((w + self.max_ang_vel) / (2.0 * self.max_ang_vel), 0.0, 1.0)

        extra = np.array(
            [dist_norm, heading_norm, v_norm, w_norm], dtype=np.float32
        )
        obs = np.concatenate([scan, extra], axis=0).astype(np.float32)
        return obs

    def _send_action(self, action):
        """
        action in [-1,1]^2 -> v, w
        """
        a = np.clip(action, -1.0, 1.0)
        v_cmd = (a[0] + 1.0) / 2.0 * self.max_lin_vel  # [0, max_lin_vel]
        w_cmd = a[1] * self.max_ang_vel               # [-max_ang_vel, max_ang_vel]

        cmd = Twist()
        cmd.linear.x = float(v_cmd)
        cmd.angular.z = float(w_cmd)
        self.cmd_pub.publish(cmd)

    def _stop_robot(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def _is_collision(self, scan_norm):
        """
        scan_norm: 0~1, 需要还原到真实距离再判断
        """
        min_range = float(np.min(scan_norm) * self.scan_range_max)
        return min_range < self.collision_distance

    # ---------- 目标可视化 ----------
    def _spawn_or_move_goal_model(self):
        """
        在 Gazebo 中根据当前 self.goal 放置目标模型。

        策略：
          - 进程内第一次调用：spawn 一个模型到当前 goal 位置；
          - 后续调用：只用 SetModelState 把它移动到新位置。
        """
        x = float(self.goal[0])
        y = float(self.goal[1])
        z = 0.025  # 圆柱高度 0.05，取中间略高一点

        if self.spawn_model_srv is None or self.set_model_state_srv is None:
            return

        # 1) 如果还没 spawn 过，先尝试 spawn 一次
        if not self.goal_model_spawned:
            try:
                req = SpawnModelRequest()
                req.model_name = GOAL_MODEL_NAME
                req.model_xml = GOAL_MODEL_SDF
                req.robot_namespace = ""
                req.initial_pose.position.x = x
                req.initial_pose.position.y = y
                req.initial_pose.position.z = z
                req.initial_pose.orientation.x = 0.0
                req.initial_pose.orientation.y = 0.0
                req.initial_pose.orientation.z = 0.0
                req.initial_pose.orientation.w = 1.0

                resp = self.spawn_model_srv(req)

                ok = True
                msg = ""
                if hasattr(resp, "success"):
                    ok = resp.success
                if hasattr(resp, "status_message"):
                    msg = resp.status_message

                if not ok:
                    if "already exists" in msg:
                        rospy.logwarn("Goal model already exists in Gazebo, will just move it.")
                        self.goal_model_spawned = True
                    else:
                        rospy.logwarn("Spawn goal model failed: %s", msg)
                else:
                    self.goal_model_spawned = True
                    rospy.loginfo("Spawned Gazebo goal model at (%.2f, %.2f)", x, y)

            except Exception as e:
                rospy.logwarn("Failed to spawn goal model: %s", str(e))
                return

        # 2) 如果已经 spawn 过（或者确认已存在），就用 SetModelState 移动
        if self.goal_model_spawned:
            try:
                state = ModelState()
                state.model_name = GOAL_MODEL_NAME
                state.pose.position.x = x
                state.pose.position.y = y
                state.pose.position.z = z
                state.pose.orientation.x = 0.0
                state.pose.orientation.y = 0.0
                state.pose.orientation.z = 0.0
                state.pose.orientation.w = 1.0

                self.set_model_state_srv(state)
            except Exception as e:
                rospy.logwarn("Failed to move goal model: %s", str(e))

    def _publish_goal_marker_rviz(self):
        """
        在 RViz 中发布一个 Marker，frame_id 用 'odom'。
        RViz 中添加 Marker 显示：topic=/rl_goal_marker, frame_id=odom
        """
        marker = Marker()
        marker.header.frame_id = "odom"  # 与 odom 坐标系一致
        marker.header.stamp = rospy.Time.now()
        marker.ns = "rl_goal"
        marker.id = 0
        marker.type = Marker.CYLINDER  # 或 Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = float(self.goal[0])
        marker.pose.position.y = float(self.goal[1])
        marker.pose.position.z = 0.025
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.3  # 直径
        marker.scale.y = 0.3
        marker.scale.z = 0.05

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8

        marker.lifetime = rospy.Duration(0.0)  # 0 = 永久

        self.goal_marker_pub.publish(marker)

    # ---------- Gymnasium API ----------
    def reset(self, *, seed=None, options=None):
        """
        Gymnasium 新接口:
            obs, info = env.reset(seed=..., options=...)
        """
        super().reset(seed=seed)  # 让 Gymnasium 内部随机数也同步

        self._stop_robot()
        rospy.sleep(0.1)

        # reset world
        if self.reset_world_srv is not None:
            try:
                self.reset_world_srv()
            except Exception as e:
                rospy.logwarn("reset_world failed: %s", str(e))

        # 等仿真稳定一点
        rospy.sleep(0.5)

        # 确保拿到最新传感器数据
        self.last_scan = rospy.wait_for_message(self.scan_topic, LaserScan)
        self.last_odom = rospy.wait_for_message(self.odom_topic, Odometry)

        # 随机目标 + 在 Gazebo & RViz 中显示
        self._sample_goal()
        self._spawn_or_move_goal_model()
        self._publish_goal_marker_rviz()

        dist, _ = self._compute_dist_and_heading()
        self.prev_dist_to_goal = dist
        self.step_count = 0

        obs = self._get_obs()
        info = {"goal": self.goal.copy()}
        return obs, info

    def step(self, action):
        """
        Gymnasium 新接口:
            obs, reward, terminated, truncated, info
        """
        self.step_count += 1

        # 1) 发送动作
        self._send_action(action)

        # 为了计算奖励里对“前进/转圈”的惩罚和鼓励，这里再算一次 v_cmd, w_cmd
        a = np.clip(action, -1.0, 1.0)
        v_cmd = (a[0] + 1.0) / 2.0 * self.max_lin_vel      # [0, max_lin_vel]
        w_cmd = a[1] * self.max_ang_vel                   # [-max_ang_vel, max_ang_vel]

        # 2) 等待 step_time
        rospy.sleep(self.step_time)

        # 3) 构造观测
        obs = self._get_obs()
        scan_norm = obs[: self.num_beams]
        dist, heading = self._compute_dist_and_heading()
        min_range = float(np.min(scan_norm) * self.scan_range_max)

        # 4) 奖励 & 终止条件
        terminated = False  # 碰撞 / 到达目标
        truncated = False   # 时间用完
        info = {}

        # (1) 距离进展奖励：朝目标前进就加分
        progress = self.prev_dist_to_goal - dist   # >0 表示离目标更近
        reward = 1.5 * progress

        # (2) 朝向惩罚
        reward -= 0.1 * abs(heading)

        # (3) 安全前进奖励
        safe_dist = self.collision_distance + 0.1
        if abs(heading) < np.deg2rad(30) and min_range > safe_dist:
            reward += 0.05 * (v_cmd / self.max_lin_vel)

        # (4) 转圈惩罚
        reward -= 0.02 * (abs(w_cmd) / self.max_ang_vel)

        # (5) 离障碍太近惩罚
        if min_range < safe_dist:
            proximity_penalty = (safe_dist - min_range) / safe_dist
            reward -= 0.25 * proximity_penalty

        # (6) 时间惩罚
        reward -= 0.01

        # 碰撞（终止）
        if self._is_collision(scan_norm):
            reward -= 10.0
            terminated = True
            info["done_reason"] = "collision"

        # 到达目标（终止）
        if dist < self.goal_tolerance:
            reward += 10.0
            terminated = True
            info["done_reason"] = "goal_reached"
            print("Goal reached!")

        # 超时（截断）
        if self.step_count >= self.max_episode_steps and not terminated:
            truncated = True
            info["done_reason"] = "timeout"

        self.prev_dist_to_goal = dist

        # 顺便保持 RViz 里的目标 Marker 新鲜一点（时间戳更新）
        self._publish_goal_marker_rviz()

        return obs, float(reward), terminated, truncated, info

    def render(self):
        pass  # Gazebo 已经在渲染

    def close(self):
        self._stop_robot()
