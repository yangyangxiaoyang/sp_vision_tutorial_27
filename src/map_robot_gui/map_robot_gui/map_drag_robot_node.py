#!/usr/bin/env python3
"""
map_drag_robot_node.py
ROS 2 节点入口：
  - 加载地图，创建若干 RobotAgent
  - 主机器人（sentry）订阅 /sentry/cmd_vel 和 /gimbal/control
  - 所有机器人支持 Pygame 鼠标拖拽
  - 发布 RobotKinematicsArray（/robots 话题）
  - 发布 Odometry（主机器人，/Odometry 话题）
    - 发布局部代价地图（/local_costmap/costmap，内容为其他机器人）
  - 广播 TF：map -> lidar_odom（静态）-> base_link（动态）
             map -> base_link_2（动态）...

GimbalControlMsg 模式（/gimbal/control，与下位机一致）：
  mode == 0  全向扫描：忽略所有角度参数，big_yaw 匀速自主扫描
  mode == 1  三轴指定角度：big_yaw / small_yaw / pitch 设为目标，限位无效
  mode == 2  范围扫描：small_yaw、pitch 在各自限位内往复扫描，目标角度无效
  mode == 3  仅大 yaw 锁定：big_yaw 保持不动，忽略其余参数
  mode == 4  仅大 yaw 指向：仅 big_yaw 生效，其余参数忽略

里程计说明：
  odom.twist.twist.linear.x/y 是 lidar_odom（=map）系下 base_link 原点的线速度。
  odom.header.frame_id = 'lidar_odom'，child_frame_id = 'base_link'。
"""

import math
import threading
from typing import List

import numpy as np
import pygame

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Header
from std_msgs.msg import String
from geometry_msgs.msg import (
    PoseWithCovariance,
    TwistWithCovariance,
    TransformStamped,
    Twist,
)
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from robot_msg.msg import (
    ChassisModeMsg,
    GimbalControlMsg,
    TeamRobotHpMsg,
    RobotKinematics,
    RobotKinematicsArray,
    PostureControlMsg,
    PostureFeedbackMsg,
    ChassisMsg,
)
from std_msgs.msg import Float32
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

from map_robot_gui.drag_robot.map_meta import (
    load_map,
    pixel_to_world,
)
from map_robot_gui.drag_robot.robot_agent import RobotAgent
from map_robot_gui.drag_robot.map_gui import MapGui


class MapDragRobotNode(Node):
    def __init__(self):
        super().__init__('map_drag_robot_node')

        # ---- 参数声明 ----
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('topic', '/robots')
        self.declare_parameter('frame_id', 'map')

        # GUI/仿真频率
        self.declare_parameter('publish_hz', 30.0)
        self.declare_parameter('pub_hz',     30.0)
        self.declare_parameter('odom_hz',    30.0)

        # 动力学约束
        self.declare_parameter('v_max',        3.5)
        self.declare_parameter('a_max',        4.5)
        self.declare_parameter('robot_radius', 0.25)

        # FOPDT 参数（与 controller_minco.launch.py 保持一致）
        self.declare_parameter('tau_x',      0.2923)   # x 轴一阶时间常数 (s)
        self.declare_parameter('tau_y',      0.2986)   # y 轴一阶时间常数 (s)
        self.declare_parameter('delay_x_s',  0.1716)   # x 轴纯延迟 (s)
        self.declare_parameter('delay_y_s',  0.1645)   # y 轴纯延迟 (s)

        # 窗口尺寸
        self.declare_parameter('window_width',  1600)
        self.declare_parameter('window_height', 1000)

        # 地图处理
        self.declare_parameter('treat_unknown_as_occupied', True)

        # 位置测量噪声
        self.declare_parameter('pos_noise_std', 0.1)
        self.declare_parameter('seed', 0)

        # 主机器人（sentry）
        self.declare_parameter('robot_id',            1)
        self.declare_parameter('init_x',              4.0)
        self.declare_parameter('init_y',              7.0)
        self.declare_parameter('tf_child_frame_id',   'base_link')
        self.declare_parameter('cmd_vel_topic',       '/sentry/cmd_vel')
        self.declare_parameter('cmd_vel_timeout',     5.0)
        self.declare_parameter('odom_topic',          '/Odometry')
        self.declare_parameter('odom_frame_id',       'lidar_odom')
        self.declare_parameter('local_costmap_topic',  '/local_costmap/costmap')
        self.declare_parameter('local_costmap_width_m',  6.0)
        self.declare_parameter('local_costmap_height_m', 6.0)
        self.declare_parameter('gimbal_topic',        '/gimbal/control')
        self.declare_parameter('scan_speed',          1.0)
        self.declare_parameter('chassis_topic',       '/chassis/mode')
        self.declare_parameter('team_robot_hp_topic', '/team_robot_hp')
        self.declare_parameter('posture_topic',       '/sentry/posture_control')
        self.declare_parameter('posture_feedback_topic', '/sentry/posture_feedback')
        self.declare_parameter('posture_feedback_hz', 10.0)

        # 其他机器人：以逗号分隔的字符串（可配置任意数量）
        # 例：robot_ids="2,3"  init_xs="20.0,10.0"  init_ys="7.0,5.0"
        #      tf_child_frame_ids="base_link_2,base_link_3"
        self.declare_parameter('robot_ids',           '2')
        self.declare_parameter('init_xs',             '20.0')
        self.declare_parameter('init_ys',             '7.0')
        self.declare_parameter('tf_child_frame_ids',  'base_link_2')

        # ---- 读取参数 ----
        map_yaml = self.get_parameter('map_yaml').get_parameter_value().string_value
        if not map_yaml:
            raise RuntimeError(
                "Parameter 'map_yaml' is empty. "
                "Example: --ros-args -p map_yaml:=/path/to/map.yaml"
            )

        self.topic    = self.get_parameter('topic').value
        self.frame_id = self.get_parameter('frame_id').value

        self.sim_hz  = float(self.get_parameter('publish_hz').value)
        self.pub_hz  = float(self.get_parameter('pub_hz').value)
        self.odom_hz = float(self.get_parameter('odom_hz').value)

        v_max        = float(self.get_parameter('v_max').value)
        a_max        = float(self.get_parameter('a_max').value)
        robot_radius = float(self.get_parameter('robot_radius').value)

        tau_x     = float(self.get_parameter('tau_x').value)
        tau_y     = float(self.get_parameter('tau_y').value)
        delay_x_s = float(self.get_parameter('delay_x_s').value)
        delay_y_s = float(self.get_parameter('delay_y_s').value)

        win_w  = int(self.get_parameter('window_width').value)
        win_h  = int(self.get_parameter('window_height').value)

        treat_unknown = bool(self.get_parameter('treat_unknown_as_occupied').value)

        self.pos_noise_std = float(self.get_parameter('pos_noise_std').value)
        seed = int(self.get_parameter('seed').value)
        if seed == 0:
            seed = int(self.get_clock().now().nanoseconds % (2 ** 31 - 1))
        self.rng = np.random.default_rng(seed)

        sentry_id       = int(self.get_parameter('robot_id').value)
        sentry_x        = float(self.get_parameter('init_x').value)
        sentry_y        = float(self.get_parameter('init_y').value)
        sentry_tf       = self.get_parameter('tf_child_frame_id').get_parameter_value().string_value
        cmd_vel_topic   = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        cmd_vel_timeout = float(self.get_parameter('cmd_vel_timeout').value)
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        self.local_costmap_topic = self.get_parameter('local_costmap_topic').get_parameter_value().string_value
        self.local_costmap_width_m = float(self.get_parameter('local_costmap_width_m').value)
        self.local_costmap_height_m = float(self.get_parameter('local_costmap_height_m').value)
        gimbal_topic    = self.get_parameter('gimbal_topic').get_parameter_value().string_value
        scan_speed      = float(self.get_parameter('scan_speed').value)
        chassis_topic   = self.get_parameter('chassis_topic').get_parameter_value().string_value
        team_robot_hp_topic = self.get_parameter('team_robot_hp_topic').get_parameter_value().string_value
        posture_topic   = self.get_parameter('posture_topic').get_parameter_value().string_value
        posture_feedback_topic = self.get_parameter('posture_feedback_topic').get_parameter_value().string_value
        posture_feedback_hz = float(self.get_parameter('posture_feedback_hz').value)

        extra_ids  = [int(x)   for x in self.get_parameter('robot_ids').value.split(',')          if x.strip()]
        extra_xs   = [float(x) for x in self.get_parameter('init_xs').value.split(',')            if x.strip()]
        extra_ys   = [float(x) for x in self.get_parameter('init_ys').value.split(',')            if x.strip()]
        extra_tfs  = [x.strip() for x in self.get_parameter('tf_child_frame_ids').value.split(',') if x.strip()]

        # ---- 加载地图 ----
        map_rgb, map_occ, meta, map_path = load_map(map_yaml, treat_unknown)
        map_h, map_w = map_rgb.shape[:2]
        self.map_w = map_w
        self.map_h = map_h
        self.map_res = float(meta.resolution)
        self.robot_radius = robot_radius
        self.local_costmap_w = max(1, int(round(self.local_costmap_width_m / self.map_res)))
        self.local_costmap_h = max(1, int(round(self.local_costmap_height_m / self.map_res)))

        default_pos = pixel_to_world(
            np.array([map_w / 2.0, map_h / 2.0]), meta, map_h
        )

        # ---- 创建机器人 ----
        common_kwargs = dict(
            v_max=v_max, a_max=a_max, robot_radius=robot_radius,
            map_occ=map_occ, meta=meta, map_w=map_w, map_h=map_h,
            sim_hz=self.sim_hz, scan_speed=scan_speed,
            tau_x=tau_x, tau_y=tau_y,
            delay_x_s=delay_x_s, delay_y_s=delay_y_s,
        )

        sentry_pos = (
            np.array([sentry_x, sentry_y], dtype=float)
            if not (math.isnan(sentry_x) or math.isnan(sentry_y))
            else default_pos.copy()
        )
        self.sentry = RobotAgent(
            robot_id=sentry_id,
            init_pos=sentry_pos,
            tf_child_frame_id=sentry_tf,
            **common_kwargs,
        )
        self.sentry.set_cmd_vel_timeout(cmd_vel_timeout)
        self._sentry_motion_gui_allowed = True
        self._sentry_motion_hp_allowed = True

        # 其他机器人
        self.extra_robots: List[RobotAgent] = []
        for i, rid in enumerate(extra_ids):
            rx   = extra_xs[i]  if i < len(extra_xs)  else default_pos[0] + 2.0 * (i + 1)
            ry   = extra_ys[i]  if i < len(extra_ys)  else default_pos[1]
            rtf  = extra_tfs[i] if i < len(extra_tfs) else f'base_link_{i + 2}'
            pos_i = (
                np.array([rx, ry], dtype=float)
                if not (math.isnan(rx) or math.isnan(ry))
                else default_pos.copy() + np.array([2.0 * (i + 1), 0.0])
            )
            agent = RobotAgent(
                robot_id=rid, init_pos=pos_i,
                tf_child_frame_id=rtf,
                **common_kwargs,
            )
            self.extra_robots.append(agent)

        # 所有机器人（sentry 在首位，GUI 左键控制）
        self.all_robots: List[RobotAgent] = [self.sentry] + self.extra_robots

        # ---- GUI ----
        self.gui = MapGui(
            robots=self.all_robots,
            map_rgb=map_rgb,
            meta=meta,
            map_w=map_w,
            map_h=map_h,
            win_w=win_w,
            win_h=win_h,
            sim_hz=self.sim_hz,
        )
        self.gui._node_ref = self

        # ---- TF broadcaster ----
        self.tf_broadcaster        = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_map_to_lidar_odom()

        # ---- Publishers ----
        self.robots_pub = self.create_publisher(RobotKinematicsArray, self.topic, 10)
        self.odom_pub   = self.create_publisher(Odometry, self.odom_topic, 10)
        self.local_costmap_pub = self.create_publisher(OccupancyGrid, self.local_costmap_topic, 10)
        
        self.imu_yaw_pub = self.create_publisher(Float32, '/sentry/imu_yaw_deg', 10)
        self.chassis_info_pub = self.create_publisher(ChassisMsg, '/chassis_info', 10)
        self.posture_feedback_pub = self.create_publisher(PostureFeedbackMsg, posture_feedback_topic, 10)
        self.auto_aim_target_pos_pub = self.create_publisher(String, '/auto_aim_target_pos', 10)
        
        self.local_costmap_enabled = True
        self.auto_aim_enemy_detected = False
        self.be_attacked = False
        self.create_service(SetBool, '/set_local_costmap_enable', self._on_set_local_costmap_enable)

        self.last_posture_time = -1.0
        self.current_posture = 3 # default to move posture

        pub_dt  = 1.0 / max(self.pub_hz,  0.1)
        odom_dt = 1.0 / max(self.odom_hz, 1.0)
        posture_feedback_dt = 1.0 / max(posture_feedback_hz, 0.1)
        self.create_timer(pub_dt,  self._on_pub_timer)
        self.create_timer(odom_dt, self._on_odom_timer)
        self.create_timer(posture_feedback_dt, self._on_posture_feedback_timer)

        # ---- Subscribers ----
        self.create_subscription(Twist, cmd_vel_topic, self._on_cmd_vel, 10)
        self.create_subscription(GimbalControlMsg, gimbal_topic, self._on_gimbal, 10)
        self.create_subscription(ChassisModeMsg, chassis_topic, self._on_chassis_mode, 10)
        self.create_subscription(TeamRobotHpMsg, team_robot_hp_topic, self._on_team_robot_hp, 10)
        self.create_subscription(PostureControlMsg, posture_topic, self._on_posture, 10)

        self.get_logger().info(
            f"MapDragRobotNode ready | map={map_path} | "
            f"robots={[r.robot_id for r in self.all_robots]} | "
            f"sim={self.sim_hz}Hz pub={self.pub_hz}Hz odom={self.odom_hz}Hz"
        )
        self.get_logger().info(
            f"gimbal_topic={gimbal_topic} scan_speed={scan_speed} rad/s"
        )
        self.get_logger().info(
            f"chassis_topic={chassis_topic}"
        )
        self.get_logger().info(
            f"team_robot_hp_topic={team_robot_hp_topic}"
        )
        self.get_logger().info(
            f"local_costmap_topic={self.local_costmap_topic} frame={self.frame_id} "
            f"window={self.local_costmap_width_m:.2f}x{self.local_costmap_height_m:.2f}m"
        )
        self.get_logger().info(
            f"posture_feedback_topic={posture_feedback_topic} hz={posture_feedback_hz:.1f}"
        )

    # ------------------------------------------------------------------
    # 静态 TF：map -> lidar_odom（恒等，一次性广播）
    # ------------------------------------------------------------------

    def _publish_static_map_to_lidar_odom(self):
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame_id    # map
        t.child_frame_id  = self.odom_frame  # lidar_odom
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(t)

    # ------------------------------------------------------------------
    # 订阅回调
    # ------------------------------------------------------------------

    def _on_cmd_vel(self, msg: Twist):
        now_ns = self.get_clock().now().nanoseconds
        self.sentry.on_cmd_vel(float(msg.linear.x), float(msg.linear.y), now_ns)

    def _on_gimbal(self, msg: GimbalControlMsg):
        self.sentry.on_gimbal_control(
            int(msg.mode),
            float(msg.big_yaw),
            float(msg.small_yaw),
            float(msg.pitch),
            float(msg.small_yaw_lower_limit),
            float(msg.small_yaw_upper_limit),
            float(msg.pitch_lower_limit),
            float(msg.pitch_upper_limit),
        )

    def _on_chassis_mode(self, msg: ChassisModeMsg):
        self.sentry.set_chassis_stop(bool(msg.is_stop))
        self.sentry.chassis_mode = getattr(msg, 'mode', 0)
        if not bool(msg.is_stop):
            self.sentry.chassis_rotate_velocity = float(msg.rotate_velocity)
        else:
            self.sentry.chassis_rotate_velocity = 0.0

    def _on_team_robot_hp(self, msg: TeamRobotHpMsg):
        self._sentry_motion_hp_allowed = int(msg.sentry_hp) > 0
        self.sentry.set_hp_enabled(self.is_sentry_motion_allowed())
        
    def _on_posture(self, msg: PostureControlMsg):
        now_sec = self.get_clock().now().nanoseconds / 1e9
        # 仅当姿态不同时才检查冷却，相同姿态不刷新 CD
        if msg.posture_type != self.current_posture:
            if now_sec - self.last_posture_time > 5.0:
                self.current_posture = msg.posture_type
                self.last_posture_time = now_sec

    def _on_set_local_costmap_enable(self, request, response):
        self.local_costmap_enabled = request.data
        response.success = True
        return response

    def _on_posture_feedback_timer(self):
        msg = PostureFeedbackMsg()
        msg.current_posture = int(self.current_posture)
        msg.be_attacked = bool(self.be_attacked)
        msg.shooter_17mm_1_barrel_heat = 0
        self.posture_feedback_pub.publish(msg)

    def is_sentry_motion_gui_allowed(self) -> bool:
        return self._sentry_motion_gui_allowed

    def is_sentry_motion_allowed(self) -> bool:
        return self._sentry_motion_gui_allowed and self._sentry_motion_hp_allowed

    def toggle_sentry_motion_allowed(self) -> bool:
        self._sentry_motion_gui_allowed = not self._sentry_motion_gui_allowed
        self.sentry.set_hp_enabled(self.is_sentry_motion_allowed())
        self.get_logger().info(
            f"Sentry motion GUI {'ALLOW' if self._sentry_motion_gui_allowed else 'BLOCK'} "
            f"(effective={'allowed' if self.is_sentry_motion_allowed() else 'forbidden'})"
        )
        return self._sentry_motion_gui_allowed

    def toggle_auto_aim_enemy_detected(self) -> bool:
        self.auto_aim_enemy_detected = not self.auto_aim_enemy_detected

        self.get_logger().info(
            f"Set /auto_aim_target_pos enemy_detected={int(self.auto_aim_enemy_detected)}"
        )
        return self.auto_aim_enemy_detected

    def toggle_be_attacked(self) -> bool:
        self.be_attacked = not self.be_attacked

        self.get_logger().info(
            f"Set posture feedback be_attacked={int(self.be_attacked)}"
        )
        return self.be_attacked

    def _publish_auto_aim_target_pos(self):
        msg = String()
        msg.data = f"0,0,{1 if self.auto_aim_enemy_detected else 0},0"
        self.auto_aim_target_pos_pub.publish(msg)

    # ------------------------------------------------------------------
    # 定时发布
    # ------------------------------------------------------------------

    def _on_pub_timer(self):
        now = self.get_clock().now().to_msg()
        arr = RobotKinematicsArray()
        arr.header = Header()
        arr.header.stamp    = now
        arr.header.frame_id = self.frame_id

        kinematic_list = []
        for robot in self.all_robots:
            pos, vel, yaw = robot.snapshot()
            nx, ny = self.rng.normal(0.0, self.pos_noise_std, size=2)

            r = RobotKinematics()
            r.id = int(robot.robot_id)

            pose = PoseWithCovariance()
            pose.pose.position.x = float(pos[0] + nx)
            pose.pose.position.y = float(pos[1] + ny)
            pose.pose.position.z = 0.0
            half_yaw = yaw * 0.5
            pose.pose.orientation.z = math.sin(half_yaw)
            pose.pose.orientation.w = math.cos(half_yaw)
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            cov = [0.0] * 36
            cov[0] = self.pos_noise_std ** 2
            cov[7] = self.pos_noise_std ** 2
            pose.covariance = cov
            r.pose = pose

            tw = TwistWithCovariance()
            tw.twist.linear.x = float(vel[0])
            tw.twist.linear.y = float(vel[1])
            tw.twist.linear.z = 0.0
            tw.covariance = [0.0] * 36
            r.twist = tw

            kinematic_list.append(r)

        arr.robots = kinematic_list
        self.robots_pub.publish(arr)
        self._publish_local_costmap(now)
        self._publish_auto_aim_target_pos()

    def _publish_local_costmap(self, stamp):
        sentry_pos, _, _ = self.sentry.snapshot()

        local = OccupancyGrid()
        local.header.stamp = stamp
        local.header.frame_id = self.frame_id
        local.info.resolution = self.map_res
        local.info.width = self.local_costmap_w
        local.info.height = self.local_costmap_h
        local.info.origin.position.x = float(sentry_pos[0] - 0.5 * self.local_costmap_w * self.map_res)
        local.info.origin.position.y = float(sentry_pos[1] - 0.5 * self.local_costmap_h * self.map_res)
        local.info.origin.position.z = 0.0
        local.info.origin.orientation.w = 1.0
        local.data = [0] * (self.local_costmap_w * self.local_costmap_h)

        if self.local_costmap_enabled:
            for robot in self.extra_robots:
                pos, vel, yaw = robot.snapshot()
                self._paint_robot_footprint(
                    local.data,
                    local.info.origin.position.x,
                    local.info.origin.position.y,
                    self.local_costmap_w,
                    self.local_costmap_h,
                    pos
                )

        self.local_costmap_pub.publish(local)

    def _paint_robot_footprint(
        self,
        grid_data,
        origin_x: float,
        origin_y: float,
        grid_w: int,
        grid_h: int,
        pos: np.ndarray,
    ):
        radius_cells = max(1, int(math.ceil(self.robot_radius / self.map_res)))
        col_center = int((float(pos[0]) - origin_x) / self.map_res)
        row_center = int((float(pos[1]) - origin_y) / self.map_res)

        col0 = max(0, col_center - radius_cells)
        col1 = min(grid_w - 1, col_center + radius_cells)
        row0 = max(0, row_center - radius_cells)
        row1 = min(grid_h - 1, row_center + radius_cells)
        radius_sq = self.robot_radius * self.robot_radius

        for row in range(row0, row1 + 1):
            cy = origin_y + (row + 0.5) * self.map_res
            dy = cy - float(pos[1])
            for col in range(col0, col1 + 1):
                cx = origin_x + (col + 0.5) * self.map_res
                dx = cx - float(pos[0])
                if dx * dx + dy * dy <= radius_sq:
                    grid_data[row * grid_w + col] = 100

    def _on_odom_timer(self):
        """
        发布 Odometry（主机器人）+ 动态 TF。

        TF 树：
          map ──(static)──> lidar_odom ──(dynamic)──> base_link   (sentry)
          map ──(dynamic)──> base_link_2, base_link_3, ...

        Odometry：
          header.frame_id = lidar_odom
          child_frame_id  = base_link
          twist.linear.x/y = lidar_odom 系下 base_link 原点速度
            （lidar_odom 与 map 重合，数值即 map 系速度）
        """
        now = self.get_clock().now().to_msg()

        # 主机器人 Odometry
        pos, vel, yaw = self.sentry.snapshot()
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.sentry.tf_child_frame_id
        odom.pose.pose.position.x = float(pos[0])
        odom.pose.pose.position.y = float(pos[1])
        odom.pose.pose.position.z = 0.0
        half_yaw = yaw * 0.5
        odom.pose.pose.orientation.z = math.sin(half_yaw)
        odom.pose.pose.orientation.w = math.cos(half_yaw)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.covariance    = [0.0] * 36
        odom.pose.covariance[0] = self.pos_noise_std ** 2
        odom.pose.covariance[7] = self.pos_noise_std ** 2
        odom.twist.twist.linear.x  = float(vel[0])
        odom.twist.twist.linear.y  = float(vel[1])
        
        z_v_override = self.sentry.get_z_vel_override()
        if z_v_override != 0.0:
            odom.twist.twist.linear.z  = z_v_override
        else:
            odom.twist.twist.linear.z  = 0.0
            
        odom.twist.twist.angular.z = 0.0
        odom.twist.covariance = [0.0] * 36
        self.odom_pub.publish(odom)
        
        imu_yaw_msg = Float32()
        imu_yaw_msg.data = yaw * 180.0 / math.pi
        self.imu_yaw_pub.publish(imu_yaw_msg)
        
        chassis_info_msg = ChassisMsg()
        chassis_info_msg.stamp = now
        chassis_info_msg.vx = float(vel[0])
        chassis_info_msg.vy = float(vel[1])
        chassis_yaw = getattr(self.sentry, 'chassis_yaw', 0.0)
        dt = 1.0 / self.odom_hz
        
        # mode 1=底盘跟随, 3=差速对正(仿真等同跟随): rotate chassis towards gimbal front
        chassis_mode = getattr(self.sentry, 'chassis_mode', 0)
        target_chassis_yaw = yaw
        
        if chassis_mode in (1, 3):
            # Rotate towards yaw with a set angular velocity
            yaw_err = target_chassis_yaw - chassis_yaw
            while yaw_err > math.pi:
                yaw_err -= 2 * math.pi
            while yaw_err < -math.pi:
                yaw_err += 2 * math.pi
                
            rotate_speed = 3.0 # radians per second speed limit
            step = rotate_speed * dt
            if abs(yaw_err) < step:
                self.sentry.chassis_yaw = target_chassis_yaw
            else:
                self.sentry.chassis_yaw = chassis_yaw + math.copysign(step, yaw_err)
        elif chassis_mode == 2:
            base_vel = getattr(self.sentry, 'chassis_rotate_velocity', 3.0)
            if base_vel == 0.0:
                base_vel = 3.0 # Fallback default angular velocity instead of 0 if msg doesn't provide
            now_sec = now.sec + now.nanosec / 1e9
            # 变化幅度设置大一些：波动范围在 0.2*base_vel 到 1.8*base_vel 之间，周期根据时间改变
            current_vel = base_vel * (1.0 + 0.8 * math.sin(now_sec * 2.0))
            self.sentry.chassis_yaw = chassis_yaw + current_vel * dt
        else:
            self.sentry.chassis_yaw = chassis_yaw + getattr(self.sentry, 'chassis_rotate_velocity', 0.0) * dt
        
        yaw_diff = self.sentry.chassis_yaw - yaw
        while yaw_diff > math.pi:
            yaw_diff -= 2 * math.pi
        while yaw_diff < -math.pi:
            yaw_diff += 2 * math.pi
        chassis_info_msg.yaw = yaw_diff
        
        self.chassis_info_pub.publish(chassis_info_msg)

        # 动态 TF：lidar_odom -> base_link（sentry）
        self.tf_broadcaster.sendTransform(
            self._make_tf(self.odom_frame, self.sentry.tf_child_frame_id,
                          pos, yaw, now)
        )

        # 动态 TF：map -> base_link_N（其他机器人）
        for robot in self.extra_robots:
            pos_r, _, yaw_r = robot.snapshot()
            self.tf_broadcaster.sendTransform(
                self._make_tf(self.frame_id, robot.tf_child_frame_id,
                              pos_r, yaw_r, now)
            )

    @staticmethod
    def _make_tf(parent: str, child: str,
                 pos: np.ndarray, yaw: float, stamp) -> TransformStamped:
        t = TransformStamped()
        t.header.stamp    = stamp
        t.header.frame_id = parent
        t.child_frame_id  = child
        t.transform.translation.x = float(pos[0])
        t.transform.translation.y = float(pos[1])
        t.transform.translation.z = 0.0
        half_yaw = yaw * 0.5
        t.transform.rotation.z = math.sin(half_yaw)
        t.transform.rotation.w = math.cos(half_yaw)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        return t


# --------------------------------------------------------------------------


def main():
    rclpy.init()
    node = None
    try:
        node = MapDragRobotNode()

        executor = MultiThreadedExecutor()
        executor.add_node(node)
        ros_thread = threading.Thread(target=executor.spin, daemon=True)
        ros_thread.start()

        while rclpy.ok():
            now_ns = node.get_clock().now().nanoseconds
            sentry_cmd_vel_active = node.sentry.is_cmd_vel_active(now_ns)
            sentry_motion_allowed = node.is_sentry_motion_allowed()
            sentry_motion_blocked = sentry_cmd_vel_active or not sentry_motion_allowed
            node.gui.tick(sentry_motion_blocked, now_ns)

    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        pygame.quit()


if __name__ == '__main__':
    main()
