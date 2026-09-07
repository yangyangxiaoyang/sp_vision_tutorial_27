#!/usr/bin/env python3
"""
robot_agent.py
面向对象封装单个机器人的运动状态与动力学。

每个 RobotAgent 持有：
  - 位置 pos、速度 vel（均在 map/world 坐标系下）
  - big_yaw：map 系大 yaw，由 GimbalControlMsg 控制
  - small_yaw / pitch：云台小 yaw 与俯仰（rad），仿真中独立维护
  - cmd_vel：**云台系（gimbal frame）下的速度指令**，
    由 yaw 旋转到 map 系后驱动底盘位移

GimbalControlMsg 模式（与下位机一致）：
  mode == 0  全向扫描：忽略所有角度参数，big_yaw 匀速自主扫描
  mode == 1  三轴指定角度：big_yaw / small_yaw / pitch 设为目标，限位无效
  mode == 2  范围扫描：small_yaw、pitch 在各自限位内往复扫描，目标角度无效
  mode == 3  仅大 yaw 锁定：big_yaw 保持不动，忽略其余参数
  mode == 4  仅大 yaw 指向：仅 big_yaw 生效，其余参数忽略
"""

import collections
import math
import threading
from typing import Optional, Tuple

import numpy as np

from map_robot_gui.drag_robot.map_meta import (
    MapMeta,
    clamp_norm,
    in_collision_world,
    segment_in_collision,
    estimate_obstacle_normal,
    find_nearest_free_world,
)


class RobotAgent:
    """
    单个机器人的运动状态与动力学。

    yaw 语义
    --------
    self.yaw 表示 big_yaw（map 系，rad），用于 cmd_vel 旋转与里程计朝向。
    self.small_yaw / self.pitch 为云台小 yaw 与俯仰（rad），由 GimbalControlMsg 按模式更新。
    - 底盘位移（cmd_vel / 拖拽）不会覆盖云台角度
    - cmd_vel 的 x/y 是云台系下的速度，step() 中用当前 yaw 旋转到 map 系后驱动底盘

    Parameters
    ----------
    robot_id : int
        机器人 ID（在 RobotKinematics.id 中发布）
    init_pos : np.ndarray
        初始世界坐标 [x, y]（米）
    v_max : float
        最大速度（m/s）
    a_max : float
        最大加速度（m/s²）
    robot_radius : float
        碰撞半径（m）
    map_occ : np.ndarray
        占用栅格 bool 数组 [H, W]
    meta : MapMeta
        地图元信息
    map_w, map_h : int
        地图像素尺寸
    tf_child_frame_id : str
        TF 子帧名称（map -> lidar_odom -> tf_child_frame_id）
    scan_speed : float
        全向扫描角速度（mode==0），rad/s
    """

    def __init__(
        self,
        robot_id: int,
        init_pos: np.ndarray,
        v_max: float,
        a_max: float,
        robot_radius: float,
        map_occ: np.ndarray,
        meta: MapMeta,
        map_w: int,
        map_h: int,
        tf_child_frame_id: str = 'base_link',
        sim_hz: float = 30.0,
        scan_speed: float = 1.0,
        # FOPDT 一阶惯性+纯延迟参数（与控制器保持一致）
        tau_x: float = 0.2923,    # x 轴时间常数 (s)
        tau_y: float = 0.2986,    # y 轴时间常数 (s)
        delay_x_s: float = 0.1716,  # x 轴纯延迟 (s)
        delay_y_s: float = 0.1645,  # y 轴纯延迟 (s)
    ):
        self.robot_id = robot_id
        self.v_max = v_max
        self.a_max = a_max
        self.robot_radius = robot_radius
        self.map_occ = map_occ
        self.meta = meta
        self.map_w = map_w
        self.map_h = map_h
        self.tf_child_frame_id = tf_child_frame_id
        self.dt = 1.0 / max(sim_hz, 1.0)
        self.scan_speed = scan_speed  # rad/s for mode 0

        # --- FOPDT 一阶惯性 + 纯延迟（离散化） ---
        # beta = 1 - exp(-dt/tau)：一阶离散化增益
        self._beta_x = 1.0 - math.exp(-self.dt / max(tau_x, 1e-6))
        self._beta_y = 1.0 - math.exp(-self.dt / max(tau_y, 1e-6))
        # 纯延迟步数 = round(L / dt)
        _delay_steps_x = max(0, round(delay_x_s / self.dt))
        _delay_steps_y = max(0, round(delay_y_s / self.dt))
        # 延迟缓冲队列：maxlen = delay_steps+1，队首为最旧（已延迟）的输入
        self._delay_buf_x: collections.deque = collections.deque(
            [0.0] * (_delay_steps_x + 1), maxlen=_delay_steps_x + 1)
        self._delay_buf_y: collections.deque = collections.deque(
            [0.0] * (_delay_steps_y + 1), maxlen=_delay_steps_y + 1)
        # FOPDT 等效制动滞后时间：delay + tau（用于拖拽减速剖面）
        # 机器人从速度 v 命令归零后，约需 (delay + tau) 秒才能完全停止
        self._fopdt_lag_x = delay_x_s + tau_x   # ≈ 0.464 s
        self._fopdt_lag_y = delay_y_s + tau_y   # ≈ 0.461 s

        # --- 运动状态 ---
        self.pos = init_pos.copy().astype(float)
        self.vel = np.array([0.0, 0.0], dtype=float)
        self.yaw = 0.0          # big_yaw，map 系，rad
        self.small_yaw = 0.0    # 小 yaw，rad
        self.pitch = 0.0        # 俯仰，rad
        self.target = self.pos.copy()
        self.dragging = False

        # --- 云台控制状态（GimbalControlMsg 最新值，deg）---
        self.gimbal_mode: int = 0
        self.gimbal_big_yaw_deg: float = 0.0
        self.gimbal_small_yaw_deg: float = 0.0
        self.gimbal_pitch_deg: float = 0.0
        self.gimbal_small_yaw_lower_deg: float = -180.0
        self.gimbal_small_yaw_upper_deg: float = 180.0
        self.gimbal_pitch_lower_deg: float = -90.0
        self.gimbal_pitch_upper_deg: float = 90.0
        # mode==2 往复扫描方向
        self._small_yaw_scan_dir: float = 1.0
        self._pitch_scan_dir: float = 1.0

        # --- cmd_vel（仅主机器人使用） ---
        # 以 base_link 系存储，每帧在 step() 中用当前 yaw 转换到 map 系
        # 这样云台旋转（mode==0/2）时运动方向实时跟随 yaw 变化
        self._cmd_vel_vx_bl: float = 0.0
        self._cmd_vel_vy_bl: float = 0.0
        self._cmd_vel_last_time_ns: Optional[int] = None
        self._cmd_vel_timeout_ns: int = int(5.0 * 1e9)

        # 底盘强制停止标志（由 ChassisModeMsg.is_stop 控制）
        self._chassis_stop: bool = False
        # HP 禁用标志：hp<=0 时无条件冻结位移，不覆盖 chassis_stop 状态
        self._hp_disabled: bool = False
        
        # 强制 z 轴速度
        self.z_vel_override = 0.0

        # 线程锁（供外部 GUI 线程和 ROS 线程共享）
        self.lock = threading.Lock()

        # 若初始位置碰撞则自动修正
        self._fix_initial_collision()

    # ------------------------------------------------------------------
    # 碰撞辅助（转发到模块级函数）
    # ------------------------------------------------------------------

    def _in_collision(self, pos: np.ndarray) -> bool:
        return in_collision_world(
            pos, self.map_occ, self.meta, self.map_w, self.map_h, self.robot_radius
        )

    def _seg_collision(self, p0: np.ndarray, p1: np.ndarray) -> bool:
        return segment_in_collision(
            p0, p1, self.map_occ, self.meta, self.map_w, self.map_h, self.robot_radius
        )

    def _obstacle_normal(self, pos: np.ndarray) -> Optional[np.ndarray]:
        return estimate_obstacle_normal(
            pos, self.map_occ, self.meta, self.map_w, self.map_h, self.robot_radius
        )

    def _fix_initial_collision(self):
        if self._in_collision(self.pos):
            ok, new_pos = find_nearest_free_world(
                self.pos, self.map_occ, self.meta,
                self.map_w, self.map_h, self.robot_radius, max_radius_m=2.0
            )
            if ok:
                self.pos = new_pos
                self.target = self.pos.copy()

    # ------------------------------------------------------------------
    # cmd_vel（主机器人专用）
    # ------------------------------------------------------------------

    def set_cmd_vel_timeout(self, timeout_s: float):
        self._cmd_vel_timeout_ns = int(timeout_s * 1e9)

    def on_cmd_vel(self, vx_bl: float, vy_bl: float, now_ns: int):
        """
        接收并缓存机体系（base_link）速度，不在此处旋转到世界系。
        每帧 step() 中使用当前 yaw 实时旋转，确保云台旋转时运动方向同步更新。
        接收 cmd_vel 不影响云台运动模式，gimbal 状态保持不变，继续在 step() 中更新。
        """
        with self.lock:
            self._cmd_vel_vx_bl = vx_bl
            self._cmd_vel_vy_bl = vy_bl
            self._cmd_vel_last_time_ns = now_ns

    def set_chassis_stop(self, stop: bool):
        """设置底盘强制停止标志。stop=True 时冻结运动。"""
        with self.lock:
            self._chassis_stop = stop

    def set_hp_enabled(self, enabled: bool):
        """设置 HP 运动门控。enabled=False 时无条件禁止位移。"""
        with self.lock:
            self._hp_disabled = not enabled
            
    def set_z_vel(self, val: float):
        """设置 z 轴速度覆盖值"""
        with self.lock:
            self.z_vel_override = val

    def is_cmd_vel_active(self, now_ns: int) -> bool:
        if self._cmd_vel_last_time_ns is None:
            return False
        return (now_ns - self._cmd_vel_last_time_ns) < self._cmd_vel_timeout_ns

    # ------------------------------------------------------------------
    # 云台控制
    # ------------------------------------------------------------------

    def on_gimbal_control(
        self,
        mode: int,
        big_yaw_deg: float,
        small_yaw_deg: float,
        pitch_deg: float,
        small_yaw_lower_deg: float,
        small_yaw_upper_deg: float,
        pitch_lower_deg: float,
        pitch_upper_deg: float,
    ):
        """缓存 GimbalControlMsg 全部字段；具体生效逻辑见 _update_gimbal_from_control。"""
        with self.lock:
            if self.gimbal_mode != mode:
                self._small_yaw_scan_dir = 1.0
                self._pitch_scan_dir = 1.0
            self.gimbal_mode = mode
            self.gimbal_big_yaw_deg = big_yaw_deg
            self.gimbal_small_yaw_deg = small_yaw_deg
            self.gimbal_pitch_deg = pitch_deg
            self.gimbal_small_yaw_lower_deg = small_yaw_lower_deg
            self.gimbal_small_yaw_upper_deg = small_yaw_upper_deg
            self.gimbal_pitch_lower_deg = pitch_lower_deg
            self.gimbal_pitch_upper_deg = pitch_upper_deg

    @staticmethod
    def _normalize_angle_rad(angle: float) -> float:
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def _reciprocate_scan_(
        self,
        current_rad: float,
        lower_deg: float,
        upper_deg: float,
        scan_dir_attr: str,
    ) -> float:
        lo = math.radians(lower_deg)
        hi = math.radians(upper_deg)
        if lo > hi:
            lo, hi = hi, lo
        scan_dir = getattr(self, scan_dir_attr)
        value = current_rad + scan_dir * self.scan_speed * self.dt
        if value >= hi:
            value = hi
            setattr(self, scan_dir_attr, -1.0)
        elif value <= lo:
            value = lo
            setattr(self, scan_dir_attr, 1.0)
        return value

    def _update_gimbal_from_control(self):
        """根据 gimbal_mode 更新 big_yaw / small_yaw / pitch，必须在 lock 内调用。"""
        mode = self.gimbal_mode
        if mode == 0:
            # 全向扫描：忽略所有参数，big_yaw 匀速旋转
            self.yaw += self.scan_speed * self.dt
            self.yaw = self._normalize_angle_rad(self.yaw)
        elif mode == 1:
            # 三轴指定角度：目标角度生效，限位无效
            self.yaw = math.radians(self.gimbal_big_yaw_deg)
            self.small_yaw = math.radians(self.gimbal_small_yaw_deg)
            self.pitch = math.radians(self.gimbal_pitch_deg)
        elif mode == 2:
            # 范围扫描：small_yaw、pitch 在限位内往复，目标角度无效
            self.small_yaw = self._reciprocate_scan_(
                self.small_yaw,
                self.gimbal_small_yaw_lower_deg,
                self.gimbal_small_yaw_upper_deg,
                '_small_yaw_scan_dir',
            )
            self.pitch = self._reciprocate_scan_(
                self.pitch,
                self.gimbal_pitch_lower_deg,
                self.gimbal_pitch_upper_deg,
                '_pitch_scan_dir',
            )
        elif mode == 3:
            # 仅 big_yaw 锁定：保持当前 big_yaw，忽略其余参数
            pass
        elif mode == 4:
            # 仅大 yaw 指向：仅 big_yaw 生效
            self.yaw = math.radians(self.gimbal_big_yaw_deg)

    # ------------------------------------------------------------------
    # 动力学步进（含碰撞 + 墙壁滑动）
    # ------------------------------------------------------------------

    def step(self, now_ns: int):
        """一帧动力学步进，含碰撞检测与云台 yaw 更新。"""
        with self.lock:
            self._step_locked(now_ns)

    def _step_locked(self, now_ns: int):
        """必须在 lock 内调用。"""
        prev_pos = self.pos.copy()

        # HP 禁用优先级最高：无条件冻结位移，但不篡改 chassis_stop 状态
        if self._hp_disabled:
            self.vel = np.array([0.0, 0.0], dtype=float)
            # 重置延迟缓冲，避免恢复后积压输入突然释放
            for buf in (self._delay_buf_x, self._delay_buf_y):
                for i in range(len(buf)):
                    buf[i] = 0.0
            self._update_gimbal_from_control()
            return

        # 底盘强制停止：速度归零，不更新位置
        if self._chassis_stop:
            self.vel = np.array([0.0, 0.0], dtype=float)
            for buf in (self._delay_buf_x, self._delay_buf_y):
                for i in range(len(buf)):
                    buf[i] = 0.0
            self._update_gimbal_from_control()
            return

        use_cmd_vel = self.is_cmd_vel_active(now_ns)

        if use_cmd_vel:
            self.target = self.pos.copy()
            # 用当前 yaw 将 base_link 系速度实时旋转到 map 系
            # 这样 mode==0/2 云台旋转时，运动方向随 yaw 同步变化
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            vx_map = c * self._cmd_vel_vx_bl - s * self._cmd_vel_vy_bl
            vy_map = s * self._cmd_vel_vx_bl + c * self._cmd_vel_vy_bl
            v_des = clamp_norm(np.array([vx_map, vy_map], dtype=float), self.v_max)
        else:
            to_goal = self.target - self.pos
            dist = float(np.linalg.norm(to_goal))
            if dist < 0.05:
                # 到达目标：清空延迟缓冲，防止历史非零输入继续驱动导致震荡
                v_des = np.array([0.0, 0.0], dtype=float)
                self.vel = np.array([0.0, 0.0], dtype=float)
                for buf in (self._delay_buf_x, self._delay_buf_y):
                    for i in range(len(buf)):
                        buf[i] = 0.0
            else:
                dir_vec = to_goal / max(dist, 1e-9)
                # FOPDT-aware 减速剖面：制动距离 = v * (delay + tau)
                # 取 x/y 两轴中较大的滞后时间作为保守估计
                fopdt_lag = max(self._fopdt_lag_x, self._fopdt_lag_y)
                # 当前速度在滞后时间内的惯性滑行距离不超过剩余距离
                v_stop = dist / max(fopdt_lag, self.dt)
                speed = min(self.v_max, v_stop)
                v_des = dir_vec * speed

        # FOPDT：将期望速度推入延迟缓冲队列，读取已延迟的输入
        self._delay_buf_x.append(v_des[0])
        self._delay_buf_y.append(v_des[1])
        u_x = self._delay_buf_x[0]  # 队首 = 最旧 = 延迟 delay_steps 步的输入
        u_y = self._delay_buf_y[0]

        vel_new = np.array([
            (1.0 - self._beta_x) * self.vel[0] + self._beta_x * u_x,
            (1.0 - self._beta_y) * self.vel[1] + self._beta_y * u_y,
        ], dtype=float)
        vel_new = clamp_norm(vel_new, self.v_max)
        pos_new = self.pos + vel_new * self.dt

        if not self._seg_collision(self.pos, pos_new):
            self.pos = pos_new
            self.vel = vel_new
        else:
            # 碰撞：尝试墙壁滑动
            n = self._obstacle_normal(self.pos)
            if n is None:
                self.pos = prev_pos
                self.vel = np.array([0.0, 0.0], dtype=float)
            else:
                vn = float(np.dot(vel_new, n))
                v_t = vel_new - vn * n
                if float(np.linalg.norm(v_t)) < 1e-4:
                    self.pos = prev_pos
                    self.vel = np.array([0.0, 0.0], dtype=float)
                else:
                    v_t = clamp_norm(v_t, self.v_max)
                    slid = False
                    for s in (1.0, 0.6, 0.3, 0.15, 0.0):
                        v_try = v_t * s
                        pos_try = self.pos + v_try * self.dt
                        if not self._seg_collision(self.pos, pos_try):
                            self.pos = pos_try
                            self.vel = v_try
                            slid = True
                            break
                    if not slid:
                        self.pos = prev_pos
                        self.vel = np.array([0.0, 0.0], dtype=float)

        # 云台控制更新（无论底盘是否运动，始终执行）
        self._update_gimbal_from_control()

    # ------------------------------------------------------------------
    # 快照读取（线程安全）
    # ------------------------------------------------------------------

    def snapshot(self) -> Tuple[np.ndarray, np.ndarray, float]:
        """返回 (pos, vel, yaw) 的副本。"""
        with self.lock:
            return self.pos.copy(), self.vel.copy(), float(self.yaw)
            
    def get_z_vel_override(self) -> float:
        with self.lock:
            return self.z_vel_override

    # ------------------------------------------------------------------
    # 拖拽接口（由 GUI 线程调用）
    # ------------------------------------------------------------------

    def set_target(self, target: np.ndarray):
        with self.lock:
            self.target = target.copy()

    def set_dragging(self, flag: bool):
        with self.lock:
            self.dragging = flag

    def get_dragging(self) -> bool:
        with self.lock:
            return self.dragging

    def get_pos_for_drag_check(self) -> np.ndarray:
        with self.lock:
            return self.pos.copy()
