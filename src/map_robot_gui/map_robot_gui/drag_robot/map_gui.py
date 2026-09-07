#!/usr/bin/env python3
"""
map_gui.py
Pygame GUI：地图渲染、机器人绘制、鼠标拖拽事件处理。
"""
import math
import numpy as np
import pygame
import threading
from typing import List, Optional, Tuple

from map_robot_gui.drag_robot.map_meta import (
    MapMeta,
    world_to_pixel,
    pixel_to_world,
)
from map_robot_gui.drag_robot.robot_agent import RobotAgent


RobotAgentColor  = (0, 0, 255)


# 每个机器人在 GUI 中使用不同配色
_ROBOT_COLORS = [
    ((200, 50,  50),  (255, 100, 100), (0, 200,  50)),   # robot 0: 红
    ((50,  80, 200),  (80, 150, 255),  (0, 200, 200)),   # robot 1: 蓝
    ((50, 180,  50),  (100, 255, 100), (200, 200, 0)),   # robot 2: 绿
    ((180, 100,  0),  (255, 180,  50), (200, 100, 200)), # robot 3: 橙
]


def _robot_color(idx: int):
    return _ROBOT_COLORS[idx % len(_ROBOT_COLORS)]


class MapGui:
    """
    封装 Pygame 窗口，持有对所有 RobotAgent 的引用。

    Parameters
    ----------
    robots : List[RobotAgent]
        所有机器人列表；robots[0] 为主机器人（sentry）
    map_rgb : np.ndarray [H, W, 3]
        地图 RGB 图像
    meta : MapMeta
        地图元信息
    map_w, map_h : int
        地图像素尺寸
    win_w, win_h : int
        窗口像素尺寸
    sim_hz : float
        GUI tick 频率
    """

    def __init__(
        self,
        robots: List[RobotAgent],
        map_rgb: np.ndarray,
        meta: MapMeta,
        map_w: int,
        map_h: int,
        win_w: int = 1600,
        win_h: int = 1000,
        sim_hz: float = 30.0,
    ):
        self.robots = robots
        self.map_rgb = map_rgb
        self.meta = meta
        self.map_w = map_w
        self.map_h = map_h
        self.win_w = win_w
        self.win_h = win_h
        self.sim_hz = sim_hz
        self.map_res = getattr(self.meta, 'resolution', 0.05)

        pygame.init()
        self.screen = pygame.display.set_mode((win_w, win_h))
        pygame.display.set_caption("Map Drag Robot — multi-robot sim")
        self.clock = pygame.time.Clock()

        # 缩放因子（自适应窗口）
        sx = win_w / float(map_w)
        sy = win_h / float(map_h)
        self.window_scale = max(0.05, min(sx, sy))

        # 地图 Surface
        self.map_surface = self._make_map_surface(map_rgb, self.window_scale)
        self.map_rect = self.map_surface.get_rect()
        screen_rect = self.screen.get_rect()
        self.map_offset = (
            (screen_rect.width  - self.map_rect.width)  // 2,
            (screen_rect.height - self.map_rect.height) // 2,
        )

        # 右键当前选中的非主机器人索引（在 self.robots[1:] 中）
        # None 表示尚未选中任何机器人
        self._right_selected: Optional[int] = None   # index into self.robots
        self._right_dragging: bool = False
        self._auto_aim_button_rect = pygame.Rect(12, 12, 210, 34)
        self._be_attacked_button_rect = pygame.Rect(234, 12, 180, 34)
        self._sentry_motion_button_rect = pygame.Rect(426, 12, 200, 34)

        # 字体（可选）
        try:
            self._font = pygame.font.SysFont('monospace', 14)
        except Exception:
            self._font = None

    # ------------------------------------------------------------------
    # 内部工具
    # ------------------------------------------------------------------

    @staticmethod
    def _make_map_surface(rgb_img: np.ndarray, scale: float) -> pygame.Surface:
        surf = pygame.surfarray.make_surface(np.transpose(rgb_img, (1, 0, 2)))
        if abs(scale - 1.0) > 1e-6:
            new_size = (
                int(rgb_img.shape[1] * scale),
                int(rgb_img.shape[0] * scale),
            )
            surf = pygame.transform.smoothscale(surf, new_size)
        return surf

    def _world_to_screen(self, xy_m: np.ndarray) -> Tuple[int, int]:
        px = world_to_pixel(xy_m, self.meta, self.map_h) * self.window_scale
        return (
            int(px[0] + self.map_offset[0]),
            int(px[1] + self.map_offset[1]),
        )

    def _mouse_to_world(self, mx: int, my: int) -> Optional[np.ndarray]:
        mx_map = mx - self.map_offset[0]
        my_map = my - self.map_offset[1]
        if (mx_map < 0 or my_map < 0
                or mx_map >= self.map_rect.width
                or my_map >= self.map_rect.height):
            return None
        px = mx_map / self.window_scale
        py = my_map / self.window_scale
        return pixel_to_world(np.array([px, py], dtype=float), self.meta, self.map_h)

    # ------------------------------------------------------------------
    # 事件处理
    # ------------------------------------------------------------------

    def _pick_radius(self) -> float:
        """屏幕 20px 对应的世界距离，至少 1.0m，用于拾取机器人。"""
        return max(1.0, 20.0 / self.window_scale * self.meta.resolution)

    def _nearest_extra_robot(self, world_pos: np.ndarray) -> Optional[int]:
        """
        在 self.robots[1:] 中找距 world_pos 最近的机器人索引（在 self.robots 中的下标）。
        若无非主机器人则返回 None。
        """
        if len(self.robots) <= 1:
            return None
        best_idx, best_dist = None, float('inf')
        for i, robot in enumerate(self.robots[1:], start=1):
            pos = robot.get_pos_for_drag_check()
            d = float(np.linalg.norm(world_pos - pos))
            if d < best_dist:
                best_dist = d
                best_idx = i
        return best_idx

    def _handle_auto_aim_button_click(self, event) -> bool:
        if event.type != pygame.MOUSEBUTTONDOWN or event.button != 1:
            return False
        if not self._auto_aim_button_rect.collidepoint(event.pos):
            return False

        node = getattr(self, '_node_ref', None)
        if node is not None and hasattr(node, 'toggle_auto_aim_enemy_detected'):
            node.toggle_auto_aim_enemy_detected()
        return True

    def _handle_be_attacked_button_click(self, event) -> bool:
        if event.type != pygame.MOUSEBUTTONDOWN or event.button != 1:
            return False
        if not self._be_attacked_button_rect.collidepoint(event.pos):
            return False

        node = getattr(self, '_node_ref', None)
        if node is not None and hasattr(node, 'toggle_be_attacked'):
            node.toggle_be_attacked()
        return True

    def _handle_sentry_motion_button_click(self, event) -> bool:
        if event.type != pygame.MOUSEBUTTONDOWN or event.button != 1:
            return False
        if not self._sentry_motion_button_rect.collidepoint(event.pos):
            return False

        node = getattr(self, '_node_ref', None)
        if node is not None and hasattr(node, 'toggle_sentry_motion_allowed'):
            node.toggle_sentry_motion_allowed()
        return True

    def handle_events(self, sentry_motion_blocked: bool):
        """
        鼠标控制逻辑：
          左键       → 主机器人（robots[0]）；被禁用时忽略
          右键单击   → 选中距离最近的非主机器人，并设置目标点
          右键拖动   → 持续移动已选中的非主机器人
        """
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt

            if self._handle_auto_aim_button_click(event):
                continue

            if self._handle_be_attacked_button_click(event):
                continue

            if self._handle_sentry_motion_button_click(event):
                continue

            if event.type == pygame.KEYDOWN and event.key == pygame.K_z:
                self.robots[0].set_z_vel(-1.3)
            elif event.type == pygame.KEYUP and event.key == pygame.K_z:
                self.robots[0].set_z_vel(0.0)

            # ── 左键：主机器人 ──────────────────────────────────────────
            if len(self.robots) > 0:
                sentry = self.robots[0]

                if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                    if not sentry_motion_blocked:
                        target = self._mouse_to_world(*pygame.mouse.get_pos())
                        if target is not None:
                            sentry.set_target(target)
                            pos = sentry.get_pos_for_drag_check()
                            if np.linalg.norm(target - pos) <= self._pick_radius():
                                sentry.set_dragging(True)

                if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                    sentry.set_dragging(False)

                if event.type == pygame.MOUSEMOTION and sentry.get_dragging():
                    if not sentry_motion_blocked:
                        target = self._mouse_to_world(*pygame.mouse.get_pos())
                        if target is not None:
                            sentry.set_target(target)
                    else:
                        sentry.set_dragging(False)

            # ── 右键：选中最近的非主机器人并控制 ──────────────────────
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 3:
                target = self._mouse_to_world(*pygame.mouse.get_pos())
                if target is not None:
                    # 找到距点击位置最近的非主机器人
                    idx = self._nearest_extra_robot(target)
                    if idx is not None:
                        robot = self.robots[idx]
                        robot.set_target(target)
                        pos = robot.get_pos_for_drag_check()
                        # 选中该机器人（无论是否点中都选中最近的）
                        self._right_selected = idx
                        # 只有点在其附近才开启拖动跟随
                        if np.linalg.norm(target - pos) <= self._pick_radius():
                            self._right_dragging = True
                        else:
                            self._right_dragging = False

            if event.type == pygame.MOUSEBUTTONUP and event.button == 3:
                self._right_dragging = False

            if event.type == pygame.MOUSEMOTION and self._right_dragging:
                if self._right_selected is not None:
                    target = self._mouse_to_world(*pygame.mouse.get_pos())
                    if target is not None:
                        self.robots[self._right_selected].set_target(target)

    def _draw_auto_aim_button(self):
        node = getattr(self, '_node_ref', None)
        detected = bool(getattr(node, 'auto_aim_enemy_detected', False)) if node else False
        bg_color = (40, 150, 70) if detected else (85, 85, 85)
        border_color = (230, 255, 230) if detected else (210, 210, 210)
        text_color = (255, 255, 255)
        text = f"Enemy Detected: {'ON' if detected else 'OFF'}"

        pygame.draw.rect(self.screen, bg_color, self._auto_aim_button_rect, border_radius=6)
        pygame.draw.rect(self.screen, border_color, self._auto_aim_button_rect, width=2, border_radius=6)
        if self._font:
            label = self._font.render(text, True, text_color)
            label_rect = label.get_rect(center=self._auto_aim_button_rect.center)
            self.screen.blit(label, label_rect)

    def _draw_be_attacked_button(self):
        node = getattr(self, '_node_ref', None)
        be_attacked = bool(getattr(node, 'be_attacked', False)) if node else False
        bg_color = (180, 55, 55) if be_attacked else (85, 85, 85)
        border_color = (255, 220, 220) if be_attacked else (210, 210, 210)
        text_color = (255, 255, 255)
        text = f"Be Attacked: {'ON' if be_attacked else 'OFF'}"

        pygame.draw.rect(self.screen, bg_color, self._be_attacked_button_rect, border_radius=6)
        pygame.draw.rect(self.screen, border_color, self._be_attacked_button_rect, width=2, border_radius=6)
        if self._font:
            label = self._font.render(text, True, text_color)
            label_rect = label.get_rect(center=self._be_attacked_button_rect.center)
            self.screen.blit(label, label_rect)

    def _draw_sentry_motion_button(self):
        node = getattr(self, '_node_ref', None)
        allowed = (
            bool(node.is_sentry_motion_gui_allowed())
            if node and hasattr(node, 'is_sentry_motion_gui_allowed')
            else True
        )
        bg_color = (40, 150, 70) if allowed else (180, 55, 55)
        border_color = (230, 255, 230) if allowed else (255, 220, 220)
        text_color = (255, 255, 255)
        text = f"Sentry Move: {'ALLOW' if allowed else 'BLOCK'}"

        pygame.draw.rect(self.screen, bg_color, self._sentry_motion_button_rect, border_radius=6)
        pygame.draw.rect(self.screen, border_color, self._sentry_motion_button_rect, width=2, border_radius=6)
        if self._font:
            label = self._font.render(text, True, text_color)
            label_rect = label.get_rect(center=self._sentry_motion_button_rect.center)
            self.screen.blit(label, label_rect)

    # ------------------------------------------------------------------
    # 渲染
    # ------------------------------------------------------------------

    def draw(self):
        self.screen.fill((30, 30, 30))
        self.screen.blit(self.map_surface, self.map_offset)

        for idx, robot in enumerate(self.robots):
            pos, vel, yaw = robot.snapshot()
            body_color, arrow_color, cross_color = _robot_color(idx)

            # 目标十字
            with robot.lock:
                target = robot.target.copy()
            tx, ty = self._world_to_screen(target)
            pygame.draw.line(self.screen, cross_color, (tx - 8, ty), (tx + 8, ty), 2)
            pygame.draw.line(self.screen, cross_color, (tx, ty - 8), (tx, ty + 8), 2)

            # 机器人圆（选中的非主机器人加外圈高亮）
            x, y = self._world_to_screen(pos)
            rad_px = max(2, int((robot.robot_radius / self.meta.resolution) * self.window_scale))
            
            # 先画底盘矩形再画圆，免得被圆覆盖
            chassis_yaw = getattr(robot, 'chassis_yaw', yaw)
            is_main = (robot == self.robots[0])
            self._get_rotated_rect(self.screen, (0, 255, 255) if is_main else body_color,
                                pos[0], pos[1], yaw, override_yaw=chassis_yaw)
            
            pygame.draw.circle(self.screen, body_color, (x, y), rad_px, 0)
            if idx > 0 and idx == self._right_selected:
                pygame.draw.circle(self.screen, (255, 255, 255), (x, y), rad_px + 4, 2)

            # 朝向箭头（始终显示 yaw 方向）
            arrow_len = 30
            ex = int(x + math.cos(yaw) * arrow_len)
            ey = int(y - math.sin(yaw) * arrow_len)   # 屏幕 y 轴向下
            pygame.draw.line(self.screen, arrow_color, (x, y), (ex, ey), 2)

            # 速度箭头（细线）
            spd = float(np.linalg.norm(vel))
            if spd > 1e-3:
                dv = vel / spd
                vex = int(x + dv[0] * 20)
                vey = int(y - dv[1] * 20)
                pygame.draw.line(self.screen, (220, 220, 80), (x, y), (vex, vey), 1)

            # ID 标签
            if self._font:
                label = self._font.render(f"R{robot.robot_id}", True, (240, 240, 240))
                self.screen.blit(label, (x + rad_px + 2, y - 8))

        # GUI 更新：获取最新被选中的姿态并在机器人周围画圈
        node = getattr(self.robots[0], '_node', None)
        if hasattr(self, '_node_ref'):
            node = self._node_ref

        # Draw main robot posture circle
        main_robot = self.robots[0]
        pos, _, _ = main_robot.snapshot()
        cx, cy = self._world_to_screen(pos)

        current_posture = getattr(node, 'current_posture', 3) if node else 3
        posture_colors = {
            1: (255, 0, 0),    # 进攻: 红
            2: (0, 255, 0),    # 防御: 绿
            3: (0, 0, 255),    # 移动: 蓝
        }
        color = posture_colors.get(current_posture, (0, 0, 255))
        # 减小直径，比如半径从3.0改成1.0
        pygame.draw.circle(self.screen, color, (cx, cy), int(1.0 / self.map_res * self.window_scale), 2)

        self._draw_auto_aim_button()
        self._draw_be_attacked_button()
        self._draw_sentry_motion_button()

        pygame.display.flip()

    # ------------------------------------------------------------------
    # 主循环（单帧）
    # ------------------------------------------------------------------

    def tick(self, sentry_motion_blocked: bool, now_ns: int):
        """处理事件 + 所有机器人动力学步进 + 渲染，限速到 sim_hz。"""
        self.handle_events(sentry_motion_blocked)
        for robot in self.robots:
            robot.step(now_ns)
        self.draw()
        self.clock.tick(int(self.sim_hz))

    def _get_rotated_rect(self, surf, color, x, y, yaw, override_yaw=None):
        """
        在给定位置绘制一个旋转的矩形（表示底盘），用于调试。
        """
        l = 0.60   # 底盘长 (加大尺寸，以防被表示身体的圆形覆盖)
        w = 0.45   # 底盘宽

        dx2 = l / 2.0
        dy2 = w / 2.0

        yaw_use = yaw
        if override_yaw is not None:
             yaw_use = override_yaw

        # _get_rotated_rect 之前是在原点算，现在需要加上 x, y 坐标转换
        p_local = np.array([
            [dx2, dy2],
            [-dx2, dy2],
            [-dx2, -dy2],
            [dx2, -dy2],
            [dx2, dy2],
        ])
        R = np.array([
            [ math.cos(yaw_use), -math.sin(yaw_use)],
            [ math.sin(yaw_use),  math.cos(yaw_use)],
        ])
        p_rot = p_local @ R
        
        # 将局部坐标系加上中心的坐标，由于 y 轴相反所以 y 加个负号或者重新做转换
        pts_screen = []
        for point in p_rot:
            cx, cy = self._world_to_screen(np.array([x + point[0], y + point[1]]))
            pts_screen.append((cx, cy))
            
        pygame.draw.lines(surf, color, False, pts_screen, 2)
