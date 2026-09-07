# sp_vision_tutorial_27

同济大学 SuperPower 战队 2027 赛季算法组招新仓库

| Lecture | 自瞄 | 导航雷达 |
|---------|------|----------|
| 1 | Hello SP Vision | — |
| 2 | Hello C++ && OOP | — |
| 3 | Hello Modern C++ | — |
| 4 | Hello Armor && PnP | Hello ROS2 |
| 5 | Hello Kalman && Target | **Hello SP Nav** |

后续课程视频将会全部上传至战队 Bilibili 官方账号：[TJ-SuperPower战队](https://space.bilibili.com/651837546)

---

## Lecture 5 · Hello SP Nav（本仓库 nav 教程栈）

从 `sp_nav_26` 的仿真启动链路裁剪而来，功能与参数对齐：

- 上层决策树：`click_nav.xml`
- 下层导航树：`default_nav_with_fallback.xml`
- 全局规划器：`AStar`（`sp_global_planner/AStarPlanner`）
- 控制器：`PidController`（跟踪 `/global_path`）

已移除：Topo/Hybrid/JPS 规划器、MPC/TinyMPC、局部规划器、地形行为树节点、比赛决策树及相关依赖。

### 依赖

- ROS 2（Jazzy / Humble 均可，需与系统 BT / Qt 版本匹配）
- `behaviortree_cpp_v3`、`pluginlib`、`qtbase5-dev`、`libopencv-dev`、`libyaml-cpp-dev`
- Python：`pygame`、`numpy`、`Pillow`

### 编译

```bash
cd /path/to/sp_vision_tutorial_27
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 启动仿真

```bash
bash bash/sim_nav_all_start.sh
```

启动内容：

1. `base_link` → `livox_frame` 静态 TF
2. ESDF 地图 + A* 规划器 + 下层 BT（`default_nav_with_fallback`）+ RViz
3. PID 控制器
4. 决策层（`click_nav`）
5. 地图拖拽仿真 GUI（发布里程计 / TF）

### 使用

1. 在决策 GUI 中加载并运行 `click_nav`
2. 在 RViz 中用 **2D Goal Pose**（或发布 `/goal_pose`）下发目标
3. 机器人沿 A* 全局路径由 PID 跟踪至目标点

### 包一览

| 包 | 作用 |
|----|------|
| `robot_msg` / `sp_msgs` | 消息定义 |
| `sp_map_server` | ESDF / 全局代价地图 |
| `sp_global_planner` | A* 全局规划 |
| `sp_controller_server` | PID 路径跟踪 |
| `sp_nav_bt` | 下层导航行为树 |
| `sp_decision` | 上层点击导航决策 |
| `sp_nav_bringup` | 一键启动与地图 |
| `map_robot_gui` | 仿真拖拽 GUI |

---

##### 注意

本仓库 `main` 分支为算法组自瞄方向招新使用，`nav` 分支为算法组导航方向招新使用，请各位同学在进行 clone 等操作时注意区分。
