from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="sp_global_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[{
                "costmap_topic": "/global_costmap",
                "path_topic": "/global_path",

                # =======================
                # Option 1: Original A*
                # =======================
                # "plugin_name": "AStar",
                # "plugin_type": "sp_global_planner/AStarPlanner",

                # # plugin params (namespaced under plugin_name)
                # "AStar.lethal_cost": 100,
                # "AStar.cost_weight": 2.0,

                # =======================
                # Option 2: Topo A*
                # =======================
                # "plugin_name": "TopoAStar",
                # "plugin_type": "sp_global_planner/TopoAStarPlanner",
                
                # # plugin params (namespaced under plugin_name)
                # "TopoAStar.topo_yaml": "/home/rm/Desktop/sp_nav_26/src/tools/map_process/topo/topology.yaml",
                # "TopoAStar.lethal_cost": 100,
                # "TopoAStar.cost_weight": 2.0,
                # "TopoAStar.unknown_is_obstacle": True,
                # "TopoAStar.max_topo_paths": 50,
                # "TopoAStar.max_astar_iters": 200000,
                # "TopoAStar.region_tolerance_m": 0.3,
                # "TopoAStar.heuristic_weight": 1.0,

                # =======================
                # Option 3: Hybrid A* + Topo (适用于全向轮)
                # =======================
                # "plugin_name": "HybridTopoAStar",
                # "plugin_type": "sp_global_planner/HybridTopoAStarPlanner",
                
                # # plugin params (namespaced under plugin_name)
                # "HybridTopoAStar.topo_yaml": "/home/rm/Desktop/sp_nav_26/src/tools/map_process/topo/topology.yaml",
                # "HybridTopoAStar.lethal_cost": 100,
                # "HybridTopoAStar.cost_weight": 2.0,
                # "HybridTopoAStar.unknown_is_obstacle": True,
                # "HybridTopoAStar.max_topo_paths": 50,
                # "HybridTopoAStar.max_astar_iters": 200000,
                # "HybridTopoAStar.region_tolerance_m": 0.3,
                # "HybridTopoAStar.heuristic_weight": 1.2,
                
                # # 混合A*特有参数
                # "HybridTopoAStar.angle_bins": 8,               # 角度离散化 (8 = 45度/bin) - 降低搜索空间
                # "HybridTopoAStar.turning_radius": 0.3,         # 转弯半径(米) - 全向轮可以很小
                # "HybridTopoAStar.step_size": 0.1,              # 步长(米)
                # "HybridTopoAStar.angle_cost_weight": 0.5,      # 角度变化代价权重
                
                # # 代价惩罚系数 
                # "HybridTopoAStar.non_straight_penalty": 1.2,   # 非直线运动惩罚 (斜向/旋转)
                # "HybridTopoAStar.change_penalty": 0.5,         # 改变运动类型惩罚
                # "HybridTopoAStar.rotate_penalty": 1.5,         # 平移↔旋转切换惩罚
                # "HybridTopoAStar.cost_penalty": 20.0,          # 障碍物代价惩罚系数
                
                # # 🆕 门点附近惩罚参数（改进3：门点平滑性）
                # "HybridTopoAStar.connector_proximity_radius": 0.5,      # 门点附近区域半径(米)
                # "HybridTopoAStar.sharp_turn_penalty": 3.0,              # 急转弯惩罚
                # "HybridTopoAStar.orientation_mismatch_penalty": 2.0,    # 朝向不匹配惩罚
                
                # # 🆕 段间连续性参数（改进2：路径段连接）
                # "HybridTopoAStar.segment_continuity_weight": 5.0,       # 段间连续性权重

                # =======================
                # Option 4: Topo JPS (Jump Point Search)
                # =======================
                "plugin_name": "TopoJPS",
                "plugin_type": "sp_global_planner/TopoJPSPlanner",
                
                # plugin params (namespaced under plugin_name)
                "TopoJPS.topo_yaml": "/home/rm/Desktop/sp_nav_26/src/tools/map_process/topo/topology.yaml",
                "TopoJPS.lethal_cost": 100,              # 致命代价阈值（>=此值视为障碍物）
                "TopoJPS.cost_weight": 2.0,            # 代价权重
                "TopoJPS.unknown_is_obstacle": True,   # 未知区域是否视为障碍物
                "TopoJPS.max_topo_paths": 50,          # 最大拓扑路径枚举数
                "TopoJPS.max_jps_iters": 200000,       # JPS最大迭代次数
                "TopoJPS.region_tolerance_m": 0.3,     # 区域边界容差（米）
                "TopoJPS.heuristic_weight": 1.0,       # 启发式权重（>=1.0，越大搜索越快但可能不最优）
                "TopoJPS.goal_tolerance_m": 0.40,      # 终点脱困半径（米），终点代价>=100时在圆内搜索代价<lethal_cost的点
                "TopoJPS.start_tolerance_m": 0.40,     # 起点脱困半径（米），起点代价>=100时在圆内搜索代价<lethal_cost的点
            }]
        )
    ])
