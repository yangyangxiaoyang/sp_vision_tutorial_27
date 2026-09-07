# 重定位结果消息
bool converged                # 是否收敛
float64 x                     # 位置 X (map 坐标系)
float64 y                     # 位置 Y (map 坐标系)
float64 z                     # 位置 Z (map 坐标系)
float64 roll                  # 欧拉角 Roll (度)
float64 pitch                 # 欧拉角 Pitch (度)
float64 yaw                   # 欧拉角 Yaw (度)
float64 fitness               # 适应度 (越小越好)
uint32 iterations            # 迭代次数
float64 time_ms              # 配准耗时 (毫秒)
bool is_locked               # 是否已锁定 (锁定后不再更新)
int32 status                 # 0=IDLE, 1=PROCESSING, 2=SUCCESS, 3=FAILED, 4=LOCKED
