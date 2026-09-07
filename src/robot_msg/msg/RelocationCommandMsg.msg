# 重定位控制命令消息
int32 CMD_IDLE = 0
int32 CMD_TRIGGER_RELOCALIZE = 1
int32 CMD_LOCK_RESULT = 2
int32 CMD_UNLOCK_RESULT = 3
int32 CMD_SET_POSE = 4
int32 CMD_CLEAR_AND_RESET = 5
int32 CMD_HOLD_TF = 6       # 冻结 TF 发布（不停配准；状态机自动恢复用）
int32 CMD_RELEASE_TF = 7    # 解除 TF 冻结，恢复播最新 result/best

int32 command                 # 命令类型
float64 x                     # (仅CMD_SET_POSE 使用) 新位置 X
float64 y                     # (仅CMD_SET_POSE 使用) 新位置 Y
float64 z                     # (仅CMD_SET_POSE 使用) 新位置 Z
float64 roll                  # (仅CMD_SET_POSE 使用) 欧拉角 Roll (度)
float64 pitch                 # (仅CMD_SET_POSE 使用) 欧拉角 Pitch (度)
float64 yaw                   # (仅CMD_SET_POSE 使用) 欧拉角 Yaw (度)
