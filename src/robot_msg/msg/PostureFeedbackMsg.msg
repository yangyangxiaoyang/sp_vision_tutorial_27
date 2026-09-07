# Posture Feedback Message from lower computer
# 当前姿态：1=进攻姿态, 2=防御姿态, 3=移动姿态
uint8 current_posture

# 哨兵当前姿态是否为强化姿态 (0x70D bit15)
bool enhanced_pose

# 各姿态剩余时长（单位：秒，来自 0x70E）
uint8 attack_pose_duration
uint8 defense_pose_duration
uint8 move_pose_duration
uint8 enhanced_attack_pose_duration
uint8 enhanced_defense_pose_duration
uint8 enhanced_move_pose_duration

# 是否被打: 0=未被打, 1=被打 (0x004 data[1])
bool be_attacked

# 17mm枪管热量 (uint16, 最大约 400, 来自 0x701)
uint16 shooter_17mm_1_barrel_heat
