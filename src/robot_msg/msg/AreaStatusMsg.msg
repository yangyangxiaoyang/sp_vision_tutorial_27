int8 centrum         # 0: neutral, 1: friendly, 2: enemy, 3: both (0x601)
int8 team_bastion    # 己方堡垒增益点占领状态: 0未占领, 1己方, 2对方, 3双方
int8 enemy_bastion   # 对方堡垒增益点占领状态: 0未占领, 1对方, 2己方, 3双方

# 对方飞镖最后一次击中己方前哨站或基地（0x601 bit11-22）
uint16 last_hit_time    # 命中时间，范围 0-420，开局默认 0；每次命中刷新
uint8  last_hit_target  # 1前哨站 2基地固定 3基地随机固定 4基地随机移动 5基地末端移动
