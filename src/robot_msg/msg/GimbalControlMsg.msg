uint8 mode                      # 运动模式: 0=全向扫描 1=三轴指定角度 2=范围扫描 3=仅yaw锁定 4=仅大yaw指定角度
float32 big_yaw                 # 大yaw目标角度 (deg)，mode=1/4 生效
float32 small_yaw               # 小yaw目标角度 (deg)，mode=1 生效
float32 pitch                   # pitch目标角度 (deg)，mode=1 生效
float32 small_yaw_lower_limit   # 小yaw范围下限 (deg)，mode=2 生效
float32 small_yaw_upper_limit   # 小yaw范围上限 (deg)，mode=2 生效
float32 pitch_lower_limit       # pitch范围下限 (deg)，mode=2 生效
float32 pitch_upper_limit       # pitch范围上限 (deg)，mode=2 生效
