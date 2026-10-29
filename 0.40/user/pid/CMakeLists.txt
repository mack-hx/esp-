# PID 控制环组件
#
# 包含:
#   pid.c          通用 PID 算法 (位置式)
#   balance_pid.c  平衡环 (内环): 姿态误差 → 电机输出 (%)
#   speed_pid.c    速度环 (外环): 轮速误差 → 目标倾角 (°)
#   steer_pid.c    转向环 (差速环): 左右轮速差 → 两轮输出差 (%)
#   position_pid.c 位置保持环 (最外环): 位置误差 + 平移速度 → 目标倾角 (°)
#
# 依赖 param: 各环的增益从参数表实时读取 (App / 网页改完下一周期生效)
idf_component_register(
    SRCS "pid.c"
         "balance_pid.c"
         "speed_pid.c"
         "steer_pid.c"
         "position_pid.c"
    INCLUDE_DIRS "."
    REQUIRES param
)
