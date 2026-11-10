# PID 控制环组件
#
# 包含:
#   pid.c          通用 PID 算法 (位置式) + 死区软化
#   speed_pid.c    速度环 (外环): 轮速误差 → 目标倾角 (°)
#   steer_pid.c    转向环 (差速环): 左右轮速差 → 两轮输出差 (%)
#   lqr.c          LQR 状态反馈: 位置/速度/倾角/角速度 → 电机输出 (%)
#
# 依赖 param: 各环的增益从参数表实时读取 (App / 网页改完下一周期生效)
idf_component_register(
    SRCS "pid.c"
         "speed_pid.c"
         "steer_pid.c"
         "lqr.c"
    INCLUDE_DIRS "."
    REQUIRES param
)
