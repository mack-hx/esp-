# 里程计组件
#
# 包含:
#   odom.c  编码器脉冲 → 行进距离 (cm)
#
# 依赖 param: 换算要用参数表里的 gear (减速比) 和 wheel_dia (轮径)
idf_component_register(
    SRCS "odom.c"
    INCLUDE_DIRS "."
    REQUIRES param encoder
)
