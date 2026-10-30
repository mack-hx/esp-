# 控制回路跟踪组件 (trace)
#
# 包含:
#   trace.c  环形缓冲采集 (控制任务写) + 独立任务输出串口 (实时 + 回灌)
#
# 依赖: 只用 FreeRTOS, 不依赖 param / 控制环 ——
#       这样它就是一个纯"记录器", 换控制算法也不用动。
idf_component_register(
    SRCS "trace.c"
    INCLUDE_DIRS "."
    REQUIRES freertos esp_partition heap
)
