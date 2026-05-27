# magnetic_sensor.msg
# 通用磁性传感器/霍尔传感器数据话题
# 适用于：单极/全极/双极霍尔开关、1D/3D线性数字磁力计等

uint64 timestamp             # 系统启动后的时间 (微秒)

# 传感器硬件类型枚举
uint8 SENSOR_TYPE_SWITCH_UNIPOLAR = 0   # 单极性开关 (如 DRV5023)
uint8 SENSOR_TYPE_SWITCH_OMNIPOLAR = 1  # 全极性开关 (如 DRV5032)
uint8 SENSOR_TYPE_SWITCH_BIPOLAR = 2    # 双极性锁存开关
uint8 SENSOR_TYPE_LINEAR_1D = 3         # 单轴线性传感器
uint8 SENSOR_TYPE_LINEAR_3D = 4         # 3D 线性传感器 (如 TMAG5273)

uint8 sensor_type            # 当前传感器的类型

uint32 device_id             # 传感器设备的唯一 ID (用于区分总线上的不同传感器，标准 PX4 规范)

# ================= 数据字段 =================

# 数字/中断状态 (布尔值)
# 适用场景：
# 1. 纯开关型传感器 (如 DRV5032)：磁铁靠近触发时为 true，释放为 false。
# 2. 智能数字传感器 (如 TMAG5273)：其内部阈值比较器触发，或外部 INT 引脚拉低时为 true。
bool is_triggered

# 线性磁场绝对值 (单位：毫特斯拉 mT)
# 适用场景：仅针对 LINEAR_1D 和 LINEAR_3D 类型有效。
# 注意：如果是纯开关型传感器 (如 DRV5032)，驱动层应将这些字段填充为 NAN (Not a Number)。
float32 mag_x                # X轴磁场分量
float32 mag_y                # Y轴磁场分量
float32 mag_z                # Z轴磁场分量

# 芯片内部温度 (单位：摄氏度)
# 适用场景：许多现代 I2C 磁传感器 (如 TMAG5273) 内置高精度温度计，有助于温度补偿或监控环境。
# 注意：如果不具备温度检测功能，驱动层应填充为 NAN。
float32 temperature
