#include "tmag5273.hpp"

TMAG5273::TMAG5273(const I2CSPIDriverConfig &config) :
    device::I2C(config),
    I2CSPIDriver<TMAG5273>(config)
{
}

// I2C 单字节写入辅助函数
int TMAG5273::write_reg(uint8_t reg, uint8_t val)
{
    uint8_t cmd[2] = {reg, val};
    return transfer(cmd, 2, nullptr, 0);
}

int TMAG5273::init()
{
    int ret = I2C::init();
    if (ret != PX4_OK) {
        return ret;
    }

    // 0. DEVICE_CONFIG_1: 恢复标准的 3-byte I2C read 模式
    if (write_reg(TMAG5273_REG_DEVICE_CONFIG_1, 0x00) != PX4_OK) {
        return PX4_ERROR;
    }

    // 1. DEVICE_CONFIG_2: 设置为 Continuous measure mode (连续测量模式)
    if (write_reg(TMAG5273_REG_DEVICE_CONFIG_2, 0x02) != PX4_OK) {
        return PX4_ERROR;
    }

    // 2. SENSOR_CONFIG_1: 开启 X, Y, Z 三轴转换通道
    if (write_reg(TMAG5273_REG_SENSOR_CONFIG_1, 0x70) != PX4_OK) {
        return PX4_ERROR;
    }

    // 开启系统调度：10000微秒 = 100Hz
    ScheduleOnInterval(10000);

    return PX4_OK;
}

int TMAG5273::probe()
{
    uint8_t cmd = TMAG5273_REG_MANUFACTURER_ID;
    uint8_t id_data[2] = {0, 0};

    if (transfer(&cmd, 1, id_data, 2) != PX4_OK) {
        return PX4_ERROR;
    }

    // 严谨校验：TI 芯片的 0x1E 寄存器返回 0x49 ('I')，0x1F 返回 0x54 ('T')
    if (id_data[0] == 0x49 && id_data[1] == 0x54) {
        return PX4_OK;
    }

    return PX4_ERROR;
}

void TMAG5273::RunImpl()
{
    // 连续读取 X, Y, Z 数据，共 6 个字节 (地址 0x12 到 0x17)
    uint8_t reg_data_cmd = TMAG5273_REG_X_MSB;
    uint8_t mag_data[6] = {0};

    if (transfer(&reg_data_cmd, 1, mag_data, 6) != PX4_OK) {
        _comms_errors++;
        return;
    }

    // 解析 16 位 2的补码 数据
    int16_t raw_x = (int16_t)((mag_data[0] << 8) | mag_data[1]);
    int16_t raw_y = (int16_t)((mag_data[2] << 8) | mag_data[3]);
    int16_t raw_z = (int16_t)((mag_data[4] << 8) | mag_data[5]);

    // 转换为真实的毫特斯拉 (mT) 物理单位
    float mag_x_mt = (float)raw_x * TMAG5273A1_MT_PER_LSB;
    float mag_y_mt = (float)raw_y * TMAG5273A1_MT_PER_LSB;
    float mag_z_mt = (float)raw_z * TMAG5273A1_MT_PER_LSB;

    _last_mag_z = mag_z_mt; // 缓存用于后台打印监控

    // --- 组装通用的 uORB 消息 ---
    magnetic_sensor_s msg{};
    msg.timestamp = hrt_absolute_time();
    msg.device_id = get_device_id();
    msg.sensor_type = magnetic_sensor_s::SENSOR_TYPE_LINEAR_3D;

    // 你可以在这里加入阈值判定逻辑，或者将判定放在混控器/应用层
    // 这里设定一个示例阈值：当 Z 轴磁场绝对值 > 15.0 mT 时认为触发
    msg.is_triggered = (fabsf(mag_z_mt) > 15.0f);

    msg.mag_x = mag_x_mt;
    msg.mag_y = mag_y_mt;
    msg.mag_z = mag_z_mt;

    // 简写，此处不读取温度，填入无效值
    msg.temperature = NAN;

    // 发布消息
    _magnetic_pub.publish(msg);

    // 发送调试信息到 QGroundControl
    debug_value_s dbg_msg{};
    dbg_msg.timestamp = hrt_absolute_time();
    dbg_msg.value = mag_z_mt;
    dbg_msg.ind = 2; // 区分不同的传感器可以给不同的索引
    _debug_pub.publish(dbg_msg);
}

I2CSPIDriverBase *TMAG5273::instantiate(const I2CSPIDriverConfig &config, int runtime_instance)
{
    // 【关键修改】：不再强行覆盖 config.i2c_address。
    // 直接使用命令行传入的 config，这样才能拉起两个不同地址的芯片
    TMAG5273 *instance = new TMAG5273(config);
    if (!instance) {
        PX4_ERR("alloc failed");
        return nullptr;
    }
    if (instance->init() != PX4_OK) {
        delete instance;
        return nullptr;
    }
    return instance;
}

void TMAG5273::print_status()
{
    I2CSPIDriverBase::print_status();
    PX4_INFO("TMAG5273 3D Hall Sensor is running smoothly.");
    PX4_INFO("Current Z-Axis Magnetic Field (mT): %.3f", (double)_last_mag_z);
}

void TMAG5273::print_usage()
{
    PRINT_MODULE_USAGE_NAME("tmag5273", "driver");
    PRINT_MODULE_USAGE_SUBCATEGORY("sensor");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int tmag5273_main(int argc, char *argv[])
{
    using ThisDriver = TMAG5273;
    BusCLIArguments cli{true, false};
    cli.default_i2c_frequency = 400000;

    // 拦截自定义的 -a 参数用于多实例挂载
    uint8_t custom_i2c_address = 0;
    int new_argc = 0;
    char *new_argv[argc + 1];

    for (int i = 0; i < argc; ++i) {
        if (strcmp(argv[i], "-a") == 0 && i + 1 < argc) {
            custom_i2c_address = (uint8_t)strtol(argv[i + 1], nullptr, 16);
            i++;
        } else {
            new_argv[new_argc++] = argv[i];
        }
    }
    new_argv[new_argc] = nullptr;

    const char *verb = cli.parseDefaultArguments(new_argc, new_argv);
    if (!verb) {
        ThisDriver::print_usage();
        return -1;
    }

    if (custom_i2c_address != 0) {
        cli.i2c_address = custom_i2c_address;
    }

    BusInstanceIterator iterator(MODULE_NAME, cli, 0);

    if (!strcmp(verb, "start")) {
        return ThisDriver::module_start(cli, iterator);
    }
    if (!strcmp(verb, "stop")) {
        return ThisDriver::module_stop(iterator);
    }
    if (!strcmp(verb, "status")) {
        return ThisDriver::module_status(iterator);
    }

    ThisDriver::print_usage();
    return -1;
}
