#pragma once

#include <px4_platform_common/i2c_spi_buses.h>
#include <drivers/device/i2c.h>
#include <px4_platform_common/module.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/magnetic_sensor.h>
#include <lib/mathlib/mathlib.h>
#include <uORB/topics/debug_value.h>

// TMAG5273 寄存器地址定义
#define TMAG5273_REG_DEVICE_CONFIG_1  0x00
#define TMAG5273_REG_DEVICE_CONFIG_2  0x01
#define TMAG5273_REG_SENSOR_CONFIG_1  0x02
#define TMAG5273_REG_X_MSB            0x12 // X轴数据起始寄存器
#define TMAG5273_REG_T_MSB            0x18 // 温度数据起始寄存器
#define TMAG5273_REG_MANUFACTURER_ID  0x1E // 厂商ID

// 磁场量程转换系数 (TMAG5273A1 满量程 +/- 40mT)
constexpr float TMAG5273A1_MT_PER_LSB = 40.0f / 32768.0f;

class TMAG5273 : public device::I2C, public I2CSPIDriver<TMAG5273>
{
public:
    TMAG5273(const I2CSPIDriverConfig &config);
    ~TMAG5273() override = default;

    static I2CSPIDriverBase *instantiate(const I2CSPIDriverConfig &config, int runtime_instance);
    static void print_usage();

    int init();
    void RunImpl();
    void print_status() override;

private:
    int probe();
    int write_reg(uint8_t reg, uint8_t val);

    // uORB 发布器
    uORB::PublicationMulti<magnetic_sensor_s> _magnetic_pub{ORB_ID(magnetic_sensor)};
    uORB::Publication<debug_value_s> _debug_pub{ORB_ID(debug_value)};

    uint8_t _comms_errors{0};
    float _last_mag_z{0.0f}; // 缓存最新Z轴数据用于状态打印
};
