#ifndef BNO085_HAL
#define BNO085_HAL

extern "C" {
#include <sh2.h>
#include <sh2_SensorValue.h>
#include <sh2_err.h>
#include <sh2_hal.h>
}

#define DEFAULT_ADDR 0x4A
#define I2C_ADAPTER "/dev/i2c-0"

class BNO085 {
public:
    explicit BNO085(int addr = DEFAULT_ADDR);
    auto begin() -> bool;
    auto reset() -> int;
    auto set_sensor_config(sh2_SensorId_t sensorId) -> int;

    sh2_ProductIds_t prod_ids;
    sh2_SensorValue_t sensor_value;

private:
    sh2_Hal_t _sh2_hal;
};

#endif
