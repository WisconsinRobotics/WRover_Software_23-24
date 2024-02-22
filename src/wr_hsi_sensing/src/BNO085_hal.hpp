#ifndef BNO085_HAL
#define BNO085_HAL

extern "C" {
#include <euler.h>
#include <sh2.h>
#include <sh2_SensorValue.h>
#include <sh2_err.h>
#include <sh2_hal.h>
}

#define DEFAULT_ADDR 0x4A
#define I2C_ADAPTER "/dev/i2c-1"

class BNO085 {
public:
    explicit BNO085(bool debug = false, int addr = DEFAULT_ADDR);
    ~BNO085();
    auto begin() -> bool;
    auto reset() -> int;
    void close();
    auto set_sensor_config(sh2_SensorId_t sensorId) -> int;
    auto get_sensor_event() -> bool;
    auto get_accuracy() -> int;
    auto get_mag_x() -> float;
    auto get_mag_y() -> float;
    auto get_mag_z() -> float;
    auto get_raw_mag_x() -> int;
    auto get_raw_mag_y() -> int;
    auto get_raw_mag_z() -> int;
    auto get_yaw() -> float;
    auto get_yaw(float &accuracy) -> float;
    auto tare(bool zAxis, sh2_TareBasis_t basis) -> bool;
    auto persist_tare() -> bool;
    auto clear_tare() -> bool;

    sh2_ProductIds_t prod_ids;
    sh2_SensorValue_t sensor_value;

private:
    sh2_Hal_t _sh2_hal;
    bool _debug;
};

#endif
