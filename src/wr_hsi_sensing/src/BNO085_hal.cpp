#include "BNO085_hal.hpp"
#include "sh2.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <memory>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <unistd.h>

#define US_TO_NS 1000;

static int fd;
static int i2c_addr;
static bool debug_i2c;
static sh2_SensorValue_t *sensor_value_ptr = nullptr;

static auto get_timestamp_us() -> uint32_t;

static auto sh2_hal_open(sh2_Hal_t *self) -> int;
static void sh2_hal_close(sh2_Hal_t *self);
static auto sh2_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us) -> int;
static auto sh2_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) -> int;
static auto sh2_hal_getTimeUs(sh2_Hal_t *self) -> uint32_t;

static void event_handler(void *cookie, sh2_AsyncEvent_t *pEvent);
static void sensor_handler(void *cookie, sh2_SensorEvent_t *pEvent);

BNO085::BNO085(bool debug, int addr)
    : _debug(debug),
      _sh2_hal{
          .open = sh2_hal_open,
          .close = sh2_hal_close,
          .read = sh2_hal_read,
          .write = sh2_hal_write,
          .getTimeUs = sh2_hal_getTimeUs},
      sensor_value(),
      prod_ids() {
    i2c_addr = addr;
    debug_i2c = debug;
}

BNO085::~BNO085() {
    close();
}

auto BNO085::begin() -> bool {
    int status = sh2_open(&_sh2_hal, event_handler, nullptr);
    if (status != SH2_OK) {
        return false;
    }

    // Check connection
    memset(&prod_ids, 0, sizeof(prod_ids));
    status = sh2_getProdIds(&prod_ids);
    if (status != SH2_OK) {
        return false;
    }

    sensor_value_ptr = &sensor_value;
    sh2_setSensorCallback(sensor_handler, nullptr);

    return true;
}

auto BNO085::reset() -> int {
    return sh2_devReset();
}

void BNO085::close() {
    sh2_close();
}

auto BNO085::set_sensor_config(sh2_SensorId_t sensorId) -> int {

    sh2_SensorConfig_t sensor_config;
    // TODO make this not a magic number
    sensor_config.reportInterval_us = 10000;

    return sh2_setSensorConfig(sensorId, &sensor_config);
}

auto BNO085::get_sensor_event() -> bool {
    sensor_value.timestamp = 0;

    sh2_service();

    return sensor_value.timestamp != 0;
}

auto BNO085::get_accuracy() -> int {
    return sensor_value.status;
}

auto BNO085::get_mag_x() -> float {
    return sensor_value.un.magneticField.x;
}

auto BNO085::get_mag_y() -> float {
    return sensor_value.un.magneticField.y;
}

auto BNO085::get_mag_z() -> float {
    return sensor_value.un.magneticField.z;
}

auto BNO085::get_raw_mag_x() -> int {
    return sensor_value.un.rawMagnetometer.x;
}

auto BNO085::get_raw_mag_y() -> int {
    return sensor_value.un.rawMagnetometer.y;
}

auto BNO085::get_raw_mag_z() -> int {
    return sensor_value.un.rawMagnetometer.z;
}

auto BNO085::get_yaw() -> float {
    return q_to_yaw(sensor_value.un.rotationVector.real,
                    sensor_value.un.rotationVector.i,
                    sensor_value.un.rotationVector.j,
                    sensor_value.un.rotationVector.k);
}

auto BNO085::get_yaw(float &accuracy) -> float {
    accuracy = sensor_value.un.rotationVector.accuracy;
    return get_yaw();
}

auto BNO085::tare(bool zAxis, sh2_TareBasis_t basis) -> bool {
    int status = sh2_setTareNow(zAxis ? SH2_TARE_Z : (SH2_TARE_X & SH2_TARE_Y & SH2_TARE_Z), basis);

    return status == SH2_OK;
}

auto BNO085::persist_tare() -> bool {
    int status = sh2_persistTare();

    return status == SH2_OK;
}

auto BNO085::clear_tare() -> bool {
    int status = sh2_clearTare();

    return status == SH2_OK;
}

static auto get_timestamp_us() -> uint32_t {
    struct timespec ts {};

    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return ts.tv_nsec / US_TO_NS;
}

static auto sh2_hal_open(sh2_Hal_t *self) -> int {
    // Open i2c device
    fd = open(I2C_ADAPTER, O_RDWR);
    if (fd < 0) {
        // TODO: add error message
        return SH2_ERR;
    }

    if (ioctl(fd, I2C_SLAVE, i2c_addr) < 0) {
        // TODO: add error message
        return SH2_ERR;
    };
    return 0;
}

static void sh2_hal_close(sh2_Hal_t *self) {
    close(fd);
}

static auto sh2_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned int len, uint32_t *t_us) -> int {
    auto const buf = std::make_unique<uint8_t[]>(len);

    ssize_t bytes_read = read(fd, buf.get(), len);
    if (bytes_read < len) {
        return 0;
    }

    if (debug_i2c) {
        uint16_t packet_size = (buf[0] | (buf[1] << 8)) & ~0x8000U;
        uint8_t seq_num = buf[3];

        printf("Recieved packet of %d bytes, Sequence number %d\n", packet_size, seq_num);
    }

    memcpy(pBuffer, buf.get(), len);

    *t_us = get_timestamp_us();
    return bytes_read;
}

static auto sh2_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) -> int {
    ssize_t bytes_written = write(fd, pBuffer, len);

    if (debug_i2c) {
        printf("Wrote packet of %d bytes, %ld bytes written\n", len, bytes_written);
    }

    return bytes_written;
}

static auto sh2_hal_getTimeUs(sh2_Hal_t *self) -> uint32_t {
    return get_timestamp_us();
}

// Handle non-sensor events from the sensor hub
static void event_handler(void *cookie, sh2_AsyncEvent_t *pEvent) {
    // TODO - figure out what events need to be handled

    if (debug_i2c) {
        if (pEvent->eventId == SH2_RESET) {
            printf("Sensor Reset\n");
            // // If we see a reset, set a flag so that sensors will be reconfigured.
            //     resetOccurred = true;
        } else if (pEvent->eventId == SH2_SHTP_EVENT) {
            printf("EventHandler recieved SHTP event ID: %d\n", pEvent->shtpEvent);
        } else if (pEvent->eventId == SH2_GET_FEATURE_RESP) {
            printf("EventHandler Sensor Config: %d\n", pEvent->sh2SensorConfigResp.sensorId);
        } else {
            printf("EventHandler reieved unknown event ID: %d\n", pEvent->eventId);
        }
    }
}

// Handle sensor events from the sensor hub
static void sensor_handler(void *cookie, sh2_SensorEvent_t *pEvent) {

    if (sensor_value_ptr != nullptr) {
        int status = sh2_decodeSensorEvent(sensor_value_ptr, pEvent);

        if (status != SH2_OK) {
            sensor_value_ptr->timestamp = 0;
            if (debug_i2c) {
                printf("Error decoding SensorEvent\n");
            }
        }
    }
}
