#include "BNO085_hal.hpp"

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <i2c/smbus.h>
#include <iostream>
#include <linux/i2c-dev.h>
#include <memory>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <unistd.h>

#define US_TO_NS 1000;

static int fd;
static int i2c_addr;
static sh2_SensorValue_t *sensor_value_ptr = nullptr;

static auto get_timestamp_us() -> uint32_t;

static auto sh2_hal_open(sh2_Hal_t *self) -> int;
static void sh2_hal_close(sh2_Hal_t *self);
static auto sh2_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us) -> int;
static auto sh2_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) -> int;
static auto sh2_hal_getTimeUs(sh2_Hal_t *self) -> uint32_t;

static void event_handler(void *cookie, sh2_AsyncEvent_t *pEvent);
static void sensor_handler(void *cookie, sh2_SensorEvent_t *pEvent);

BNO085::BNO085(int addr)
    : _sh2_hal{
          .open = sh2_hal_open,
          .close = sh2_hal_close,
          .read = sh2_hal_read,
          .write = sh2_hal_write,
          .getTimeUs = sh2_hal_getTimeUs} {
    i2c_addr = addr;
}

auto BNO085::begin() -> bool {
    int status;

    status = sh2_open(&_sh2_hal, event_handler, nullptr);
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

auto BNO085::set_sensor_config(sh2_SensorId_t sensorId) -> int {

    sh2_SensorConfig_t sensor_config;
    sensor_config.reportInterval_us = 100000;

    return sh2_setSensorConfig(sensorId, &sensor_config);
}

static auto get_timestamp_us() -> uint32_t {
    struct timespec ts;
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
    // Empty for now
    close(fd);
}

static auto sh2_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned int len, uint32_t *t_us) -> int {
    auto const buf = std::make_unique<uint8_t[]>(len);

    ssize_t bytes_read = read(fd, buf.get(), len);
    if (bytes_read < len) {
        return 0;
    }

    memcpy(pBuffer, buf.get(), len);

    *t_us = get_timestamp_us();
    return bytes_read;
}

static auto sh2_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) -> int {
    ssize_t bytes_written = write(fd, pBuffer, len);

    return bytes_written;
}

static auto sh2_hal_getTimeUs(sh2_Hal_t *self) -> uint32_t {
    return get_timestamp_us();
}

// Handle non-sensor events from the sensor hub
static void event_handler(void *cookie, sh2_AsyncEvent_t *pEvent) {
    // // If we see a reset, set a flag so that sensors will be reconfigured.
    // if (pEvent->eventId == SH2_RESET) {
    //     resetOccurred = true;
    // }
    // else if (pEvent->eventId == SH2_SHTP_EVENT) {
    //     printf("EventHandler  id:SHTP, %d\n", pEvent->shtpEvent);
    // }
    // else if (pEvent->eventId == SH2_GET_FEATURE_RESP) {
    //     // printf("EventHandler Sensor Config, %d\n", pEvent->sh2SensorConfigResp.sensorId);
    // }
    // else {
    //     printf("EventHandler, unknown event Id: %d\n", pEvent->eventId);
    // }

    // TODO - figure out what events need to be handled
    std::cout << "EventHandler recieved event Id: " << pEvent->eventId << std::endl;
}

// Handle sensor events from the sensor hub
static void sensor_handler(void *cookie, sh2_SensorEvent_t *pEvent) {

    if (sensor_value_ptr != nullptr) {
        int status = sh2_decodeSensorEvent(sensor_value_ptr, pEvent);

        // if (status != SH2_OK) {
        // }
    }
}
