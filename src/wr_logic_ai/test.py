#!/usr/bin/env python3
import adafruit_gps


if __name__=="__main__":
    gps = adafruit_gps.GPS_GtopI2C(i2c_bus=adafruit_gps.I2C()))
    gps.readline()
    gps.nmea_sentence