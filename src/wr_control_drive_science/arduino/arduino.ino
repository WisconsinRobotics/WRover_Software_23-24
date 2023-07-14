
/**
 * @file
 * @author Ben Nowotny
 * @brief Arduino sensor code and ideolized servo code
 * @details
 * This file contains code to talk to the TEROS12 moisture sensor over the SDI protocol and
 *
 */

/**
 * @addtogroup wr_control_drive_science
 * @{
 *
 * @defgroup wr_control_drive_science_arduino Science Arduino
 * @brief The Arduino managing science hardware (TEROS12 and servo)
 *
 * @{
 */

// TODO(@bennowotny): ROSserial?

#include "Arduino.h"
#include "pins_arduino.h"
#include <SDI12.h>
#include <Servo.h>

// Constants
/// Servo pin (not used)
#define CACHE_SERVO_PIN 9
/// Expected code word to start the Arduino code
constexpr uint8_t CODE_WORD{0xAA};

/// Data pin in the SDI protocol
constexpr int8_t DATA_PIN{12}; // A Guess
/// Object handling the SDI12 protocol
SDI12 sdi12Bus{DATA_PIN};

// Globals
/// Current position of the servo (unused)
uint8_t servoPos = 0;
/// Servo control object for the Arduino (unused)
Servo cacheServo;

/**
 * @brief One-shot code to set up the Arduino
 *
 * Sets up the SDI protocol, servo, and waits for a magic word from the Pi to start the main loop
 */
void setup() {
    // Initialize serial
    Serial.begin(115200);

    // Initialize servo
    cacheServo.attach(CACHE_SERVO_PIN);
    cacheServo.write(0);

    sdi12Bus.begin();

    // Wait for code word from Pi so that we know it has booted
    while (true) {
        if (Serial.available()) {
            Serial.readBytes(&servoPos, 1);

            // Break when the code word is received
            if (servoPos == CODE_WORD)
                break;
        }
    }
}

/// Iteration counter to implement the blinky LED and sense timing loop
uint32_t iter{0};

/**
 * @brief The main loop of the Arudino code
 *
 * Writes to a servo (not plugged in) and reads periodically from the TEROS12 sensor and streams over serial.
 * Also, blinks blinky.
 */
void loop() {
    // When a servo position is avaiable from serial, update the servo's position
    if (Serial.available()) {
        Serial.readBytes(&servoPos, 1);
        // FIXME: This value needs to be restricted to the range [0, 180]
        // Probably on the Pi side?
        cacheServo.write(servoPos);
    }

    if (iter % 100 == 0) {
        sdi12Bus.sendCommand("aR4!");
        (void)sdi12Bus.readStringUntil('\t');
        const auto resp{sdi12Bus.readStringUntil(' ').toInt()};
        // Big-endian?
        Serial.write(static_cast<uint8_t>((resp >> 24) & 0xFF));
        Serial.write(static_cast<uint8_t>((resp >> 16) & 0xFF));
        Serial.write(static_cast<uint8_t>((resp >> 8) & 0xFF));
        Serial.write(static_cast<uint8_t>((resp)&0xFF));
    }

    if (++iter % 1000 > 500) {
        digitalWrite(LED_BUILTIN, LOW);
    } else {
        digitalWrite(LED_BUILTIN, HIGH);
    }

    delay(1);
}

/// @}
/// @}
