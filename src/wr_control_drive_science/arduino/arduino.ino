
// TODO(@bennowotny): ROSserial?

#include "Arduino.h"
#include "pins_arduino.h"
#include <SDI12.h>
#include <Servo.h>

// Constants
#define CACHE_SERVO_PIN 9
constexpr uint8_t CODE_WORD{0xAA};

constexpr int8_t DATA_PIN{12}; // A Guess
SDI12 sdi12Bus{DATA_PIN};

// Globals
uint8_t servoPos = 0;
Servo cacheServo;

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

uint32_t iter{0};

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
