#include <Servo.h>

// Constants
#define SCOOP_SERVO_PIN 9
constexpr uint8_t CODE_WORD{0xAA};

// Globals
uint8_t servoPos = 0;
Servo scoopServo;

void setup() {
    // Initialize serial
    Serial.begin(115200);

    // Initialize servo
    scoopServo.attach(SCOOP_SERVO_PIN);
    scoopServo.write(0);

    // Wait for code word from Pi so that we know it has booted
    while(true) {
        if(Serial.available()) {
            Serial.readBytes(&servoPos, 1);
            
            // Break when the code word is received
            if(servoPos == CODE_WORD) break;
        }
    }
}

void loop() {
    // When a servo position is avaiable from serial, update the servo's position
    if(Serial.available()) {
        Serial.readBytes(&servoPos, 1);
        scoopServo.write(servoPos);
    }
    else {
        // Wait to slow down looping just a bit
        delay(2);
    }
}