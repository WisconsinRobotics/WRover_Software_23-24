// Code to translate serial data to led panel ouput

// TODO(@bennowotny): ROSserial?

/**
 * @file arduino.ino
 * @defgroup wr_led_matrix
 * @{
 * @defgroup wr_led_matrix_arduino LED Matrix Arduino
 * @brief Arduino code to interpret serial messages from the ROS service and control the LED matrix
 *
 * This code sits on the other end of the serial line and reboots when the serial port is opened.  The communication from here to the physical LED Matrix is handled by the Arduino Neopixel library.
 * @{
 */

#include "Arduino.h"
#include "pins_arduino.h"
#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix.h>
#include <Adafruit_NeoPixel.h>

/// Data pin for the NeoMatrix
#define PIN 6

/// Representation of control for the NeoPixel matrix
Adafruit_NeoMatrix matrix{8, 8, PIN,
                          NEO_MATRIX_BOTTOM + NEO_MATRIX_LEFT +
                              NEO_MATRIX_ROWS + NEO_MATRIX_ZIGZAG,
                          NEO_GRB + NEO_KHZ800};

/**
 * @brief Runs once on Arduino boot.
 *
 * Responsible for initializing the NeoPixel matrix, clearing out the serial buffer, and synchronizing with the ROS service by sending the magic word.
 */
void setup() {
    Serial.begin(9600);

    matrix.begin();
    matrix.clear();
    matrix.show();
    matrix.setBrightness(255);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);

    while (Serial.available() > 0)
        Serial.read();

    // Magic word to sync startup
    constexpr uint8_t data[]{
        0x31,
        0x41,
        0x59,
        0x26,
        0xDE,
        0xAD,
        0xBE,
        0xEF};

    Serial.write(data, 8);
}

/// Used to control the blinky LED
/**
 * The blinky LED is a practice in embedded systems to dedicate an LED pin to soley toggling an LED over time.  This acts as feedback to the developers that something has halted the processor if the light stops blinking.  This is especially important on systems like Arduinos, since if there was a fault, there is no common logging available in a production run to make the error visible.
 */
unsigned int pass = 0;
/// Stores the RGB values sent over serial
uint8_t serial_input[3]{0, 0, 0};

/**
 * @brief Set the color of the physical LED panel based on the stored values in the serial_input variable
 *
 */
void setColor();

/**
 * @brief Is called in a loop during the Arduino runtime after setup().
 *
 * Is responsible for reading in messages off of serial, computing the CRC, and, if appropriate, dispatching the color to the LED matrix.  Also toggles the blinkly LED at ~0.5Hz.
 */
void loop() {
    // Wait for color
    if (Serial.available() >= 4) {

        // Read color into byte array
        uint8_t crc_calc{0};
        for (unsigned char &color_repr : serial_input) {
            color_repr = Serial.read();
            crc_calc ^= color_repr;
        }

        uint8_t crc = Serial.read();
        if (crc_calc == crc)
            setColor();

        // Flush any extra bytes
        while (Serial.available() != 0) {
            Serial.read();
        }
    }

    if (++pass % 1000 > 500) {
        digitalWrite(LED_BUILTIN, LOW);
    } else {
        digitalWrite(LED_BUILTIN, HIGH);
    }

    delay(1);
}

void setColor() {
    uint8_t r = serial_input[0];
    uint8_t g = serial_input[1];
    uint8_t b = serial_input[2];

    // Convert colors to 32 bit number

    uint32_t color;
    color = color | r;
    color = color << 8;
    color = color | g;
    color = color << 8;
    color = color | b;

    matrix.fill(color);
    matrix.show();
}

/// @} @}
