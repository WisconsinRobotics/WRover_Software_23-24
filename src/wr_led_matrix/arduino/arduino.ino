// Code to translate serial data to led panel ouput

#include "Arduino.h"
#include "pins_arduino.h"
#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix.h>
#include <Adafruit_NeoPixel.h>

#define PIN 6

Adafruit_NeoMatrix matrix{8, 8, PIN,
                          NEO_MATRIX_BOTTOM + NEO_MATRIX_LEFT +
                              NEO_MATRIX_ROWS + NEO_MATRIX_ZIGZAG,
                          NEO_GRB + NEO_KHZ800};

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

unsigned int pass = 0;
uint8_t serial_input[3]{0, 0, 0};

void setColor();

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