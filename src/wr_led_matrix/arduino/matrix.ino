// Code to translate serial data to led panel ouput

#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix.h>
#include <Adafruit_NeoPixel.h>

#define PIN 6

Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(8, 8, PIN,
  NEO_MATRIX_BOTTOM     + NEO_MATRIX_LEFT +
  NEO_MATRIX_ROWS + NEO_MATRIX_ZIGZAG,
  NEO_GRB           + NEO_KHZ800);

const uint16_t colors[] = {
  matrix.Color(255, 0, 0), matrix.Color(0, 255, 0), matrix.Color(0, 0, 255) };

void setup() {
  Serial.begin(9600);
 
  matrix.begin();
  matrix.setBrightness(255);
}

int x    = matrix.width();
int pass = 0;
uint8_t serial_input[3];

void loop() {
  // Wait for color
  while(Serial.available() < 3) {delay(0.1);};

  // Read color into byte array
  for (int i=0; i<3; i++) {
    serial_input[i] = Serial.read();
  }

  setColor();

  // Flush any extra bytes
  while(Serial.available()) {
    Serial.read();
  }
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