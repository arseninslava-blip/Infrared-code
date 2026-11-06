#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include "config.h"
#include "functions.h"

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

volatile uint32_t lineState = 0;   // line + angle in one word, see packLine()

uint16_t minGreen[SENSOR_COUNT];
uint16_t adc_buffer[SENSOR_COUNT];

void setup() {
#if SERIAL_DEBUG
    Serial.begin(115200);
#endif
    strip.begin();
    strip.setBrightness(75);
    setStartupPattern();
    calibrateSensors();
}

void loop() {
    processLineFast();
#if SERIAL_DEBUG
    printDebug();
#endif
    refreshLeds();
}

// I2C slave — transmit only, replaces the old CAN bus.
void setup1() {
    Wire1.setSDA(I2C_SDA_PIN);
    Wire1.setSCL(I2C_SCL_PIN);
    Wire1.begin(I2C_SLAVE_ADDR);
    Wire1.onRequest(onI2CRequest);
}

void loop1() {
}
