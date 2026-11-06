#pragma once

#include <Arduino.h>

#define NO_SERIAL_DEBUG 0

// I2C — transmit-only slave, replaces the old CAN bus
#define I2C_SDA_PIN     14
#define I2C_SCL_PIN     15
#define I2C_SLAVE_ADDR  0x42

// LED strip
#define LED_PIN     13
#define LED_COUNT   6

// Sensors — direct ADC reads, no multiplexer
#define SENSOR_COUNT 4
static const uint8_t SENSOR_PINS[SENSOR_COUNT] = { A0, A1, A2, A3 };

static const uint8_t weight_lut[32] PROGMEM = {
    0,  18, 25, 31, 36, 40, 44, 48, 51, 54, 57, 60, 62, 65, 67, 69,
    71, 73, 75, 77, 78, 80, 82, 83, 85, 86, 88, 89, 90, 92, 93, 94
};

// Direction vector per sensor, 90° apart: A0=0°, A1=90°, A2=180°, A3=270°.
// Adjust if the sensors aren't mounted in a circle.
static const float sensVecs_fixed[SENSOR_COUNT][2] PROGMEM = {
    { 1,   0},
    {  0,  1},
    {-1,   0},
    {  0, -1}
};
