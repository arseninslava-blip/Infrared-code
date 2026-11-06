#pragma once

#include <Arduino.h>

// 1 — print line, angle, raw ADC and thresholds to Serial every 200 ms (for tuning).
// 0 — Serial is not used at all. Can also be set from build_flags: -DSERIAL_DEBUG=1
#ifndef SERIAL_DEBUG
#define SERIAL_DEBUG 0
#endif

// I2C — transmit-only slave, replaces the old CAN bus
#define I2C_SDA_PIN     14
#define I2C_SCL_PIN     15
#define I2C_SLAVE_ADDR  0x42

// LED strip
#define LED_PIN     13
#define LED_COUNT   6
// How often to resend the LED colour, ms (0 — only once in setup)
#define LED_REFRESH_MS 1000

// Sensors — direct ADC reads, no multiplexer
#define SENSOR_COUNT 4
static const uint8_t SENSOR_PINS[SENSOR_COUNT] = { A0, A1, A2, A3 };

// Direction vector per sensor, 90° apart: A0=0°, A1=90°, A2=180°, A3=270°.
// Adjust if the sensors aren't mounted in a circle.
static const float sensVecs_fixed[SENSOR_COUNT][2] PROGMEM = {
    { 1,   0},
    {  0,  1},
    {-1,   0},
    {  0, -1}
};
