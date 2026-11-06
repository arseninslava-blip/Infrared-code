#pragma once

#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include "config.h"

extern Adafruit_NeoPixel strip;
extern uint8_t cmd;
extern int16_t angle;
extern uint8_t line;
extern bool package;
extern uint16_t minGreen[SENSOR_COUNT];
extern uint16_t adc_buffer[SENSOR_COUNT];
extern uint8_t information[8];
extern uint8_t operation[8];
extern uint32_t last_calib[SENSOR_COUNT];

inline uint16_t readADCFast(uint8_t channel) {
    return analogRead(SENSOR_PINS[channel]);
}

inline void readAllSensors() {
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        adc_buffer[i] = analogRead(SENSOR_PINS[i]);
    }
}

inline uint8_t fastWeight(uint16_t value, uint16_t min_val) {
    if (value >= min_val) return 0;
    uint16_t diff = min_val - value;
    uint8_t idx = (diff * 32) / min_val;
    if (idx >= 32) idx = 31;
    return pgm_read_byte(&weight_lut[idx]);
}

inline int16_t fastAtan2Lookup(int32_t y, int32_t x) {
    if (x == 0 && y == 0) return 0;
    uint8_t octant = 0;
    if (x < 0) { x = -x; octant += 4; }
    if (y < 0) { y = -y; octant += 2; }
    bool swap = false;
    if (y > x) { int32_t tmp = x; x = y; y = tmp; swap = true; }
    if (x == 0) return 0;
    uint16_t ratio = (y << 10) / x;
    if (ratio > 1023) ratio = 1023;
    uint16_t angle_deg = (ratio * 90) >> 10;
    if (swap) angle_deg = 90 - angle_deg;
    if (octant & 4) angle_deg = 180 - angle_deg;
    if (octant & 2) angle_deg = -angle_deg;
    return (int16_t)angle_deg;
}

inline void processLineFast() {
    for (uint8_t s = 0; s < SENSOR_COUNT; s++) {
        adc_buffer[s] = analogRead(SENSOR_PINS[s]);
    }

    int8_t sum_x = 0, sum_y = 0;
    int32_t total_weight = 0;
    for (uint8_t s = 0; s < SENSOR_COUNT; s++) {
        if (adc_buffer[s] < minGreen[s] - 15) {
            sum_x += sensVecs_fixed[s][0];
            sum_y += sensVecs_fixed[s][1];
            total_weight++;
        }
    }

    if (total_weight < 1) {
        line = 0;
        angle = 0;
    } else {
        line = 1;
        float norm_x = sum_x;//(sum_x << 10) / total_weight;
        float norm_y = sum_y;//(sum_y << 10) / total_weight;
        angle = atan2(norm_y, norm_x) * 57.296;
    }

#if NO_SERIAL_DEBUG == 0
    //Serial.println(angle);
#endif
    package = true;
}

inline void calibrateSensors() {
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        minGreen[i] = 1023;
        last_calib[i] = 0;
    }
    for (uint8_t sample = 0; sample < 100; sample++) {
        for (uint8_t s = 0; s < SENSOR_COUNT; s++) {
            uint16_t val = readADCFast(s);
            if (val < minGreen[s]) minGreen[s] = val;
        }
        delayMicroseconds(100);
    }
}

// Green-to-white fade across the strip (ported from main.cpp's LED pattern).
inline void setStartupPattern() {
    strip.clear();
    for (uint16_t i = 0; i < LED_COUNT; i++) {
        uint8_t rb = (uint8_t)(255UL * (LED_COUNT - 1 - i) / (LED_COUNT - 1));
        strip.setPixelColor(i, strip.Color(255, 255, 150));
    }
    strip.show();
}

// Non-blocking: gated by millis() so it can run every loop() iteration
// without stalling sensor reads.
inline void rainbow(int wait) {
    static uint32_t hue = 0;
    static uint32_t last_update = 0;
    uint32_t now = millis();
    if (now - last_update < (uint32_t)wait) return;
    last_update = now;
    for (uint16_t i = 0; i < LED_COUNT; i++) {
        strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(hue + (i * 65536UL / LED_COUNT))));
    }
    strip.show();
    hue += 4096;
}

inline void colorWipe(uint32_t color, int wait) {
    for (int i = 0; i < strip.numPixels(); i++) strip.setPixelColor(i, color);
    strip.show();
    if (wait) delay(wait);
}

inline int readADC(uint8_t channel, int pin) {
    (void)pin;
    return readADCFast(channel);
}

// Transmit-only I2C callback: answers read requests with the latest line
// data. No onReceive is registered, so nothing written to the Pico is acted on.
inline void onI2CRequest() {
    information[0] = line;
    information[1] = angle & 0xFF;
    information[2] = (angle >> 8) & 0xFF;
    Wire1.write(information, 3);
#if NO_SERIAL_DEBUG == 0
    static uint32_t last_sent = 0;
    if (millis() - last_sent > 200) {
        last_sent = millis();
        Serial.printf("I2C response sent: line=%d angle=%d\n", line, angle);
    }
#endif
}
