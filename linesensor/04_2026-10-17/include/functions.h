#pragma once

#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include "config.h"

extern Adafruit_NeoPixel strip;
extern volatile uint32_t lineState;
extern uint16_t minGreen[SENSOR_COUNT];
extern uint16_t adc_buffer[SENSOR_COUNT];

inline uint16_t readADCFast(uint8_t channel) {
    return analogRead(SENSOR_PINS[channel]);
}

// line and angle packed into one 32-bit word in I2C packet byte order: byte 0 = line,
// bytes 1-2 = angle (little-endian). An aligned 32-bit store/load is a single instruction
// on RP2040, so the I2C ISR can never send line from one reading and angle from another.
inline uint32_t packLine(uint8_t ln, int16_t ang) {
    return (uint32_t)ln | ((uint32_t)(uint16_t)ang << 8);
}

inline void processLineFast() {
    // Last angle that could be determined; kept when the line disappears
    static int16_t lastAngle = 0;

    for (uint8_t s = 0; s < SENSOR_COUNT; s++) {
        adc_buffer[s] = analogRead(SENSOR_PINS[s]);
    }

    int8_t sum_x = 0, sum_y = 0;
    uint8_t hits = 0;
    for (uint8_t s = 0; s < SENSOR_COUNT; s++) {
        if (adc_buffer[s] < minGreen[s] - 15) {
            sum_x += sensVecs_fixed[s][0];
            sum_y += sensVecs_fixed[s][1];
            hits++;
        }
    }

    // The angle is computed in locals and published with a single store
    if (hits == 0) {
        lineState = packLine(0, 0);
    } else {
        if (sum_x != 0 || sum_y != 0) {
            float norm_x = sum_x;
            float norm_y = sum_y;
            lastAngle = atan2(norm_y, norm_x) * 57.296;
        }
        // Sum (0,0): the line is under opposite sensors (A0+A2, A1+A3 or all four),
        // its direction can't be told, so the last known angle is sent
        lineState = packLine(1, lastAngle);
    }
}

inline void calibrateSensors() {
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        minGreen[i] = 1023;
    }
    for (uint8_t sample = 0; sample < 100; sample++) {
        for (uint8_t s = 0; s < SENSOR_COUNT; s++) {
            uint16_t val = readADCFast(s);
            if (val < minGreen[s]) minGreen[s] = val;
        }
        delayMicroseconds(100);
    }
}

// One constant colour on the whole strip: it lights the floor for the sensors,
// which are calibrated under this light right after it is switched on.
inline void setStartupPattern() {
    strip.clear();
    for (uint16_t i = 0; i < LED_COUNT; i++) {
        strip.setPixelColor(i, strip.Color(255, 255, 150));
    }
    strip.show();
}

// WS2812 pixels hold their colour, so show() is not needed every loop() pass (it took ~80%
// of the loop). The buffer is resent rarely, only to restore a pixel that noise on the data
// line may have changed: the sensors were calibrated under this light.
inline void refreshLeds() {
#if LED_REFRESH_MS > 0
    static uint32_t last = 0;
    if (millis() - last < LED_REFRESH_MS) return;
    last = millis();
    strip.show();
#endif
}

#if SERIAL_DEBUG
// Every 200 ms: what is sent over I2C, and raw ADC values against their thresholds
inline void printDebug() {
    static uint32_t last = 0;
    if (millis() - last < 200) return;
    last = millis();
    uint32_t s = lineState;
    Serial.printf("line=%d angle=%d | adc/threshold:", (int)(s & 0xFF), (int)(int16_t)(s >> 8));
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        Serial.printf(" %d/%d", (int)adc_buffer[i], (int)minGreen[i] - 15);
    }
    Serial.println();
}
#endif

// Transmit-only I2C callback: answers read requests with the latest line
// data. No onReceive is registered, so nothing written to the Pico is acted on.
// No printing here: an ISR must not wait for the USB serial mutex.
inline void onI2CRequest() {
    uint32_t s = lineState;              // one load: line and angle from the same reading
    uint8_t packet[3] = {
        (uint8_t)(s & 0xFF),             // line
        (uint8_t)((s >> 8) & 0xFF),      // angle, low byte
        (uint8_t)((s >> 16) & 0xFF),     // angle, high byte
    };
    Wire1.write(packet, 3);
}
