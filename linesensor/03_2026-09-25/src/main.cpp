#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include "config.h"
#include "functions.h"

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

uint8_t cmd = 0;
int16_t angle = 0;
uint8_t line = 0;
bool package = false;

uint16_t minGreen[SENSOR_COUNT];
uint16_t adc_buffer[SENSOR_COUNT];
uint8_t information[8];
uint8_t operation[8];
uint32_t last_calib[SENSOR_COUNT];

void setup() {
#if NO_SERIAL_DEBUG == 0
    Serial.begin(115200);
#endif
    strip.begin();
    strip.setBrightness(150);
    setStartupPattern();
    calibrateSensors();
}

void loop() {
    switch (cmd) {
        case 0xFF:
            for (uint8_t i = 0; i < SENSOR_COUNT; i++) minGreen[i] = 1023;
            cmd = 0xFE;
            operation[0] = 0xFE;
            break;
        case 0xFE:
            for (uint8_t s = 0; s < SENSOR_COUNT; s++) {
                uint16_t val = readADCFast(s);
                if (val < minGreen[s]) minGreen[s] = val;
            }
            break;
        default:
            processLineFast();
            break;
    }
    char buf[64];
    sprintf(buf, "detected: %d | direct %d", line, angle);
    Serial.println(buf);
    setStartupPattern();
    //rainbow(150);
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
