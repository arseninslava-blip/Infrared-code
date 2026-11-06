#pragma once

#include <Wire.h>
#include "config.h"
#include "structs.h"

inline void bnoWriteReg(uint8_t reg, uint8_t val) {
    Wire1.beginTransmission(BNO_ADDR);
    Wire1.write(reg);
    Wire1.write(val);
    Wire1.endTransmission();
    delay(30);
}

inline bool bnoReadRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
    Wire1.beginTransmission(BNO_ADDR);
    Wire1.write(reg);
    if (Wire1.endTransmission(false) != 0) return false;
    if (Wire1.requestFrom((uint8_t)BNO_ADDR, len) != len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = Wire1.read();
    return true;
}

inline bool beginIMU() {
    uint8_t id;
    if (!bnoReadRegs(BNO_CHIP_ID_REG, &id, 1) || id != 0xA0) {
        Serial.println("не удалось считать гироскоп (BNO055 не найден)");
        return false;
    }
    bnoWriteReg(0x3D, 0x00); // CONFIGMODE
    bnoWriteReg(0x3F, 0x20); // system reset
    delay(700);
    bnoWriteReg(0x3B, 0x00); // units: m/s^2, dps, degrees, celsius
    bnoWriteReg(0x3D, 0x0C); // NDOF (sensor fusion)
    delay(100);
    return true;
}

inline bool readEuler(BNOData &data) {
    uint8_t buf[6];
    if (!bnoReadRegs(0x1A, buf, 6)) {
        Serial.println("не удалось считать гироскоп (Euler)");
        return false;
    }
    for (int i = 0; i < 3; i++) data.euler[i] = (int16_t)((buf[i * 2 + 1] << 8) | buf[i * 2]) / 16.0f;
    return true;
}

inline bool readBNOData(BNOData &data) {
    uint8_t buf[8];
    bool ok = true;

    ok &= bnoReadRegs(0x08, buf, 6);
    for (int i = 0; i < 3; i++) data.acc[i] = (int16_t)((buf[i * 2 + 1] << 8) | buf[i * 2]) / 100.0f;

    ok &= bnoReadRegs(0x0E, buf, 6);
    for (int i = 0; i < 3; i++) data.mag[i] = (int16_t)((buf[i * 2 + 1] << 8) | buf[i * 2]) / 16.0f;

    ok &= bnoReadRegs(0x14, buf, 6);
    for (int i = 0; i < 3; i++) data.gyro[i] = (int16_t)((buf[i * 2 + 1] << 8) | buf[i * 2]) / 16.0f;

    ok &= bnoReadRegs(0x1A, buf, 6);
    for (int i = 0; i < 3; i++) data.euler[i] = (int16_t)((buf[i * 2 + 1] << 8) | buf[i * 2]) / 16.0f;

    ok &= bnoReadRegs(0x20, buf, 8);
    for (int i = 0; i < 4; i++) data.quat[i] = (int16_t)((buf[i * 2 + 1] << 8) | buf[i * 2]) / 16384.0f;

    ok &= bnoReadRegs(0x28, buf, 6);
    for (int i = 0; i < 3; i++) data.linAcc[i] = (int16_t)((buf[i * 2 + 1] << 8) | buf[i * 2]) / 100.0f;

    ok &= bnoReadRegs(0x2E, buf, 6);
    for (int i = 0; i < 3; i++) data.grav[i] = (int16_t)((buf[i * 2 + 1] << 8) | buf[i * 2]) / 100.0f;

    ok &= bnoReadRegs(0x34, buf, 1);
    data.temp = (int8_t)buf[0];

    ok &= bnoReadRegs(0x35, buf, 1);
    data.cal[0] = (buf[0] >> 6) & 0x03;
    data.cal[1] = (buf[0] >> 4) & 0x03;
    data.cal[2] = (buf[0] >> 2) & 0x03;
    data.cal[3] = buf[0] & 0x03;

    if (!ok) Serial.println("не удалось считать гироскоп (BNO055)");
    return ok;
}

inline void printBNOData(const BNOData &data) {
    Serial.print("GYRO  heading=");  Serial.print(data.euler[0]);
    Serial.print(" roll=");          Serial.print(data.euler[1]);
    Serial.print(" pitch=");         Serial.print(data.euler[2]);
    Serial.print(" gyroXYZ=");       Serial.print(data.gyro[0]);
    Serial.print(",");               Serial.print(data.gyro[1]);
    Serial.print(",");               Serial.print(data.gyro[2]);
    Serial.print(" calib(S/G/A/M)="); Serial.print(data.cal[0]);
    Serial.print("/");               Serial.print(data.cal[1]);
    Serial.print("/");               Serial.print(data.cal[2]);
    Serial.print("/");               Serial.println(data.cal[3]);
}
