// ============================================================================
//  BNO055 (гироскоп/IMU) - читается по уже настроенному Wire1 (см. setup1())
// ============================================================================

#pragma once
#include <Wire.h>
#include "state.h"

// Реальный адрес определяется в beginIMU() (0x29 или 0x28 - зависит от COM3).
uint8_t bnoAddr = BNO_ADDR;

bool bnoWriteReg(uint8_t reg, uint8_t val) {
    Wire1.beginTransmission(bnoAddr);
    Wire1.write(reg);
    Wire1.write(val);
    Wire1.endTransmission();
    delay(30);
    return true;
}

bool bnoReadRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
    Wire1.beginTransmission(bnoAddr);
    Wire1.write(reg);
    uint8_t txErr = Wire1.endTransmission(false);
    if (txErr != 0) return false; // нет ACK на этом адресе - устройства там нет
    if (Wire1.requestFrom((uint8_t)bnoAddr, len) != len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = Wire1.read();
    return true;
}

bool beginIMU() {
    uint8_t id;
    const uint8_t candidates[2] = { BNO_ADDR, BNO_ADDR_ALT };

    for (uint8_t c = 0; c < 2; c++) {
        bnoAddr = candidates[c];
        if (bnoReadRegs(BNO_CHIP_ID_REG, &id, 1)) {
            if (id == 0xA0) {
                //Serial.print("BNO055 найден на 0x"); Serial.println(bnoAddr, HEX);
                break;
            }
        } else {
            //Serial.print("нет ответа (NACK) по адресу 0x"); Serial.println(bnoAddr, HEX);
        }
        if (c == 1) {
            //Serial.println("не удалось считать гироскоп (BNO055 не найден ни на 0x28, ни на 0x29) - проверь питание, SDA/SCL и подтяжки (4.7к на 3.3В)");
            return false;
        }
    }

    bnoWriteReg(0x3D, 0x00); // CONFIGMODE
    bnoWriteReg(0x3F, 0x20); // system reset
    delay(700);
    bnoWriteReg(0x3B, 0x00); // units: m/s^2, dps, degrees, celsius
    bnoWriteReg(0x3D, 0x0C); // NDOF (sensor fusion)
    delay(100);
    return true;
}

bool readBNOeuler(BNOData &data){
  uint8_t buf[6];

  // При неудачном чтении buf не заполнен - data не трогаем, остаётся прошлое значение,
  // а imuStamp не обновляется (ядро 0 по нему поймёт, что курс устарел).
  if (!bnoReadRegs(0x1A, buf, 6)) return false;

  for (int i = 0; i < 3; i++) data.euler[i] = (int16_t)((buf[i * 2 + 1] << 8) | buf[i * 2]) / 16.0f;
  imuStamp = millis();
  return true;
}

bool imuFresh() { return isFresh(imuStamp, SENSOR_TIMEOUT); }
