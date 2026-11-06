// ============================================================================
//  BNO055 (гироскоп/IMU) - читается по уже настроенному Wire1 (см. setup1())
// ============================================================================

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
    bnoWriteReg(0x3D, BNO_OPR_MODE); // режим fusion, см. BNO_OPR_MODE в config.h (IMUPLUS / NDOF)
    delay(100);
    return true;
}

bool readBNOData(BNOData &data) {
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

    //if (!ok) Serial.println("не удалось считать гироскоп (BNO055)");
    return ok;
}

static float imuHPrev = 0;
static uint32_t imuHPrevUs = 0;

bool readBNOeuler(BNOData &data){
  uint8_t buf[6];

  // При неудачном чтении buf не заполнен - data не трогаем, остаётся прошлое значение,
  // а imuStamp не обновляется (ядро 0 по нему поймёт, что курс устарел).
  if (!bnoReadRegs(0x1A, buf, 6)) return false;

  for (int i = 0; i < 3; i++) data.euler[i] = (int16_t)((buf[i * 2 + 1] << 8) | buf[i * 2]) / 16.0f;
  imuStamp = millis();

  // Скорость рысканья (для D-члена стабилизации курса) - производная самого heading, а не
  // регистр гироскопа: знак автоматически совпадает с heading (не зависит от установки датчика
  // и осей). BNO обновляет выходы на 100 Гц, а loop1 читает чаще - считаем только при смене heading.
  float h = data.euler[0];
  uint32_t us = micros();
  if (h != imuHPrev) {
    float dt = (us - imuHPrevUs) * 1e-6f;
    float d = h - imuHPrev;
    if (d > 180) d -= 360; else if (d < -180) d += 360;
    if (dt >= 0.1f)        imuYawRate = 0;                                  // большая пауза - производной нет
    else if (dt > 0.003f)  imuYawRate = 0.5f * imuYawRate + 0.5f * (d / dt);
    imuHPrev = h;
    imuHPrevUs = us;
  } else if (us - imuHPrevUs > 30000) {
    imuYawRate = 0;                                                         // heading стоит - не вращаемся
  }
  return true;
}

void printBNOData(const BNOData &data) {
    //Serial.print("GYRO  heading=");  Serial.print(data.euler[0]);
    //Serial.print(" roll=");          Serial.print(data.euler[1]);
    //Serial.print(" pitch=");         Serial.print(data.euler[2]);
    //Serial.print(" gyroXYZ=");       Serial.print(data.gyro[0]);
    //Serial.print(",");               Serial.print(data.gyro[1]);
    //Serial.print(",");               Serial.print(data.gyro[2]);
    //Serial.print(" calib(S/G/A/M)="); Serial.print(data.cal[0]);
    //Serial.print("/");               Serial.print(data.cal[1]);
    //Serial.print("/");               Serial.print(data.cal[2]);
    //Serial.print("/");               Serial.println(data.cal[3]);
}
