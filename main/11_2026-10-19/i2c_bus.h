// ============================================================================
//  Шина Wire1 (камера, IMU, датчик линии): запуск, таймаут, освобождение
// ============================================================================
// Если ведомый не отпускает шину (оборвалась посылка, и он держит SDA), каждое чтение
// ждёт до конца таймаута и молчат все три датчика сразу. По умолчанию таймаут Wire — 1 с,
// и цикл ядра 1 растягивался до 3 с. Поэтому:
//   - таймаут одной посылки I2C_TIMEOUT_MS; после него ядро Wire перезапускает свой I2C;
//   - если ни одно устройство не отвечает дольше I2C_STUCK_MS, шина освобождается вручную:
//     до 9 тактов SCL (ведомый дошлёт байт и отпустит SDA), STOP и Wire1 заново.
// Если SCL держит сама камера (растянула такт и не отпускает), отсюда это не лечится:
// камера через 0,5 с без обмена перезапускает свой I2C (I2C_RESET_MS в EXP-5.py).

#pragma once
#include <Wire.h>
#include "config.h"

volatile uint32_t i2cRecoveries = 0;   // сколько раз освобождали шину (пишет ядро 1)

void i2cBegin() {
  Wire1.setSDA(SDA_PIN);
  Wire1.setSCL(SCL_PIN);
  Wire1.begin();
  Wire1.setClock(I2C_CLOCK_HZ);
  Wire1.setTimeout(I2C_TIMEOUT_MS, true);   // true — после таймаута ядро Wire перезапускает I2C
}

// Линия шины как «открытый сток»: у вывода с INPUT_PULLUP digitalWrite(LOW) тянет к земле,
// digitalWrite(HIGH) отпускает на подтяжку (так устроен digitalWrite в arduino-pico)
static void i2cPulse(uint8_t pin, PinStatus level) {
  digitalWrite(pin, level);
  delayMicroseconds(5);   // ~100 кГц
}

void i2cRecover() {
  Wire1.end();
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
  delayMicroseconds(5);

  // Ведомый посреди байта держит SDA: тактуем, пока не отпустит (больше 9 тактов не нужно)
  for (uint8_t i = 0; i < 9 && digitalRead(SDA_PIN) == LOW; i++) {
    i2cPulse(SCL_PIN, LOW);
    i2cPulse(SCL_PIN, HIGH);
  }
  // STOP: SDA снизу вверх при отпущенной SCL — ведомые сбрасывают своё состояние
  i2cPulse(SCL_PIN, LOW);
  i2cPulse(SDA_PIN, LOW);
  i2cPulse(SCL_PIN, HIGH);
  i2cPulse(SDA_PIN, HIGH);

  i2cBegin();
  i2cRecoveries++;
}

// Ядро 1, в конце прохода loop1: anyOk — ответило ли хоть одно устройство.
// Никто не отвечает дольше I2C_STUCK_MS — шина зависла, освобождаем (не чаще раза в I2C_STUCK_MS).
void i2cWatch(bool anyOk) {
  static uint32_t lastOk = 0;
  if (anyOk) {
    lastOk = millis();
    return;
  }
  if (millis() - lastOk < I2C_STUCK_MS) return;
  i2cRecover();
  lastOk = millis();
}
