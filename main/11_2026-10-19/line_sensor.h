// ============================================================================
//  Датчик линии (PicoLineFollower) — 3 байта по I2C (адрес LINE_ADDR)
// ============================================================================
// readLine() — ядро 1, каждый проход loop1.

#pragma once
#include <Wire.h>
#include "state.h"

bool readLine() {
  uint8_t buf[3];
  if (Wire1.requestFrom(LINE_ADDR, 3) != 3) {
    // Нет ответа: не держим старое "линия видна" дольше LINE_TIMEOUT,
    // иначе робот будет вечно ехать "от линии".
    if (millis() - lineStamp > LINE_TIMEOUT) line_be = false;
    return false;
  }
  for (uint8_t i = 0; i < 3; i++) buf[i] = Wire1.read();
  // line_dir - одним присваиванием и раньше line_be: ядро 0 читает их без блокировок
  // и не должно увидеть промежуточное значение (до вычитания 180).
  line_dir = (int16_t)(buf[1] | (buf[2] << 8)) - 180;
  line_be = buf[0];
  lineStamp = millis();
  return true;
}
