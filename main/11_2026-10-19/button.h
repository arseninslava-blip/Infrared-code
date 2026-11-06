// ============================================================================
//  Кнопка режима атака/стоп (MODE_BTN)
// ============================================================================

#pragma once
#include "config.h"

// true один раз — когда кнопку отпустили после нажатия (как и раньше, по отпусканию).
// Уровень принимается, только если продержался DEBOUNCE_MS: дребезг не переключает режим дважды.
bool modeButtonClicked() {
  static bool pressed = false;      // состояние после антидребезга
  static bool lastRaw = false;
  static uint32_t changedAt = 0;
  bool raw = !digitalRead(MODE_BTN);   // INPUT_PULLUP: нажата = LOW
  if (raw != lastRaw) {
    lastRaw = raw;
    changedAt = millis();
  }
  if (raw != pressed && millis() - changedAt >= DEBOUNCE_MS) {
    pressed = raw;
    return !pressed;
  }
  return false;
}
