// ============================================================================
//  Пеналка: импульс по аппаратному таймеру, перезарядка от последнего удара
// ============================================================================

#pragma once
#include <pico/time.h>
#include "config.h"

// Пеналка заряжена: с конца прошлого импульса (или со старта) прошло KICK_COOLDOWN_MS.
bool kickFlag = false;
uint32_t kickEndMs = 0;   // когда закончился (или закончится) последний импульс

// Вызывается в каждом проходе loop(). Перезарядка считается от последнего удара,
// а не по сетке таймера (с Ticker два удара могли пройти с разницей в доли секунды).
void updateKick() {
  if (!kickFlag && (int32_t)(millis() - kickEndMs) >= (int32_t)KICK_COOLDOWN_MS) kickFlag = true;
}

// Конец импульса — по аппаратному таймеру, точно через KICK_PULSE_MS, даже если loop() задержался
int64_t kickPulseEnd(alarm_id_t, void *) {
  digitalWrite(KICK, 0);
  return 0;   // не повторять
}

// Не блокирует: цикл управления продолжает работать, пока идёт импульс
void kicker() {
  if (kickFlag) {
    digitalWrite(KICK, 1);
    if (add_alarm_in_ms(KICK_PULSE_MS, kickPulseEnd, nullptr, true) < 0) {
      delay(KICK_PULSE_MS);   // нет свободного таймера — импульс по-старому, пин не останется включённым
      digitalWrite(KICK, 0);
    }
    kickFlag = false;
    kickEndMs = millis() + KICK_PULSE_MS;
  }
}
