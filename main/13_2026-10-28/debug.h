// ============================================================================
//  Отладка в Serial: строка состояния (ядро 0) и время циклов (ядро 1)
// ============================================================================
// Включается в config.h: SERIAL_DEBUG, LOOP_TIMING, STATUS_INTERVAL.

#pragma once
#include "state.h"
#include "sensors.h"

// Длительность участка кода, мкс: среднее и максимум с прошлой печати
struct TimeStat {
  uint32_t sum = 0, max = 0, n = 0;
  void add(uint32_t us) { sum += us; if (us > max) max = us; n++; }
  unsigned long avg() const { return n ? sum / n : 0; }
  void reset() { sum = max = n = 0; }
};

// Почему плата стартовала. «просадка питания» или «сбой питания» — значит, питание платы
// роняет что-то мощное: удар пеналки или разгон моторов.
void printBootReason() {
  static const char *const names[] = { "неизвестно", "включение питания", "кнопка RUN",
                                       "программный сброс", "сторожевой таймер", "отладчик",
                                       "сбой питания (glitch)", "просадка питания (brownout)" };
  unsigned r = (unsigned)rp2040.getResetReason();
  char buf[96];
  snprintf(buf, sizeof(buf), "СТАРТ платы: %s", r < 8 ? names[r] : "?");
  Serial.println(buf);
}

// Ядро 0, раз в STATUS_INTERVAL: режим и всё, что пришло от датчиков (+ время цикла ядра 0).
// Каждая строка уходит одним println, чтобы не перемешиваться с печатью ядра 1.
void printStatus(bool attack, uint32_t loopUs) {
#if SERIAL_DEBUG
  static TimeStat loop0;
  static uint32_t last = 0, lastCamErrors = 0, lastLineErrors = 0, lastImuErrors = 0, lastRecoveries = 0;
  static bool bootShown = false;
  loop0.add(loopUs);
  if (millis() - last < STATUS_INTERVAL) return;
  last = millis();
  // Через 3 с после старта, а не в setup(): к этому времени монитор порта успевает
  // переподключиться к USB после перезагрузки платы
  if (!bootShown && millis() > 3000) {
    printBootReason();
    bootShown = true;
  }

  uint32_t errors = camErrors, lErrors = lineErrors;
  char buf[320];
  snprintf(buf, sizeof(buf),
           "%s | камера %s, сбоев +%lu | мяч %d d=%d | ворота1 %d ворота2 %d | от линии %d до неё (%d), дальней %d (%d) | захват %d"
           " | датчик линии %s, сбоев +%lu: %d (%d)",
           attack ? "АТАКА" : "СТОП", camFresh() ? "ok" : "МОЛЧИТ", (unsigned long)(errors - lastCamErrors),
           ball_dir, ball_dist, gate_dir_1, gate_dir_2, cam_line_dir, out_dist, cam_line_dir_2, out_dist_2, (int)trig,
           lineFresh() ? "ok" : "МОЛЧИТ", (unsigned long)(lErrors - lastLineErrors),
           (int)line_be, line_dir);
  lastCamErrors = errors;
  lastLineErrors = lErrors;
  Serial.println(buf);
  // Гироскоп: читается ли курс и не сбросился ли BNO055 (после сброса — режим CONFIG, курс 0)
  uint32_t iErrors = imuErrors;
  uint8_t mode = imuMode;
  const char *modeName = mode == BNO_MODE_NDOF ? "NDOF"
                       : mode == 0             ? "CONFIG — гироскоп сбросился, курс не считается"
                       : mode == 0xFF          ? "не прочитан"
                                               : "другой";
  snprintf(buf, sizeof(buf), "  гироскоп %s, сбоев +%lu | режим %s (0x%02X) | курс %d",
           imuFresh() ? "ok" : "МОЛЧИТ", (unsigned long)(iErrors - lastImuErrors), modeName,
           (unsigned)mode, (int)imuData.euler[0]);
  lastImuErrors = iErrors;
  Serial.println(buf);
  if (field_valid) {
    snprintf(buf, sizeof(buf), "  поле: X %d Y %d, ось %d", (int)lroundf(field_x),
             (int)lroundf(field_y), field_axis);
  } else {
    snprintf(buf, sizeof(buf), "  поле: неизвестно (нужны обе пары ворот)");
  }
  Serial.println(buf);
  uint32_t recoveries = i2cRecoveries;
  if (recoveries != lastRecoveries) {   // строка только когда шина зависала
    snprintf(buf, sizeof(buf), "  шина I2C зависала: освобождена +%lu раз (всего %lu)",
             (unsigned long)(recoveries - lastRecoveries), (unsigned long)recoveries);
    Serial.println(buf);
    lastRecoveries = recoveries;
  }
#if LOOP_TIMING
  snprintf(buf, sizeof(buf), "  ядро 0: цикл ср %lu макс %lu мкс", loop0.avg(), (unsigned long)loop0.max);
  Serial.println(buf);
#endif
  loop0.reset();
#else
  (void)attack;
  (void)loopUs;
#endif
}

// Ядро 1, раз в STATUS_INTERVAL: сколько занимает опрос камеры, IMU и датчика линии.
// Если «камера» в максимуме близка к кадру камеры (десятки мс) — она держит шину и тормозит линию.
// IMU читается раз в IMU_INTERVAL: в его статистику попадают только проходы с чтением (imuRead).
void core1Timing(uint32_t camUs, uint32_t imuUs, uint32_t lineUs, bool imuRead) {
#if SERIAL_DEBUG && LOOP_TIMING
  static TimeStat cam, imu, line, total;
  static uint32_t last = 0;
  cam.add(camUs);
  if (imuRead) imu.add(imuUs);
  line.add(lineUs);
  total.add(camUs + imuUs + lineUs);
  if (millis() - last < STATUS_INTERVAL) return;
  last = millis();

  char buf[192];
  snprintf(buf, sizeof(buf),
           "  ядро 1: цикл ср %lu макс %lu мкс (камера %lu/%lu, IMU %lu/%lu, линия %lu/%lu)",
           total.avg(), (unsigned long)total.max, cam.avg(), (unsigned long)cam.max,
           imu.avg(), (unsigned long)imu.max, line.avg(), (unsigned long)line.max);
  Serial.println(buf);
  cam.reset();
  imu.reset();
  line.reset();
  total.reset();
#else
  (void)camUs;
  (void)imuUs;
  (void)lineUs;
  (void)imuRead;
#endif
}
