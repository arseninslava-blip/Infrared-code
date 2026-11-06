// ============================================================================
//  Отладка в Serial: строка состояния (ядро 0) и время циклов (ядро 1)
// ============================================================================
// Включается в config.h: SERIAL_DEBUG, LOOP_TIMING, STATUS_INTERVAL.

#pragma once
#include "state.h"
#include "i2c_bus.h"
#include "camera.h"

// Длительность участка кода, мкс: среднее и максимум с прошлой печати
struct TimeStat {
  uint32_t sum = 0, max = 0, n = 0;
  void add(uint32_t us) { sum += us; if (us > max) max = us; n++; }
  unsigned long avg() const { return n ? sum / n : 0; }
  void reset() { sum = max = n = 0; }
};

// Ядро 0, раз в STATUS_INTERVAL: режим и всё, что пришло от датчиков (+ время цикла ядра 0).
// Каждая строка уходит одним println, чтобы не перемешиваться с печатью ядра 1.
void printStatus(bool attack, uint32_t loopUs) {
#if SERIAL_DEBUG
  static TimeStat loop0;
  static uint32_t last = 0, lastCamErrors = 0, lastRecoveries = 0;
  loop0.add(loopUs);
  if (millis() - last < STATUS_INTERVAL) return;
  last = millis();

  uint32_t errors = camErrors;
  char buf[256];
  snprintf(buf, sizeof(buf),
           "%s | камера %s, сбоев +%lu | мяч %d d=%d | ворота1 %d ворота2 %d | от линии %d | захват %d"
           " | датчик линии %d (%d) | курс %d",
           attack ? "АТАКА" : "СТОП", camFresh() ? "ok" : "МОЛЧИТ", (unsigned long)(errors - lastCamErrors),
           ball_dir, ball_dist, gate_dir_1, gate_dir_2, cam_line_dir, (int)trig,
           (int)line_be, line_dir, (int)imuData.euler[0]);
  lastCamErrors = errors;
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
