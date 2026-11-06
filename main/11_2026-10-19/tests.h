// ============================================================================
//  Испытания на поле
// ============================================================================
// Включаются в loop() (main.ino) вместо AttackMode() на время проверки: RUN_MODE в config.h.
// Настройки — раздел «Испытания на поле» в config.h.

#pragma once
#include "config.h"
#include "state.h"
#include "navigation.h"

// Доворот корпуса на ворота 2 — жёлтые, пока в камере SWAP_GATES = False.
// Как атака держит корпус на воротах 1 (rot_speed = gate_dir_1 * 10).
int32_t faceGate2() { return gate_dir_2 * FACE_GATE_KP; }

// Угол прямоугольника TEST2_RECT_W × TEST2_RECT_H по номеру 1..4 (против часовой от (+X, +Y))
Vec2 rectCorner(uint8_t n) {
  const float hx = TEST2_RECT_W / 2, hy = TEST2_RECT_H / 2;
  switch (n) {
    case 2:  return Vec2(-hx, hy);
    case 3:  return Vec2(-hx, -hy);
    case 4:  return Vec2(hx, -hy);
    default: return Vec2(hx, hy);
  }
}

// Светодиод платы мигает (раз в TEST_LED_BLINK_MS), пока робот в области приезда; иначе погашен
void zoneLed(bool inZone) {
  static uint32_t toggledAt = 0;
  static bool on = false;
  if (!inZone) {
    if (on) digitalWrite(LED_BUILTIN, 0);
    on = false;
    return;
  }
  if (millis() - toggledAt >= TEST_LED_BLINK_MS) {
    toggledAt = millis();
    on = !on;
    digitalWrite(LED_BUILTIN, on);
  }
}

// Число с одним знаком после запятой: printf с %f в ядре может быть выключен
char *putTenths(char *p, float v) {
  long t = lroundf(v * 10);
  if (t < 0) {
    *p++ = '-';
    t = -t;
  }
  return p + sprintf(p, "%ld.%ld", t / 10, t % 10);
}

// Мини-лог испытания в Serial: строка LOG,... раз в TEST_LOG_MS — удобно записать в файл
// (pio device monitor --filter log2file) или скопировать из монитора порта и открыть как CSV.
// Колонки: время от старта испытания, шаг, цель X/Y, положение X/Y, расстояние до цели,
// робот в области ±GOTO_TOLERANCE, driveTo считает «приехал», положение известно, ось поля.
void testLog(bool header, uint32_t tMs, uint8_t step, Vec2 target, bool inZone, bool arrived) {
#if TEST_LOG_MS > 0
  static uint32_t last = 0;
  if (header) {
    Serial.println("LOG,t_ms,step,target_x,target_y,x,y,dist,zone,arrived,valid,axis");
    last = 0;
  }
  if (!header && millis() - last < TEST_LOG_MS) return;
  last = millis();

  char buf[128];
  char *p = buf + sprintf(buf, "LOG,%lu,%u,", (unsigned long)tMs, (unsigned)step);
  p = putTenths(p, target.x); *p++ = ',';
  p = putTenths(p, target.y); *p++ = ',';
  p = putTenths(p, field_x); *p++ = ',';
  p = putTenths(p, field_y); *p++ = ',';
  p = putTenths(p, (target - fieldPos()).length());
  sprintf(p, ",%d,%d,%d,%d", (int)inZone, (int)arrived, (int)fieldPosValid(), field_axis);
  Serial.println(buf);
#else
  (void)header; (void)tMs; (void)step; (void)target; (void)inZone; (void)arrived;
#endif
}

// Один отрезок испытания: ехать к target, корпус на жёлтых воротах, мигать светодиодом
// в области приезда, писать мини-лог (restart — с заголовком). Возвращает true, когда driveTo
// приехал и робот простоял в точке TEST2_PAUSE_MS — пора к следующей точке.
bool testLeg(Vec2 target, uint8_t step, bool restart, uint32_t tMs) {
  static uint32_t arrivedAt = 0;
  if (restart) arrivedAt = 0;

  bool here = driveTo(target, faceGate2());
  Vec2 err = target - fieldPos();
  bool inZone = fieldPosValid() && fabsf(err.x) <= GOTO_TOLERANCE && fabsf(err.y) <= GOTO_TOLERANCE;
  zoneLed(inZone);
  testLog(restart, tMs, step, target, inZone, here);

  if (!here) {
    arrivedAt = 0;
    return false;
  }
  uint32_t now = millis();
  if (!arrivedAt) arrivedAt = now ? now : 1;
  if (now - arrivedAt < TEST2_PAUSE_MS) return false;
  arrivedAt = 0;
  return true;
}

// Испытание запускается заново, если его не вызывали дольше 200 мс (режим выключали кнопкой)
bool testRestarted(uint32_t &lastCall) {
  uint32_t now = millis();
  bool restart = now - lastCall > 200;
  lastCall = now;
  return restart;
}

// Испытание координат 2: из центра поля в угол TEST2_CORNER прямоугольника и обратно в центр.
// Шаги: 0 — в центр (если робот поставлен не точно), 1 — в угол, 2 — обратно в центр,
// 3 — испытание закончено, робот стоит в центре.
void CoordTest2() {
  static uint8_t step = 0;
  static uint32_t lastCall = 0, startMs = 0;
  bool restart = testRestarted(lastCall);
  if (restart) {
    step = 0;
    startMs = millis();
  }

  Vec2 target = step == 1 ? rectCorner(TEST2_CORNER) : Vec2(0, 0);
  bool next = testLeg(target, step, restart, millis() - startMs);
  if (step >= 3 || !next) return;   // после шага 3 driveTo просто держит робота в центре

  step++;
#if SERIAL_DEBUG
  static const char *const done[] = { "в центре, еду в угол", "в углу, еду обратно в центр",
                                      "вернулся в центр, испытание закончено" };
  char buf[96];
  snprintf(buf, sizeof(buf), "Испытание 2: %s (поле X %d Y %d)", done[step - 1],
           (int)lroundf(field_x), (int)lroundf(field_y));
  Serial.println(buf);
#endif
}

// Испытание координат 3: четыре угла прямоугольника по кругу без конца, и каждый раз через
// центр: центр -> 1 (+X, +Y) -> центр -> 2 (−X, +Y) -> центр -> 3 (−X, −Y) -> центр -> 4 (+X, −Y)
// -> центр -> 1 ... В каждой точке пауза TEST2_PAUSE_MS. Шаг в мини-логе: 0 — едем в центр,
// 1..4 — едем в этот угол.
void CoordTest3() {
  static uint8_t leg = 0;   // 0..7: чётные — центр, нечётные — угол leg / 2 + 1
  static uint32_t lastCall = 0, startMs = 0, laps = 0;
  bool restart = testRestarted(lastCall);
  if (restart) {
    leg = 0;
    laps = 0;
    startMs = millis();
  }

  uint8_t corner = leg % 2 ? leg / 2 + 1 : 0;   // 0 — центр
  Vec2 target = corner ? rectCorner(corner) : Vec2(0, 0);
  if (!testLeg(target, corner, restart, millis() - startMs)) return;

  leg = (leg + 1) % 8;
  if (leg == 0) laps++;   // вернулись в центр после угла 4
#if SERIAL_DEBUG
  char buf[160];
  uint8_t next = leg % 2 ? leg / 2 + 1 : 0;
  int n = corner ? snprintf(buf, sizeof(buf), "Испытание 3: в углу %u", (unsigned)corner)
                 : snprintf(buf, sizeof(buf), "Испытание 3: в центре");
  n += snprintf(buf + n, sizeof(buf) - n, " (поле X %d Y %d), ", (int)lroundf(field_x),
                (int)lroundf(field_y));
  if (next) snprintf(buf + n, sizeof(buf) - n, "еду в угол %u; кругов %lu", (unsigned)next,
                     (unsigned long)laps);
  else snprintf(buf + n, sizeof(buf) - n, "еду в центр; кругов %lu", (unsigned long)laps);
  Serial.println(buf);
#endif
}
