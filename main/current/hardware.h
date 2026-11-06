// ============================================================================
//  Моторы, пеналка, кнопка режима
// ============================================================================
// Моторы по CAN с синхронным разгоном и омни-кинематикой, пеналка, кнопка атака/стоп.

#pragma once
#include <SPI.h>
#include <mcp_can.h>
#include <pico/time.h>
#include "config.h"
#include "geometry.h"

// ============================================================================
//  Моторы: CAN (MCP2515), синхронный разгон, омни-кинематика
// ============================================================================
// Режимы двигают робота только через omniMoveVector() / omniMoveDirection().


MCP_CAN motorCAN(CAN_CS_MOTOR);

// Запуск CAN-контроллера моторов. Одна попытка: на плате без MCP2515 ожидание зависало навсегда.
// Если MCP2515 не ответил с первого раза, моторы не поедут до перезагрузки.
void motorsBegin() {
  SPI.setMOSI(CAN_MOSI_MOTOR);
  SPI.setMISO(CAN_MISO_MOTOR);
  SPI.setSCK(CAN_SCK_MOTOR);
  SPI.begin();
  // while (motorCAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) delay(100);
  motorCAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ);
  motorCAN.setMode(MCP_NORMAL);
}

struct Motor {
  int32_t target;
  uint32_t lastCmd;
  float current = 0;  // текущая точка разгона (плавно догоняет desired)
  // Новая скорость уходит не чаще MOTOR_SEND_PERIOD_MS (рампа меняет её почти в каждом проходе,
  // и без ограничения шина CAN была занята целиком); urgent (instant) — сразу.
  // Та же скорость повторяется раз в MOTOR_DATA_SEND_TIMEOUT.
  bool needsUpdate(int32_t n, bool urgent) {
    uint32_t since = millis() - lastCmd;
    if (n == target) return since > MOTOR_DATA_SEND_TIMEOUT;
    return urgent || since >= MOTOR_SEND_PERIOD_MS;
  }
} motors[4];

bool motorAccelEnabled = true;

bool sendMotor(uint8_t i, uint8_t cmd, uint8_t *data = nullptr) {
  if (i >= 4) return false;

  uint8_t d[8] = { cmd, 0, 0, 0, 0, 0, 0, 0 };
  if (data) memcpy(&d[4], data, 4);

  uint32_t ids[4] = { MOTOR_RL, MOTOR_FL, MOTOR_FR, MOTOR_RR };

  if (motorCAN.sendMsgBuf(ids[i], 0, 8, d) == CAN_OK) {
    motors[i].lastCmd = millis();
    return true;
  }
  return false;
}

// Разгон всех четырёх моторов вместе: за шаг каждый проходит одну и ту же долю оставшегося
// пути, поэтому все приходят к новой скорости одновременно и соотношение скоростей колёс
// (а значит, направление движения) по дороге не искажается. Раньше каждый мотор разгонялся
// сам по себе: тот, кому меняться меньше, приходил раньше — и робот уводило в сторону.
// Самый «дальний» мотор разгоняется с MOTOR_ACCEL_PER_SEC, как и раньше.
void setMotorSpeeds(const int32_t desired[4]) {
  static uint32_t lastRamp = 0;
  uint32_t now = millis();
  uint32_t dt = now - lastRamp;
  if (dt > 50) dt = 50;  // защита от гигантского шага после паузы/старта
  lastRamp = now;

  float goal[4];
  float maxDiff = 0;
  for (int i = 0; i < 4; i++) {
    goal[i] = constrain(desired[i], -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
    maxDiff = fmaxf(maxDiff, fabsf(goal[i] - motors[i].current));
  }

  float k = 1.0f;   // доля оставшегося пути за этот шаг — общая для всех моторов
  if (motorAccelEnabled) {
    float maxStep = MOTOR_ACCEL_PER_SEC * dt / 1000.0f;
    if (maxDiff > maxStep) k = maxStep / maxDiff;
  }

  for (int i = 0; i < 4; i++) {
    motors[i].current += (goal[i] - motors[i].current) * k;

    int32_t s = (int32_t)lroundf(motors[i].current);
    if (!motors[i].needsUpdate(s, !motorAccelEnabled)) continue;

    int32_t v = s * 100;
    uint8_t d[4] = { (uint8_t)v, (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24) };
    if (sendMotor(i, CMD_SPEED, d)) motors[i].target = s;
  }
}

void allMotorsOn() {
  for (int i = 0; i < 4; i++) sendMotor(i, CMD_MOTOR_ON);
}

void allMotorsStop() {
  for (int i = 0; i < 4; i++) {
    sendMotor(i, CMD_MOTOR_STOP);
    // Рампа с нуля: при включении атаки моторы разгоняются, а не дёргаются со старой скоростью
    motors[i].current = 0;
    motors[i].target = 0;
  }
}

// Режим СТОП: остановка уходит сразу при входе в режим и потом раз в STOP_RESEND_MS,
// а не в каждом проходе loop() (4 кадра CAN за проход занимали шину целиком).
void stopMode(bool entered) {
  static uint32_t lastSent = 0;
  if (!entered && millis() - lastSent < STOP_RESEND_MS) return;
  allMotorsStop();
  lastSent = millis();
}

// Ехать с вектором скорости v (x — вперёд, y — влево, длина — как wheelSpeed) и вращением w.
// instant=true - разгон (рампа) выключен на этот вызов, скорость выставляется
// сразу (например, отталкивание от линии - нужен резкий рывок, а не плавный
// набор). По умолчанию false - обычная плавная рампа.
void omniMoveVector(Vec2 v, int32_t w, bool instant = false) {
  motorAccelEnabled = !instant;

  int32_t motor_dps[4];

  static constexpr float SIN_A[4] = { -0.5f, 0.5f, 0.5f, -0.5f };
  static constexpr float COS_A[4] = { 0.8660254f, 0.8660254f, -0.8660254f, -0.8660254f };

  for (int i = 0; i < 4; i++) {
    motor_dps[i] = ((v.x * COS_A[i]) + v.y * SIN_A[i]) / 2 + w;
  }
  setMotorSpeeds(motor_dps);
}

// То же через скорость и направление в градусах (0 — вперёд, плюс — влево)
void omniMoveDirection(int32_t wheelSpeed, float direction, int32_t w, bool instant = false) {
  float directionRad = (direction)*DEG_TO_RAD;
  omniMoveVector(Vec2(wheelSpeed * cos(directionRad), wheelSpeed * sin(directionRad)), w, instant);
}

// ============================================================================
//  Пеналка: импульс по аппаратному таймеру, перезарядка от последнего удара
// ============================================================================


// Пеналка заряжена: с конца прошлого импульса (или со старта) прошло KICK_COOLDOWN_MS.
bool kickFlag = false;
uint32_t kickEndMs = 0;   // когда закончился (или закончится) последний импульс

// Вызывается в каждом проходе loop(). Перезарядка считается от последнего удара,
// а не по сетке таймера (с Ticker два удара могли пройти с разницей в доли секунды).
void updateKick() {
  if (!kickFlag && (int32_t)(millis() - kickEndMs) >= (int32_t)KICK_COOLDOWN_MS) kickFlag = true;
}

// Идёт импульс пеналки или прошло меньше KICK_SETTLE_MS после него (боёк ещё возвращается)
bool kickActive() { return kickEndMs != 0 && (int32_t)(millis() - kickEndMs) < KICK_SETTLE_MS; }

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
#if SERIAL_DEBUG
    // Каждый импульс — строка в Serial: сверить с тем, бьёт ли пеналка на самом деле.
    // t — от старта платы: если t начался заново, плата перезагрузилась (причина — «СТАРТ платы»)
    static uint32_t kicks = 0;
    char buf[64];
    snprintf(buf, sizeof(buf), "УДАР %lu (t=%lu мс)", (unsigned long)++kicks, (unsigned long)millis());
    Serial.println(buf);
#endif
  }
}

// ============================================================================
//  Кнопка режима атака/стоп (MODE_BTN)
// ============================================================================


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
