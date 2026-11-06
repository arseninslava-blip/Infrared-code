// ============================================================================
//  Моторы: CAN (MCP2515), синхронный разгон, омни-кинематика
// ============================================================================
// Режимы двигают робота только через omniMoveVector() / omniMoveDirection().

#pragma once
#include <SPI.h>
#include <mcp_can.h>
#include "config.h"
#include "vec2.h"

MCP_CAN motorCAN(CAN_CS_MOTOR);

// Запуск CAN-контроллера моторов; ждёт, пока MCP2515 не ответит
void motorsBegin() {
  SPI.setMOSI(CAN_MOSI_MOTOR);
  SPI.setMISO(CAN_MISO_MOTOR);
  SPI.setSCK(CAN_SCK_MOTOR);
  SPI.begin();
  while (motorCAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) delay(100);
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
