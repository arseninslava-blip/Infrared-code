// ============================================================================
//  Главный контроллер робота (RP2350)
// ============================================================================
// Ядро 1 (setup1/loop1) опрашивает датчики по Wire1: камеру, IMU, датчик линии.
// Ядро 0 (setup/loop) принимает решения, управляет моторами по CAN и пеналкой.
//
// Файлы скетча:
//   config.h               — пины, адреса, константы
//   structs.h, state.h     — общие данные датчиков (пишет ядро 1, читает ядро 0)
//   i2c_bus.h              — шина Wire1: таймаут и освобождение, если зависла
//   camera.h               — пакет камеры OpenMV
//   line_sensor.h          — датчик линии
//   imu.h                  — гироскоп BNO055
//   motors.h               — CAN, разгон, движение (omniMoveVector / omniMoveDirection)
//   kicker.h, button.h     — пеналка, кнопка режима
//   vec2.h, angles.h       — векторы и углы
//   navigation.h           — езда по координатам поля (driveTo)
//   tests.h                — испытания на поле (CoordTest2, CoordTest3)
//   attack.h, defense.h    — режимы игры
//   debug.h                — отладочная печать
//   archive/               — старый код, не компилируется

#include <Arduino.h>

// Отдельный стек 8 КБ для ядра 1. Без этого 8 КБ делятся пополам между ядрами,
// и переполнение стека ядра 0 молча портит стек ядра 1. Стоит 8 КБ RAM из 520.
bool core1_separate_stack = true;

#include "config.h"
#include "structs.h"
#include "vec2.h"
#include "state.h"
#include "i2c_bus.h"
#include "camera.h"
#include "line_sensor.h"
#include "imu.h"
#include "motors.h"
#include "kicker.h"
#include "button.h"
#include "angles.h"
#include "navigation.h"
#include "tests.h"
#include "attack.h"
#include "defense.h"
#include "debug.h"

void setup() {
  Serial.begin(115200);
  motorsBegin();

  pinMode(MODE_BTN, INPUT_PULLUP);
  pinMode(KICK, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  allMotorsOn();
}

// Режим, который включает кнопка; выбирается RUN_MODE в config.h
void runMode() {
#if RUN_MODE == RUN_ATTACK
  AttackMode();
#elif RUN_MODE == RUN_DEFENSE
  DefenseMode();
#elif RUN_MODE == RUN_GOTO_CENTER
  driveTo(Vec2(0, 0));
#elif RUN_MODE == RUN_TEST2
  CoordTest2();
#elif RUN_MODE == RUN_CORNERS
  CoordTest3();
#else
#error "RUN_MODE: неизвестный режим, см. config.h"
#endif
}

void loop() {
  static bool modeOn = false;   // режим включён или стоп — переключается кнопкой

  uint32_t t0 = micros();
  camTake();
  updateKick();

  bool wasOn = modeOn;
  if (modeButtonClicked()) modeOn = !modeOn;

  if (modeOn) {
    runMode();
  } else {
    stopMode(wasOn);   // только что выключили — остановка сразу
  }

  printStatus(modeOn, micros() - t0);   // печать — только при SERIAL_DEBUG
}

void setup1() {
  i2cBegin();

  // BNO055 после подачи питания просыпается не мгновенно (~650-850мс POR).
  delay(800);
  for (uint8_t attempt = 0; attempt < 5 && !beginIMU(); attempt++) {
    delay(200);
  }
}

void loop1() {
  static uint32_t lastImu = 0;

  uint32_t t0 = micros();
  bool anyOk = cam_read();
  uint32_t t1 = micros();
  // BNO055 обновляет курс 100 раз в секунду: чаще IMU_INTERVAL читать незачем, шина нужнее линии
  bool imuNow = millis() - lastImu >= IMU_INTERVAL;
  if (imuNow) {
    lastImu = millis();
    anyOk |= readBNOeuler(imuData);
  }
  uint32_t t2 = micros();
  anyOk |= readLine();
  i2cWatch(anyOk);   // никто не ответил дольше I2C_STUCK_MS — шина зависла, освобождаем

  core1Timing(t1 - t0, t2 - t1, micros() - t2, imuNow);   // печать — только при SERIAL_DEBUG и LOOP_TIMING
}
