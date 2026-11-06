// ============================================================================
//  Главный контроллер робота (RP2350)
// ============================================================================
// Ядро 1 (setup1/loop1) опрашивает датчики по Wire1: камеру, IMU, датчик линии.
// Ядро 0 (setup/loop) принимает решения, управляет моторами по CAN и пеналкой.
//
// Файлы скетча:
//   config.h      — пины, адреса, константы, выбор режима RUN_MODE
//   geometry.h    — векторы (Vec2) и углы
//   state.h       — общие данные датчиков (пишет ядро 1, читает ядро 0)
//   sensors.h     — шина Wire1, камера OpenMV, датчик линии, гироскоп BNO055
//   hardware.h    — моторы по CAN, пеналка, кнопка режима
//   navigation.h  — езда по координатам поля (driveTo) и испытания на поле
//   modes.h       — режимы игры: атака, защита
//   debug.h       — отладочная печать
//   remote.h      — запуск испытаний с пульта
//   link.h        — команды через Bluetooth-модуль HC-05 (пульт — ros-atom/panel.py)
//   archive/      — старый код, не компилируется

#include <Arduino.h>

// Отдельный стек 8 КБ для ядра 1. Без этого 8 КБ делятся пополам между ядрами,
// и переполнение стека ядра 0 молча портит стек ядра 1. Стоит 8 КБ RAM из 520.
bool core1_separate_stack = true;

#include "config.h"
#include "geometry.h"
#include "state.h"
#include "sensors.h"
#include "hardware.h"
#include "navigation.h"
#include "modes.h"
#include "debug.h"
#include "remote.h"
#include "link.h"

void setup() {
  Serial.begin(115200);
  motorsBegin();

  pinMode(MODE_BTN, INPUT_PULLUP);
  //pinMode(BALL_TRIG, BALL_TRIG_ACTIVE ? INPUT_PULLDOWN : INPUT_PULLUP);   // без датчика — «мяча нет»
  pinMode(KICK, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  allMotorsOn();
  linkBegin();
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
}// 

void loop() {
  static bool modeOn = false;   // режим включён или стоп — переключается кнопкой

  uint32_t t0 = micros();
  linkPoll();
  static uint32_t lastAnalog = 0;
  if (millis() - lastAnalog >= ANALOG_PRINT_MS) {
    lastAnalog = millis();
    Serial.println(analogRead(A2));
  }
  camTake();
  APPTrig = analogRead(A2) > 500 && !kickActive();   // мяч в лунке — и для атаки, и для защиты
  updateKick();

  static bool wasRemote = false;
  bool wasOn = modeOn || wasRemote;
  if (modeButtonClicked()) {
    if (remoteTask) remoteTask = 0;   // кнопка на роботе останавливает и испытание с пульта
    else modeOn = !modeOn;
  }

  wasRemote = remoteRun();
  if (wasRemote) {
    modeOn = false;
  } else if (modeOn) {
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

  linkPassesCore1++;
  uint32_t t0 = micros();
  bool anyOk = cam_read();
  uint32_t t1 = micros();
  // BNO055 обновляет курс 100 раз в секунду: чаще IMU_INTERVAL читать незачем, шина нужнее линии
  bool imuNow = millis() - lastImu >= IMU_INTERVAL;
  if (imuNow) {
    lastImu = millis();
    anyOk |= readBNOeuler(imuData);
    checkIMUMode();   // не сбросился ли гироскоп (раз в IMU_MODE_CHECK_MS)
  }
  uint32_t t2 = micros();
#if LINE_ENABLED
  anyOk |= readLine();
#endif
  i2cWatch(anyOk);   // никто не ответил дольше I2C_STUCK_MS — шина зависла, освобождаем

  core1Timing(t1 - t0, t2 - t1, micros() - t2, imuNow);   // печать — только при SERIAL_DEBUG и LOOP_TIMING
}
