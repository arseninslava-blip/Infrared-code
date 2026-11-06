// ============================================================================
//  Общие данные датчиков
// ============================================================================
// Ядро 1 опрашивает датчики (loop1) и пишет сюда, ядро 0 принимает решения (loop) и читает.
// Данные камеры — копия последнего пакета (camTake() в начале loop), линия и курс — напрямую.

#pragma once
#include "structs.h"

// Датчик линии (readLine)
volatile bool line_be = 0;
volatile int line_dir = 0;

// Данные камеры для ядра 0: копия последнего пакета, обновляется в начале loop() (camTake),
// поэтому в пределах прохода все поля из одного пакета и не меняются посреди решения.
int16_t ball_dir = 0;
int16_t gate_dir_1 = 0;
int16_t gate_dir_2 = 0;
int16_t cam_line_dir = 0;
int16_t ball_dist = 0;
bool trig;   // захват мяча — из того же пакета камеры, что и ball_dir (копия ядра 0)

// Положение на поле — из того же пакета камеры (см. navigation.h).
// X — вдоль поля: +110 у ворот 1 (цель атаки), −110 у ворот 2, 0 — середина;
// Y — смещение от линии между воротами, плюс — левее, если смотреть на ворота 1 (до ±80).
bool field_valid = false;    // в последнем кадре были видны обе пары ворот
float field_x = 0, field_y = 0;
int16_t field_axis = 0;      // куда в осях робота смотрит +X поля (на ворота 1), °

BNOData imuData;   // курс BNO055 (readBNOeuler)

// Время (millis) последнего УСПЕШНОГО чтения датчика; 0 = ещё ни разу.
// Пишет ядро 1, проверяет ядро 0 (camFresh в camera.h, imuFresh в imu.h).
volatile uint32_t camStamp = 0;
volatile uint32_t lineStamp = 0;
volatile uint32_t imuStamp = 0;

static inline bool isFresh(uint32_t stamp, uint32_t timeoutMs) {
  return stamp != 0 && millis() - stamp <= timeoutMs;
}
