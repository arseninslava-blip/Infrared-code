// ============================================================================
//  Езда по координатам поля
// ============================================================================
// Координаты считает камера по двум воротам (state.h: field_x, field_y, field_axis):
//   X — вдоль поля: +110 у ворот 1 (цель атаки), −110 у ворот 2, 0 — середина;
//   Y — смещение от линии между воротами, плюс — левее, если смотреть на ворота 1 (до ±80).
// Оси поля в осях робота задаёт field_axis — направление на ворота 1 вдоль поля.
// Единицы X и Y — сантиметры поля: камера переводит ворота из кадра в см по таблице
// зеркала (MIRROR_CAL в EXP-5.py, замер 19.10).
//
// Пример: driveTo(Vec2(0, 0)) — ехать в центр поля; возвращает true, когда приехал.

#pragma once
#include "config.h"
#include "state.h"
#include "camera.h"
#include "motors.h"
#include "vec2.h"

// Знаем ли мы сейчас, где робот: камера на связи и видит обе пары ворот
bool fieldPosValid() { return field_valid && camFresh(); }

// Положение робота на поле (имеет смысл, только если fieldPosValid())
Vec2 fieldPos() { return Vec2(field_x, field_y); }

// Перевод вектора из осей поля в оси робота (для omniMoveVector)
Vec2 fieldToRobot(Vec2 v) {
  Vec2 ax = Vec2::polar(field_axis);   // +X поля в осях робота
  return ax * v.x + ax.perp() * v.y;   // +Y поля — влево от +X
}

// Ехать к точке target (координаты поля) с вращением w.
// Торможение равномерное: скорость ~ корень из пути до края области приезда — дальше
// GOTO_BRAKE_DIST от него GOTO_SPEED_MAX, у самого края — не меньше GOTO_SPEED_MIN.
// Так робот гасит скорость с постоянным замедлением, а не почти всю — на последних
// сантиметрах (линейный спад с 20 см давал перелёт ~50 см). В области приезда (отклонение и по X, и по Y
// не больше GOTO_TOLERANCE) робот плавно останавливается (рампа моторов), вращение w
// остаётся, функция возвращает true. Снова трогаемся, только если отклонение превысит
// GOTO_RESUME_TOL, чтобы шум камеры не дёргал робота на месте.
// Нет координат (камера молчит или не видит обе пары ворот) — стоим и возвращаем false.
bool driveTo(Vec2 target, int32_t w = 0) {
  static bool arrived = false;
  static Vec2 lastTarget;
  if (target.distanceTo(lastTarget) > 0.01f) {   // новая точка — едем заново
    arrived = false;
    lastTarget = target;
  }

  if (!fieldPosValid()) {
    omniMoveVector(Vec2(), 0);
    return false;
  }

  Vec2 err = target - fieldPos();   // в осях поля
  float tol = arrived ? GOTO_RESUME_TOL : GOTO_TOLERANCE;
  arrived = fabsf(err.x) <= tol && fabsf(err.y) <= tol;
  if (arrived) {
    omniMoveVector(Vec2(), w);         // плавная остановка: моторы тормозят по рампе
    return true;
  }

  // Доля тормозного пути до края области приезда: 0 — на краю, 1 — дальше GOTO_BRAKE_DIST
  float k = constrain((err.length() - GOTO_TOLERANCE) / GOTO_BRAKE_DIST, 0.0f, 1.0f);
  float speed = fmaxf(GOTO_SPEED_MAX * sqrtf(k), GOTO_SPEED_MIN);   // v² ~ путь: замедление постоянное
  omniMoveVector(fieldToRobot(err).withLength(speed), w);
  return false;
}
