// ============================================================================
//  Углы в градусах: приведение к (-180, 180] и сложение направлений
// ============================================================================
// Для расчётов в координатах — Vec2 (vec2.h).

#pragma once
#include <math.h>

// Приводит угол к диапазону (-180, 180]. Работает и для отрицательных a < -180.
int norm180(int a) {
  a %= 360;
  if (a > 180) a -= 360;
  else if (a <= -180) a += 360;
  return a;
}

// Во float: на Cortex-M33 он аппаратный, double — программный.
// DEG_TO_RAD/RAD_TO_DEG из Arduino.h — double, поэтому здесь свои float-константы.
int VectorSum(int alfa1, int alfa2, float w1, float w2) {
  const float DEG2RAD = 0.017453292f, RAD2DEG = 57.29578f;
  float a1 = alfa1 * DEG2RAD, a2 = alfa2 * DEG2RAD;
  float x3 = w1 * cosf(a1) + w2 * cosf(a2);
  float y3 = w1 * sinf(a1) + w2 * sinf(a2);
  return (int)lroundf(atan2f(y3, x3) * RAD2DEG);
}
