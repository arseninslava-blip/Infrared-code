// ============================================================================
//  Векторы и углы
// ============================================================================
// Vec2 — координаты и скорости; norm180/VectorSum — углы в градусах.

#pragma once
#include <math.h>

// ============================================================================
//  Vec2 — двумерный вектор для координатно-векторных расчётов
// ============================================================================
// Оси робота — как в omniMoveDirection(): x — вперёд, y — влево.
// Угол — в градусах от оси x, плюс — против часовой:
//   0 — вперёд, 90 — влево, -90 — вправо, 180 — назад (как ball_dir с камеры).
//
// Примеры:
//   Vec2 ball = Vec2::polar(ball_dir, ball_dist);    // мяч в осях робота
//   Vec2 away = Vec2::polar(line_dir, 2000);         // скорость «от линии»
//   omniMoveVector(ball.withLength(3000) + away, 0); // на мяч, но с отталкиванием от линии
//   float turn = Vec2::polar(gate_dir_1).angleTo(ball);  // на сколько мяч левее ворот, °


struct Vec2 {
  float x = 0, y = 0;

  static constexpr float DEG2RAD = 0.017453292f, RAD2DEG = 57.29578f;

  constexpr Vec2() = default;
  constexpr Vec2(float x_, float y_) : x(x_), y(y_) {}

  // Вектор длины len под углом deg
  static Vec2 polar(float deg, float len = 1.0f) {
    float r = deg * DEG2RAD;
    return Vec2(len * cosf(r), len * sinf(r));
  }

  // ---- арифметика ----
  constexpr Vec2 operator+(Vec2 b) const { return Vec2(x + b.x, y + b.y); }
  constexpr Vec2 operator-(Vec2 b) const { return Vec2(x - b.x, y - b.y); }
  constexpr Vec2 operator-() const { return Vec2(-x, -y); }
  constexpr Vec2 operator*(float k) const { return Vec2(x * k, y * k); }
  constexpr Vec2 operator/(float k) const { return Vec2(x / k, y / k); }
  Vec2 &operator+=(Vec2 b) { x += b.x; y += b.y; return *this; }
  Vec2 &operator-=(Vec2 b) { x -= b.x; y -= b.y; return *this; }
  Vec2 &operator*=(float k) { x *= k; y *= k; return *this; }
  Vec2 &operator/=(float k) { x /= k; y /= k; return *this; }

  // ---- скалярные величины ----
  constexpr float dot(Vec2 b) const { return x * b.x + y * b.y; }     // |a|·|b|·cos: > 0 — смотрят в одну сторону
  constexpr float cross(Vec2 b) const { return x * b.y - y * b.x; }   // |a|·|b|·sin: > 0 — b левее a
  constexpr float lengthSq() const { return x * x + y * y; }         // для сравнений — без sqrt
  float length() const { return sqrtf(lengthSq()); }
  float angle() const { return atan2f(y, x) * RAD2DEG; }             // (-180, 180]; у нулевого вектора 0
  float distanceTo(Vec2 b) const { return (b - *this).length(); }
  float angleTo(Vec2 b) const { return atan2f(cross(b), dot(b)) * RAD2DEG; }   // от этого вектора до b, (-180, 180]

  // ---- новые векторы ----
  Vec2 normalized() const {                       // длина 1; нулевой вектор остаётся нулевым
    float l = length();
    return l > 0 ? *this / l : Vec2();
  }
  Vec2 withLength(float len) const { return normalized() * len; }
  Vec2 clampLength(float maxLen) const {          // длина не больше maxLen, направление то же
    float l = length();
    return l > maxLen && l > 0 ? *this * (maxLen / l) : *this;
  }
  Vec2 rotated(float deg) const {                 // повёрнут на deg (плюс — против часовой)
    float r = deg * DEG2RAD, c = cosf(r), s = sinf(r);
    return Vec2(x * c - y * s, x * s + y * c);
  }
  constexpr Vec2 perp() const { return Vec2(-y, x); }   // повёрнут на +90° (влево)
  Vec2 projectOn(Vec2 axis) const {               // составляющая вдоль axis
    float l2 = axis.lengthSq();
    return l2 > 0 ? axis * (dot(axis) / l2) : Vec2();
  }

  // Линейная интерполяция: t = 0 — a, t = 1 — b
  static constexpr Vec2 lerp(Vec2 a, Vec2 b, float t) { return a + (b - a) * t; }

  // Шаг от from к to длиной не больше maxStep — плавная смена вектора скорости
  static Vec2 moveToward(Vec2 from, Vec2 to, float maxStep) {
    Vec2 d = to - from;
    float l = d.length();
    return l <= maxStep ? to : from + d * (maxStep / l);
  }
};

constexpr Vec2 operator*(float k, Vec2 v) { return v * k; }

// ============================================================================
//  Углы в градусах: приведение к (-180, 180] и сложение направлений
// ============================================================================
// Для расчётов в координатах — Vec2 (выше).


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
