// ============================================================================
//  Режим атаки
// ============================================================================
// Вход — данные камеры и линии (state.h), выход — omniMoveDirection() и kicker().

#pragma once
#include "state.h"
#include "camera.h"
#include "motors.h"
#include "kicker.h"
#include "angles.h"

int accel_via_distance() {
  return ACCEL_DISTANCE_SPEED_MULTIPLIER * ball_dist;
}

// Скорость объезда мяча: чем дальше мяч по углу от 0 (центра), тем быстрее едем.
int attackSpeed(int dir) {
  int a = constrain(abs(dir), 0, 180);
  return map(a, 0, 180, ATTACK_SPEED_MIN, ATTACK_SPEED_MAX);
}

float rot_speed = 0;
void AttackMode() {
  // Камера молчит дольше SENSOR_TIMEOUT — её углы устарели (при ошибке она перезапускается ~2 с).
  // С линии съезжаем по датчику линии, иначе плавно останавливаемся и не крутимся.
  if (!camFresh()) {
    if (line_be) omniMoveDirection(1600, line_dir, 0, true);
    else omniMoveDirection(0, 0, 0);
    return;
  }

  rot_speed = gate_dir_1 * 10;
  if ((cam_line_dir > -1 || line_be)) {
    if (line_be) {
      omniMoveDirection(1600, line_dir, rot_speed, true);
    } else {

      omniMoveDirection(1600, VectorSum(cam_line_dir, ball_dir, 3, 1), rot_speed, true);
      //VectorSum(cam_line_dir, ball_dir, 2, 1)
    }
    return;
  }


  if (!trig) {
    digitalWrite(LED_BUILTIN, 0);
    if (abs(ball_dir) < 10) {
      // Мяч перед роботом и не захвачен - едем прямо на него
      omniMoveDirection(5500, ball_dir, rot_speed);
      if (abs(gate_dir_1) < 15) {
        kicker();
      }
    } else {
      // Мяч не перед роботом - объезжаем, скорость растёт с углом
      if (ball_dist <= 20) {
        omniMoveDirection(attackSpeed(ball_dir) + accel_via_distance(), (abs(ball_dir) + 90) * (ball_dir / abs(ball_dir)), rot_speed);
      } else {
        omniMoveDirection(5000, ball_dir, rot_speed);
      }
    }
  } else {
    digitalWrite(LED_BUILTIN, 1);
    // Мяч захвачен - целимся в ворота и бьём
    if (abs(gate_dir_1) < 15) {
      kicker();
    }
    omniMoveDirection(5000, 0, rot_speed);
  }
}
