// ============================================================================
//  Запуск испытаний с пульта (ros-atom/panel.py)
// ============================================================================
// Команды приходят через link.h:
//   TASK 0        — стоп
//   TASK 1        — испытание 1: в центр поля
//   TASK 2        — углы по кругу, каждый раз через центр (CoordTest3)
//   GOTO x y      — испытание 3: ехать в точку поля, целые см
// Пульт спрашивает состояние несколько раз в секунду. Если он замолчал дольше REMOTE_TIMEOUT_MS,
// робот останавливается: без связи его нечем было бы остановить.

#pragma once
#include "config.h"
#include "state.h"
#include "sensors.h"
#include "navigation.h"

uint8_t remoteTask = 0;   // 0 — пульт ничем не управляет
Vec2 remoteTarget;
uint32_t remoteLastCmd = 0;

// Возвращает true, если команда — для этого файла. Любая команда значит, что пульт на связи
bool remoteCommand(const char *cmd) {
  remoteLastCmd = millis();
  int task, x, y;
  if (sscanf(cmd, "TASK %d", &task) == 1 && task >= 0 && task <= 2) {
    remoteTask = task;
    return true;
  }
  if (sscanf(cmd, "GOTO %d %d", &x, &y) == 2) {
    remoteTarget = Vec2(constrain(x, -REMOTE_MAX_X, REMOTE_MAX_X), constrain(y, -REMOTE_MAX_Y, REMOTE_MAX_Y));
    remoteTask = 3;
    return true;
  }
  return false;
}

// Что видит робот: дописывается к ответу пульту. Углы в осях робота, X и Y в см, BD в пикселях камеры
void remoteStatus(char *out, size_t size) {
  snprintf(out, size, " T=%d CAM=%d V=%d X=%d Y=%d A=%d B=%d BD=%d G1=%d G2=%d L=%d TX=%d TY=%d",
           remoteTask, camFresh() ? 1 : 0, fieldPosValid() ? 1 : 0, (int)lroundf(field_x), (int)lroundf(field_y),
           field_axis, ball_dir, ball_dist, gate_dir_1, gate_dir_2, cam_line_dir,
           (int)lroundf(remoteTarget.x), (int)lroundf(remoteTarget.y));
}

// Проход испытания; false — пульт ничем не управляет
bool remoteRun() {
  if (remoteTask && millis() - remoteLastCmd > REMOTE_TIMEOUT_MS) remoteTask = 0;
  switch (remoteTask) {
    case 1:  driveTo(Vec2(0, 0)); return true;
    case 2:  CoordTest3(); return true;
    case 3:  driveTo(remoteTarget); return true;
    default: return false;
  }
}
