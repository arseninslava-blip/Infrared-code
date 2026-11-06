// ============================================================================
//  Камера OpenMV — пакет 18 байт по I2C (адрес CAM_ADDR)
// ============================================================================
// cam_read() — ядро 1, каждый проход loop1; camTake() — ядро 0, начало loop().
// Пакет — 9 чисел int16: мяч°, ворота 2°, ворота 1°, от линии°, дистанция до мяча, захват,
// X×10, Y×10 (положение на поле), ось поля°. Формат должен совпадать с _PACKET_FMT в камере.

#pragma once
#include <Wire.h>
#include <pico/mutex.h>
#include "state.h"

const size_t CAM_PACKET_LEN = 18;
const int16_t CAM_POS_NONE = INT16_MIN;   // положение неизвестно: в кадре не обе пары ворот

volatile uint32_t camErrors = 0;   // неудачных чтений камеры с запуска (пишет ядро 1)

bool camFresh() { return isFresh(camStamp, SENSOR_TIMEOUT); }

// Пакет камеры целиком. Ядро 1 кладёт его в camShared под мьютексом, ядро 0 забирает копию
// в начале loop(): раньше ядро 0 могло прочитать ball_dir из нового пакета, а ball_dist из старого.
struct CamPacket {
  int16_t ballDir, gate2, gate1, lineDir, ballDist;
  bool trig;
  int16_t posX10, posY10, fieldAxis;
  uint32_t seq;   // номер принятого пакета: камера шлёт один пакет на кадр
};
CamPacket camShared = { 0, 0, 0, 0, 0, false, CAM_POS_NONE, CAM_POS_NONE, 0, 0 };
auto_init_mutex(camMutex);

// Простой фильтр координат поля: экспоненциальное сглаживание по кадрам камеры
// (доля нового кадра — FIELD_FILTER_ALPHA). После перерыва, когда положение было неизвестно,
// фильтр начинает заново с нового значения, а не догоняет старое.
void filterFieldPos(float x, float y, bool restart) {
  if (restart) {
    field_x = x;
    field_y = y;
  } else {
    field_x += (x - field_x) * FIELD_FILTER_ALPHA;
    field_y += (y - field_y) * FIELD_FILTER_ALPHA;
  }
}

// true — пакет принят
bool cam_read() {
  uint8_t buffer[CAM_PACKET_LEN];
  size_t bytesRead = Wire1.requestFrom(CAM_ADDR, CAM_PACKET_LEN);

  if (bytesRead == CAM_PACKET_LEN) {
    for (size_t i = 0; i < CAM_PACKET_LEN; i++) {
      buffer[i] = Wire1.read();
    }

    // Распаковка углов (little-endian)
    CamPacket p;
    p.ballDir = buffer[0] | (buffer[1] << 8);
    //p.ballDir += 18;
    p.gate2 = buffer[2] | (buffer[3] << 8);
    p.gate1 = buffer[4] | (buffer[5] << 8);
    p.lineDir = buffer[6] | (buffer[7] << 8);
    p.ballDist = buffer[8] | (buffer[9] << 8);
    p.trig = buffer[10] | (buffer[11] << 8);
    p.posX10 = buffer[12] | (buffer[13] << 8);   // X уже в см поля: камера пересчитывает по зеркалу
    p.posY10 = buffer[14] | (buffer[15] << 8);
    p.fieldAxis = buffer[16] | (buffer[17] << 8);
    static uint32_t seq = 0;
    p.seq = ++seq;

    mutex_enter_blocking(&camMutex);
    camShared = p;
    mutex_exit(&camMutex);
    camStamp = millis();
    return true;
  }
  camErrors++;   // число сбоев за интервал — в строке состояния
  return false;
}

// Ядро 0, начало loop(): копия последнего пакета камеры в рабочие переменные
void camTake() {
  mutex_enter_blocking(&camMutex);
  CamPacket p = camShared;
  mutex_exit(&camMutex);

  ball_dir = p.ballDir;
  gate_dir_2 = p.gate2;
  gate_dir_1 = p.gate1;
  cam_line_dir = p.lineDir;
  ball_dist = p.ballDist;
  trig = p.trig;

  // Фильтр — только по новому кадру: camTake() вызывается чаще, чем приходят пакеты
  static uint32_t lastSeq = 0;
  bool newFrame = p.seq != lastSeq;
  lastSeq = p.seq;

  bool wasValid = field_valid;
  field_valid = p.posX10 != CAM_POS_NONE;
  if (field_valid && newFrame) {
    filterFieldPos(p.posX10 / 10.0f, p.posY10 / 10.0f, !wasValid);
    field_axis = p.fieldAxis;
  }
}
