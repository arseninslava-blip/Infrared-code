#pragma once

#include <Arduino.h>
#include "config.h"

struct Motor {
    int32_t target;      // последняя ramped-скорость, реально отправленная по CAN
    float current = 0;   // текущая точка разгона (плавно догоняет desired)
    uint32_t lastCmd;
    uint32_t lastRamp = 0;
    int32_t encoderAccum = 0;  // накопленная разница энкодера за текущее 10с окно (для печати)
    int32_t syncAccum = 0;     // накопленная разница энкодера за текущее окно синхронизации
    int16_t lastEncoder = 0;   // последнее сырое значение энкодера (0..16383)
    bool hasEncoder = false;   // false до первого полученного кадра
    float trim = 0;            // поправка к команде (dps) — подстраивается syncMotorsByEncoder()
    bool needsUpdate(int32_t n) { return n != target || millis() - lastCmd > MOTOR_DATA_SEND_TIMEOUT; }
};

struct CamData {
    int16_t ball_dir;
    int16_t gate_dir_1;
    int16_t gate_dir_2;
    int16_t cam_line_dir;
    int16_t ball_dist;
    bool trig;
};

struct LineData {
    bool detected;
    int16_t angle;
};

// BNO055: [0]=X, [1]=Y, [2]=Z unless noted otherwise.
struct BNOData {
    float euler[3];   // Heading, Roll, Pitch (deg)
    float quat[4];    // W, X, Y, Z
    float linAcc[3];  // m/s^2, gravity removed
    float grav[3];    // m/s^2
    float acc[3];     // m/s^2, total
    float gyro[3];    // deg/s
    float mag[3];     // uT
    int8_t temp;      // deg C
    uint8_t cal[4];   // System, Gyro, Accel, Mag (0..3)
};
