#pragma once

#include <Arduino.h>

#ifndef LED_BUILTIN
#define LED_BUILTIN 25
#endif

// ---- Pins ----
#define SDA_PIN 26
#define SCL_PIN 27

#define CAN_CS_MOTOR   17
#define CAN_MISO_MOTOR 16
#define CAN_MOSI_MOTOR 19
#define CAN_SCK_MOTOR  18

#define MODE_BTN 28
#define KICK     15
#define BALL_DETECT_PIN 5
#define DRIBBLE_PIN 20

// ---- I2C device addresses (shared Wire1 bus) ----
#define CAM_ADDR  0x12
#define BNO_ADDR  0x29   // BNO055 gyro/IMU
#define LINE_ADDR 0x42   // PicoLineFollower I2C slave (line sensor)

#define BNO_CHIP_ID_REG 0x00

// ---- CAN IDs (motors) ----
#define MOTOR_FL 0x144
#define MOTOR_FR 0x141
#define MOTOR_RL 0x143
#define MOTOR_RR 0x142

// ---- Robot geometry ----
#define ROBOT_RADIUS_MM 200.0f
#define WHEEL_RADIUS_MM 65.0f
#define MAX_MOTOR_SPEED 2000
#define MOVEMENT_SPEED  1600

// ---- Timings ----
#define SERIAL_INTERVAL         200
#define SENSOR_TIMEOUT          500
#define DEBOUNCE_MS             50
#define KICK_COOLDOWN_MS        3500
#define KICK_PULSE_MS           25
#define MOTOR_DATA_SEND_TIMEOUT 10

// Разгон моторов: сколько единиц скорости набирается за 1 секунду.
// Линейная рампа (постоянное ускорение) -> график speed(t) прямая линия,
// не мгновенный скачок (90°) и не пологая кривая. Чем больше значение,
// тем круче наклон. При этой константе разгон с 0 до MAX_MOTOR_SPEED
// занимает MAX_MOTOR_SPEED / MOTOR_ACCEL_PER_SEC секунд (~0.25с сейчас).
#define MOTOR_ACCEL_PER_SEC 8000.0f

// Резкое торможение/переход на новую скорость — например, когда датчик
// линии сработал и нужно оттолкнуться, а не плавно затормозить обычной
// рампой. В несколько раз круче MOTOR_ACCEL_PER_SEC.
#define LINE_BRAKE_ACCEL_PER_SEC (MOTOR_ACCEL_PER_SEC * 4.0f)

// ---- CAN motor commands ----
#define CMD_MOTOR_ON   0x88
#define CMD_MOTOR_STOP 0x81
#define CMD_SPEED      0xA2

// ---- Motor speed sync по энкодерам (согласование реального пройденного пути колёс) ----
#define ENCODER_TICKS_PER_REV 16384.0f  // однооборотный энкодер, 14 бит
#define SYNC_INTERVAL_MS      100       // окно, за которое сравниваем тики между колёсами
#define SYNC_MIN_TARGET       100       // ниже этого |target| колесо в синхронизации не участвует
#define SYNC_GAIN             0.3f      // доля ошибки (в dps), забираемая в trim за шаг
#define SYNC_TRIM_MAX         400.0f    // ограничение поправки, dps

// ---- Attack tuning ----
#define ACCEL_DISTANCE_SPEED_MULTIPLIER 20
#define ATTACK_SPEED_MIN 1300
#define ATTACK_SPEED_MAX 3500
#define STALL_PROGRESS_DELTA 5
#define STALL_TIMEOUT_MS 300
#define STALL_BOOST_SPEED 800

// ---- Defense tuning ----
#define BALL_FRONT_ANGLE    15
#define BALL_FRONT_HOLD_MS  500
#define DEF_HOME_DIRECTION  180
