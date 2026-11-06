#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <mcp_can.h>
#include <Servo.h>
#include "config.h"
#include "structs.h"
#include "imu.h"

extern MCP_CAN motorCAN;
extern Servo dribble;
extern Motor motors[4];
extern CamData camData;
extern LineData lineData;
extern BNOData imuData;
extern volatile bool kickFlag;

// ============================================================================
// Dribbler
// ============================================================================

inline void dribbleInit() {
    pinMode(DRIBBLE_PIN, OUTPUT);
    digitalWrite(DRIBBLE_PIN, HIGH);
    dribble.attach(DRIBBLE_PIN);
    delay(2000);
    dribble.writeMicroseconds(2000);
    delay(2000);
    dribble.writeMicroseconds(1000);
    delay(2000);
}

// flag=false сбрасывает отсчёт разгона (например, когда мяч не захвачен);
// flag=true раскручивает дриблер плавно в течение ~512 мс.
inline void dribbleWork(bool flag) {
    static uint32_t t = millis();
    if (!flag) {
        t = millis();
        return;
    }
    dribble.writeMicroseconds(1080 + min<uint32_t>(512, millis() - t));
}

// ============================================================================
// Camera (I2C, CAM_ADDR)
// ============================================================================

inline bool cam_read() {
    uint8_t buf[12];
    if (Wire1.requestFrom(CAM_ADDR, 12) != 12) {
        Serial.println("не удалось считать камеру");
        return false;
    }
    for (uint8_t i = 0; i < 12; i++) buf[i] = Wire1.read();

    camData.ball_dir     = (buf[0] | (buf[1] << 8));
    camData.gate_dir_1   = buf[2] | (buf[3] << 8);
    camData.gate_dir_2   = buf[4] | (buf[5] << 8);
    camData.cam_line_dir = buf[6] | (buf[7] << 8);
    camData.ball_dist    = buf[8] | (buf[9] << 8);
    camData.trig         = buf[10] | (buf[11] << 8);
    return true;
}

inline void printCamData() {
    Serial.print("CAM   ball_dir=");   Serial.print(camData.ball_dir);
    Serial.print(" gate1=");           Serial.print(camData.gate_dir_1);
    Serial.print(" gate2=");           Serial.print(camData.gate_dir_2);
    Serial.print(" line_dir=");        Serial.print(camData.cam_line_dir);
    Serial.print(" ball_dist=");       Serial.print(camData.ball_dist);
    Serial.print(" trig=");            Serial.println(camData.trig);
}

inline bool ball_detect() {
    return digitalRead(BALL_DETECT_PIN);
}

// ============================================================================
// Line sensor (I2C, LINE_ADDR) — reads 3 bytes: bool line, int16 angle.
// Same protocol as the PicoLineFollower I2C slave.
// ============================================================================

inline bool readLine() {
    uint8_t buf[3];
    if (Wire1.requestFrom(LINE_ADDR, 3) != 3) {
        Serial.println("не удалось считать линию");
        return false;
    }
    for (uint8_t i = 0; i < 3; i++) buf[i] = Wire1.read();
    lineData.detected = buf[0];
    lineData.angle = (int16_t)(buf[1] | (buf[2] << 8));
    lineData.angle -= 180;
    return true;
}

inline void printLineData() {
    Serial.print("LINE  detected="); Serial.print(lineData.detected);
    Serial.print(" angle=");         Serial.println(lineData.angle);
}

// Prints the latest reading from every sensor. Rate-limited by SERIAL_INTERVAL.
inline void printAllSensors() {
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint < SERIAL_INTERVAL) return;
    lastPrint = millis();

    printCamData();
    printLineData();
    printBNOData(imuData);
    Serial.println();
}

// ============================================================================
// Motor control (CAN)
// ============================================================================

inline bool sendMotor(uint8_t i, uint8_t cmd, uint8_t *data = nullptr) {
    if (i >= 4) return false;

    uint8_t d[8] = { cmd, 0, 0, 0, 0, 0, 0, 0 };
    if (data) memcpy(&d[4], data, 4);

    uint32_t ids[4] = { MOTOR_RL, MOTOR_FL, MOTOR_FR, MOTOR_RR };
    if (motorCAN.sendMsgBuf(ids[i], 0, 8, d) != CAN_OK) {
        return false;
    }
    motors[i].lastCmd = millis();
    return true;
}

// Плавно подгоняет ПОЛНУЮ желаемую скорость мотора i (поступательная часть
// linearDesired + вращение w вместе) к цели с постоянным ускорением
// MOTOR_ACCEL_PER_SEC (или LINE_BRAKE_ACCEL_PER_SEC, если fastRamp=true —
// резкое торможение/переход, например при съезде на линию). Раньше w
// добавлялся поверх БЕЗ рампы (доворот на объект — выход следящего контура,
// казалось что задержка через рампу делает выравнивание вялым), но по факту
// это давало рывок при резкой смене w — теперь ускорение ограничивает и его
// тоже. Вызывается на каждый тик из omniMoveDirection, сама считает время по
// motors[i].lastRamp.
inline void setMotorSpeed(uint8_t i, int32_t linearDesired, int32_t w, bool fastRamp = false) {
    int32_t desired = constrain(linearDesired + w, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);

    uint32_t now = millis();
    uint32_t dt = now - motors[i].lastRamp;
    if (dt > 50) dt = 50; // защита от гигантского шага после паузы/старта
    motors[i].lastRamp = now;

    float accelRate = fastRamp ? LINE_BRAKE_ACCEL_PER_SEC : MOTOR_ACCEL_PER_SEC;
    float maxStep = accelRate * dt / 1000.0f;
    float diff = desired - motors[i].current;
    if (diff > maxStep) diff = maxStep;
    else if (diff < -maxStep) diff = -maxStep;
    motors[i].current += diff;

    int32_t s = constrain((int32_t)lroundf(motors[i].current + motors[i].trim), -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
    if (!motors[i].needsUpdate(s)) return;

    int32_t v = s * 100;
    uint8_t d[4] = { (uint8_t)v, (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24) };
    if (sendMotor(i, CMD_SPEED, d)) motors[i].target = s;
}

inline void allMotorsOn() {
    for (int i = 0; i < 4; i++) sendMotor(i, CMD_MOTOR_ON);
}

inline void allMotorsStop() {
    for (int i = 0; i < 4; i++) sendMotor(i, CMD_MOTOR_STOP);
}

inline void omniMoveDirection(int32_t wheelSpeed, float direction, int32_t w, bool fastRamp = false) {
    float directionRad = direction * DEG_TO_RAD;
    float Vx = wheelSpeed * cos(directionRad);
    float Vy = wheelSpeed * sin(directionRad);

    static constexpr float SIN_A[4] = { -0.8660254f, 0.8660254f, 0.8660254f, -0.8660254f };
    static constexpr float COS_A[4] = { 0.5f, 0.5f, -0.5f, -0.5f };

    for (int i = 0; i < 4; i++) {
        int32_t linear = ((Vx * COS_A[i]) + Vy * SIN_A[i]) / 2;
        setMotorSpeed(i, linear, w, fastRamp);
    }
}

// ============================================================================
// Encoders (читаются из ответных CAN-кадров моторов на команду CMD_SPEED —
// байты 6-7 ответа = однооборотное значение энкодера, 0..16383).
// readMotorEncoders() вычитывает всё, что накопилось в приёмном буфере CAN,
// и копит знаковую разницу (с учётом переполнения 14 бит) в motors[i].encoderAccum.
// printAndResetEncoders() раз в 10 секунд печатает накопленное и обнуляет счётчики.
// ============================================================================

inline void readMotorEncoders() {
    uint32_t canId;
    uint8_t len;
    uint8_t buf[8];

    while (motorCAN.checkReceive() == CAN_MSGAVAIL) {
        if (motorCAN.readMsgBuf(&canId, &len, buf) != CAN_OK || len < 8) continue;

        int idx;
        switch (canId) {
            case MOTOR_RL: idx = 0; break;
            case MOTOR_FL: idx = 1; break;
            case MOTOR_FR: idx = 2; break;
            case MOTOR_RR: idx = 3; break;
            default: continue;
        }

        int16_t enc = (int16_t)(buf[6] | (buf[7] << 8));
        Motor &m = motors[idx];
        if (m.hasEncoder) {
            int32_t delta = enc - m.lastEncoder;
            if (delta > 8192) delta -= 16384;
            else if (delta < -8192) delta += 16384;
            m.encoderAccum += delta;
            m.syncAccum += delta;
        }
        m.lastEncoder = enc;
        m.hasEncoder = true;
    }
}

// Синхронизация реальных скоростей колёс по энкодерам: даже с closed-loop
// speed на самом моторе колёса могут реально проезжать разное расстояние за
// одно и то же время (трение, развесовка, разброс редуктора). Раз в
// SYNC_INTERVAL_MS сравниваем "прогресс в своём направлении" каждого активного
// колеса (тики энкодера, умноженные на знак его же target — т.к. соседние
// колёса могут иметь противоположный знак команды при одинаковом реальном
// движении, зеркальная установка моторов) со средним по всем активным, и
// небольшим шагом подстраиваем trim — персональную поправку к команде мотора
// (dps, прибавляется в setMotorSpeed, знак поправки тоже приводится к знаку
// target). Колёса с |target| < SYNC_MIN_TARGET (стоят или почти стоят) в
// сравнении не участвуют — там разница в тиках это шум.
inline void syncMotorsByEncoder() {
    static uint32_t lastSync = 0;
    uint32_t now = millis();
    if (now - lastSync < SYNC_INTERVAL_MS) return;
    uint32_t dt = now - lastSync;
    lastSync = now;

    float progress[4] = {0, 0, 0, 0}; // тики в направлении собственной команды
    float sum = 0;
    int active = 0;
    for (int i = 0; i < 4; i++) {
        if (abs(motors[i].target) < SYNC_MIN_TARGET) continue;
        float sign = motors[i].target > 0 ? 1.0f : -1.0f;
        progress[i] = motors[i].syncAccum * sign;
        sum += progress[i];
        active++;
    }

    // тики/сек -> dps: dps = ticks/dt(s) / ENCODER_TICKS_PER_REV * 360
    float ticksToDps = (dt > 0) ? (360.0f / ENCODER_TICKS_PER_REV / (dt / 1000.0f)) : 0;

    if (active >= 2) {
        float avg = (float)sum / active;
        for (int i = 0; i < 4; i++) {
            if (abs(motors[i].target) < SYNC_MIN_TARGET) continue;
            float sign = motors[i].target > 0 ? 1.0f : -1.0f;
            float errProgress = avg - progress[i]; // >0 — колесо отстаёт от среднего
            float errDps = errProgress * ticksToDps;
            // отстающему колесу нужно "прибавить газу" в его же направлении (sign)
            motors[i].trim = constrain(motors[i].trim + sign * errDps * SYNC_GAIN, -SYNC_TRIM_MAX, SYNC_TRIM_MAX);
        }
    }

    for (int i = 0; i < 4; i++) motors[i].syncAccum = 0;
}

inline void printAndResetEncoders() {
    static uint32_t lastPrint = 0;
    uint32_t now = millis();
    if (now - lastPrint < 10000) return;
    lastPrint = now;

    char buf[150];
    sprintf(buf, "encoders(10s) RL:%ld FL:%ld FR:%ld RR:%ld | trim RL:%d FL:%d FR:%d RR:%d",
            (long)motors[0].encoderAccum, (long)motors[1].encoderAccum,
            (long)motors[2].encoderAccum, (long)motors[3].encoderAccum,
            (int)motors[0].trim, (int)motors[1].trim,
            (int)motors[2].trim, (int)motors[3].trim);
    Serial.println(buf);

    for (int i = 0; i < 4; i++) motors[i].encoderAccum = 0;
}

// ============================================================================
// Math helpers
// ============================================================================

inline int getBallZone(float angle) {
    return constrain((int)(angle / 30), 0, 11);
}

inline float getZoneSpeed(int zone) {
    const float zoneSpeeds[12] = {
        0.25f, 0.4f, 0.45f, 0.55f, 0.75f, 1.0f,
        1.0f, 0.75f, 0.55f, 0.45f, 0.4f, 0.25f
    };
    return zoneSpeeds[zone];
}

inline int VectorSum(int alfa1, int alfa2, int w1, int w2) {
    double x = w1 * cos(alfa1 * DEG_TO_RAD) + w2 * cos(alfa2 * DEG_TO_RAD);
    double y = w1 * sin(alfa1 * DEG_TO_RAD) + w2 * sin(alfa2 * DEG_TO_RAD);
    return (int)round(atan2(y, x) * RAD_TO_DEG);
}

inline int accel_via_distance() {
    return ACCEL_DISTANCE_SPEED_MULTIPLIER * camData.ball_dist;
}

// Скорость объезда мяча: чем дальше мяч по углу от центра, тем быстрее едем.
inline int attackSpeed(int dir) {
    int a = constrain(abs(dir), 0, 180);
    return map(a, 0, 180, ATTACK_SPEED_MIN, ATTACK_SPEED_MAX);
}

// ============================================================================
// Обход мяча по дуге (когда ball_dist <= 20): едем не прямо на мяч, а по
// касательной, чтобы обогнуть его. Раньше сторона обхода считалась как
// ball_dir / abs(ball_dir) — знак мгновенно переключался при переходе
// ball_dir через ноль (или границу мёртвой зоны ±10°), из-за чего d
// скачком менялся почти на 180° — дугу штормило. Ниже — версия с
// гистерезисом (сторона не меняется от шума камеры возле центра) и
// плавным нарастанием бокового смещения вместо жёсткого шага 0→90°.
inline int arcSide(int ball_dir) {
    static int side = 1;
    const int HYSTERESIS_DEG = 5;
    if (ball_dir > HYSTERESIS_DEG) side = 1;
    else if (ball_dir < -HYSTERESIS_DEG) side = -1;
    return side;
}

inline int arcOffsetDeg(int ball_dir) {
    // 0° на границе мёртвой зоны (|ball_dir| ~ 10°), плавно растёт до 90°
    // к |ball_dir| ~ 40° и дальше держится на 90° — без скачка на стыке.
    int mag = constrain(abs(ball_dir) - 10, 0, 30);
    return map(mag, 0, 30, 0, 90);
}

inline int arcDirection(int ball_dir) {
    return ball_dir + arcOffsetDeg(ball_dir) * arcSide(ball_dir);
}

// "Застряли": мяч почти по центру, но дистанция до него не уменьшается
// дольше STALL_TIMEOUT_MS — форсируем скорость.
int16_t stall_best_dist = 32767;
uint32_t stall_since = 0;

inline int stallBoost() {
    if (camData.ball_dist < stall_best_dist - STALL_PROGRESS_DELTA) {
        stall_best_dist = camData.ball_dist;
        stall_since = millis();
        return 0;
    }
    if (millis() - stall_since > STALL_TIMEOUT_MS) return STALL_BOOST_SPEED;
    return 0;
}

inline void resetStall() {
    stall_best_dist = 32767;
    stall_since = millis();
}

// ============================================================================
// Kicker
// ============================================================================

inline void armKick() {
    kickFlag = true;
}

inline void kicker() {
    if (!kickFlag) return;
    digitalWrite(KICK, 1);
    delay(KICK_PULSE_MS);
    digitalWrite(KICK, 0);
    Serial.println("kick");
    kickFlag = false;
}

// ============================================================================
// Modes
// ============================================================================

inline void DefenseMode() {
    if (camData.cam_line_dir < 0) {
        omniMoveDirection(camData.ball_dir * 25, 90, camData.gate_dir_1 * 10);
    } else {
        omniMoveDirection(2000, camData.cam_line_dir, camData.gate_dir_1 * 10);
    }
}

inline void AtackMode() {
    if (camData.cam_line_dir < 0) {
        if (!camData.trig) {
            dribble.writeMicroseconds(0);
            delay(10);
            digitalWrite(LED_BUILTIN, 0);
            if (abs(camData.ball_dir) < 10) {
                omniMoveDirection(accel_via_distance() + stallBoost(), camData.ball_dir, camData.gate_dir_1 * 10);
            } else if (camData.ball_dist <= 20) {
                resetStall();
                int side = camData.ball_dir / abs(camData.ball_dir);
                omniMoveDirection(attackSpeed(camData.ball_dir) + accel_via_distance(),
                                   (abs(camData.ball_dir) + 90) * side, camData.gate_dir_1 * 10);
            } else {
                omniMoveDirection(2000, camData.ball_dir, camData.gate_dir_1 * 10);
            }
        } else {
            dribble.writeMicroseconds(1200);
            delay(10);
            digitalWrite(LED_BUILTIN, 1);
            if (abs(camData.gate_dir_1) < 15) {
                kicker();
            }
            omniMoveDirection(3600, 0, camData.gate_dir_1 * 10);
        }
    } else {
        dribble.writeMicroseconds(0);
        delay(10);
        omniMoveDirection(2500, camData.cam_line_dir, camData.gate_dir_1 * 15);
    }
}

void setDribbleSpeed(int speed) {
    dribble.writeMicroseconds(speed);
    delay(10);
}
void AtackMode2() {
    float k = 1;
    if (camData.cam_line_dir < 0) {
        k = 0.1;
    }
    if (lineData.detected) {
        if (!camData.trig) {
            setDribbleSpeed(0);
            digitalWrite(LED_BUILTIN, 0);
            if (abs(camData.ball_dir) < 10) {
                omniMoveDirection(accel_via_distance() + stallBoost(), camData.ball_dir, camData.gate_dir_1 * 10);
            } else if (camData.ball_dist <= 20) {
                resetStall();
                int side = camData.ball_dir / abs(camData.ball_dir);
                omniMoveDirection(attackSpeed(camData.ball_dir) + accel_via_distance() * k,
                                   (abs(camData.ball_dir) + 90) * side, camData.gate_dir_1 * 10);
            } else {
                omniMoveDirection(2000 * k, camData.ball_dir, camData.gate_dir_1 * 10);
            }
        } else {
            setDribbleSpeed(1200);
            digitalWrite(LED_BUILTIN, 1);
            if (abs(camData.gate_dir_1) < 15) {
                kicker();
            }
            omniMoveDirection(3600 * k, 0, camData.gate_dir_1 * 10);
        }
    } else {
        setDribbleSpeed(0);
        omniMoveDirection(2500 * k, lineData.angle, camData.gate_dir_1 * 15);
    }
}

bool flag_ob = 0;

void attack_3() {
    float koeff = 1;
    int v = 2500;
    int d = camData.ball_dir-5;
    int w = 0;

    // ------------- v (correct speed) -------

    if (abs(camData.ball_dir) < 20) {
        v = accel_via_distance() + stallBoost();
    }
    else {
        if (camData.trig or abs(camData.ball_dir) < 10) {
            v = 2500;
        }
        else {
            if (camData.ball_dist <= 20) {
                resetStall();
                v = attackSpeed(camData.ball_dir) + accel_via_distance();
            }
            else {
                v = 2500;
            }
        }
    }

    // ------------- k (koef for v) ---------------

    if (camData.cam_line_dir != -1 or lineData.detected) {
        koeff = 0.2;
    }
    else {
        if (0) { /* timer in past */
            koeff = 2;
        }
        else {
            if (camData.gate_dir_1 < d  && d < camData.gate_dir_2) {
                koeff = 1.2;
            }
        }
    }

    // ----------------- d (direction) ------------------

    if (camData.ball_dist <= 20) {
        flag_ob = 1;
    }
    else if (camData.ball_dist >= 30 or lineData.detected or abs(camData.ball_dir) < 10) {
        flag_ob = 0;
    }
    if (lineData.detected) {
        d = lineData.angle;
    }
    else {
        if (camData.trig or abs(camData.ball_dir) < 10) {
            d = 0;
        }
        else {
            if (flag_ob) {
                //d = arcDirection(camData.ball_dir);
                int side = camData.ball_dir / abs(camData.ball_dir);
                d = (abs(camData.ball_dir) + 90) * side;
            }
            else {
                d = camData.ball_dir;
            }
        }
    }

    // ---------------- w (rotate speed) ---------------

    w = camData.gate_dir_1 * 12.5;

    // -------------------------------------------------

    v *= koeff;
    char buf[64];
    sprintf(buf, "speed: %d | direction: %d | rotate_speed: %d | dist %d", v, d, w,  camData.ball_dist);
    Serial.println(buf);
    if (camData.trig) {
        kicker();
    }
    if (abs(camData.ball_dir) < 30) {
        dribble.writeMicroseconds(1100);
        delay(10);
    }
    else {
        dribble.writeMicroseconds(1100);
        delay(10);
    }
    omniMoveDirection(v, d, w);
}

// ---- attack_4: v/d/w/koeff разнесены по отдельным функциям ----

// Целевая поступательная скорость (без koeff).
inline int attack4_speed() {
    if (abs(camData.ball_dir) < 20) return accel_via_distance() + stallBoost();
    if (camData.ball_dist <= 20) {
        resetStall();
        return attackSpeed(camData.ball_dir) + accel_via_distance();
    }
    return 2500;
}

// Направление движения. На линии — от неё; мяч по центру — прямо на него;
// мяч рядом (ball_dist<=20) — стабилизированная дуга обхода (arcDirection);
// иначе — прямо на мяч.
inline int attack4_direction() {
    if (lineData.detected) return lineData.angle;
    if (abs(camData.ball_dir) < 10) return 0;
    if (camData.ball_dist <= 20) return arcDirection(camData.ball_dir);
    return camData.ball_dir;
}

// Множитель скорости: резко снижаем на линии/вне поля видимости линии
// разметки, слегка форсируем, когда мяч уже между воротами (можно бить).
inline float attack4_koeff(int d) {
    if (camData.cam_line_dir != -1 or lineData.detected) return 0.6f;
    if (camData.gate_dir_1 < d && d < camData.gate_dir_2) return 1.2f;
    return 1.0f;
}

// Угловая скорость доворота на ворота (теперь тоже под рампой — см. setMotorSpeed).
inline int attack4_rotate() {
    return camData.gate_dir_1 * 12.5f;
}

// Как attack_3, но при сработавшем датчике линии (lineData.detected) не
// плавно замедляется обычной рампой, а тормозит/перестраивается резко
// (LINE_BRAKE_ACCEL_PER_SEC вместо MOTOR_ACCEL_PER_SEC), чтобы оттолкнуться
// от линии, а не сползать на неё по инерции.
void attack_4() {
    int d = attack4_direction();      // считаем первым: koeff зависит от d
    int v = (int)(attack4_speed() * attack4_koeff(d));
    int w = attack4_rotate();

    char buf[100];
    sprintf(buf, "attack_4 speed: %d | direction: %d | rotate_speed: %d | dist %d | line %d",
            v, d, w, camData.ball_dist, lineData.detected);
    Serial.println(buf);

    if (camData.trig) {
        kicker();
    }
    dribble.writeMicroseconds(1100);
    delay(10);

    // На линии — резкое торможение/отталкивание (fastRamp=true), иначе —
    // обычная плавная рампа, как в attack_3.
    omniMoveDirection(v, d, w, /*fastRamp=*/ lineData.detected);
}

unsigned long tmr_counterattack = 0;
bool counterattack = false;

void defense_2() {
    float k = 1;
    int v = 2500;
    int d = camData.ball_dir;;
    int w = 0;

    bool attack = false;

    // ------------- v (correct speed) ------------

    // ------------- k (koef for v) ---------------

    if (lineData.detected or camData.cam_line_dir < 0) {
        k = 0.75;
    }
    else {
        if (abs(camData.ball_dir) < 20) {
            k = 1.75;
        }
        else {
            k = 1;
        }
    }

    // ------------- d (direction) ----------------

    if (lineData.detected) {
        d = VectorSum(d, lineData.angle, 0.5, 1);
        tmr_counterattack = millis();
    }
    else {
        if (abs(camData.ball_dir) < 20) {
            if (millis() - tmr_counterattack > 500) {
                d = 0;
                attack = true;
            }
        }
        else {

            if (camData.gate_dir_1 < 0) {
                d = -100;
            }
            else {
                d = 100;
            }
            tmr_counterattack = millis();
        }
    }
    // ------------- w (rotate speed) -------------

    if (attack) {
        w = 1.25 * camData.gate_dir_1;
    }
    else {
        const double course = -1.0 * (imuData.euler[0] - 360.0 * (imuData.euler[0] > 180));
        w = static_cast<int>(course) * 1.25;
    }


    omniMoveDirection(v, d, w);
}