// ============================================================================
//  Обработка датчиков
// ============================================================================

bool trig;
uint32_t lastCamDebugPrint = 0;

static inline bool isFresh(uint32_t stamp, uint32_t timeoutMs) {
  return stamp != 0 && millis() - stamp <= timeoutMs;
}
bool camFresh() { return isFresh(camStamp, SENSOR_TIMEOUT); }
bool imuFresh() { return isFresh(imuStamp, SENSOR_TIMEOUT); }

// Нулевой курс ("вперёд"). Ставится при старте кнопкой (IMU_ZERO_ON_START), только если IMU отвечает.
float imuZero = 0;
void zeroHeading() {
  if (imuFresh()) imuZero = imuData.euler[0];
}

void printCamData() {
  if (millis() - lastCamDebugPrint < SERIAL_INTERVAL) return;
  lastCamDebugPrint = millis();

  //Serial.print("ball_dir: ");      Serial.print(ball_dir);
  //Serial.print("  gate_dir_1: ");  Serial.print(gate_dir_1);
  //Serial.print("  gate_dir_2: ");  Serial.print(gate_dir_2);
  Serial.print("  cam_line_dir: ");Serial.println(cam_line_dir);
  //Serial.print("  ball_dist: ");   Serial.print(ball_dist);
  //Serial.print("  trig: ");        Serial.println(trig);
}

static uint8_t camPrev[4] = { 0, 0, 0, 0 };  // прошлые ball_dir/gate_dir_2 (сырые байты) - для поиска нового кадра

void cam_read() {
  uint8_t buffer[12];
  size_t bytesRead = Wire1.requestFrom(CAM_ADDR, 12);

  if (bytesRead == 12) {
    for (size_t i = 0; i < 12; i++) {
      buffer[i] = Wire1.read();
    }

    // У камеры нет счётчика кадров: новым считаем пакет, где поменялись ball_dir или gate_dir_2.
    bool newFrame = memcmp(buffer, camPrev, 4) != 0;

    camSeqLock++;                                 // нечётный: идёт запись
    __sync_synchronize();
    // Распаковка углов (little-endian)
    ball_dir = buffer[0] | (buffer[1] << 8);
    //ball_dir += 18;
    gate_dir_2 = buffer[2] | (buffer[3] << 8);
    gate_dir_1 = buffer[4] | (buffer[5] << 8);
    cam_line_dir = buffer[6] | (buffer[7] << 8);
    ball_dist = buffer[8] | (buffer[9] << 8);
    trig = buffer[10] | (buffer[11] << 8);
    if (newFrame) {
      memcpy(camPrev, buffer, 4);
      camFrameMs = millis();
      camFrameCount++;
    }
    __sync_synchronize();
    camSeqLock++;                                 // чётный: запись закончена
    camStamp = millis();

    printCamData();
  } else {
    static uint32_t lastCamErr = 0;
    if (millis() - lastCamErr >= SERIAL_INTERVAL) {
      lastCamErr = millis();
      Serial.println("CAM: error read");
    }
  }
}

// Согласованная копия полей камеры, нужных DefenseMode (ядро 0). Без неё ball_dir и gate_dir_2
// могли быть взяты из разных пакетов.
struct CamSnap {
  int16_t ball_dir, gate_dir_1, gate_dir_2, cam_line_dir, ball_dist;
  bool trig;
  uint32_t frameCount, frameMs;
};

CamSnap camSnapshot() {
  CamSnap s;
  for (uint8_t tries = 0; tries < 5; tries++) {   // ограничение на случай зависшего ядра 1 посреди записи
    uint32_t a = camSeqLock;
    __sync_synchronize();
    s.ball_dir = ball_dir;
    s.gate_dir_1 = gate_dir_1;
    s.ball_dist = ball_dist;
    s.trig = trig;
    s.gate_dir_2 = gate_dir_2;
    s.cam_line_dir = cam_line_dir;
    s.frameCount = camFrameCount;
    s.frameMs = camFrameMs;
    __sync_synchronize();
    if (!(a & 1) && a == camSeqLock) break;
  }
  return s;
}

volatile uint32_t lineFails = 0;                  // сколько раз чтение датчика линии по I2C не удалось (диагностика)
static uint32_t lineLastOnMs = 0;                 // когда датчик последний раз видел линию
static int16_t lineDirLast = 0;                   // направление "от линии" в тот момент

bool readLine() {
  uint8_t buf[3];
  if (Wire1.requestFrom(LINE_ADDR, 3) != 3) {
    lineFails++;
    // Нет ответа: не держим старое "линия видна" дольше LINE_TIMEOUT,
    // иначе робот будет вечно ехать "от линии".
    if (millis() - lineStamp > LINE_TIMEOUT) line_be = false;
    return false;
  }
  for (uint8_t i = 0; i < 3; i++) buf[i] = Wire1.read();
  uint32_t now = millis();
  int16_t dir = (int16_t)(buf[1] | (buf[2] << 8)) - 180;
  bool on = buf[0];
  if (on) { lineLastOnMs = now; lineDirLast = dir; }
  // Короткое касание линии (несколько мс на скорости) ядро 0 с периодом 5 мс могло пропустить между опросами -
  // защитник отталкивался "через раз". Держим "линия есть" ещё 30 мс после последнего касания и в это время
  // отдаём направление последнего касания (сам датчик вне линии направление уже не отдаёт).
  bool hold = !on && lineLastOnMs != 0 && (now - lineLastOnMs) < 30;
  // line_dir - одним присваиванием и раньше line_be: ядро 0 читает их без блокировок
  // и не должно увидеть промежуточное значение (до вычитания 180).
  line_dir = hold ? lineDirLast : dir;
  line_be = on || hold;
  lineStamp = now;
  return true;
}

void printLineData() {
  //Serial.print("LINE  detected="); Serial.print(line_be);
  //Serial.print(" angle=");         Serial.println(line_dir);
}

// ============================================================================
//  Управление моторами
// ============================================================================

struct Motor {
  int32_t target;
  uint32_t lastCmd;
  float current = 0;  // текущая точка разгона (плавно догоняет desired)
  uint32_t lastRamp = 0;
  int16_t actualSpeed = 0;  // реальная скорость мотора (dps) из ответного CAN-кадра - для одометрии
  bool needsUpdate(int32_t n) {
    return n != target || millis() - lastCmd > MOTOR_DATA_SEND_TIMEOUT;
  }
} motors[4];

bool motorAccelEnabled = true;

bool sendMotor(uint8_t i, uint8_t cmd, uint8_t *data = nullptr) {
  if (i >= 4) return false;

  uint8_t d[8] = { cmd, 0, 0, 0, 0, 0, 0, 0 };
  if (data) memcpy(&d[4], data, 4);

  uint32_t ids[4] = { MOTOR_RL, MOTOR_FL, MOTOR_FR, MOTOR_RR };

  if (motorCAN.sendMsgBuf(ids[i], 0, 8, d) == CAN_OK) {
    motors[i].lastCmd = millis();
    return true;
  }
  return false;
}

void setMotorSpeed(uint8_t i, int32_t desired) {
  desired = constrain(desired, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);

  uint32_t now = millis();
  uint32_t dt = now - motors[i].lastRamp;
  if (dt > 50) dt = 50;  // защита от гигантского шага после паузы/старта
  motors[i].lastRamp = now;

  if (!motorAccelEnabled) {
    motors[i].current = desired;
  } else {
    float maxStep = MOTOR_ACCEL_PER_SEC * dt / 1000.0f;
    float diff = desired - motors[i].current;
    if (diff > maxStep) diff = maxStep;
    else if (diff < -maxStep) diff = -maxStep;
    motors[i].current += diff;
  }

  int32_t s = (int32_t)lroundf(motors[i].current);
  if (!motors[i].needsUpdate(s)) return;

  int32_t v = s * 100;
  uint8_t d[4] = { (uint8_t)v, (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24) };
  if (sendMotor(i, CMD_SPEED, d)) motors[i].target = s;
}
uint32_t canRxAny = 0, canRxMotor = 0;

void readMotorFeedback() {
  uint32_t canId;
  uint8_t len, buf[8];
  while (motorCAN.checkReceive() == CAN_MSGAVAIL) {
    canRxAny++;
    if (motorCAN.readMsgBuf(&canId, &len, buf) != CAN_OK || len < 8) continue;
    int idx;
    switch (canId) {
      case MOTOR_RL: idx = 0; break;
      case MOTOR_FL: idx = 1; break;
      case MOTOR_FR: idx = 2; break;
      case MOTOR_RR: idx = 3; break;
      default: continue;
    }
    canRxMotor++;
    motors[idx].actualSpeed = (int16_t)(buf[4] | (buf[5] << 8));
  }

  static uint32_t lastCanDebug = 0;
  if (millis() - lastCanDebug >= 1000) {
    lastCanDebug = millis();
    char buf2[48];
    sprintf(buf2, "CANFB any=%lu motor=%lu", (unsigned long)canRxAny, (unsigned long)canRxMotor);
    Serial.println(buf2);
  }
}

void allMotorsOn() {
  for (int i = 0; i < 4; i++) sendMotor(i, CMD_MOTOR_ON);
}

void allMotorsStop() {
  for (int i = 0; i < 4; i++) sendMotor(i, CMD_MOTOR_STOP);
}

// instant=true - разгон (рампа) выключен на этот вызов, скорость выставляется
// сразу (например, отталкивание от линии - нужен резкий рывок, а не плавный
// набор). По умолчанию false - обычная плавная рампа.
// Здесь вектор скорости задаётся компонентами (Vx, Vy): так вызывающий код может сам
// ограничивать разгон на уровне вектора, а не по колёсам (DefenseMode).
void omniMoveXY(float Vx, float Vy, int32_t w, bool instant = false) {
  motorAccelEnabled = !instant;

  int32_t motor_dps[4];

  static constexpr float SIN_A[4] = { -0.5f, 0.5f, 0.5f, -0.5f };
  static constexpr float COS_A[4] = { 0.8660254f, 0.8660254f, -0.8660254f, -0.8660254f };

  for (int i = 0; i < 4; i++) {
    motor_dps[i] = ((Vx * COS_A[i]) + Vy * SIN_A[i]) / 2 + w;
    setMotorSpeed(i, motor_dps[i]);
  }
}

void omniMoveDirection(int32_t wheelSpeed, float direction, int32_t w, bool instant = false) {
  float directionRad = (direction)*DEG_TO_RAD;
  omniMoveXY(wheelSpeed * cos(directionRad), wheelSpeed * sin(directionRad), w, instant);
}

// ============================================================================
//  Математика
// ============================================================================

void computeRobotVelocity(float &vx, float &vy, float &wOut) {
  float s0 = motors[0].actualSpeed, s1 = motors[1].actualSpeed;  // RL, FL
  float s2 = motors[2].actualSpeed, s3 = motors[3].actualSpeed;  // FR, RR

  wOut = (s0 + s1 + s2 + s3) / 4.0f;
  vy = ((s1 - s0) + (s2 - s3)) / 2.0f;
  vx = ((s0 - s3) + (s1 - s2)) / (2.0f * 1.7320508f);
}

// Поворот вектора (x,y) на angleDeg против часовой стрелки.
void rotate2D(float x, float y, float angleDeg, float &outX, float &outY) {
  float r = angleDeg * DEG_TO_RAD;
  float c = cos(r), s = sin(r);
  outX = x * c - y * s;
  outY = x * s + y * c;
}

// Разница углов target-current, приведённая к диапазону (-180, 180].
float angleDiff(float target, float current) {
  float d = target - current;
  while (d > 180) d -= 360;
  while (d <= -180) d += 360;
  return d;
}

// Трекер угловой ошибки (alpha-beta): оценивает положение x (град) и скорость v (град/с)
// по редким измерениям с РЕАЛЬНЫМ интервалом между ними, а между измерениями
// экстраполирует. Углы считаются по кратчайшей дуге (переход через ±180).
struct AngleTracker {
  float x = 0, v = 0;   // оценка и скорость на момент tMs
  uint32_t tMs = 0;
  float g = DEF_TRK_G, h = DEF_TRK_H;   // коэффициенты; для отдельного трекера можно переопределить

  void reset(float meas, uint32_t ms) { x = meas; v = 0; tMs = ms; }

  // changed=false: измерение не изменилось (камера "молчит" или ничего не двигается) -
  // скорость дополнительно гасим, чтобы экстраполяция не тянула оценку мимо цели.
  void update(float meas, uint32_t ms, bool changed) {
    float dt = (float)(int32_t)(ms - tMs) * 1e-3f;
    if (dt < 0.005f) dt = 0.005f;                 // защита от деления на ~0 при дрожании меток времени
    if (dt > 0.1f) dt = 0.1f;
    float pred = angleDiff(x + v * dt, 0);
    float r = angleDiff(meas, pred);
    x = angleDiff(pred + g * r, 0);
    v += h * r / dt;
    if (!changed) v *= 0.5f;
    tMs = ms;
  }

  // Оценка на момент nowMs (не дальше DEF_EXTRAP_MAX_MS от последнего измерения).
  float at(uint32_t nowMs) const {
    int32_t age = (int32_t)(nowMs - tMs);
    if (age < 0) age = 0;
    if (age > DEF_EXTRAP_MAX_MS) age = DEF_EXTRAP_MAX_MS;
    return angleDiff(x + v * age * 1e-3f, 0);
  }
};

int VectorSum(int alfa1, int alfa2, float w1, float w2) {
  double x1 = w1 * cos(alfa1 * DEG_TO_RAD);
  double y1 = w1 * sin(alfa1 * DEG_TO_RAD);
  double x2 = w2 * cos(alfa2 * DEG_TO_RAD);
  double y2 = w2 * sin(alfa2 * DEG_TO_RAD);
  double x3 = x1 + x2;
  double y3 = y1 + y2;
  return (int)round(atan2(y3, x3) * RAD_TO_DEG);
}

int accel_via_distance() {
  return ACCEL_DISTANCE_SPEED_MULTIPLIER * ball_dist;
}

// Скорость объезда мяча: чем дальше мяч по углу от 0 (центра), тем быстрее едем.
int attackSpeed(int dir) {
  int a = constrain(abs(dir), 0, 180);
  return map(a, 0, 180, ATTACK_SPEED_MIN, ATTACK_SPEED_MAX);
}

/*
 "Застряли": мяч почти по центру (|ball_dir| < 15), но дистанция до него
 не уменьшается дольше STALL_TIMEOUT_MS - форсируем скорость, чтобы не
 зависнуть в дребезге на месте.
*/

int16_t stall_best_dist = 32767;
uint32_t stall_since = 0;

int stallBoost() {
  if (ball_dist < stall_best_dist - STALL_PROGRESS_DELTA) {
    stall_best_dist = ball_dist;
    stall_since = millis();
    return 0;
  }
  if (millis() - stall_since > STALL_TIMEOUT_MS) return STALL_BOOST_SPEED;
  return 0;
}

void resetStall() {
  stall_best_dist = 32767;
  stall_since = millis();
}

// ============================================================================
//  Пеналка
// ============================================================================

volatile bool kickFlag = false;

// Вызывается по таймеру Ticker раз в KICK_COOLDOWN_MS - "взводит" пеналку.
void armKick() {
  kickFlag = true;
}

void kicker() {
  if (kickFlag) {
    digitalWrite(KICK, 1);
    delay(KICK_PULSE_MS);
    digitalWrite(KICK, 0);
    //Serial.println("kick");
    kickFlag = false;
  }
}

// ============================================================================
//  Режимы работы
// ============================================================================
// Приводит угол к диапазону (-180, 180]. Работает и для отрицательных a < -180.
int norm180(int a) {
  a %= 360;
  if (a > 180) a -= 360;
  else if (a <= -180) a += 360;
  return a;
}

// Не даёт направлению уйти вперёд больше, чем на maxFwdDeg от чисто бокового хода (+-90 град).
// Назад и вбок не трогает. Для 0 град (строго вперёд) сторону берёт из side.
int limitForward(int target, int side, int maxFwdDeg) {
  int t = norm180(target);
  int minA = 90 - maxFwdDeg;
  if (abs(t) >= minA) return t;
  int s = (t > 0) ? 1 : ((t < 0) ? -1 : side);
  return s * minA;
}

unsigned long tmrDefA = 0;
int degreesForDef = 0, speedForDef = 0;
// Прежняя, более сложная версия защитника (трекер ошибки, упреждение, профиль скорости, смесь линий и т.д.).
void DefenseMode_save002() {
  // ---------- параметры ----------
  const int   SPEED_MAX = DEF_SPEED_MAX, SPEED_MIN = DEF_SPEED_MIN, SPEED_RAMP = DEF_SPEED_RAMP; // скорость растёт с ошибкой
  const int   ALIGN_IN = DEF_ALIGN_IN, ALIGN_OUT = DEF_ALIGN_OUT;   // гистерезис "встал на линию мяч-ворота"
  const int   BEHIND_IN = 170, BEHIND_OUT = 155;  // мяч сзади -> стоим
  const int   SIDE_HOLD = 150;                    // |e| больше: сторону не переключаем
  const float LEAD_S  = 0.12f;                    // тормозим заранее, за LEAD_S сек до цели (0 = выкл)
  const float ANG_RATE = 500.0f;                  // град/с: макс. скорость поворота вектора движения
  // стабилизация курса (ПД): rot = -(K*ошибка + KD*скорость_рысканья); мёртвая зона только у П-части.
  // Со скоростью в контуре K можно поднимать (25-35), не боясь раскачки.
  const float ROT_K = 20.0f, ROT_KD = 1.0f, ROT_MAX = 600.0f, ROT_DEAD = 1.0f;
  // разгон/торможение вектора скорости - DEF_ACC_RATE / DEF_DEC_RATE (config.h)

  // ---------- состояние между вызовами ----------
  static uint32_t lastUs = 0, lastFrameCount = 0, lastDbgMs = 0;
  static int   lastSide = 1;
  static bool  aligned = false, ballBehind = false;
  static float dirDeg = 0;                        // направление движения (после сглаживания), град
  static float velX = 0, velY = 0;                // текущий вектор скорости (после рампы)
  static AngleTracker eTrk;                       // оценка ошибки e и её скорости
  static float biasDeg = DEF_BIAS_NEAR;           // текущий радиальный доворот (плавно едет к NEAR/FAR)
  static int   lastLineDir = 0;                   // последнее направление "от линии", которое видела камера
  static uint32_t camLineSeenMs = 0;              // когда камера последний раз видела линию

  // Фиксированный период: без println в цикле loop() крутится десятки кГц,
  // а все ограничения по скорости/углу считаются от реального dt.
  uint32_t nowUs = micros();
  uint32_t dtUs = nowUs - lastUs;
  if (dtUs < DEF_CTRL_PERIOD_US) return;
  lastUs = nowUs;
  bool fresh = dtUs > 100000;                     // давно не вызывали: начинаем заново
  if (dtUs > 50000) dtUs = 50000;
  float dt = dtUs * 1e-6f;                        // сек

#if DEF_LOG_MOTORS
  readMotorFeedback();                            // забираем ответные кадры моторов (для лога "команда/факт")
#endif

  // ---------- входные величины ----------
  CamSnap cs = camSnapshot();                     // ball_dir/gate_dir_2/cam_line_dir из одного пакета
  int ballDir = cs.ball_dir, gateDir2 = cs.gate_dir_2, camLineDir = cs.cam_line_dir;
  uint32_t nowMs = millis();                      // берём ПОСЛЕ снимка: cs.frameMs <= nowMs

  float imU        = angleDiff(imuData.euler[0] - imuZero, 0);   // курс, град (float: без потери дробной части)
  float yawRate    = imuYawRate;                                 // град/с, знак как у imU
  int angleForLine = norm180(line_dir);
  int e            = norm180(ballDir - 180 - gateDir2);      // со знаком, с переходом через ±180

  // Трекер обновляется только на новом кадре камеры (с реальным интервалом между кадрами).
  // Кадр, где ничего не поменялось, камера отличить не даёт, поэтому после DEF_FRAME_HOLD_MS
  // тишины делаем "кадр без изменений" - скорость ошибки затухает.
  if (fresh) {
    eTrk.reset(e, nowMs);
    lastFrameCount = cs.frameCount;
  } else if (cs.frameCount != lastFrameCount) {
    lastFrameCount = cs.frameCount;
    eTrk.update(e, cs.frameMs, true);
  } else if ((int32_t)(nowMs - eTrk.tMs) > DEF_FRAME_HOLD_MS) {
    eTrk.update(e, nowMs, false);
  }
  float eF = eTrk.at(nowMs);                      // ошибка "сейчас" (экстраполяция между кадрами)
  float dEdt = eTrk.v;                            // град/с

  int DEFA = (int)fabsf(eF);

  // "эффективная" ошибка для скорости: учитываем, что мы уже приближаемся к цели
  float ePred = eF + dEdt * LEAD_S;
  float eEff  = fabsf(eF);
  if (ePred * eF <= 0)            eEff = 0;                 // скоро пересечём цель -> ползём
  else if (fabsf(ePred) < eEff)   eEff = fabsf(ePred);

  // ---------- гистерезисы ----------
  // Упреждающий старт: если ошибка быстро растёт (мяч уезжает), выходим из "выровнен" не дожидаясь
  // ALIGN_OUT - иначе робот стартует с места уже с отставанием. Ошибка убывает - на выход не влияет.
  int defaLead = (int)fabsf(angleDiff(eF + dEdt * DEF_START_LEAD_S, 0));
  if (DEFA <= ALIGN_IN) aligned = true; else if (DEFA > ALIGN_OUT || defaLead > ALIGN_OUT) aligned = false;
  if (abs(ballDir) > BEHIND_IN) ballBehind = true; else if (abs(ballDir) < BEHIND_OUT) ballBehind = false;

  // сторона = куда надо сместиться, чтобы уменьшить ошибку
  if (DEFA <= SIDE_HOLD && eF != 0) lastSide = (eF > 0) ? 1 : -1;
  int side = lastSide;

  bool farFromLine = (camLineDir < 0) && abs(gateDir2) > 135;

  // ---------- 1. цель: угол и скорость ----------
  int target = (int)lroundf(dirDeg);              // по умолчанию держим прошлое направление
  int speed  = 0;

  // Радиальный доворот: раньше направление скакало 80 <-> 135 град (вперёд-вбок <-> назад-вбок)
  // каждый раз, когда камера видела/теряла линию, отсюда "волны" вперёд-назад и просадка
  // скорости на каждом скачке. Теперь доворот плавно едет между NEAR и FAR.
  float biasT = farFromLine ? DEF_BIAS_FAR : DEF_BIAS_NEAR;
  if (fresh) biasDeg = biasT;
  else       biasDeg += constrain(biasT - biasDeg, -DEF_BIAS_RATE * dt, DEF_BIAS_RATE * dt);

  float gateRel = constrain(angleDiff(gateDir2, 180), -DEF_GATE_REL_MAX, DEF_GATE_REL_MAX);   // ворота относительно "строго сзади"

  if (!aligned && !ballBehind) {
    tmrDefA = millis();
    // Едем по касательной к воротам: ворота не строго сзади -> направление поворачивается вместе с ними
    // (по дуге вокруг ворот, а не по прямой). При воротах ровно сзади и bias=0 - строго вбок (90 град).
    target = norm180((int)lroundf(180.0f + gateRel - side * (90.0f - biasDeg)));
    // профиль скорости: DEF_PROFILE_EXP = 1 линейно (как раньше), 0.5 корень (торможение с постоянным замедлением)
    float x = constrain((eEff - ALIGN_OUT) / SPEED_RAMP, 0.0f, 1.0f);
    speed  = SPEED_MIN + (int)((SPEED_MAX - SPEED_MIN) * powf(x, DEF_PROFILE_EXP));
    // Упреждение по скорости мяча: пока ошибка РАСТЁТ (grow > 0), догоняем его не только по ошибке,
    // но и по её скорости - иначе на ходу всегда отстаём на время реакции.
    float grow = dEdt * (eF > 0 ? 1.0f : -1.0f);
    if (grow > 0) speed += (int)(DEF_FF_GAIN * grow);
    if (speed > SPEED_MAX) speed = SPEED_MAX;
  } else if (ballBehind && camLineDir < 0) {
    // Мяч сзади: боком защищать нечем, а линии камера не видит - медленно едем к своей линии
    // (в сторону ворот). Как только камера увидит линию, ветка выключается и робот стоит.
    // Съезд с линии по датчику (line_be) ниже по-прежнему главнее.
    target = norm180((int)lroundf(180.0f + gateRel));
    speed  = DEF_BEHIND_SPEED;
  } else {
    // здесь стоим; сюда можно вернуть проверку millis() - tmrDefA > 10000
  }
  if (!camFresh()) speed = 0;                     // камера молчит - стоим, а не едем по старым данным

  // ---------- 2. реакция на линию ----------
  if (line_be) {
    target = line_dir;
    if (speed < DEF_LINE_ESCAPE_SPEED) speed = DEF_LINE_ESCAPE_SPEED;   // съезжаем с линии, даже если "выровнены"
  }
  // Линию в камере учитываем на уровне вектора скорости (ниже, п.4), а не смесью направлений: смесь "от линии" и цели
  // делила боковую скорость почти пополам всякий раз, когда камера видит линию (а защитник живёт у линии),
  // при этом реально останавливал у линии только датчик.
  if (fresh) camLineSeenMs = 0;
  if (camLineDir > -1) { lastLineDir = camLineDir; camLineSeenMs = nowMs; }
  bool camLineVis = camLineSeenMs != 0 && (nowMs - camLineSeenMs) < 100;   // держим 100 мс, чтобы камера не мигала

  // Не уезжаем вперёд: и касательная по дуге, и примесь линии камеры (она обычно "от линии" = вперёд)
  // тянули направление вперёд. Съезд с линии по датчику (line_be) не ограничиваем.
  if (!line_be) target = limitForward(target, side, DEF_FWD_MAX_DEG);

  // ---------- 3. сглаживание направления ----------
  float err = angleDiff(target, dirDeg);
  if (line_be || fresh) {
    dirDeg = target;                              // у линии реагируем сразу
  } else {
    float step = ANG_RATE * dt;
    dirDeg = angleDiff(dirDeg + constrain(err, -step, step), 0);
  }
  degreesForDef = (int)lroundf(dirDeg);
  float turn = fabsf(angleDiff(target, dirDeg));
  speed = (int)(speed * constrain(180.0f - turn, 40.0f, 180.0f) / 180.0f);   // на развороте едем медленнее

  // ---------- 4. рампа на уровне ВЕКТОРА скорости ----------
  // Раньше рампа была по колёсам: при повороте вектора колёса догоняли цель с разной
  // скоростью, и траектория искажалась. Здесь ограничиваем изменение самого вектора
  // (сохраняя его направление), а рампу по колёсам ниже отключаем. Торможение
  // (вектор уменьшается) можно резче разгона - DEF_DEC_RATE.
  float rad = dirDeg * DEG_TO_RAD;
  float cmdX = speed * cosf(rad), cmdY = speed * sinf(rad);
  if (fresh) { velX = 0; velY = 0; }              // старт с нуля, а не с остатков до остановки

  // Камера видит линию: опора не только на датчик линии. (lastLineDir - направление "ОТ линии".)
  // Если команда или скорость идут НА линию, толкаем от неё тем сильнее, чем быстрее на неё едем, и гасим ход на линию
  // быстро (не больше ARC_FAST_RATE за такт). Стоим или едем вдоль линии - не трогаем: скорость вдоль неё остаётся.
  if (camLineVis && !line_be) {
    float ax = cosf(lastLineDir * DEG_TO_RAD), ay = sinf(lastLineDir * DEG_TO_RAD);
    float n  = cmdX * ax + cmdY * ay;             // < 0 - команда идёт на линию
    float vn = velX * ax + velY * ay;             // < 0 - едем на линию
    float approach = fmaxf(fmaxf(0.0f, -n), fmaxf(0.0f, -vn));
    if (approach > 0) {
      float wantAway = ARC_PRE_GAIN * approach;
      if (n < wantAway) { float add = wantAway - n; cmdX += add * ax; cmdY += add * ay; }
      if (vn < 0) { float rem = fminf(-vn, ARC_FAST_RATE * dt); velX += rem * ax; velY += rem * ay; }
    }
  }

  if (line_be) {
    velX = cmdX; velY = cmdY;                     // у линии рывок без ограничений
  } else {
    float dX = cmdX - velX, dY = cmdY - velY;
    float dm = sqrtf(dX * dX + dY * dY);
    bool braking = (dX * velX + dY * velY) < 0;
    float maxD = (braking ? DEF_DEC_RATE : DEF_ACC_RATE) * dt;
    if (dm > maxD) { float k = maxD / dm; dX *= k; dY *= k; }
    velX += dX; velY += dY;
  }
  speedForDef = (int)lroundf(sqrtf(velX * velX + velY * velY));

  // ---------- курс ----------
  // нет свежего курса - не крутимся (иначе вращение по мусору/устаревшему значению)
  int rot = 0;
  if (imuFresh()) {
    float p = (fabsf(imU) <= ROT_DEAD) ? 0.0f : ROT_K * imU;
    rot = (int)constrain(-(p + ROT_KD * yawRate), -ROT_MAX, ROT_MAX);
  }

  // ---------- отладка (не чаще SERIAL_INTERVAL) и движение ----------
  if (nowMs - lastDbgMs >= SERIAL_INTERVAL) {
    lastDbgMs = nowMs;
    char buffer[128];
    // sp - желаемая скорость до рампы, spd - фактическая после рампы; lb - датчик линии видит линию, cl - линию видит камера,
    // lf - сколько чтений датчика линии по I2C не удалось (растёт - железо/шина), cf - камера отвечает (0 = стоим, данных нет)
    snprintf(buffer, sizeof(buffer), "def = %d, e = %d, v = %d, sp = %d, spd = %d, b = %d, h = %d, w = %d, lb = %d, cl = %d, lf = %d, cf = %d",
             degreesForDef, (int)eF, (int)dEdt, speed, speedForDef, (int)biasDeg, (int)imU, (int)yawRate,
             (int)line_be, (int)camLineVis, (int)lineFails, (int)camFresh());
    Serial.println(buffer);
#if DEF_LOG_MOTORS
    // команда (последняя отправленная, град/с) / факт из ответа мотора: если факт сильно отстаёт -
    // упираемся в сами моторы/драйвер, а не в код
    char wb[80];
    snprintf(wb, sizeof(wb), "wheels cmd/act: %ld/%d %ld/%d %ld/%d %ld/%d",
             (long)motors[0].target, motors[0].actualSpeed, (long)motors[1].target, motors[1].actualSpeed,
             (long)motors[2].target, motors[2].actualSpeed, (long)motors[3].target, motors[3].actualSpeed);
    Serial.println(wb);
#endif
  }
  omniMoveXY(velX, velY, rot, true);              // рампа уже на векторе - по колёсам не нужна
}

// ============================================================================
//  DefenseMode - компактная версия. Едем вбок вдоль своих ворот так, чтобы мяч оказался напротив ворот (e = 0),
//  курс держим по BNO, от линий отталкиваемся датчиком и камерой. Мяч пропал - стоим.
// ============================================================================
void DefenseMode() {
  const float SPEED_MAX = 5000, SPEED_MIN = 900, SPEED_K = 130;   // скорость = SPEED_K * ошибка (град) в пределах MIN..MAX
  const float ALIGN_IN = 6, ALIGN_OUT = 10;                        // гистерезис "встали на линию мяч-ворота"
  const float ACCEL = 12000, DECEL = 18000;                        // разгон / торможение вектора, ед/с
  const float ROT_K = 20, ROT_KD = 1, ROT_MAX = 600;               // удержание курса

  static uint32_t lastUs = 0, lastDbgMs = 0, ballOkMs = 0, ballOutSince = 0, lineSeenMs = 0;
  static float dirDeg = 0, velX = 0, velY = 0, ballF = 0;
  static bool aligned = false, behind = false;
  static int side = 1, lineAway = 0;

  // период 5 мс
  uint32_t nowUs = micros();
  uint32_t dtUs = nowUs - lastUs;
  if (dtUs < 5000) return;
  lastUs = nowUs;
  bool fresh = dtUs > 100000;                     // давно не вызывали: начинаем заново
  if (dtUs > 50000) dtUs = 50000;
  float dt = dtUs * 1e-6f;

  CamSnap cs = camSnapshot();
  uint32_t nowMs = millis();
  float imU = angleDiff(imuData.euler[0] - imuZero, 0);

  // ---------- мяч ----------
  // Мяч ушёл из обзора - камера может схватить что-то другое, и угол "телепортируется". Скачок больше 40 град
  // не принимаем, пока он не продержится 250 мс; 400 мс без нормального мяча - стоим.
  float bm = angleDiff(cs.ball_dir, 0);
  if (fresh || nowMs - ballOkMs > 1500) {
    ballF = bm; ballOkMs = nowMs; ballOutSince = 0;
  } else if (fabsf(angleDiff(bm, ballF)) < 40) {
    ballF = angleDiff(ballF + (1.0f - expf(-dt / 0.03f)) * angleDiff(bm, ballF), 0);   // лёгкое сглаживание
    ballOkMs = nowMs; ballOutSince = 0;
  } else {
    if (!ballOutSince) ballOutSince = nowMs;
    if (nowMs - ballOutSince > 250) { ballF = bm; ballOkMs = nowMs; ballOutSince = 0; }
  }
  bool ballLost = (nowMs - ballOkMs > 400) || !camFresh();

  // ---------- ошибка и режим ----------
  float e  = angleDiff(ballF - 180 - cs.gate_dir_2, 0);   // 0 - мяч строго напротив своих ворот
  float ae = fabsf(e);
  if (ae <= ALIGN_IN) aligned = true; else if (ae > ALIGN_OUT) aligned = false;
  if (fabsf(ballF) > 170) behind = true; else if (fabsf(ballF) < 155) behind = false;   // мяч сзади
  if (ae <= 150 && e != 0) side = (e > 0) ? 1 : -1;       // сторона; у +-180 не переключаем

  // ---------- камера видит линию (держим 100 мс, чтобы не мигала) ----------
  if (fresh) lineSeenMs = 0;
  if (cs.cam_line_dir > -1) { lineAway = cs.cam_line_dir; lineSeenMs = nowMs; }
  bool lineVis = lineSeenMs != 0 && nowMs - lineSeenMs < 100;   // lineAway - направление "ОТ линии"

  // ---------- направление и скорость ----------
  float gateRel = constrain(angleDiff(cs.gate_dir_2, 180), -60.0f, 60.0f);   // ворота относительно "строго сзади"
  bool farFromLine = cs.cam_line_dir < 0 && abs(cs.gate_dir_2) > 135;        // линии не видно - чуть к воротам
  int target = (int)lroundf(dirDeg);
  float speed = 0;
  if (!ballLost) {
    if (!aligned && !behind) {
      target = norm180((int)lroundf(180.0f + gateRel - side * (90.0f - (farFromLine ? 30.0f : 0.0f))));   // по дуге вокруг ворот
      speed = constrain(SPEED_K * ae, SPEED_MIN, SPEED_MAX);
    } else if (behind && cs.cam_line_dir < 0) {
      target = norm180((int)lroundf(180.0f + gateRel));                       // мяч сзади: медленно к своей линии
      speed = 300;
    }
  }
  if (!line_be) target = limitForward(target, side, 8);                       // вперёд не уезжаем
  if (line_be) { target = line_dir; if (speed < 2000) speed = 2000; }         // датчик линии: съезжаем сразу

  if (line_be || fresh) dirDeg = target;
  else dirDeg = angleDiff(dirDeg + constrain(angleDiff(target, dirDeg), -720 * dt, 720 * dt), 0);

  // ---------- вектор скорости ----------
  float rad = dirDeg * DEG_TO_RAD;
  float cmdX = speed * cosf(rad), cmdY = speed * sinf(rad);
  if (fresh) { velX = 0; velY = 0; }
  if (lineVis && !line_be) {
    // Камера видит линию. Команда или скорость идут НА линию - отталкиваемся тем сильнее, чем быстрее на неё едем,
    // и гасим ход на линию быстро. Стоим или едем вдоль линии - не трогаем.
    float ax = cosf(lineAway * DEG_TO_RAD), ay = sinf(lineAway * DEG_TO_RAD);
    float n = cmdX * ax + cmdY * ay, vn = velX * ax + velY * ay;      // < 0 - на линию
    float approach = fmaxf(fmaxf(0.0f, -n), fmaxf(0.0f, -vn));
    if (approach > 0) {
      if (n < 0.7f * approach) { float add = 0.7f * approach - n; cmdX += add * ax; cmdY += add * ay; }
      if (vn < 0) { float rem = fminf(-vn, 36000 * dt); velX += rem * ax; velY += rem * ay; }
    }
  }
  if (line_be) {
    velX = cmdX; velY = cmdY;                                            // у линии рывок без рампы
  } else {
    float dX = cmdX - velX, dY = cmdY - velY;
    float dm = sqrtf(dX * dX + dY * dY);
    float maxD = (((dX * velX + dY * velY) < 0) ? DECEL : ACCEL) * dt;  // тормозить можно резче, чем разгоняться
    if (dm > maxD) { float k = maxD / dm; dX *= k; dY *= k; }
    velX += dX; velY += dY;
  }
  degreesForDef = (int)lroundf(dirDeg);
  speedForDef = (int)lroundf(sqrtf(velX * velX + velY * velY));

  // ---------- курс ----------
  int rot = 0;
  if (imuFresh()) rot = (int)constrain(-((fabsf(imU) <= 1 ? 0 : ROT_K * imU) + ROT_KD * (float)imuYawRate), -ROT_MAX, ROT_MAX);

  if (nowMs - lastDbgMs >= SERIAL_INTERVAL) {
    lastDbgMs = nowMs;
    char buffer[128];
    // br/bd - сырые ball_dir и ball_dist с камеры (что она присылает, когда мяча нет), lost - мяча нет, lb - датчик линии,
    // cl - камера видит линию, lf - сбои чтения датчика линии по I2C
    snprintf(buffer, sizeof(buffer), "def=%d e=%d sp=%d spd=%d h=%d br=%d bd=%d lost=%d lb=%d cl=%d lf=%d",
             degreesForDef, (int)e, (int)speed, speedForDef, (int)imU, (int)cs.ball_dir, (int)cs.ball_dist,
             (int)ballLost, (int)line_be, (int)lineVis, (int)lineFails);
    Serial.println(buffer);
  }
  omniMoveXY(velX, velY, rot, true);
}

void DefenseMode_save001() {
  int imU = (imuData.euler[0] - 360 * (imuData.euler[0] > 180));
  int angleForLine = norm180(line_dir);
  int side = ball_dir / max(abs(ball_dir), 1);
  int DEFA = abs(ball_dir - norm180((180 + gate_dir_2)));
  speedForDef = 4000;

  if ((cam_line_dir < 0) && abs(gate_dir_2) > 135) {
    if (DEFA > 10 && abs(ball_dir) < 170) {
      tmrDefA = millis();
      degreesForDef = 135 * side;
    } else {
      if (millis() - tmrDefA > 10000){
      }
      speedForDef = 0;
    }
  } else {
    if (DEFA > 10 && abs(ball_dir) < 170) {
      tmrDefA = millis();
      degreesForDef = 80 * side;
    } else {
      if (millis() - tmrDefA > 10000){
      }
      speedForDef = 0;
    }
  }
  if (line_be || cam_line_dir > -1) {
    if (line_be) {
      degreesForDef = line_dir;
    } else if (cam_line_dir > -1 && abs(angleForLine) > 80 && abs(angleForLine) < 100) {
      degreesForDef = cam_line_dir;
    } else if (cam_line_dir > -1 && DEFA > 10 && DEFA < 170) {
      degreesForDef = VectorSum(cam_line_dir, degreesForDef, 1.2, 1);
    }
  }

  char buffer[64];

  sprintf(buffer, "def = %d, trig = %d", degreesForDef, abs(ball_dir - ((180 + gate_dir_2) - 360 * ((180 + gate_dir_2) > 180))));
  Serial.println(buffer);
  omniMoveDirection(speedForDef, degreesForDef, imU * -20);
}
bool toLine(int d) {
  int deltaD = 0;
  if (cam_line_dir > -1) {
    deltaD = abs(cam_line_dir - d);
  }
  if (line_be) {
    deltaD = abs(line_dir - d);
  }
  return deltaD % 180 > 90;
}

float rot_speed = 0;
void AtackMode() {
  rot_speed = gate_dir_1 * 10;  // не больше 700: если ворот не видно, не крутимся волчком

  if (!camFresh()) {                                   // камера молчит - стоим, а не едем по старым данным
    omniMoveDirection(0, 0, 0);
    return;
  }

  if ((cam_line_dir > -1 || line_be)) {
    if (line_be) {
      omniMoveDirection(500, line_dir, rot_speed, true);
    } else {
      omniMoveDirection(1500, cam_line_dir, rot_speed, true);
    }
    return;
  }


  if (!trig) {
    digitalWrite(LED_BUILTIN, 0);
    if (abs(ball_dir) < 8) {
      // Мяч перед роботом и не захвачен - едем прямо, форсируем скорость при простое accel_via_distance() + stallBoost()
      // Скорость по расстоянию: далеко - быстро (было 3500, а мяч сбоку ехал на 4500 - спереди было медленнее), ближе - притормаживаем
      omniMoveDirection(constrain(ball_dist * 100, 2000, 4500), ball_dir, rot_speed);
      if (abs(gate_dir_1) < 10) {
        kicker();
      }
    } else {
      // Мяч не перед роботом - объезжаем, скорость растёт с углом
      if (ball_dist <= 20) {
        resetStall();
        omniMoveDirection(attackSpeed(ball_dir) + accel_via_distance(), constrain(abs(ball_dir) + 60, 0, 120) * (ball_dir / abs(ball_dir)), rot_speed);   // не дальше 120 град: обход не уводит далеко назад
      } else {
        omniMoveDirection(1500, ball_dir, rot_speed);
      }
    }
  } else {
    digitalWrite(LED_BUILTIN, 1);
    // Мяч захвачен - целимся в ворота и бьём
    if (abs(gate_dir_1) < 15) {
      kicker();
    }
    omniMoveDirection(500, 0, rot_speed);
  }
}


void AtackMode_save002() {
  rot_speed = gate_dir_1 * 10;
  if ((cam_line_dir > -1 || line_be)) {
    if (line_be) {
      omniMoveDirection(1000, line_dir, rot_speed, true);
    } else {

      omniMoveDirection(1000, VectorSum(cam_line_dir, ball_dir, 2, 1), rot_speed, true);
    }
    return;
  }


  if (!trig) {
    digitalWrite(LED_BUILTIN, 0);
    if (abs(ball_dir) < 10) {
      // Мяч перед роботом и не захвачен - едем прямо, форсируем скорость при простое accel_via_distance() + stallBoost()
      omniMoveDirection(3500, ball_dir, rot_speed);
      if (abs(gate_dir_1) < 15) {
        kicker();
      }
    } else {
      // Мяч не перед роботом - объезжаем, скорость растёт с углом
      if (ball_dist <= 20) {
        resetStall();
        omniMoveDirection(attackSpeed(ball_dir) + accel_via_distance(), (abs(ball_dir) + 90) * (ball_dir / abs(ball_dir)), rot_speed);
      } else {
        omniMoveDirection(3000, ball_dir, rot_speed);
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

void AtackMode_save001() {
  if (cam_line_dir < 0 || !line_be) {
    if (!trig) {
      digitalWrite(LED_BUILTIN, 0);
      if (abs(ball_dir) < 10) {
        // Мяч перед роботом и не захвачен - едем прямо, форсируем скорость при простое
        omniMoveDirection(accel_via_distance() + stallBoost(), ball_dir, gate_dir_1 * 10);
      } else {
        // Мяч не перед роботом - объезжаем, скорость растёт с углом
        if (ball_dist <= 20) {
          resetStall();
          omniMoveDirection(attackSpeed(ball_dir) + accel_via_distance(), (abs(ball_dir) + 90) * (ball_dir / abs(ball_dir)), gate_dir_1 * 10);
        } else {
          omniMoveDirection(2000, ball_dir, gate_dir_1 * 10);
        }
      }
    } else {
      digitalWrite(LED_BUILTIN, 1);
      // Мяч захвачен - целимся в ворота и бьём
      if (abs(gate_dir_1) < 15) {
        //kicker();
      }
      omniMoveDirection(2500, 0, gate_dir_1 * 10);
    }
  } else {
    // Отталкивание от линии - без разгона, резкий рывок
    if (line_be) {
      omniMoveDirection(2000, line_dir, gate_dir_1 * 10, true);
    } else {
      omniMoveDirection(2000, cam_line_dir, gate_dir_1 * 10, true);
    }
  }
}