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

void printCamData() {
  if (millis() - lastCamDebugPrint < SERIAL_INTERVAL) return;
  lastCamDebugPrint = millis();

  Serial.print("ball_dir: ");      Serial.print(ball_dir);
  Serial.print("  gate_dir_1: ");  Serial.print(gate_dir_1);
  Serial.print("  gate_dir_2: ");  Serial.print(gate_dir_2);
  Serial.print("  cam_line_dir: ");Serial.print(cam_line_dir);
  Serial.print("  ball_dist: ");   Serial.print(ball_dist);
  Serial.print("  trig: ");        Serial.println(trig);
}

void cam_read() {
  uint8_t buffer[12];
  size_t bytesRead = Wire1.requestFrom(CAM_ADDR, 12);

  if (bytesRead == 12) {
    for (size_t i = 0; i < 12; i++) {
      buffer[i] = Wire1.read();
    }

    // Распаковка углов (little-endian)
    ball_dir = buffer[0] | (buffer[1] << 8);
    //ball_dir += 18;
    gate_dir_2 = buffer[2] | (buffer[3] << 8);
    gate_dir_1 = buffer[4] | (buffer[5] << 8);
    cam_line_dir = buffer[6] | (buffer[7] << 8);
    ball_dist = buffer[8] | (buffer[9] << 8);
    trig = buffer[10] | (buffer[11] << 8);
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

bool readLine() {
  uint8_t buf[3];
  if (Wire1.requestFrom(LINE_ADDR, 3) != 3) {
    // Нет ответа: не держим старое "линия видна" дольше LINE_TIMEOUT,
    // иначе робот будет вечно ехать "от линии".
    if (millis() - lineStamp > LINE_TIMEOUT) line_be = false;
    return false;
  }
  for (uint8_t i = 0; i < 3; i++) buf[i] = Wire1.read();
  // line_dir - одним присваиванием и раньше line_be: ядро 0 читает их без блокировок
  // и не должно увидеть промежуточное значение (до вычитания 180).
  line_dir = (int16_t)(buf[1] | (buf[2] << 8)) - 180;
  line_be = buf[0];
  lineStamp = millis();
  return true;
}

void printLineData() {
  Serial.print("LINE  detected="); Serial.print(line_be);
  Serial.print(" angle=");         Serial.println(line_dir);
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
void omniMoveDirection(int32_t wheelSpeed, float direction, int32_t w, bool instant = false) {
  motorAccelEnabled = !instant;

  float directionRad = (direction)*DEG_TO_RAD;

  float Vx = wheelSpeed * cos(directionRad);
  float Vy = wheelSpeed * sin(directionRad);

  int32_t motor_dps[4];

  static constexpr float SIN_A[4] = { -0.5f, 0.5f, 0.5f, -0.5f };
  static constexpr float COS_A[4] = { 0.8660254f, 0.8660254f, -0.8660254f, -0.8660254f };

  for (int i = 0; i < 4; i++) {
    motor_dps[i] = ((Vx * COS_A[i]) + Vy * SIN_A[i]) / 2 + w;
    setMotorSpeed(i, motor_dps[i]);
  }
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

unsigned long tmrDefA = 0;
int degreesForDef = 0, speedForDef = 0;
void DefenseMode() {
  // ---------- параметры ----------
  const int   DIR_SIGN = 1;                       // -1, если защита уезжает ОТ линии мяч-ворота, а не к ней
  const float SPEED_MAX = 6000;                   // потолок скорости (на колесо это <= 3000 + rot, запас до MAX_MOTOR_SPEED есть)
  const float KS = 750.0f;                        // дальняя зона: скорость = KS*sqrt(ошибка): 10° -> ~2000, 20° -> ~3100, 40° -> ~4550
  const float KL = 300.0f;                        // ближняя зона: скорость = KL*ошибка (до ~6° сверх E_IN), у цели без бесконечного усиления sqrt
  const float KD = 8.0f;                          // ед. скорости на 1°/с изменения ошибки: тормозит заранее, гасит перелёт
  const float E_IN = 3.0f, E_OUT = 5.0f;          // гистерезис: |e| <= E_IN - встали, > E_OUT - поехали
  const float E_TAU = 0.02f, D_TAU = 0.05f;       // фильтры ошибки и её производной, с
  const float ACC = 24000, DECEL = 32000;         // разгон/торможение вектора скорости, ед/с
  const float BASE_TAU = 0.15f;                   // плавная смена курса 80° <-> 135°, с
  const int   BEHIND_IN = 170, BEHIND_OUT = 155;  // мяч строго сзади -> стоим
  const int   BACK_IN = 110, BACK_OUT = 95;       // мяч сзади-сбоку -> едем, но не быстрее BACK_SPEED_MAX
  const float BACK_SPEED_MAX = 1500;              // потолок скорости, пока мяч сзади (раньше до 6000 - назад уезжал слишком быстро)
  const int   SIDE_HOLD = 150;                    // |e| больше: сторону не переключаем
  const float ROT_K = 20, ROT_MAX = 600, ROT_DEAD = 1;   // стабилизация курса
  // рывок с ударом: мяч в "области ворот 1" - в секторе +-DASH_GATE вокруг направления на ворота соперника
  const int   DASH_GATE = 20;                     // полуширина области ворот, град (было 5, и мяч отдельно "спереди" +-15)
  const int   DASH_DIST = 35;                     // ball_dist не больше этого (в атаке "рядом" - 20; 100 - без ограничения)
  const int   KICK_DIST = 12;                     // ball_dist не больше этого (или trig) - мяч у пеналки, бьём
  const float DASH_SPEED = 5500;                  // скорость рывка
  const float DASH_ACC = 40000;                   // разгон на рывке, ед/с (обычный ACC 24000)
  const float DASH_ROT_K = 10, DASH_ROT_MAX = 600;    // на рывке доворачиваем корпус (пеналку) на мяч
  const uint32_t KICK_LATEST_MS = 450;            // мяч так и не дошёл до пеналки - бьём всё равно через столько мс
  const uint32_t DASH_FOLLOW_MS = 100;            // после удара ещё столько мс едем на мяч (дожимаем)
  const uint32_t DASH_COOL_MS = 800;              // пауза между рывками, мс
  const uint32_t DASH_HOLD_MS = BALL_FRONT_HOLD_MS;   // условие рывка должно держаться непрерывно столько мс (500)
  const float RETURN_SPEED = 1500;                // после рывка сдаём назад с такой скоростью, пока камера не увидит линию
  const uint32_t RETURN_MAX_MS = 2000;            // но не дольше этого

  // ---------- состояние между вызовами ----------
  static unsigned long lastUs = 0, lastPrintMs = 0;
  static int   lastSide = 1;
  static bool  moving = false, ballBehind = false, ballBack = false;
  static float eF = 0, prevEF = 0, dEF = 0;       // отфильтрованная ошибка и её производная, °/с
  static float vF = 0, vS = 0;                    // текущий вектор скорости: вперёд (cos) и вбок (sin)
  static float base = 80;                         // текущий "боковой" курс, °
  static uint32_t dashStart = 0, kickAt = 0, dashReadyAt = 0, returnUntil = 0;
  static uint32_t dashCondSince = 0;              // с какого момента условие рывка держится непрерывно; 0 - не держится
  static bool  dash = false, kicked = false;

  // dt по micros(): цикл бывает короче 1 мс, по millis() dt скакал бы 0/1/2
  unsigned long nowUs = micros();
  float dt = (nowUs - lastUs) * 1e-6f;
  lastUs = nowUs;
  bool fresh = dt > 0.1f;                         // давно не вызывали: начинаем заново
  if (dt > 0.05f)   dt = 0.05f;
  if (dt < 0.0001f) dt = 0.0001f;
  uint32_t nowMs = millis();

  // ---------- входные величины ----------
  float imU        = imuData.euler[0];
  if (imU > 180) imU -= 360;
  int e            = norm180(ball_dir - 180 - gate_dir_2);   // со знаком, с переходом через ±180
  bool farFromLine = (cam_line_dir < 0) && abs(gate_dir_2) > 135;

  if (fresh) {
    eF = e; prevEF = e; dEF = 0; vF = vS = 0; base = farFromLine ? 135 : 80;
    returnUntil = nowMs; dash = false; dashCondSince = 0;
  }

  // фильтр ошибки по времени, а не по числу вызовов (по кратчайшей дуге около ±180)
  float d = e - eF;
  if (d > 180) d -= 360; else if (d < -180) d += 360;
  eF += d * dt / (E_TAU + dt);
  if (eF > 180) eF -= 360; else if (eF < -180) eF += 360;

  float dd = eF - prevEF;
  if (dd > 180) dd -= 360; else if (dd < -180) dd += 360;
  prevEF = eF;
  dEF += (dd - dEF * dt) / (D_TAU + dt);          // сглаженная dE/dt

  // ---------- гистерезисы ----------
  if (fabsf(eF) > E_OUT) moving = true; else if (fabsf(eF) <= E_IN) moving = false;
  if (abs(ball_dir) > BEHIND_IN) ballBehind = true; else if (abs(ball_dir) < BEHIND_OUT) ballBehind = false;
  if (abs(ball_dir) > BACK_IN) ballBack = true; else if (abs(ball_dir) < BACK_OUT) ballBack = false;
  if (fabsf(eF) <= SIDE_HOLD && eF != 0) lastSide = (eF > 0) ? 1 : -1;

  // ---------- 1. желаемая боковая скорость ----------
  // Профиль: далеко sqrt (как торможение с постоянным замедлением - едем быстро),
  // у цели линейный: у sqrt около нуля усиление бесконечное, и с задержкой камеры
  // робот бы перелетал. KD сбрасывает скорость заранее,
  // пока робот ещё подъезжает к линии мяч-ворота (камера запаздывает).
  float vLat = 0;                                 // со знаком: > 0 - в сторону +base
  float mag  = fabsf(eF) - E_IN;
  if (moving && !ballBehind && mag > 0 && camFresh()) {   // камера молчит - стоим, а не едем по старым данным
    tmrDefA = millis();
    float p = fminf(KL * mag, KS * sqrtf(mag)) * lastSide;
    vLat = p + KD * dEF;
    if (vLat * p < 0) vLat = 0;                   // D только тормозит, назад не разворачивает
    // мяч сзади: ошибка огромная, без потолка робот на полной скорости уезжал назад/вбок
    float vMax = ballBack ? BACK_SPEED_MAX : SPEED_MAX;
    vLat = DIR_SIGN * constrain(vLat, -vMax, vMax);
  }

  // плавно меняем "боковой" курс 80° <-> 135°
  base += ((farFromLine ? 135 : 80) - base) * dt / (BASE_TAU + dt);
  float b  = base * DEG_TO_RAD;
  float tF = fabsf(vLat) * cosf(b);               // вперёд/назад
  float tS = vLat * sinf(b);                      // вбок

  // ---------- 1б. рывок с ударом ----------
  // Мяч в области ворот 1 (не дальше DASH_GATE от направления на них; ворота - в передней полусфере) -
  // тупо едем на мяч и бьём. Начатый рывок выполняется гарантированно: не прерывается, пока не будет удара,
  // а удар - как только мяч у пеналки, но не позже KICK_LATEST_MS. Стартуем, только когда пеналка заряжена
  // (kickFlag), иначе удар мог бы не случиться. Съезд с линии по датчику ниже всё равно главнее (безопасность).
  // Таймер перед рывком: мяч должен пробыть в области ворот (и рядом) непрерывно DASH_HOLD_MS - случайный
  // проскок мяча через область или дребезг камеры рывка не вызывают.
  if (!dash) {
    bool cond = camFresh() && !line_be && abs(gate_dir_1) < 90 &&
                abs(norm180(ball_dir - gate_dir_1)) <= DASH_GATE && ball_dist <= DASH_DIST;
    if (!cond)              dashCondSince = 0;
    else if (!dashCondSince) dashCondSince = nowMs ? nowMs : 1;
    if (cond && nowMs - dashCondSince >= DASH_HOLD_MS && kickFlag && (int32_t)(nowMs - dashReadyAt) >= 0) {
      dash = true; kicked = false; dashStart = nowMs;
      dashCondSince = 0;                          // следующий рывок - снова только после DASH_HOLD_MS
    }
  }
  if (dash) {
    if (!kicked && (trig || ball_dist <= KICK_DIST || nowMs - dashStart >= KICK_LATEST_MS)) {
      kicker();                                   // пеналка заряжена (проверили на старте) - удар будет
      kicked = true; kickAt = millis();           // millis(), а не nowMs: kicker() держит импульс KICK_PULSE_MS
    }
    if (kicked && millis() - kickAt >= DASH_FOLLOW_MS) {
      dash = false;                               // рывок закончен - возвращаемся назад к линии
      dashReadyAt = millis() + DASH_COOL_MS;
      returnUntil = millis() + RETURN_MAX_MS;
    }
  }
  if (dash) {
    float a = ball_dir * DEG_TO_RAD;              // тупо на мяч
    tF = DASH_SPEED * cosf(a);
    tS = DASH_SPEED * sinf(a);
  }
  if (cam_line_dir > -1 || line_be || ballBack) returnUntil = nowMs;   // линия видна / мяч сзади - возврат не нужен
  if (!dash && (int32_t)(returnUntil - nowMs) > 0 && tF > -RETURN_SPEED) tF = -RETURN_SPEED;

  // ---------- 2. реакция на линию ----------
  // Линия не заменяет боковое движение, а только срезает его составляющую "в линию":
  // робот скользит вдоль линии с полной боковой скоростью, а не скачет вперёд-назад.
  // line_dir и cam_line_dir - направление ОТ линии.
  bool onLine = line_be;
  if (onLine || cam_line_dir > -1) {
    float a    = (onLine ? line_dir : cam_line_dir) * DEG_TO_RAD;
    float nF   = cosf(a), nS = sinf(a);
    float away = tF * nF + tS * nS;               // < 0 - едем в линию
    if (away < 0) { tF -= away * nF; tS -= away * nS; away = 0; }
    if (onLine && away < DEF_LINE_ESCAPE_SPEED) { // на линии ещё и отталкиваемся, даже если стоим
      tF += (DEF_LINE_ESCAPE_SPEED - away) * nF;
      tS += (DEF_LINE_ESCAPE_SPEED - away) * nS;
    }
  }

  // ---------- 3. сглаживание ВЕКТОРА скорости ----------
  // Сглаживаем (vF, vS), а не угол: смена стороны - это плавный проход vS через 0,
  // а не разворот вектора через "вперёд" с выездом из ворот и срезом скорости.
  if (onLine) {
    vF = tF; vS = tS;                             // у линии рывок без ограничения разгона
  } else {
    float dF = tF - vF, dS = tS - vS;
    float dist = hypotf(dF, dS);
    float step = (dash ? DASH_ACC : (hypotf(tF, tS) > hypotf(vF, vS) ? ACC : DECEL)) * dt;
    if (dist > step) { vF += dF * step / dist; vS += dS * step / dist; }
    else             { vF = tF; vS = tS; }
  }
  float dir = atan2f(vS, vF) * RAD_TO_DEG;
  speedForDef = (int)lroundf(hypotf(vF, vS));
  if (speedForDef > 0) degreesForDef = (int)lroundf(dir);

  // ---------- курс ----------
  // нет свежего курса - не крутимся; мёртвая зона плавная: rot растёт от 0, а не прыгает 0 -> 60
  int rot = 0;
  float h = fabsf(imU) - ROT_DEAD;
  if (imuFresh() && h > 0) rot = (int)constrain(-ROT_K * h * (imU > 0 ? 1 : -1), -ROT_MAX, ROT_MAX);
  // на рывке разворачиваем пеналку на мяч (мяч в области ворот - значит, заодно и на ворота);
  // после рывка удержание курса по BNO само вернёт робота лицом вперёд
  if (dash) rot = (int)constrain(ball_dir * DASH_ROT_K, -DASH_ROT_MAX, DASH_ROT_MAX);

  // ---------- отладка и движение ----------
  // печать не чаще SERIAL_INTERVAL: Serial на каждом вызове тормозит цикл управления
  if (millis() - lastPrintMs >= SERIAL_INTERVAL) {
    lastPrintMs = millis();
    char buffer[80];
    // bk - мяч сзади (скорость ограничена), dash - идёт рывок, bg - угол мяча от ворот 1 (рывок при |bg| <= DASH_GATE)
    snprintf(buffer, sizeof(buffer), "def = %d, e = %d, spd = %d, bk = %d, dash = %d, bg = %d",
             degreesForDef, (int)eF, speedForDef, (int)ballBack, (int)dash, norm180(ball_dir - gate_dir_1));
    Serial.println(buffer);
  }
  // instant = true: разгон уже ограничен выше, рампа моторов добавила бы вторую задержку
  omniMoveDirection(speedForDef, dir, rot, true);
}

void DefenseMode_save005() {
  // ---------- параметры ----------
  const int   DIR_SIGN = 1;                       // -1, если защита уезжает ОТ линии мяч-ворота, а не к ней
  const float SPEED_MAX = 6000;                   // потолок скорости (на колесо это <= 3000 + rot, запас до MAX_MOTOR_SPEED есть)
  const float KS = 750.0f;                        // дальняя зона: скорость = KS*sqrt(ошибка): 10° -> ~2000, 20° -> ~3100, 40° -> ~4550
  const float KL = 300.0f;                        // ближняя зона: скорость = KL*ошибка (до ~6° сверх E_IN), у цели без бесконечного усиления sqrt
  const float KD = 8.0f;                          // ед. скорости на 1°/с изменения ошибки: тормозит заранее, гасит перелёт
  const float E_IN = 3.0f, E_OUT = 5.0f;          // гистерезис: |e| <= E_IN - встали, > E_OUT - поехали
  const float E_TAU = 0.02f, D_TAU = 0.05f;       // фильтры ошибки и её производной, с
  const float ACC = 24000, DECEL = 32000;         // разгон/торможение вектора скорости, ед/с
  const float BASE_TAU = 0.15f;                   // плавная смена курса 80° <-> 135°, с
  const int   BEHIND_IN = 170, BEHIND_OUT = 155;  // мяч строго сзади -> стоим
  const int   BACK_IN = 110, BACK_OUT = 95;       // мяч сзади-сбоку -> едем, но не быстрее BACK_SPEED_MAX
  const float BACK_SPEED_MAX = 1500;              // потолок скорости, пока мяч сзади (раньше до 6000 - назад уезжал слишком быстро)
  const int   SIDE_HOLD = 150;                    // |e| больше: сторону не переключаем
  const float ROT_K = 20, ROT_MAX = 600, ROT_DEAD = 1;   // стабилизация курса
  // рывок с ударом: мяч в "области ворот 1" - в секторе +-DASH_GATE вокруг направления на ворота соперника
  const int   DASH_GATE = 20;                     // полуширина области ворот, град (было 5, и мяч отдельно "спереди" +-15)
  const int   DASH_DIST = 35;                     // ball_dist не больше этого (в атаке "рядом" - 20; 100 - без ограничения)
  const int   KICK_DIST = 12;                     // ball_dist не больше этого (или trig) - мяч у пеналки, бьём
  const float DASH_SPEED = 5500;                  // скорость рывка
  const float DASH_ACC = 40000;                   // разгон на рывке, ед/с (обычный ACC 24000)
  const float DASH_ROT_K = 10, DASH_ROT_MAX = 600;    // на рывке доворачиваем корпус (пеналку) на мяч
  const uint32_t KICK_LATEST_MS = 450;            // мяч так и не дошёл до пеналки - бьём всё равно через столько мс
  const uint32_t DASH_FOLLOW_MS = 100;            // после удара ещё столько мс едем на мяч (дожимаем)
  const uint32_t DASH_COOL_MS = 800;              // пауза между рывками, мс
  const float RETURN_SPEED = 1500;                // после рывка сдаём назад с такой скоростью, пока камера не увидит линию
  const uint32_t RETURN_MAX_MS = 2000;            // но не дольше этого

  // ---------- состояние между вызовами ----------
  static unsigned long lastUs = 0, lastPrintMs = 0;
  static int   lastSide = 1;
  static bool  moving = false, ballBehind = false, ballBack = false;
  static float eF = 0, prevEF = 0, dEF = 0;       // отфильтрованная ошибка и её производная, °/с
  static float vF = 0, vS = 0;                    // текущий вектор скорости: вперёд (cos) и вбок (sin)
  static float base = 80;                         // текущий "боковой" курс, °
  static uint32_t dashStart = 0, kickAt = 0, dashReadyAt = 0, returnUntil = 0;
  static bool  dash = false, kicked = false;

  // dt по micros(): цикл бывает короче 1 мс, по millis() dt скакал бы 0/1/2
  unsigned long nowUs = micros();
  float dt = (nowUs - lastUs) * 1e-6f;
  lastUs = nowUs;
  bool fresh = dt > 0.1f;                         // давно не вызывали: начинаем заново
  if (dt > 0.05f)   dt = 0.05f;
  if (dt < 0.0001f) dt = 0.0001f;
  uint32_t nowMs = millis();

  // ---------- входные величины ----------
  float imU        = imuData.euler[0];
  if (imU > 180) imU -= 360;
  int e            = norm180(ball_dir - 180 - gate_dir_2);   // со знаком, с переходом через ±180
  bool farFromLine = (cam_line_dir < 0) && abs(gate_dir_2) > 135;

  if (fresh) {
    eF = e; prevEF = e; dEF = 0; vF = vS = 0; base = farFromLine ? 135 : 80;
    returnUntil = nowMs; dash = false;
  }

  // фильтр ошибки по времени, а не по числу вызовов (по кратчайшей дуге около ±180)
  float d = e - eF;
  if (d > 180) d -= 360; else if (d < -180) d += 360;
  eF += d * dt / (E_TAU + dt);
  if (eF > 180) eF -= 360; else if (eF < -180) eF += 360;

  float dd = eF - prevEF;
  if (dd > 180) dd -= 360; else if (dd < -180) dd += 360;
  prevEF = eF;
  dEF += (dd - dEF * dt) / (D_TAU + dt);          // сглаженная dE/dt

  // ---------- гистерезисы ----------
  if (fabsf(eF) > E_OUT) moving = true; else if (fabsf(eF) <= E_IN) moving = false;
  if (abs(ball_dir) > BEHIND_IN) ballBehind = true; else if (abs(ball_dir) < BEHIND_OUT) ballBehind = false;
  if (abs(ball_dir) > BACK_IN) ballBack = true; else if (abs(ball_dir) < BACK_OUT) ballBack = false;
  if (fabsf(eF) <= SIDE_HOLD && eF != 0) lastSide = (eF > 0) ? 1 : -1;

  // ---------- 1. желаемая боковая скорость ----------
  // Профиль: далеко sqrt (как торможение с постоянным замедлением - едем быстро),
  // у цели линейный: у sqrt около нуля усиление бесконечное, и с задержкой камеры
  // робот бы перелетал. KD сбрасывает скорость заранее,
  // пока робот ещё подъезжает к линии мяч-ворота (камера запаздывает).
  float vLat = 0;                                 // со знаком: > 0 - в сторону +base
  float mag  = fabsf(eF) - E_IN;
  if (moving && !ballBehind && mag > 0 && camFresh()) {   // камера молчит - стоим, а не едем по старым данным
    tmrDefA = millis();
    float p = fminf(KL * mag, KS * sqrtf(mag)) * lastSide;
    vLat = p + KD * dEF;
    if (vLat * p < 0) vLat = 0;                   // D только тормозит, назад не разворачивает
    // мяч сзади: ошибка огромная, без потолка робот на полной скорости уезжал назад/вбок
    float vMax = ballBack ? BACK_SPEED_MAX : SPEED_MAX;
    vLat = DIR_SIGN * constrain(vLat, -vMax, vMax);
  }

  // плавно меняем "боковой" курс 80° <-> 135°
  base += ((farFromLine ? 135 : 80) - base) * dt / (BASE_TAU + dt);
  float b  = base * DEG_TO_RAD;
  float tF = fabsf(vLat) * cosf(b);               // вперёд/назад
  float tS = vLat * sinf(b);                      // вбок

  // ---------- 1б. рывок с ударом ----------
  // Мяч в области ворот 1 (не дальше DASH_GATE от направления на них; ворота - в передней полусфере) -
  // тупо едем на мяч и бьём. Начатый рывок выполняется гарантированно: не прерывается, пока не будет удара,
  // а удар - как только мяч у пеналки, но не позже KICK_LATEST_MS. Стартуем, только когда пеналка заряжена
  // (kickFlag), иначе удар мог бы не случиться. Съезд с линии по датчику ниже всё равно главнее (безопасность).
  if (!dash && kickFlag && (int32_t)(nowMs - dashReadyAt) >= 0 && camFresh() && !line_be &&
      abs(gate_dir_1) < 90 && abs(norm180(ball_dir - gate_dir_1)) <= DASH_GATE && ball_dist <= DASH_DIST) {
    dash = true; kicked = false; dashStart = nowMs;
  }
  if (dash) {
    if (!kicked && (trig || ball_dist <= KICK_DIST || nowMs - dashStart >= KICK_LATEST_MS)) {
      kicker();                                   // пеналка заряжена (проверили на старте) - удар будет
      kicked = true; kickAt = millis();           // millis(), а не nowMs: kicker() держит импульс KICK_PULSE_MS
    }
    if (kicked && millis() - kickAt >= DASH_FOLLOW_MS) {
      dash = false;                               // рывок закончен - возвращаемся назад к линии
      dashReadyAt = millis() + DASH_COOL_MS;
      returnUntil = millis() + RETURN_MAX_MS;
    }
  }
  if (dash) {
    float a = ball_dir * DEG_TO_RAD;              // тупо на мяч
    tF = DASH_SPEED * cosf(a);
    tS = DASH_SPEED * sinf(a);
  }
  if (cam_line_dir > -1 || line_be || ballBack) returnUntil = nowMs;   // линия видна / мяч сзади - возврат не нужен
  if (!dash && (int32_t)(returnUntil - nowMs) > 0 && tF > -RETURN_SPEED) tF = -RETURN_SPEED;

  // ---------- 2. реакция на линию ----------
  // Линия не заменяет боковое движение, а только срезает его составляющую "в линию":
  // робот скользит вдоль линии с полной боковой скоростью, а не скачет вперёд-назад.
  // line_dir и cam_line_dir - направление ОТ линии.
  bool onLine = line_be;
  if (onLine || cam_line_dir > -1) {
    float a    = (onLine ? line_dir : cam_line_dir) * DEG_TO_RAD;
    float nF   = cosf(a), nS = sinf(a);
    float away = tF * nF + tS * nS;               // < 0 - едем в линию
    if (away < 0) { tF -= away * nF; tS -= away * nS; away = 0; }
    if (onLine && away < DEF_LINE_ESCAPE_SPEED) { // на линии ещё и отталкиваемся, даже если стоим
      tF += (DEF_LINE_ESCAPE_SPEED - away) * nF;
      tS += (DEF_LINE_ESCAPE_SPEED - away) * nS;
    }
  }

  // ---------- 3. сглаживание ВЕКТОРА скорости ----------
  // Сглаживаем (vF, vS), а не угол: смена стороны - это плавный проход vS через 0,
  // а не разворот вектора через "вперёд" с выездом из ворот и срезом скорости.
  if (onLine) {
    vF = tF; vS = tS;                             // у линии рывок без ограничения разгона
  } else {
    float dF = tF - vF, dS = tS - vS;
    float dist = hypotf(dF, dS);
    float step = (dash ? DASH_ACC : (hypotf(tF, tS) > hypotf(vF, vS) ? ACC : DECEL)) * dt;
    if (dist > step) { vF += dF * step / dist; vS += dS * step / dist; }
    else             { vF = tF; vS = tS; }
  }
  float dir = atan2f(vS, vF) * RAD_TO_DEG;
  speedForDef = (int)lroundf(hypotf(vF, vS));
  if (speedForDef > 0) degreesForDef = (int)lroundf(dir);

  // ---------- курс ----------
  // нет свежего курса - не крутимся; мёртвая зона плавная: rot растёт от 0, а не прыгает 0 -> 60
  int rot = 0;
  float h = fabsf(imU) - ROT_DEAD;
  if (imuFresh() && h > 0) rot = (int)constrain(-ROT_K * h * (imU > 0 ? 1 : -1), -ROT_MAX, ROT_MAX);
  // на рывке разворачиваем пеналку на мяч (мяч в области ворот - значит, заодно и на ворота);
  // после рывка удержание курса по BNO само вернёт робота лицом вперёд
  if (dash) rot = (int)constrain(ball_dir * DASH_ROT_K, -DASH_ROT_MAX, DASH_ROT_MAX);

  // ---------- отладка и движение ----------
  // печать не чаще SERIAL_INTERVAL: Serial на каждом вызове тормозит цикл управления
  if (millis() - lastPrintMs >= SERIAL_INTERVAL) {
    lastPrintMs = millis();
    char buffer[80];
    // bk - мяч сзади (скорость ограничена), dash - идёт рывок, bg - угол мяча от ворот 1 (рывок при |bg| <= DASH_GATE)
    snprintf(buffer, sizeof(buffer), "def = %d, e = %d, spd = %d, bk = %d, dash = %d, bg = %d",
             degreesForDef, (int)eF, speedForDef, (int)ballBack, (int)dash, norm180(ball_dir - gate_dir_1));
    Serial.println(buffer);
  }
  // instant = true: разгон уже ограничен выше, рампа моторов добавила бы вторую задержку
  omniMoveDirection(speedForDef, dir, rot, true);
}

void DefenseMode_save004() {
  // ---------- параметры ----------
  const int   DIR_SIGN = 1;                       // -1, если защита уезжает ОТ линии мяч-ворота, а не к ней
  const float SPEED_MAX = 6000;                   // потолок скорости (на колесо это <= 3000 + rot, запас до MAX_MOTOR_SPEED есть)
  const float KS = 750.0f;                        // дальняя зона: скорость = KS*sqrt(ошибка): 10° -> ~2000, 20° -> ~3100, 40° -> ~4550
  const float KL = 300.0f;                        // ближняя зона: скорость = KL*ошибка (до ~6° сверх E_IN), у цели без бесконечного усиления sqrt
  const float KD = 8.0f;                          // ед. скорости на 1°/с изменения ошибки: тормозит заранее, гасит перелёт
  const float E_IN = 3.0f, E_OUT = 5.0f;          // гистерезис: |e| <= E_IN - встали, > E_OUT - поехали
  const float E_TAU = 0.02f, D_TAU = 0.05f;       // фильтры ошибки и её производной, с
  const float ACC = 24000, DECEL = 32000;         // разгон/торможение вектора скорости, ед/с
  const float BASE_TAU = 0.15f;                   // плавная смена курса 80° <-> 135°, с
  const int   BEHIND_IN = 170, BEHIND_OUT = 155;  // мяч строго сзади -> стоим
  const int   BACK_IN = 110, BACK_OUT = 95;       // мяч сзади-сбоку -> едем, но не быстрее BACK_SPEED_MAX
  const float BACK_SPEED_MAX = 1500;              // потолок скорости, пока мяч сзади (раньше до 6000 - назад уезжал слишком быстро)
  const int   SIDE_HOLD = 150;                    // |e| больше: сторону не переключаем
  const float ROT_K = 20, ROT_MAX = 600, ROT_DEAD = 1;   // стабилизация курса
  // рывок с ударом: мяч спереди, ворота 1 (соперника) прямо по курсу
  const int   DASH_BALL = BALL_FRONT_ANGLE;       // |ball_dir| меньше этого - мяч спереди (15°)
  const int   DASH_GATE = 5;                      // |gate_dir_1| не больше этого - ворота 1 прямо по курсу
  const int   DASH_DIST = 35;                     // ball_dist не больше этого (в атаке "рядом" - 20; 100 - без ограничения)
  const int   KICK_DIST = 12;                     // ball_dist не больше этого (или trig) - мяч у пеналки, бьём
  const float DASH_SPEED = 5500;                  // скорость рывка
  const float DASH_ACC = 40000;                   // разгон на рывке, ед/с (обычный ACC 24000)
  const uint32_t DASH_MS = 350, DASH_COOL_MS = 800;   // длительность рывка и пауза между рывками, мс
  const float RETURN_SPEED = 1500;                // после рывка сдаём назад с такой скоростью, пока камера не увидит линию
  const uint32_t RETURN_MAX_MS = 2000;            // но не дольше этого

  // ---------- состояние между вызовами ----------
  static unsigned long lastUs = 0, lastPrintMs = 0;
  static int   lastSide = 1;
  static bool  moving = false, ballBehind = false, ballBack = false;
  static float eF = 0, prevEF = 0, dEF = 0;       // отфильтрованная ошибка и её производная, °/с
  static float vF = 0, vS = 0;                    // текущий вектор скорости: вперёд (cos) и вбок (sin)
  static float base = 80;                         // текущий "боковой" курс, °
  static uint32_t dashUntil = 0, dashReadyAt = 0, returnUntil = 0;
  static bool  wasDash = false;

  // dt по micros(): цикл бывает короче 1 мс, по millis() dt скакал бы 0/1/2
  unsigned long nowUs = micros();
  float dt = (nowUs - lastUs) * 1e-6f;
  lastUs = nowUs;
  bool fresh = dt > 0.1f;                         // давно не вызывали: начинаем заново
  if (dt > 0.05f)   dt = 0.05f;
  if (dt < 0.0001f) dt = 0.0001f;
  uint32_t nowMs = millis();

  // ---------- входные величины ----------
  float imU        = imuData.euler[0];
  if (imU > 180) imU -= 360;
  int e            = norm180(ball_dir - 180 - gate_dir_2);   // со знаком, с переходом через ±180
  bool farFromLine = (cam_line_dir < 0) && abs(gate_dir_2) > 135;

  if (fresh) {
    eF = e; prevEF = e; dEF = 0; vF = vS = 0; base = farFromLine ? 135 : 80;
    dashUntil = returnUntil = nowMs; wasDash = false;
  }

  // фильтр ошибки по времени, а не по числу вызовов (по кратчайшей дуге около ±180)
  float d = e - eF;
  if (d > 180) d -= 360; else if (d < -180) d += 360;
  eF += d * dt / (E_TAU + dt);
  if (eF > 180) eF -= 360; else if (eF < -180) eF += 360;

  float dd = eF - prevEF;
  if (dd > 180) dd -= 360; else if (dd < -180) dd += 360;
  prevEF = eF;
  dEF += (dd - dEF * dt) / (D_TAU + dt);          // сглаженная dE/dt

  // ---------- гистерезисы ----------
  if (fabsf(eF) > E_OUT) moving = true; else if (fabsf(eF) <= E_IN) moving = false;
  if (abs(ball_dir) > BEHIND_IN) ballBehind = true; else if (abs(ball_dir) < BEHIND_OUT) ballBehind = false;
  if (abs(ball_dir) > BACK_IN) ballBack = true; else if (abs(ball_dir) < BACK_OUT) ballBack = false;
  if (fabsf(eF) <= SIDE_HOLD && eF != 0) lastSide = (eF > 0) ? 1 : -1;

  // ---------- 1. желаемая боковая скорость ----------
  // Профиль: далеко sqrt (как торможение с постоянным замедлением - едем быстро),
  // у цели линейный: у sqrt около нуля усиление бесконечное, и с задержкой камеры
  // робот бы перелетал. KD сбрасывает скорость заранее,
  // пока робот ещё подъезжает к линии мяч-ворота (камера запаздывает).
  float vLat = 0;                                 // со знаком: > 0 - в сторону +base
  float mag  = fabsf(eF) - E_IN;
  if (moving && !ballBehind && mag > 0 && camFresh()) {   // камера молчит - стоим, а не едем по старым данным
    tmrDefA = millis();
    float p = fminf(KL * mag, KS * sqrtf(mag)) * lastSide;
    vLat = p + KD * dEF;
    if (vLat * p < 0) vLat = 0;                   // D только тормозит, назад не разворачивает
    // мяч сзади: ошибка огромная, без потолка робот на полной скорости уезжал назад/вбок
    float vMax = ballBack ? BACK_SPEED_MAX : SPEED_MAX;
    vLat = DIR_SIGN * constrain(vLat, -vMax, vMax);
  }

  // плавно меняем "боковой" курс 80° <-> 135°
  base += ((farFromLine ? 135 : 80) - base) * dt / (BASE_TAU + dt);
  float b  = base * DEG_TO_RAD;
  float tF = fabsf(vLat) * cosf(b);               // вперёд/назад
  float tS = vLat * sinf(b);                      // вбок

  // ---------- 1б. рывок с ударом ----------
  // Мяч спереди, ворота 1 прямо по курсу (±DASH_GATE) - резко едем на мяч и бьём, когда он у пеналки.
  // Рывок длится DASH_MS, следующий - не раньше чем через DASH_COOL_MS; мяч ушёл из "спереди" - прекращаем.
  bool dash = (int32_t)(dashUntil - nowMs) > 0;
  if (dash && (!camFresh() || abs(ball_dir) > 3 * DASH_BALL)) { dash = false; dashUntil = nowMs; }
  if (!dash && (int32_t)(nowMs - dashReadyAt) >= 0 && camFresh() && !line_be &&
      abs(ball_dir) < DASH_BALL && abs(gate_dir_1) <= DASH_GATE && ball_dist <= DASH_DIST) {
    dash = true;
    dashUntil = nowMs + DASH_MS;
    dashReadyAt = dashUntil + DASH_COOL_MS;
  }
  if (dash) {
    float a = ball_dir * DEG_TO_RAD;
    tF = DASH_SPEED * cosf(a);
    tS = DASH_SPEED * sinf(a);
    if (abs(ball_dir) < DASH_BALL && (trig || ball_dist <= KICK_DIST)) kicker();   // пеналка сама не бьёт чаще KICK_COOLDOWN_MS
  }
  // рывок кончился - возвращаемся назад к линии (иначе, выровнявшись по мячу, так и остались бы в поле)
  if (wasDash && !dash) returnUntil = nowMs + RETURN_MAX_MS;
  wasDash = dash;
  if (cam_line_dir > -1 || line_be || ballBack) returnUntil = nowMs;   // линия видна / мяч сзади - возврат не нужен
  if (!dash && (int32_t)(returnUntil - nowMs) > 0 && tF > -RETURN_SPEED) tF = -RETURN_SPEED;

  // ---------- 2. реакция на линию ----------
  // Линия не заменяет боковое движение, а только срезает его составляющую "в линию":
  // робот скользит вдоль линии с полной боковой скоростью, а не скачет вперёд-назад.
  // line_dir и cam_line_dir - направление ОТ линии.
  bool onLine = line_be;
  if (onLine || cam_line_dir > -1) {
    float a    = (onLine ? line_dir : cam_line_dir) * DEG_TO_RAD;
    float nF   = cosf(a), nS = sinf(a);
    float away = tF * nF + tS * nS;               // < 0 - едем в линию
    if (away < 0) { tF -= away * nF; tS -= away * nS; away = 0; }
    if (onLine && away < DEF_LINE_ESCAPE_SPEED) { // на линии ещё и отталкиваемся, даже если стоим
      tF += (DEF_LINE_ESCAPE_SPEED - away) * nF;
      tS += (DEF_LINE_ESCAPE_SPEED - away) * nS;
    }
  }

  // ---------- 3. сглаживание ВЕКТОРА скорости ----------
  // Сглаживаем (vF, vS), а не угол: смена стороны - это плавный проход vS через 0,
  // а не разворот вектора через "вперёд" с выездом из ворот и срезом скорости.
  if (onLine) {
    vF = tF; vS = tS;                             // у линии рывок без ограничения разгона
  } else {
    float dF = tF - vF, dS = tS - vS;
    float dist = hypotf(dF, dS);
    float step = (dash ? DASH_ACC : (hypotf(tF, tS) > hypotf(vF, vS) ? ACC : DECEL)) * dt;
    if (dist > step) { vF += dF * step / dist; vS += dS * step / dist; }
    else             { vF = tF; vS = tS; }
  }
  float dir = atan2f(vS, vF) * RAD_TO_DEG;
  speedForDef = (int)lroundf(hypotf(vF, vS));
  if (speedForDef > 0) degreesForDef = (int)lroundf(dir);

  // ---------- курс ----------
  // нет свежего курса - не крутимся; мёртвая зона плавная: rot растёт от 0, а не прыгает 0 -> 60
  int rot = 0;
  float h = fabsf(imU) - ROT_DEAD;
  if (imuFresh() && h > 0) rot = (int)constrain(-ROT_K * h * (imU > 0 ? 1 : -1), -ROT_MAX, ROT_MAX);

  // ---------- отладка и движение ----------
  // печать не чаще SERIAL_INTERVAL: Serial на каждом вызове тормозит цикл управления
  if (millis() - lastPrintMs >= SERIAL_INTERVAL) {
    lastPrintMs = millis();
    char buffer[80];
    // bk - мяч сзади (скорость ограничена), dash - идёт рывок
    snprintf(buffer, sizeof(buffer), "def = %d, e = %d, spd = %d, bk = %d, dash = %d",
             degreesForDef, (int)eF, speedForDef, (int)ballBack, (int)dash);
    Serial.println(buffer);
  }
  // instant = true: разгон уже ограничен выше, рампа моторов добавила бы вторую задержку
  omniMoveDirection(speedForDef, dir, rot, true);
}

void DefenseMode_save003() {
  // ---------- параметры ----------
  const int   DIR_SIGN = 1;                       // -1, если защита уезжает ОТ линии мяч-ворота, а не к ней
  const float SPEED_MAX = 6000;                   // потолок скорости (на колесо это <= 3000 + rot, запас до MAX_MOTOR_SPEED есть)
  const float KS = 750.0f;                        // дальняя зона: скорость = KS*sqrt(ошибка): 10° -> ~2000, 20° -> ~3100, 40° -> ~4550
  const float KL = 300.0f;                        // ближняя зона: скорость = KL*ошибка (до ~6° сверх E_IN), у цели без бесконечного усиления sqrt
  const float KD = 8.0f;                          // ед. скорости на 1°/с изменения ошибки: тормозит заранее, гасит перелёт
  const float E_IN = 3.0f, E_OUT = 5.0f;          // гистерезис: |e| <= E_IN - встали, > E_OUT - поехали
  const float E_TAU = 0.02f, D_TAU = 0.05f;       // фильтры ошибки и её производной, с
  const float ACC = 24000, DECEL = 32000;         // разгон/торможение вектора скорости, ед/с
  const float BASE_TAU = 0.15f;                   // плавная смена курса 80° <-> 135°, с
  const int   BEHIND_IN = 170, BEHIND_OUT = 155;  // мяч сзади -> стоим
  const int   SIDE_HOLD = 150;                    // |e| больше: сторону не переключаем
  const float ROT_K = 20, ROT_MAX = 600, ROT_DEAD = 1;   // стабилизация курса

  // ---------- состояние между вызовами ----------
  static unsigned long lastUs = 0, lastPrintMs = 0;
  static int   lastSide = 1;
  static bool  moving = false, ballBehind = false;
  static float eF = 0, prevEF = 0, dEF = 0;       // отфильтрованная ошибка и её производная, °/с
  static float vF = 0, vS = 0;                    // текущий вектор скорости: вперёд (cos) и вбок (sin)
  static float base = 80;                         // текущий "боковой" курс, °

  // dt по micros(): цикл бывает короче 1 мс, по millis() dt скакал бы 0/1/2
  unsigned long nowUs = micros();
  float dt = (nowUs - lastUs) * 1e-6f;
  lastUs = nowUs;
  bool fresh = dt > 0.1f;                         // давно не вызывали: начинаем заново
  if (dt > 0.05f)   dt = 0.05f;
  if (dt < 0.0001f) dt = 0.0001f;

  // ---------- входные величины ----------
  float imU        = imuData.euler[0];
  if (imU > 180) imU -= 360;
  int e            = norm180(ball_dir - 180 - gate_dir_2);   // со знаком, с переходом через ±180
  bool farFromLine = (cam_line_dir < 0) && abs(gate_dir_2) > 135;

  if (fresh) { eF = e; prevEF = e; dEF = 0; vF = vS = 0; base = farFromLine ? 135 : 80; }

  // фильтр ошибки по времени, а не по числу вызовов (по кратчайшей дуге около ±180)
  float d = e - eF;
  if (d > 180) d -= 360; else if (d < -180) d += 360;
  eF += d * dt / (E_TAU + dt);
  if (eF > 180) eF -= 360; else if (eF < -180) eF += 360;

  float dd = eF - prevEF;
  if (dd > 180) dd -= 360; else if (dd < -180) dd += 360;
  prevEF = eF;
  dEF += (dd - dEF * dt) / (D_TAU + dt);          // сглаженная dE/dt

  // ---------- гистерезисы ----------
  if (fabsf(eF) > E_OUT) moving = true; else if (fabsf(eF) <= E_IN) moving = false;
  if (abs(ball_dir) > BEHIND_IN) ballBehind = true; else if (abs(ball_dir) < BEHIND_OUT) ballBehind = false;
  if (fabsf(eF) <= SIDE_HOLD && eF != 0) lastSide = (eF > 0) ? 1 : -1;

  // ---------- 1. желаемая боковая скорость ----------
  // Профиль: далеко sqrt (как торможение с постоянным замедлением - едем быстро),
  // у цели линейный: у sqrt около нуля усиление бесконечное, и с задержкой камеры
  // робот бы перелетал. KD сбрасывает скорость заранее,
  // пока робот ещё подъезжает к линии мяч-ворота (камера запаздывает).
  float vLat = 0;                                 // со знаком: > 0 - в сторону +base
  float mag  = fabsf(eF) - E_IN;
  if (moving && !ballBehind && mag > 0 && camFresh()) {   // камера молчит - стоим, а не едем по старым данным
    tmrDefA = millis();
    float p = fminf(KL * mag, KS * sqrtf(mag)) * lastSide;
    vLat = p + KD * dEF;
    if (vLat * p < 0) vLat = 0;                   // D только тормозит, назад не разворачивает
    vLat = DIR_SIGN * constrain(vLat, -SPEED_MAX, SPEED_MAX);
  }

  // плавно меняем "боковой" курс 80° <-> 135°
  base += ((farFromLine ? 135 : 80) - base) * dt / (BASE_TAU + dt);
  float b  = base * DEG_TO_RAD;
  float tF = fabsf(vLat) * cosf(b);               // вперёд/назад
  float tS = vLat * sinf(b);                      // вбок

  // ---------- 2. реакция на линию ----------
  // Линия не заменяет боковое движение, а только срезает его составляющую "в линию":
  // робот скользит вдоль линии с полной боковой скоростью, а не скачет вперёд-назад.
  // line_dir и cam_line_dir - направление ОТ линии.
  bool onLine = line_be;
  if (onLine || cam_line_dir > -1) {
    float a    = (onLine ? line_dir : cam_line_dir) * DEG_TO_RAD;
    float nF   = cosf(a), nS = sinf(a);
    float away = tF * nF + tS * nS;               // < 0 - едем в линию
    if (away < 0) { tF -= away * nF; tS -= away * nS; away = 0; }
    if (onLine && away < DEF_LINE_ESCAPE_SPEED) { // на линии ещё и отталкиваемся, даже если стоим
      tF += (DEF_LINE_ESCAPE_SPEED - away) * nF;
      tS += (DEF_LINE_ESCAPE_SPEED - away) * nS;
    }
  }

  // ---------- 3. сглаживание ВЕКТОРА скорости ----------
  // Сглаживаем (vF, vS), а не угол: смена стороны - это плавный проход vS через 0,
  // а не разворот вектора через "вперёд" с выездом из ворот и срезом скорости.
  if (onLine) {
    vF = tF; vS = tS;                             // у линии рывок без ограничения разгона
  } else {
    float dF = tF - vF, dS = tS - vS;
    float dist = hypotf(dF, dS);
    float step = (hypotf(tF, tS) > hypotf(vF, vS) ? ACC : DECEL) * dt;
    if (dist > step) { vF += dF * step / dist; vS += dS * step / dist; }
    else             { vF = tF; vS = tS; }
  }
  float dir = atan2f(vS, vF) * RAD_TO_DEG;
  speedForDef = (int)lroundf(hypotf(vF, vS));
  if (speedForDef > 0) degreesForDef = (int)lroundf(dir);

  // ---------- курс ----------
  // нет свежего курса - не крутимся; мёртвая зона плавная: rot растёт от 0, а не прыгает 0 -> 60
  int rot = 0;
  float h = fabsf(imU) - ROT_DEAD;
  if (imuFresh() && h > 0) rot = (int)constrain(-ROT_K * h * (imU > 0 ? 1 : -1), -ROT_MAX, ROT_MAX);

  // ---------- отладка и движение ----------
  // печать не чаще SERIAL_INTERVAL: Serial на каждом вызове тормозит цикл управления
  if (millis() - lastPrintMs >= SERIAL_INTERVAL) {
    lastPrintMs = millis();
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "def = %d, e = %d, spd = %d", degreesForDef, (int)eF, speedForDef);
    Serial.println(buffer);
  }
  // instant = true: разгон уже ограничен выше, рампа моторов добавила бы вторую задержку
  omniMoveDirection(speedForDef, dir, rot, true);
}

void DefenseMode_save002() {
  // ---------- параметры ----------
  const int   DIR_SIGN = 1;                       // -1, если защита уезжает ОТ линии мяч-ворота, а не к ней
  const float SPEED_MAX = 5000;                   // потолок скорости
  const float KS = 420.0f;                        // скорость = KS*sqrt(ошибка): 10° -> ~1100, 40° -> ~2550
  const float KD = 8.0f;                          // ед. скорости на 1°/с изменения ошибки: тормозит заранее, гасит перелёт
  const float E_IN = 3.0f, E_OUT = 5.0f;          // гистерезис: |e| <= E_IN - встали, > E_OUT - поехали
  const float E_TAU = 0.03f, D_TAU = 0.05f;       // фильтры ошибки и её производной, с
  const float ACC = 16000, DECEL = 26000;         // разгон/торможение вектора скорости, ед/с
  const float BASE_TAU = 0.15f;                   // плавная смена курса 80° <-> 135°, с
  const int   BEHIND_IN = 170, BEHIND_OUT = 155;  // мяч сзади -> стоим
  const int   SIDE_HOLD = 150;                    // |e| больше: сторону не переключаем
  const float ROT_K = 20, ROT_MAX = 600, ROT_DEAD = 1;   // стабилизация курса

  // ---------- состояние между вызовами ----------
  static unsigned long lastUs = 0, lastPrintMs = 0;
  static int   lastSide = 1;
  static bool  moving = false, ballBehind = false;
  static float eF = 0, prevEF = 0, dEF = 0;       // отфильтрованная ошибка и её производная, °/с
  static float vF = 0, vS = 0;                    // текущий вектор скорости: вперёд (cos) и вбок (sin)
  static float base = 80;                         // текущий "боковой" курс, °

  // dt по micros(): цикл бывает короче 1 мс, по millis() dt скакал бы 0/1/2
  unsigned long nowUs = micros();
  float dt = (nowUs - lastUs) * 1e-6f;
  lastUs = nowUs;
  bool fresh = dt > 0.1f;                         // давно не вызывали: начинаем заново
  if (dt > 0.05f)   dt = 0.05f;
  if (dt < 0.0001f) dt = 0.0001f;

  // ---------- входные величины ----------
  float imU        = imuData.euler[0];
  if (imU > 180) imU -= 360;
  int angleForLine = norm180(line_dir);
  int e            = norm180(ball_dir - 180 - gate_dir_2);   // со знаком, с переходом через ±180
  bool farFromLine = (cam_line_dir < 0) && abs(gate_dir_2) > 135;

  if (fresh) { eF = e; prevEF = e; dEF = 0; vF = vS = 0; base = farFromLine ? 135 : 80; }

  // фильтр ошибки по времени, а не по числу вызовов (по кратчайшей дуге около ±180)
  float d = e - eF;
  if (d > 180) d -= 360; else if (d < -180) d += 360;
  eF += d * dt / (E_TAU + dt);
  if (eF > 180) eF -= 360; else if (eF < -180) eF += 360;

  float dd = eF - prevEF;
  if (dd > 180) dd -= 360; else if (dd < -180) dd += 360;
  prevEF = eF;
  dEF += (dd - dEF * dt) / (D_TAU + dt);          // сглаженная dE/dt

  // ---------- гистерезисы ----------
  if (fabsf(eF) > E_OUT) moving = true; else if (fabsf(eF) <= E_IN) moving = false;
  if (abs(ball_dir) > BEHIND_IN) ballBehind = true; else if (abs(ball_dir) < BEHIND_OUT) ballBehind = false;
  if (fabsf(eF) <= SIDE_HOLD && eF != 0) lastSide = (eF > 0) ? 1 : -1;

  // ---------- 1. желаемая боковая скорость ----------
  // Профиль sqrt - как торможение с постоянным замедлением: далеко едем быстро,
  // у цели не ползём и не стартуем рывком с 900. KD сбрасывает скорость заранее,
  // пока робот ещё подъезжает к линии мяч-ворота (камера запаздывает).
  float vLat = 0;                                 // со знаком: > 0 - в сторону +base
  float mag  = fabsf(eF) - E_IN;
  if (moving && !ballBehind && mag > 0 && camFresh()) {   // камера молчит - стоим, а не едем по старым данным
    tmrDefA = millis();
    float p = KS * sqrtf(mag) * lastSide;
    vLat = p + KD * dEF;
    if (vLat * p < 0) vLat = 0;                   // D только тормозит, назад не разворачивает
    vLat = DIR_SIGN * constrain(vLat, -SPEED_MAX, SPEED_MAX);
  }

  // плавно меняем "боковой" курс 80° <-> 135°
  base += ((farFromLine ? 135 : 80) - base) * dt / (BASE_TAU + dt);
  float b  = base * DEG_TO_RAD;
  float tF = fabsf(vLat) * cosf(b);               // вперёд/назад
  float tS = vLat * sinf(b);                      // вбок

  // ---------- 2. реакция на линию ----------
  bool onLine = line_be;
  if (onLine) {
    float a = line_dir * DEG_TO_RAD;
    float s = fmaxf(DEF_LINE_ESCAPE_SPEED, fabsf(vLat));   // съезжаем с линии, даже если стоим
    tF = s * cosf(a);
    tS = s * sinf(a);
  } else if (cam_line_dir > -1 && vLat != 0) {
    int dir = (int)lroundf(atan2f(tS, tF) * RAD_TO_DEG);
    if (abs(angleForLine) > 80 && abs(angleForLine) < 100) {
      dir = cam_line_dir;
    } else if (fabsf(eF) < 170) {
      dir = VectorSum(cam_line_dir, dir, 1, 1);
    }
    tF = fabsf(vLat) * cosf(dir * DEG_TO_RAD);
    tS = fabsf(vLat) * sinf(dir * DEG_TO_RAD);
  }

  // ---------- 3. сглаживание ВЕКТОРА скорости ----------
  // Сглаживаем (vF, vS), а не угол: смена стороны - это плавный проход vS через 0,
  // а не разворот вектора через "вперёд" с выездом из ворот и срезом скорости.
  if (onLine) {
    vF = tF; vS = tS;                             // у линии рывок без ограничения разгона
  } else {
    float dF = tF - vF, dS = tS - vS;
    float dist = hypotf(dF, dS);
    float step = (hypotf(tF, tS) > hypotf(vF, vS) ? ACC : DECEL) * dt;
    if (dist > step) { vF += dF * step / dist; vS += dS * step / dist; }
    else             { vF = tF; vS = tS; }
  }
  float dir = atan2f(vS, vF) * RAD_TO_DEG;
  speedForDef = (int)lroundf(hypotf(vF, vS));
  if (speedForDef > 0) degreesForDef = (int)lroundf(dir);

  // ---------- курс ----------
  // нет свежего курса - не крутимся; мёртвая зона плавная: rot растёт от 0, а не прыгает 0 -> 60
  int rot = 0;
  float h = fabsf(imU) - ROT_DEAD;
  if (imuFresh() && h > 0) rot = (int)constrain(-ROT_K * h * (imU > 0 ? 1 : -1), -ROT_MAX, ROT_MAX);

  // ---------- отладка и движение ----------
  // печать не чаще SERIAL_INTERVAL: Serial на каждом вызове тормозит цикл управления
  if (millis() - lastPrintMs >= SERIAL_INTERVAL) {
    lastPrintMs = millis();
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "def = %d, e = %d, spd = %d", degreesForDef, (int)eF, speedForDef);
    Serial.println(buffer);
  }
  // instant = true: разгон уже ограничен выше, рампа моторов добавила бы вторую задержку
  omniMoveDirection(speedForDef, dir, rot, true);
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
      degreesForDef = VectorSum(cam_line_dir, degreesForDef, 1, 1);
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
  rot_speed = gate_dir_1 * 10;
  Serial.println(ball_dir);
  if ((cam_line_dir > -1 || line_be)) {
    if (line_be) {
      omniMoveDirection(2000, line_dir, rot_speed, true);
    } else {
      if (abs(ball_dir) < 8) {
        // Мяч перед роботом и не захвачен - едем прямо, форсируем скорость при простое accel_via_distance() + stallBoost()
        omniMoveDirection(1000, VectorSum(cam_line_dir, ball_dir, 1.5, 1), rot_speed);
        if (abs(gate_dir_1) < 10) {
          kicker();
        }
      } else {
        if (ball_dist <= 20) {
          resetStall();
          omniMoveDirection(attackSpeed(ball_dir) + accel_via_distance() * 0.2, VectorSum(cam_line_dir, (abs(ball_dir) + 60) * (ball_dir / abs(ball_dir)), 1.5, 1), rot_speed);
        } else {
          omniMoveDirection(1000, VectorSum(cam_line_dir, ball_dir, 1.5, 1), rot_speed);
        }
      }
      //omniMoveDirection(1000, VectorSum(cam_line_dir, ball_dir, 1.5, 1), rot_speed, true);
    }
    return;
  }


  if (!trig) {
    digitalWrite(LED_BUILTIN, 0);
    if (abs(ball_dir) < 8) {
      // Мяч перед роботом и не захвачен - едем прямо, форсируем скорость при простое accel_via_distance() + stallBoost()
      omniMoveDirection(5500, ball_dir, rot_speed);
      if (abs(gate_dir_1) < 10) {
        kicker();
      }
    } else {
      // Мяч не перед роботом - объезжаем, скорость растёт с углом
      if (ball_dist <= 20) {
        resetStall();
        omniMoveDirection(attackSpeed(ball_dir) + accel_via_distance(), (abs(ball_dir) + 60) * (ball_dir / abs(ball_dir)), rot_speed);
      } else {
        omniMoveDirection(4500, ball_dir, rot_speed);
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

void AtackMode_save002() {
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
      // Мяч перед роботом и не захвачен - едем прямо, форсируем скорость при простое accel_via_distance() + stallBoost()
      omniMoveDirection(5500, ball_dir, rot_speed);
      if (abs(gate_dir_1) < 15) {
        kicker();
      }
    } else {
      // Мяч не перед роботом - объезжаем, скорость растёт с углом
      if (ball_dist <= 20) {
        resetStall();
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