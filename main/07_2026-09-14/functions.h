// ============================================================================
//  Обработка датчиков
// ============================================================================

bool trig;

// Печатает все значения, полученные с камеры по I2C, с подписями.
// Вызов ограничен по времени константой SERIAL_INTERVAL, чтобы не забивать Serial.
uint32_t lastCamDebugPrint = 0;

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

void cam_read(){
  uint8_t buffer[12];
  size_t bytesRead = Wire1.requestFrom(CAM_ADDR, 12);

  if (bytesRead == 12) {
    for (size_t i = 0; i < 12; i++) {
      buffer[i] = Wire1.read();
    }

    // Распаковка углов (little-endian)
    ball_dir = buffer[0] | (buffer[1] << 8);
    //ball_dir += 18;
    gate_dir_1 = buffer[2] | (buffer[3] << 8);
    gate_dir_2 = buffer[4] | (buffer[5] << 8);
    cam_line_dir = buffer[6] | (buffer[7] << 8);
    ball_dist = buffer[8] | (buffer[9] << 8);
    trig = buffer[10] | (buffer[11] << 8);

    printCamData();
  } else {
    Serial.println("CAM: error read");
  }
}

bool ball_detect(){
  return digitalRead(5);
}

// ============================================================================
//  Управление моторами
// ============================================================================

struct Motor {
    int32_t target;
    uint32_t lastCmd;
    float current = 0;     // текущая точка разгона (плавно догоняет desired)
    uint32_t lastRamp = 0;
    bool needsUpdate(int32_t n) { return n != target || millis() - lastCmd > MOTOR_DATA_SEND_TIMEOUT; }
} motors[4];

// true - разгон работает (рампа), false - скорость выставляется мгновенно.
// Переключать в рантайме: motorAccelEnabled = false;
bool motorAccelEnabled = true;

bool sendMotor(uint8_t i, uint8_t cmd, uint8_t* data = nullptr) {
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

// Плавно подгоняет скорость мотора i к желаемой (desired) с постоянным
// ускорением MOTOR_ACCEL_PER_SEC (см. config.h) вместо мгновенного скачка.
// Считает время сама по motors[i].lastRamp. Если motorAccelEnabled == false -
// рампа отключена, скорость выставляется сразу желаемой.
void setMotorSpeed(uint8_t i, int32_t desired) {
  desired = constrain(desired, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);

  uint32_t now = millis();
  uint32_t dt = now - motors[i].lastRamp;
  if (dt > 50) dt = 50; // защита от гигантского шага после паузы/старта
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

  float directionRad = (direction) * DEG_TO_RAD;

  float Vx = wheelSpeed * cos(directionRad);
  float Vy = wheelSpeed * sin(directionRad);

  int32_t motor_dps[4];

  static constexpr float SIN_A[4] = { -0.5f, 0.5f, 0.5f,  -0.5f };
  static constexpr float COS_A[4] = { 0.8660254f, 0.8660254f, -0.8660254f, -0.8660254f };

  for (int i = 0; i < 4; i++) {
    motor_dps[i] = ((Vx / COS_A[i]) + Vy * SIN_A[i])/2 + w;
    setMotorSpeed(i, motor_dps[i]);
  }
}

// ============================================================================
//  Математика
// ============================================================================

int getBallZone(float angle) {
  int zone = (int)(angle / 30);
  return constrain(zone, 0, 11);
}

float getZoneSpeed(int zone) {
  const float zoneSpeeds[12] = {
    0.25f, 0.4f, 0.45f, 0.55f, 0.75f, 1.0f,
    1.0f, 0.75f, 0.55f, 0.45f, 0.4f, 0.25f
  };
  return zoneSpeeds[zone];
}

int VectorSum(int alfa1, int alfa2, int w1, int w2) {
  double x1 = w1 * cos(alfa1 * DEG_TO_RAD);
  double y1 = w1 * sin(alfa1 * DEG_TO_RAD);
  double x2 = w2 * cos(alfa2 * DEG_TO_RAD);
  double y2 = w2 * sin(alfa2 * DEG_TO_RAD);
  double x3 = x1 + x2;
  double y3 = y1 + y2;
  return (int)round(atan2(y3, x3) * RAD_TO_DEG);
}

int accel_via_distance(){
  return ACCEL_DISTANCE_SPEED_MULTIPLIER * ball_dist;
}

// Скорость объезда мяча: чем дальше мяч по углу от 0 (центра), тем быстрее едем.
int attackSpeed(int dir) {
  int a = constrain(abs(dir), 0, 180);
  return map(a, 0, 180, ATTACK_SPEED_MIN, ATTACK_SPEED_MAX);
}

// "Застряли": мяч почти по центру (|ball_dir| < 15), но дистанция до него
// не уменьшается дольше STALL_TIMEOUT_MS - форсируем скорость, чтобы не
// зависнуть в дребезге на месте.
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

// Вызывать из логики режима, когда нужна попытка выстрела.
// Реально бьёт только если пеналка взведена (прошёл кулдаун с прошлого удара).
void kicker() {
  if (kickFlag) {
    digitalWrite(KICK, 1);
    delay(KICK_PULSE_MS);
    digitalWrite(KICK, 0);
    Serial.println("kick");
    kickFlag = false;
  }
}

// ============================================================================
//  Режимы работы
// ============================================================================

// Не вызывается из loop() в текущей версии (нужна доп. кнопка выбора режима),
// оставлена рабочей и готовой к подключению.
//
// Автомат состояний:
//  PATROL - едем вправо/влево вдоль своих ворот, следим за мячом.
//           У границы зоны (line_be) - разворачиваемся по line_dir.
//  CHASE  - мяч был по центру (|ball_dir| < BALL_FRONT_ANGLE) дольше
//           BALL_FRONT_HOLD_MS подряд: едем на мяч, целимся в ворота
//           соперника по gate_dir_1 и бьём.
//  RETURN - после удара возвращаемся к линии своих ворот (edем "домой"
//           пока не увидим line_be), затем снова патрулируем.
//
// DEF_HOME_DIRECTION предполагает, что 0 градусов - это "вперёд, на ворота
// соперника" (как в AtackMode); если ориентация робота другая - поправьте
// константу в config.h.
enum DefState : uint8_t { DEF_PATROL, DEF_CHASE, DEF_RETURN };
DefState defState = DEF_PATROL;
bool defPatrolRight = true;
uint32_t defBallFrontSince = 0;

void DefenseMode(){
  int imU = (imuData.euler[0] - 360 * (imuData.euler[0] > 180));
  Serial.println(imU);
  if(cam_line_dir < 0) {
    omniMoveDirection(ball_dir * 25, 90, imU* -10);
  } else {
    // Отталкивание от линии - без разгона, резкий рывок
    omniMoveDirection(2000, cam_line_dir, imU * -10, true);
  }
}

void AtackMode(){
  if (cam_line_dir < 0) {
    if (!trig) {
      digitalWrite(LED_BUILTIN, 0);
      if (abs(ball_dir) < 10) {
        // Мяч перед роботом и не захвачен - едем прямо, форсируем скорость при простое
        omniMoveDirection(accel_via_distance() + stallBoost(), ball_dir, gate_dir_1 * 10);
      } else {
        // Мяч не перед роботом - объезжаем, скорость растёт с углом
        if (ball_dist <= 20){
          resetStall();
          omniMoveDirection(attackSpeed(ball_dir) + accel_via_distance(), (abs(ball_dir) + 90) * (ball_dir/abs(ball_dir)), gate_dir_1 * 10);
        } else {
          omniMoveDirection(3000, ball_dir, gate_dir_1 * 10);
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
    omniMoveDirection(2500, cam_line_dir, gate_dir_1 * 10, true);
  }
}
