// ============================================================================
//  Обработка датчиков
// ============================================================================

void cam_read() {
  uint8_t buffer[8];
  size_t bytesRead = Wire1.requestFrom(CAM_ADDR, 6);

  if (bytesRead == 6) {
    for (size_t i = 0; i < 6; i++) {
      buffer[i] = Wire1.read();
    }

    ball_dir = (buffer[0] | (buffer[1] << 8));
    ball_dir = ball_dir - 360 * (ball_dir > 180);
    y_gate_dir = (buffer[2] | (buffer[3] << 8));
    y_gate_dir = y_gate_dir - 360 * (y_gate_dir > 180);
    b_gate_dir = (buffer[4] | (buffer[5] << 8));
    cam_line_dir = (buffer[6] | (buffer[7] << 8));
    trigger = buffer[6] | (buffer[7] << 8);
    Serial.println(b_gate_dir);
  } else {
    Serial.println("CAM: error read");
  }
}

// ============================================================================
//  Управление моторами
// ============================================================================

struct Motor {
  int32_t target;
  uint32_t lastCmd;
  bool needsUpdate(int32_t n) {
    return n != target || millis() - lastCmd > MOTOR_DATA_SEND_TIMEOUT;
  }
} motors[4];

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

void setMotorSpeed(uint8_t i, int32_t s) {
  if (!motors[i].needsUpdate(s)) return;

  s = constrain(s, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
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

void omniMoveDirection(int32_t wheelSpeed, float direction, int32_t w) {
  float directionRad = (direction)*DEG_TO_RAD;

  float Vx = wheelSpeed * cos(directionRad);
  float Vy = wheelSpeed * sin(directionRad);

  int32_t motor_dps[4];

  static constexpr float SIN_A[4] = { 0.5f, 0.5f, -0.5f, -0.5f };
  static constexpr float COS_A[4] = { -0.8660254f, 0.8660254f, 0.8660254f, -0.8660254f };

  for (int i = 0; i < 4; i++) {
    motor_dps[i] = ((Vx / COS_A[i]) + Vy * SIN_A[i]) / 2 + w;
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

// ============================================================================
//  Пиналка
// ============================================================================

uint64_t kickTmr = millis();

bool kickFlag = 0;

void defKick(){
  if (millis() - kickTmr > 1000) {
    kickFlag = 1;
  }
}

void kicker() {
  if (kickFlag) {
    digitalWrite(KICK, 1);
    delay(60);
    digitalWrite(KICK, 0);
    Serial.println("kick");
    kickTmr = millis();
    kickFlag = 0;
  }
}

// ============================================================================
//  Режимы работы
// ============================================================================

uint64_t tmrForKick = 0;
bool flagK = 1, flagR = 0;
void DefenseMode() {
  if (flagR) { // на своей половине
    if (-10 <= ball_dir >= 10) { // защита
      omniMoveDirection(ball_dir * 20, 90, 0);
      flagK = 1;
    } else { // мяч перед нами
      if (flagK) { // сбрасыванем таймер
        flagK = 0;
      }
      if ((millis() - tmrForKick > 750) && !flagK) { // Проверяем есть ли взаимодействие
        // Нет взаимодействий с мячом начинам атаковать
        if (!trigger) { // Подъезжаем к мячу
          omniMoveDirection(MOVEMENT_SPEED, ball_dir, 0);
        } else {  // Атакуем ворота
          omniMoveDirection(MOVEMENT_SPEED, gate_dir, gate_dir * 15);
          kicker();
        }
      }
    }
  } else { // Возвращаемся на нашу половину
    omniMoveDirection(MOVEMENT_SPEED, gate_dir2, 0);
  }
}


void AtackMode() {
}

bool f_gate = 0;
void exp_AtackMode() {
  if (ball_dir < 15 && ball_dir > 345) {
    //kicker();
  }

  if (line_be) {
    omniMoveDirection(5000, line_dir, 0);
    f_gate = 0;
  } else {
    int64_t ll = b_gate_dir;  //(digitalRead(GATE_SELECT) ? b_gate_dir : y_gate_dir);
    if (ball_dir == 360 || f_gate == 1) {
      //f_gate = 1;
      omniMoveDirection(MOVEMENT_SPEED, ll, ll * 35);
    } else {
      omniMoveDirection(MOVEMENT_SPEED, ball_dir, ll * 35);
    }
    //int oo = VectorSum(ball_dir, cam_line_dir, 1000, 1);
    //omniMoveDirection(MOVEMENT_SPEED, oo, ll * 35);
  }
}
