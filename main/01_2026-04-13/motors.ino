/*// ==================== УЛУЧШЕННЫЕ CAN ФУНКЦИИ ====================
// Отправка команды мотору с повторами и проверкой
bool sendMotorCommandWithRetry(uint32_t id, uint8_t* data, uint8_t len, int maxRetries = CAN_MAX_RETRIES) {
  for (int attempt = 0; attempt < maxRetries; attempt++) {
    byte sndStat = motorCANLine.sendMsgBuf(id, 0, len, data);
    
    if (sndStat == CAN_OK) {
      Serial.print("[CAN] OK -> ID:0x");
      Serial.print(id, HEX);
      Serial.print(" (attempt ");
      Serial.print(attempt + 1);
      Serial.print(") Data:");
      for (int i = 0; i < len; i++) {
        Serial.print(" ");
        Serial.print(data[i], HEX);
      }
      Serial.println();
      return true;
    } else {
      Serial.print("[CAN] RETRY -> ID:0x");
      Serial.print(id, HEX);
      Serial.print(" attempt ");
      Serial.print(attempt + 1);
      Serial.print("/");
      Serial.print(maxRetries);
      Serial.println();
      delay(CAN_RETRY_DELAY);
    }
  }
  
  Serial.print("[CAN] FAILED -> ID:0x");
  Serial.println(id, HEX);
  return false;
}

// Отправка команды с контролем времени отправки (не чаще чем CAN_SEND_INTERVAL)
void sendMotorCommandThrottled(uint32_t id, uint8_t* data, uint8_t len) {
  uint32_t now = millis();
  if (now - lastSendTime < CAN_SEND_INTERVAL) {
    delay(CAN_SEND_INTERVAL - (now - lastSendTime));
  }
  sendMotorCommandWithRetry(id, data, len);
  lastSendTime = millis();
}

void motorOff(uint8_t idx) {
  if (idx >= 4) return;
  uint8_t data[8] = {CMD_MOTOR_OFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  sendMotorCommandWithRetry(ID_MOTORS[idx], data, 8);
  Serial.print("[MOTOR] M");
  Serial.print(idx+1);
  Serial.println(" OFF");
}

void motorOn(uint8_t idx) {
  if (idx >= 4) return;
  uint8_t data[8] = {CMD_MOTOR_ON, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  sendMotorCommandWithRetry(ID_MOTORS[idx], data, 8);
  Serial.print("[MOTOR] M");
  Serial.print(idx+1);
  Serial.println(" ON");
}

void motorStop(uint8_t idx) {
  if (idx >= 4) return;
  uint8_t data[8] = {CMD_MOTOR_STOP, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  sendMotorCommandWithRetry(ID_MOTORS[idx], data, 8);
  Serial.print("[MOTOR] M");
  Serial.print(idx+1);
  Serial.println(" STOP");
}

// Speed closed loop control (0xA2) с гарантированной отправкой
void setMotorSpeed(uint8_t idx, int32_t speed_dps) {
  if (idx >= 4) return;
  
  // Проверяем, изменилась ли скорость
  if (speed_dps == lastTargetSpeed[idx] && millis() - lastSpeedCommand[idx] < 100) {
    // Если скорость не изменилась и прошло меньше 100мс, не отправляем повторно
    return;
  }
  
  int32_t finalSpeed = speed_dps;
  
  // Коррекция направлений для вашей схемы
  switch(idx) {
    case 0: // M1 (0x141)
      finalSpeed = -speed_dps;
      break;
    case 1: // M2 (0x142)
      finalSpeed = -speed_dps;
      break;
    case 2: // M3 (0x143)
      // finalSpeed = -speed_dps;
      break;
    case 3: // M4 (0x144)
      // finalSpeed = -speed_dps;
      break;
  }
  
  int32_t speedControl = finalSpeed * 100;
  
  uint8_t data[8] = {CMD_SPEED_CLOSED, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  
  data[4] = (uint8_t)(speedControl & 0xFF);
  data[5] = (uint8_t)((speedControl >> 8) & 0xFF);
  data[6] = (uint8_t)((speedControl >> 16) & 0xFF);
  data[7] = (uint8_t)((speedControl >> 24) & 0xFF);
  
  // Отправляем с повторами
  if (sendMotorCommandWithRetry(ID_MOTORS[idx], data, 8)) {
    wheelSpeeds[idx] = finalSpeed;
    lastTargetSpeed[idx] = speed_dps;
    lastSpeedCommand[idx] = millis();
    
    Serial.print("[MOTOR] M");
    Serial.print(idx+1);
    Serial.print(" (ID:0x");
    Serial.print(ID_MOTORS[idx], HEX);
    Serial.print(") SPEED=");
    Serial.print(finalSpeed);
    Serial.println(" dps");
  } else {
    Serial.print("[MOTOR] M");
    Serial.print(idx+1);
    Serial.println(" SPEED COMMAND FAILED!");
  }
}

void allMotorsOn() {
  for (int i = 0; i < 4; i++) {
    motorOn(i);
    delay(50);  // Увеличенная задержка между моторами
  }
  Serial.println("[MOTOR] All motors ON");
}

void allMotorsOff() {
  for (int i = 0; i < 4; i++) {
    motorOff(i);
    delay(50);
  }
  Serial.println("[MOTOR] All motors OFF");
}

void allMotorsStop() {
  for (int i = 0; i < 4; i++) {
    motorStop(i);
    delay(30);
  }
  Serial.println("[MOTOR] All motors STOPPED");
}

// ==================== ТЕСТ ДВИЖЕНИЙ ====================
void testMovements() {
  Serial.println("\n[MOVEMENT TEST] Starting movement sequence");
  
  Serial.println("[TEST] Moving FORWARD 360 dps");
  omniMoveDirection(360, 0, 0);
  delay(2000);
  omniStop();
  delay(500);
  
  Serial.println("[TEST] Moving RIGHT 360 dps");
  omniMoveDirection(360, 90, 0);
  delay(2000);
  omniStop();
  delay(500);
  
  Serial.println("[TEST] Moving BACKWARD 360 dps");
  omniMoveDirection(360, 180, 0);
  delay(2000);
  omniStop();
  delay(500);
  
  Serial.println("[TEST] Moving LEFT 360 dps");
  omniMoveDirection(360, 270, 0);
  delay(2000);
  omniStop();
  delay(500);
  
  Serial.println("[TEST] Rotating CW 90 dps");
  omniMove(0, 0, 90);
  delay(2000);
  omniStop();
  
  Serial.println("\n[MOVEMENT TEST] Test completed!");
}*/