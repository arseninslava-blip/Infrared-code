// ============================================================================
//  OMNI PLATFORM CONTROLLER v3.2
//  Режимы: Empty Slot, Calibration, Driving Mode
//  Calibration: SELECT для входа (отправка 0xFF), повторный SELECT для выхода
//  Без ожидания ответа
// ============================================================================

#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>

// ============================================================================
//  КОНФИГУРАЦИЯ
// ============================================================================

// === Пины ===
#define SCL_PIN          2
#define SDA_PIN          3
#define OLED_ADDR        0x3C
#define BUFFER_SIZE      1024

#define CAN_CS_PIN       13
#define CAN_CS_PIN2      5
#define CAN0_INT         14

#define BTN_UP           25      // Кнопка вверх
#define BTN_DOWN         26      // Кнопка вниз
#define BTN_SELECT       27      // Кнопка выбора
#define BTN_BACK         28      // Кнопка назад

// === Параметры робота ===
#define ROBOT_RADIUS      200.0f   // мм
#define WHEEL_RADIUS      65.0f    // мм
#define RADIAN            57.296f  // 180/π

// === CAN параметры ===
#define CAN_MAX_RETRIES   5
#define CAN_RETRY_DELAY   10

// === ID моторов ===
static const uint32_t ID_MOTORS[4] = {0x141, 0x142, 0x143, 0x144};

// === CAN команды ===
#define CMD_MOTOR_OFF      0x80
#define CMD_MOTOR_STOP     0x81
#define CMD_MOTOR_ON       0x88
#define CMD_OPEN_LOOP      0xA0
#define CMD_TORQUE_CLOSED  0xA1
#define CMD_SPEED_CLOSED   0xA2

// === ID для внешнего управления ===
#define CAN_ID_SENSOR      0x111   // ID датчика расстояния/угла
#define CAN_ID_LINE        0x6E   // ID датчика линии

// === Команды калибровки ===
#define CMD_CALIBRATE      0xFF    // Команда калибровки для 0x110

// === Состояния меню ===
enum MenuState {
  MENU_MAIN,
  MENU_EMPTY,        // Пустой слот
  MENU_CALIBRATION,  // Режим калибровки
  MENU_DRIVING       // Режим езды
};

enum TaskState {
  TASK_IDLE,
  TASK_CALIBRATION,
  TASK_DRIVING
};

// ============================================================================
//  ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ
// ============================================================================

// CAN объекты
MCP_CAN motorCANLine(&SPI1, CAN_CS_PIN);
MCP_CAN CAN0(CAN_CS_PIN2);

// CAN буферы
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];

// Данные моторов
volatile int32_t wheelSpeeds[4] = {0};
uint32_t lastSpeedCommand[4] = {0};
int32_t lastTargetSpeed[4] = {0};

// Дисплей
uint8_t displayBuffer[BUFFER_SIZE];

// Меню и задачи
MenuState currentMenu = MENU_MAIN;
TaskState currentTask = TASK_IDLE;
int menuItemIndex = 0;
bool redrawMenu = true;

// Кнопки
bool upPressed = false;
bool downPressed = false;
bool selectPressed = false;
bool backPressed = false;

// === ДАННЫЕ С ДАТЧИКА 0x111 (расстояние/угол) ===
volatile uint8_t lastDistance = 0;
volatile int16_t lastAngleRaw = 0;
volatile uint32_t lastSensorTime = 0;
volatile bool hasNewSensorData = false;
float currentSensorAngle = 0;
int correctionType = 0;  // 0=нет, 1=+90°, 2=-90°
uint32_t sensorPacketCount = 0;

// === ДАННЫЕ С ДАТЧИКА 0x110 (линия + угол) ===
volatile uint8_t hasLine = 0;        // 0 - нет линии, 1 - есть линия
volatile int16_t lineAngleRaw = 0;   // угол линии (x10)
volatile uint32_t lastLineTime = 0;
volatile bool hasNewLineData = false;
float currentLineAngle = 0;
uint32_t linePacketCount = 0;

// === КАЛИБРОВКА ===
volatile bool calibrationActive = false;    // Активен ли режим калибровки
volatile bool calibrationSent = false;      // Отправлен ли пакет калибровки

// === ВЕКТОРНОЕ СЛОЖЕНИЕ ===
float currentMoveAngle = 0;          // Результирующий угол движения
float currentMoveMagnitude = 1.0f;   // Величина (0-1)
float sensorWeight = 1;           // Вес датчика 0x111
float lineWeight = 0.5f;             // Вес датчика 0x110

// Скорость движения
#define MOVEMENT_SPEED     720        // dps

// ============================================================================
//  ШРИФТ 5x7
// ============================================================================

const uint8_t font5x7[96][5] PROGMEM = {
  {0x00,0x00,0x00,0x00,0x00}, {0x00,0x00,0x5F,0x00,0x00}, {0x00,0x07,0x00,0x07,0x00},
  {0x14,0x7F,0x14,0x7F,0x14}, {0x24,0x2A,0x7F,0x2A,0x12}, {0x23,0x13,0x08,0x64,0x62},
  {0x36,0x49,0x55,0x22,0x50}, {0x00,0x05,0x03,0x00,0x00}, {0x00,0x1C,0x22,0x41,0x00},
  {0x00,0x41,0x22,0x1C,0x00}, {0x14,0x08,0x3E,0x08,0x14}, {0x08,0x08,0x3E,0x08,0x08},
  {0x00,0x50,0x30,0x00,0x00}, {0x08,0x08,0x08,0x08,0x08}, {0x00,0x30,0x30,0x00,0x00},
  {0x20,0x10,0x08,0x04,0x02}, {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00},
  {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31}, {0x18,0x14,0x12,0x7F,0x10},
  {0x27,0x45,0x45,0x45,0x39}, {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
  {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E}, {0x00,0x36,0x36,0x00,0x00},
  {0x00,0x56,0x36,0x00,0x00}, {0x08,0x14,0x22,0x41,0x00}, {0x14,0x14,0x14,0x14,0x14},
  {0x00,0x41,0x22,0x14,0x08}, {0x02,0x01,0x51,0x09,0x06}, {0x32,0x49,0x79,0x41,0x3E},
  {0x7E,0x11,0x11,0x11,0x7E}, {0x7F,0x49,0x49,0x49,0x36}, {0x3E,0x41,0x41,0x41,0x22},
  {0x7F,0x41,0x41,0x22,0x1C}, {0x7F,0x49,0x49,0x49,0x41}, {0x7F,0x09,0x09,0x09,0x01},
  {0x3E,0x41,0x49,0x49,0x7A}, {0x7F,0x08,0x08,0x08,0x7F}, {0x00,0x41,0x7F,0x41,0x00},
  {0x20,0x40,0x41,0x3F,0x01}, {0x7F,0x08,0x14,0x22,0x41}, {0x7F,0x40,0x40,0x40,0x40},
  {0x7F,0x02,0x0C,0x02,0x7F}, {0x7F,0x04,0x08,0x10,0x7F}, {0x3E,0x41,0x41,0x41,0x3E},
  {0x7F,0x09,0x09,0x09,0x06}, {0x3E,0x41,0x51,0x21,0x5E}, {0x7F,0x09,0x19,0x29,0x46},
  {0x46,0x49,0x49,0x49,0x31}, {0x01,0x01,0x7F,0x01,0x01}, {0x3F,0x40,0x40,0x40,0x3F},
  {0x1F,0x20,0x40,0x20,0x1F}, {0x3F,0x40,0x38,0x40,0x3F}, {0x63,0x14,0x08,0x14,0x63},
  {0x07,0x08,0x70,0x08,0x07}, {0x61,0x51,0x49,0x45,0x43}, {0x00,0x7F,0x41,0x41,0x00},
  {0x02,0x04,0x08,0x10,0x20}, {0x00,0x41,0x41,0x7F,0x00}, {0x04,0x02,0x01,0x02,0x04},
  {0x40,0x40,0x40,0x40,0x40}, {0x00,0x01,0x02,0x04,0x00}, {0x20,0x54,0x54,0x54,0x78},
  {0x7F,0x48,0x44,0x44,0x38}, {0x38,0x44,0x44,0x44,0x20}, {0x38,0x44,0x44,0x48,0x7F},
  {0x38,0x54,0x54,0x54,0x18}, {0x08,0x7E,0x09,0x01,0x02}, {0x0C,0x52,0x52,0x52,0x3E},
  {0x7F,0x08,0x04,0x04,0x78}, {0x00,0x44,0x7D,0x40,0x00}, {0x20,0x40,0x44,0x3D,0x00},
  {0x7F,0x10,0x28,0x44,0x00}, {0x00,0x41,0x7F,0x40,0x00}, {0x7C,0x04,0x18,0x04,0x78},
  {0x7C,0x08,0x04,0x04,0x78}, {0x38,0x44,0x44,0x44,0x38}, {0x7C,0x14,0x14,0x14,0x08},
  {0x08,0x14,0x14,0x18,0x7C}, {0x7C,0x08,0x04,0x04,0x08}, {0x48,0x54,0x54,0x54,0x20},
  {0x04,0x3F,0x44,0x40,0x20}, {0x3C,0x40,0x40,0x20,0x7C}, {0x1C,0x20,0x40,0x20,0x1C},
  {0x3C,0x40,0x30,0x40,0x3C}, {0x44,0x28,0x10,0x28,0x44}, {0x0C,0x50,0x50,0x50,0x3C},
  {0x44,0x64,0x54,0x4C,0x44}
};

// ============================================================================
//  SOFT I2C ДЛЯ OLED
// ============================================================================

void setSCL(bool level) { digitalWrite(SCL_PIN, level); }
void setSDA(bool level) { digitalWrite(SDA_PIN, level); }

void i2c_init() {
  pinMode(SCL_PIN, OUTPUT);
  pinMode(SDA_PIN, OUTPUT);
  setSCL(HIGH);
  setSDA(HIGH);
}

void i2c_start() {
  setSDA(HIGH);
  setSCL(HIGH);
  delayMicroseconds(5);
  setSDA(LOW);
  delayMicroseconds(5);
  setSCL(LOW);
}

void i2c_stop() {
  setSDA(LOW);
  setSCL(HIGH);
  delayMicroseconds(5);
  setSDA(HIGH);
}

void i2c_write_byte(uint8_t byte) {
  for (int i = 7; i >= 0; i--) {
    setSCL(LOW);
    setSDA((byte >> i) & 1);
    delayMicroseconds(2);
    setSCL(HIGH);
    delayMicroseconds(3);
    setSCL(LOW);
  }
  setSCL(LOW);
  pinMode(SDA_PIN, INPUT_PULLUP);
  setSCL(HIGH);
  delayMicroseconds(2);
  setSCL(LOW);
  pinMode(SDA_PIN, OUTPUT);
}

void sendCommand(uint8_t cmd) {
  i2c_start();
  i2c_write_byte((OLED_ADDR << 1) | 0);
  i2c_write_byte(0x00);
  i2c_write_byte(cmd);
  i2c_stop();
}

void sendDataBuffer() {
  i2c_start();
  i2c_write_byte((OLED_ADDR << 1) | 0);
  i2c_write_byte(0x40);
  for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
    i2c_write_byte(displayBuffer[i]);
  }
  i2c_stop();
}

void buffer_clear() {
  memset(displayBuffer, 0, BUFFER_SIZE);
}

void setPixel(uint8_t x, uint8_t y, bool color) {
  if (x >= 128 || y >= 64) return;
  uint16_t index = (y / 8) * 128 + x;
  if (color) 
    displayBuffer[index] |= (1 << (y % 8));
  else 
    displayBuffer[index] &= ~(1 << (y % 8));
}

void drawChar(uint8_t x, uint8_t y, char ch, bool color = true) {
  if (ch >= 32 && ch <= 126) {
    const uint8_t* glyph = font5x7[ch - 32];
    for (int col = 0; col < 5; col++) {
      uint8_t bits = pgm_read_byte(&glyph[col]);
      for (int row = 0; row < 7; row++) {
        if (bits & (1 << row)) 
          setPixel(x + col, y + row, color);
      }
    }
  }
}

void drawString(uint8_t x, uint8_t y, const char* str, bool color = true) {
  uint8_t cx = x;
  while (*str) {
    drawChar(cx, y, *str, color);
    cx += 6;
    str++;
  }
}

void drawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, bool color, bool filled) {
  if (filled) {
    for (uint8_t i = y; i < y + h; i++)
      for (uint8_t j = x; j < x + w; j++)
        setPixel(j, i, color);
  } else {
    for (uint8_t i = 0; i < w; i++) {
      setPixel(x + i, y, color);
      setPixel(x + i, y + h - 1, color);
    }
    for (uint8_t i = 0; i < h; i++) {
      setPixel(x, y + i, color);
      setPixel(x + w - 1, y + i, color);
    }
  }
}

void drawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, bool color) {
  int dx = abs(x1 - x0);
  int dy = abs(y1 - y0);
  int sx = x0 < x1 ? 1 : -1;
  int sy = y0 < y1 ? 1 : -1;
  int err = dx - dy;
  
  while (true) {
    setPixel(x0, y0, color);
    if (x0 == x1 && y0 == y1) break;
    int e2 = 2 * err;
    if (e2 > -dy) { err -= dy; x0 += sx; }
    if (e2 < dx) { err += dx; y0 += sy; }
  }
}

void display() {
  sendCommand(0x21);
  sendCommand(0x00);
  sendCommand(0x7F);
  sendCommand(0x22);
  sendCommand(0x00);
  sendCommand(0x07);
  sendDataBuffer();
}

void oled_init() {
  delay(100);
  uint8_t init_seq[] = {
    0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 
    0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x12, 
    0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0xAF
  };
  for (int i = 0; i < sizeof(init_seq); i++) 
    sendCommand(init_seq[i]);
  buffer_clear();
  display();
}

// ============================================================================
//  ФУНКЦИЯ КОРРЕКЦИИ УГЛА ДЛЯ ДАТЧИКА 0x111
// ============================================================================
unsigned long tmrB = 0;
unsigned long tmrBB = 0;

float calculateCorrectedAngle(uint8_t distance, int16_t rawAngle, int& corrType) {
  float angleDeg = rawAngle;
  
  if (distance >= 2) {
    if (angleDeg >= 15) {
      corrType = 1;
      return angleDeg + 90.0f;
    } else if (angleDeg <= -15) {
      corrType = 2;
      return angleDeg - 90.0f;
    }
  }
  
  if(angleDeg >= 15 && angleDeg <= -15){
    if(millis() - tmrB  <= 500){
      angleDeg = 0;
    }
    
  }
  corrType = 0;
  return angleDeg;
}

float normalizeAngle(float angle) {
  while (angle >= 360.0f) angle -= 360.0f;
  while (angle < 0) angle += 360.0f;
  return angle;
}

// ============================================================================
//  ВЕКТОРНОЕ СЛОЖЕНИЕ НАПРАВЛЕНИЙ
// ============================================================================

void vectorAddDirections(float angle1, float weight1, float angle2, float weight2, 
                          float& resultAngle, float& resultMagnitude) {
  float rad1 = angle1 * PI / 180.0f;
  float rad2 = angle2 * PI / 180.0f;
  
  float x1 = weight1 * cos(rad1);
  float y1 = weight1 * sin(rad1);
  float x2 = weight2 * cos(rad2);
  float y2 = weight2 * sin(rad2);
  
  float sumX = x1 + x2;
  float sumY = y1 + y2;
  
  resultAngle = atan2(sumY, sumX) * 180.0f / PI;
  resultMagnitude = sqrt(sumX * sumX + sumY * sumY);
  
  if (resultMagnitude > 1.0f) resultMagnitude = 1.0f;
  
  resultAngle = normalizeAngle(resultAngle);
}

void calculateMoveAngle() {
  bool haveSensor = (millis() - lastSensorTime < 500);
  bool haveLine = (millis() - lastLineTime < 500) && (hasLine == 1);
  
  if (!haveSensor && !haveLine) {
    currentMoveMagnitude = 0;
    return;
  }
  
  if (haveSensor && !haveLine) {
    currentMoveAngle = currentSensorAngle;
    currentMoveMagnitude = 1.0f;
    return;
  }
  
  if (!haveSensor && haveLine) {
    currentMoveAngle = currentLineAngle;
    currentMoveMagnitude = 1.0f;
    return;
  }
  
  vectorAddDirections(currentSensorAngle, sensorWeight, 
                      currentLineAngle, lineWeight,
                      currentMoveAngle, currentMoveMagnitude);
}

// ============================================================================
//  CAN ФУНКЦИИ
// ============================================================================

void sendCalibrationCommand() {
  uint8_t cmd[8] = {CMD_CALIBRATE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  
  if (CAN0.sendMsgBuf(CAN_ID_LINE, 0, 8, cmd) == CAN_OK) {
    calibrationSent = true;
    Serial.println("[CAL] Command sent: FF 00 00 00 00 00 00 00 to 0x110");
  } else {
    Serial.println("[CAL] Failed to send calibration command");
  }
}

void readCANMessages() {
  if (!digitalRead(CAN0_INT)) {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);

    // === ЛОГИРОВАНИЕ ВСЕХ CAN-СООБЩЕНИЙ В СЕРИАЛ ===
    Serial.print("[CAN] ID=0x");
    Serial.print(rxId, HEX);
    Serial.print(" LEN=");
    Serial.print(len);
    Serial.print(" DATA=");
    for (int i = 0; i < len; i++) {
      Serial.print("0x");
      if (rxBuf[i] < 0x10) Serial.print("0"); // Добавляем ведущий ноль для красоты
      Serial.print(rxBuf[i], HEX);
      if (i < len - 1) Serial.print(" ");
    }
    Serial.println();
    // === КОНЕЦ ЛОГИРОВАНИЯ ===

    // Обработка датчика 0x111
    if (rxId == CAN_ID_SENSOR && len >= 3) {
      lastDistance = rxBuf[0];
      uint16_t rawUnsigned = ((uint16_t)rxBuf[2] << 8) | rxBuf[1];
      lastAngleRaw = (int16_t)rawUnsigned;
      lastSensorTime = millis();
      hasNewSensorData = true;
      sensorPacketCount++;

      float originalAngle = lastAngleRaw;
      currentSensorAngle =  lastAngleRaw; //map(lastAngleRaw - 180, 0, -360, -180, 180);//calculateCorrectedAngle(lastDistance, lastAngleRaw, correctionType);
      //currentSensorAngle = currentSensorAngle-90;
      //currentSensorAngle = normalizeAngle(currentSensorAngle);

      if (lastDistance >= 2) {
        if (currentSensorAngle >= 15) {
          currentSensorAngle = currentSensorAngle + 90.0f;
        } else if (currentSensorAngle <= -15) {
          currentSensorAngle = currentSensorAngle - 90.0f;
        }
      }
      Serial.print("[0x111] Dist=");
      Serial.print(lastDistance);
      Serial.print(" Angle=");
      Serial.print(currentSensorAngle, 1);
      Serial.println("°");

      redrawMenu = true;
    }

    // Обработка датчика 0x110
    if (rxId == CAN_ID_LINE && len >= 3) {
      hasLine = rxBuf[0];
      int rawUnsigned = ((int)rxBuf[2] << 8) | rxBuf[1];
      lineAngleRaw = (int)rawUnsigned;
      lastLineTime = millis();
      hasNewLineData = true;
      linePacketCount++;

      if (hasLine) {
        currentLineAngle = lineAngleRaw;
        //currentLineAngle = normalizeAngle(currentLineAngle);

        Serial.print("[0x110] Line detected! Angle=");
        Serial.print(currentLineAngle, 1);
        Serial.println("°");
      } else {
        Serial.println("[0x110] No line");
      }

      redrawMenu = true;
    }
  }
}


bool sendMotorCommandWithRetry(uint32_t id, uint8_t* data, uint8_t len, int maxRetries = CAN_MAX_RETRIES) {
  for (int attempt = 0; attempt < maxRetries; attempt++) {
    if (motorCANLine.sendMsgBuf(id, 0, len, data) == CAN_OK) {
      return true;
    }
    delay(CAN_RETRY_DELAY);
  }
  return false;
}

void motorOff(uint8_t idx) {
  if (idx >= 4) return;
  uint8_t data[8] = {CMD_MOTOR_OFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  sendMotorCommandWithRetry(ID_MOTORS[idx], data, 8);
}

void motorOn(uint8_t idx) {
  if (idx >= 4) return;
  uint8_t data[8] = {CMD_MOTOR_ON, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  sendMotorCommandWithRetry(ID_MOTORS[idx], data, 8);
}

void motorStop(uint8_t idx) {
  if (idx >= 4) return;
  uint8_t data[8] = {CMD_MOTOR_STOP, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  sendMotorCommandWithRetry(ID_MOTORS[idx], data, 8);
}

void setMotorSpeed(uint8_t idx, int32_t speed_dps) {
  if (idx >= 4) return;
  
  if (speed_dps == lastTargetSpeed[idx] && millis() - lastSpeedCommand[idx] < 100) {
    return;
  }
  
  int32_t finalSpeed = speed_dps;
  switch(idx) {
    case 0: case 1: finalSpeed = -speed_dps; break;
    default: break;
  }
  
  int32_t speedControl = finalSpeed * 100;
  uint8_t data[8] = {CMD_SPEED_CLOSED, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  data[4] = (uint8_t)(speedControl & 0xFF);
  data[5] = (uint8_t)((speedControl >> 8) & 0xFF);
  data[6] = (uint8_t)((speedControl >> 16) & 0xFF);
  data[7] = (uint8_t)((speedControl >> 24) & 0xFF);
  
  if (sendMotorCommandWithRetry(ID_MOTORS[idx], data, 8)) {
    lastTargetSpeed[idx] = speed_dps;
    lastSpeedCommand[idx] = millis();
  }
}

void allMotorsOn() {
  for (int i = 0; i < 4; i++) { 
    motorOn(i); 
    delay(50); 
  }
}

void allMotorsOff() {
  for (int i = 0; i < 4; i++) { 
    motorOff(i); 
    delay(50); 
  }
}

void allMotorsStop() {
  for (int i = 0; i < 4; i++) { 
    motorStop(i); 
    delay(30); 
  }
}

// ============================================================================
//  КИНЕМАТИКА
// ============================================================================

void omniMoveDirection(int32_t wheelSpeed, float direction) {
  float directionRad = direction / RADIAN;
  float speedFactor = 0.7071f;
  float Vx = wheelSpeed * speedFactor * cos(directionRad);
  float Vy = wheelSpeed * speedFactor * sin(directionRad);
  
  int32_t motor_dps[4];
  motor_dps[0] = constrain((int32_t)(Vx - Vy), -2000, 2000);
  motor_dps[1] = constrain((int32_t)(Vx + Vy), -2000, 2000);
  motor_dps[2] = constrain((int32_t)(Vx - Vy), -2000, 2000);
  motor_dps[3] = constrain((int32_t)(Vx + Vy), -2000, 2000);
  
  for (int i = 0; i < 4; i++) {
    setMotorSpeed(i, motor_dps[i]);
  }
}

void omniMoveWithMagnitude(int32_t maxSpeed, float direction, float magnitude) {
  int32_t actualSpeed = (int32_t)(maxSpeed * magnitude);
  if (actualSpeed < 10) {
    omniStop();
    return;
  }
  omniMoveDirection(actualSpeed, direction);
}

void omniStop() {
  allMotorsStop();
}

// ============================================================================
//  ИНТЕРФЕЙС ДИСПЛЕЯ
// ============================================================================

void drawMenuFrame(const char* title) {
  drawRect(0, 0, 128, 64, true, false);
  drawRect(0, 0, 128, 12, true, true);
  drawString(4, 2, title, false);
  drawLine(0, 12, 127, 12, true);
}

void drawMainMenu() {
  drawMenuFrame("OMNI PLATFORM v3.2");
  drawMenuItem("> Empty Slot", 0, menuItemIndex == 0);
  drawMenuItem("  Calibration", 1, menuItemIndex == 1);
  drawMenuItem("  Driving Mode", 2, menuItemIndex == 2);
}

void drawMenuItem(const char* text, int index, bool selected) {
  uint8_t y = 18 + index * 12;
  if (selected) {
    drawRect(2, y - 2, 124, 11, true, true);
    drawString(6, y, text, false);
  } else {
    drawString(6, y, text, true);
  }
}

void drawEmptySlotScreen() {
  drawMenuFrame("EMPTY SLOT");
  drawString(20, 28, "No Function", true);
  drawString(20, 38, "Reserved", true);
  drawString(4, 58, "SEL:Back", false);
}

void drawCalibrationScreen() {
  drawMenuFrame("CALIBRATION MODE");
  
  char buf[30];
  
  if (!calibrationActive) {
    // Еще не вошли в режим калибровки
    drawString(4, 20, "Press SEL to enter", true);
    drawString(4, 30, "calibration mode", true);
    drawString(4, 40, "and send 0xFF to", true);
    drawString(4, 50, "device 0x110", true);
  } 
  else if (!calibrationSent) {
    // В режиме, но еще не отправили
    drawString(4, 20, "Press SEL to send", true);
    drawString(4, 30, "calibration command", true);
    sprintf(buf, "0x%02X to 0x%03X", CMD_CALIBRATE, CAN_ID_LINE);
    drawString(4, 40, buf, true);
    drawString(4, 50, "Press SEL again to exit", true);
  }
  else {
    // Команда отправлена
    drawString(4, 20, "CALIBRATION SENT!", true);
    drawString(4, 30, "Command: FF 00 00 00", true);
    drawString(4, 40, "00 00 00 00 sent", true);
    drawString(4, 50, "Press SEL to exit", true);
  }
  
  drawString(4, 58, "SEL:Action", false);
}

void drawDrivingScreen() {
  drawMenuFrame("DRIVING MODE");
  
  char buf[30];
  bool hasSensorFresh = (millis() - lastSensorTime < 500);
  bool hasLineFresh = (millis() - lastLineTime < 500);
  
  // Датчик 0x111
  if (hasSensorFresh) {
    sprintf(buf, "0x111 D:%2d A:%5.1f", lastDistance, currentSensorAngle);
    drawString(4, 14, buf, true);
  } else {
    drawString(4, 14, "0x111: NO SIGNAL", true);
  }
  
  // Датчик 0x110
  if (hasLineFresh) {
    if (hasLine) {
      sprintf(buf, "0x110: LINE  A:%5.1f", currentLineAngle);
      drawString(4, 24, buf, true);
    } else {
      drawString(4, 24, "0x110: NO LINE   ", true);
    }
  } else {
    drawString(4, 24, "0x110: NO SIGNAL ", true);
  }
  
  // Результат
  if (hasSensorFresh || hasLineFresh) {
    sprintf(buf, "RESULT A:%5.1f M:%.2f", currentMoveAngle, currentMoveMagnitude);
    drawString(4, 36, buf, true);
    sprintf(buf, "Speed: %d dps", (int)(MOVEMENT_SPEED * currentMoveMagnitude));
    drawString(4, 46, buf, true);
  } else {
    drawString(4, 36, "WAITING FOR DATA...", true);
  }
  
  drawString(4, 58, "SEL:Back", false);
}

void updateDisplay() {
  buffer_clear();
  
  switch (currentMenu) {
    case MENU_MAIN:
      drawMainMenu();
      break;
    case MENU_EMPTY:
      drawEmptySlotScreen();
      break;
    case MENU_CALIBRATION:
      drawCalibrationScreen();
      break;
    case MENU_DRIVING:
      drawDrivingScreen();
      break;
  }
  
  display();
  redrawMenu = false;
}

// ============================================================================
//  УПРАВЛЕНИЕ КНОПКАМИ
// ============================================================================

void handleButtons() {
  static bool lastUp = HIGH, lastDown = HIGH, lastSelect = HIGH, lastBack = HIGH;
  
  bool nowUp = !digitalRead(BTN_UP);
  bool nowDown = !digitalRead(BTN_DOWN);
  bool nowSelect = !digitalRead(BTN_SELECT);
  bool nowBack = !digitalRead(BTN_BACK);
  
  // Кнопка UP
  if (nowUp && !lastUp && currentMenu == MENU_MAIN) {
    menuItemIndex--;
    if (menuItemIndex < 0) menuItemIndex = 2;
    redrawMenu = true;
  }
  
  // Кнопка DOWN
  if (nowDown && !lastDown && currentMenu == MENU_MAIN) {
    menuItemIndex++;
    if (menuItemIndex > 2) menuItemIndex = 0;
    redrawMenu = true;
  }
  
  // Кнопка SELECT
  if (nowSelect && !lastSelect) {
    if (currentMenu == MENU_MAIN) {
      // Вход в выбранный режим
      switch (menuItemIndex) {
        case 0:
          currentMenu = MENU_EMPTY;
          currentTask = TASK_IDLE;
          omniStop();
          break;
        case 1:
          currentMenu = MENU_CALIBRATION;
          currentTask = TASK_CALIBRATION;
          calibrationActive = false;
          calibrationSent = false;
          omniStop();
          break;
        case 2:
          currentMenu = MENU_DRIVING;
          currentTask = TASK_DRIVING;
          omniStop();
          break;
      }
      redrawMenu = true;
    } 
    else if (currentMenu == MENU_CALIBRATION) {
      if (!calibrationActive) {
        // ПЕРВЫЙ SELECT: ВХОД в режим калибровки
        calibrationActive = true;
        Serial.println("[CAL] Entered calibration mode");
        redrawMenu = true;
      }
      else if (calibrationActive && !calibrationSent) {
        // ВТОРОЙ SELECT: Отправка команды калибровки
        sendCalibrationCommand();
        redrawMenu = true;
      }
      else if (calibrationActive && calibrationSent) {
        // ТРЕТИЙ SELECT: Выход из режима калибровки
        currentMenu = MENU_MAIN;
        currentTask = TASK_IDLE;
        calibrationActive = false;
        calibrationSent = false;
        Serial.println("[CAL] Exited calibration mode");
        redrawMenu = true;
      }
    }
    else {
      // Выход из любого другого режима в главное меню
      currentMenu = MENU_MAIN;
      currentTask = TASK_IDLE;
      omniStop();
      redrawMenu = true;
    }
  }
  
  // Кнопка BACK
  if (nowBack && !lastBack && currentMenu != MENU_MAIN) {
    if (currentMenu == MENU_CALIBRATION) {
      calibrationActive = false;
      calibrationSent = false;
      Serial.println("[CAL] Calibration cancelled");
    }
    currentMenu = MENU_MAIN;
    currentTask = TASK_IDLE;
    omniStop();
    redrawMenu = true;
  }
  
  lastUp = nowUp;
  lastDown = nowDown;
  lastSelect = nowSelect;
  lastBack = nowBack;
}

// ============================================================================
//  ВЫПОЛНЕНИЕ ЗАДАЧ
// ============================================================================

void handleTasks() {
  switch (currentTask) {
    case TASK_DRIVING:
      calculateMoveAngle();
      
      if (currentMoveMagnitude > 0.05f) {
        omniMoveWithMagnitude(MOVEMENT_SPEED, currentMoveAngle, currentMoveMagnitude);
        if (millis() % 500 < 50) redrawMenu = true;
      } else {
        omniStop();
      }
      break;
    
    default:
      break;
  }
}

// ============================================================================
//  SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n========== OMNI PLATFORM v3.2 ==========");
  Serial.println("Modes: Empty Slot | Calibration | Driving");
  Serial.println("Calibration: SELECT to enter, SELECT to send 0xFF, SELECT to exit");
  Serial.println("=========================================");
  
  // Дисплей
  i2c_init();
  oled_init();
  Serial.println("[OK] OLED Display");
  
  // CAN0 (датчики)
  SPI.setMOSI(7);
  SPI.setMISO(4);
  SPI.setSCK(6);
  SPI.begin();
  
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("[OK] CAN0 (Sensors) Initialized");
  } else {
    while(1) Serial.println("[ERROR] CAN0 Init Failed!");
  }
  
  CAN0.setMode(MCP_NORMAL);
  pinMode(CAN0_INT, INPUT);
  
  // Кнопки
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_SELECT, INPUT_PULLUP);
  pinMode(BTN_BACK, INPUT_PULLUP);
  
  // CAN моторов
  SPI1.setMOSI(11);
  SPI1.setMISO(12);
  SPI1.setSCK(10);
  SPI1.begin();
  
  while (motorCANLine.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("[WAIT] Motor CAN init...");
    delay(500);
  }
  
  motorCANLine.setMode(MCP_NORMAL);
  Serial.println("[OK] Motor CAN Bus");
  
  allMotorsOn();
  delay(500);
  
  // Настройка весов
  sensorWeight = 1;
  lineWeight = 1.5;
  
  Serial.println("[OK] System Ready");
  Serial.println("\n=== MENU ===");
  Serial.println("1. Empty Slot - Reserved");
  Serial.println("2. Calibration - SELECT: enter -> send 0xFF -> exit");
  Serial.println("3. Driving Mode - Vector addition");
  
  redrawMenu = true;
  updateDisplay();
}

// ============================================================================
//  LOOP
// ============================================================================

void loop() {
  readCANMessages();
  handleButtons();
  handleTasks();
  
  if (redrawMenu) {
    updateDisplay();
  }
}