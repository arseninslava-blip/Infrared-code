// ============================================================================
//  OMNI PLATFORM CONTROLLER v3.0
//  Объединенный скетч: дисплей, CAN, кинематика, управление моторами
//  С интерфейсом выбора задач
//  Модификация: прием данных с ID 0x111 (расстояние + угол)
//  Правила коррекции угла:
//  - Если расстояние >= 4 И угол >= 0: угол + 90°
//  - Если расстояние >= 4 И угол < 0: угол - 90°
//  - Если расстояние < 4: угол без изменений
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

#define BTN_UP           25      // Кнопка вверх/калибровка
#define BTN_DOWN         26      // Кнопка вниз/движение
#define BTN_SELECT       27      // Кнопка выбора (если есть)
#define BTN_BACK         28      // Кнопка назад (если есть)

// === Параметры робота ===
#define ROBOT_RADIUS      200.0f   // мм
#define WHEEL_RADIUS      65.0f    // мм
#define RADIAN            57.296f  // 180/π

// === CAN параметры ===
#define CAN_MAX_RETRIES   5
#define CAN_RETRY_DELAY   10
#define CAN_SEND_INTERVAL 20

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
#define CAN_ID_COMMAND     0x111   // ID для приема команд (расстояние + угол)

// === Состояния меню ===
enum MenuState {
  MENU_MAIN,
  MENU_MANUAL,
  MENU_AUTO,
  MENU_CALIBRATION,
  MENU_TEST_SEQUENCE,
  MENU_SETTINGS,
  MENU_INFO
};

enum TaskState {
  TASK_IDLE,
  TASK_CALIBRATING,
  TASK_MOVING_DIRECTION,
  TASK_ROTATING,
  TASK_TEST_SEQUENCE,
  TASK_AUTO_TRACKING
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
byte OPS[8] = {0};

// Данные моторов
volatile int32_t wheelSpeeds[4] = {0};
uint32_t lastSendTime = 0;
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
bool btnUpLastState = HIGH;
bool btnDownLastState = HIGH;
bool btnSelectLastState = HIGH;
bool btnBackLastState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// Параметры движения
int32_t currentSpeed = 360;     // dps
float currentDirection = 0;     // градусы
float currentOmega = 0;         // dps
int selectedSpeedIndex = 2;     // 0-4: 180, 360, 540, 720, 900
int selectedDirIndex = 0;       // 0-7: 0,45,90,135,180,225,270,315

// Скорости для выбора
const int32_t speedOptions[] = {180, 360, 540, 720, 900};
const char* speedLabels[] = {"180", "360", "540", "720", "900"};

// Направления для выбора
const float directionOptions[] = {0, 45, 90, 135, 180, 225, 270, 315};
const char* directionLabels[] = {"↑", "↗", "→", "↘", "↓", "↙", "←", "↖"};

// Таймеры для автоматических задач
uint32_t taskStartTime = 0;
uint32_t taskDuration = 0;
int testStep = 0;

// === ДАННЫЕ ДЛЯ АВТО-РЕЖИМА (CAN ID 0x111) ===
volatile uint8_t lastDistance = 0;      // 1 байт: расстояние (0-255)
volatile int16_t lastAngleRaw = 0;      // сырой угол (может быть отрицательным)
volatile uint32_t lastCommandTime = 0;  // время последней команды
volatile bool hasNewCommand = false;    // флаг новой команды
float currentAutoAngle = 0;             // текущий угол для движения
int correctionType = 0;                 // 0=нет коррекции, 1=+90°, 2=-90°


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

void drawProgressBar(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t percent) {
  drawRect(x, y, w, h, true, false);
  uint8_t fillWidth = map(percent, 0, 100, 0, w - 2);
  for (uint8_t i = 1; i < fillWidth + 1; i++) {
    drawLine(x + i, y + 1, x + i, y + h - 2, true);
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
//  ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
// ============================================================================

float degToRad(float deg) {
  return deg / RADIAN;
}

float radToDeg(float rad) {
  return rad * RADIAN;
}

float dpsToRpm(float dps) {
  return dps * 60.0f / 360.0f;
}


// ============================================================================
//  CAN ФУНКЦИИ
// ============================================================================

// Функция для коррекции угла на основе расстояния и знака угла
float calculateCorrectedAngle(uint8_t distance, int16_t rawAngle) {
  // Преобразуем сырой угол в градусы
  // Формат: угол передается как int16 (может быть отрицательным)
  // Значение 3600 = 360.0 градусов
  float angleDeg = rawAngle / 10.0f;
  
  // Если расстояние < 4, коррекция не применяется
  if (distance < 4) {
    correctionType = 0;
    Serial.print("[CAN] Distance <4, no correction. Original angle: ");
    Serial.println(angleDeg, 1);
    return angleDeg;
  }
  
  // Расстояние >= 4
  if (angleDeg >= 0) {
    // Положительный угол: прибавляем +90°
    correctionType = 1;
    float corrected = angleDeg + 90.0f;
    Serial.print("[CAN] Distance >=4, angle >=0 -> +90°. Original: ");
    Serial.print(angleDeg, 1);
    Serial.print(" Corrected: ");
    Serial.println(corrected, 1);
    return corrected;
  } else {
    // Отрицательный угол: вычитаем 90° (прибавляем -90°)
    correctionType = 2;
    float corrected = angleDeg - 90.0f;
    Serial.print("[CAN] Distance >=4, angle <0 -> -90°. Original: ");
    Serial.print(angleDeg, 1);
    Serial.print(" Corrected: ");
    Serial.println(corrected, 1);
    return corrected;
  }
}

void readCANMessages() {
  if (!digitalRead(CAN0_INT)) {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);
    
    // --- Обработка команд с ID 0x111 ---
    if (rxId == CAN_ID_COMMAND && len >= 3) {
      // Формат сообщения:
      // байт 0: расстояние (0-255)
      // байт 1: угол младший байт
      // байт 2: угол старший байт (знаковое int16, 0-3600 = 0-360.0°)
      //         отрицательные углы хранятся в дополнительном коде
      
      lastDistance = rxBuf[0];
      
      // Собираем 16-битное знаковое значение угла
      // Передаем как int16_t (младший байт first, старший second)
      uint16_t rawUnsigned = ((uint16_t)rxBuf[2] << 8) | rxBuf[1];
      lastAngleRaw = (int16_t)rawUnsigned;  // Интерпретируем как знаковое
      
      lastCommandTime = millis();
      hasNewCommand = true;
      
      // Вычисляем скорректированный угол
      currentAutoAngle = calculateCorrectedAngle(lastDistance, lastAngleRaw);
      
      // Нормализуем угол в диапазон 0-360
      while (currentAutoAngle >= 360.0f) currentAutoAngle -= 360.0f;
      while (currentAutoAngle < 0) currentAutoAngle += 360.0f;
      
      // Выводим отладочную информацию
      float originalAngle = lastAngleRaw / 10.0f;
      Serial.print("[CAN] ID:0x111 Dist:");
      Serial.print(lastDistance);
      Serial.print(" RawAngle:");
      Serial.print(lastAngleRaw);
      Serial.print(" (");
      Serial.print(originalAngle, 1);
      Serial.print("°) -> Corrected:");
      Serial.print(currentAutoAngle, 1);
      Serial.println("°");
      
      redrawMenu = true;
    }
    // --- Старый формат (для совместимости) ---
    else {
      if ((rxId & 0x80000000) == 0x80000000)
        Serial.printf("Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len);
      else
        Serial.printf("Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);
    
      if ((rxId & 0x40000000) == 0x40000000) {
        Serial.print(" REMOTE REQUEST FRAME");
      } else {
        for (byte i = 0; i < len; i++) {
          Serial.printf(" 0x%.2X", rxBuf[i]);
          if (i < 8) OPS[i] = rxBuf[i];
        }
      }
      Serial.println();
    }
  }
}

void send(uint16_t id, uint8_t *data) {
  CAN0.sendMsgBuf(id, 0, 8, data);
  Serial.print("Sent ID: 0x");
  Serial.print(id, HEX);
  Serial.print(", Data: ");
  for (int i = 0; i < 8; i++) {
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

bool sendMotorCommandWithRetry(uint32_t id, uint8_t* data, uint8_t len, int maxRetries = CAN_MAX_RETRIES) {
  for (int attempt = 0; attempt < maxRetries; attempt++) {
    byte sndStat = motorCANLine.sendMsgBuf(id, 0, len, data);
    
    if (sndStat == CAN_OK) {
      Serial.print("[CAN] OK -> ID:0x");
      Serial.print(id, HEX);
      Serial.println();
      return true;
    }
    delay(CAN_RETRY_DELAY);
  }
  Serial.print("[CAN] FAILED -> ID:0x");
  Serial.println(id, HEX);
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
    wheelSpeeds[idx] = finalSpeed;
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
//  КИНЕМАТИКА ОМНИ-ПЛАТФОРМЫ
// ============================================================================

void omniMove(float Vx_dps, float Vy_dps, float omega_dps) {
  float omega_rad_s = omega_dps / RADIAN;
  float V_omega = (omega_rad_s * ROBOT_RADIUS) / WHEEL_RADIUS * RADIAN;
  
  int32_t motor_dps[4];
  motor_dps[0] = constrain((int32_t)(Vx_dps - Vy_dps - V_omega), -2000, 2000);
  motor_dps[1] = constrain((int32_t)(Vx_dps + Vy_dps + V_omega), -2000, 2000);
  motor_dps[2] = constrain((int32_t)(Vx_dps - Vy_dps + V_omega), -2000, 2000);
  motor_dps[3] = constrain((int32_t)(Vx_dps + Vy_dps - V_omega), -2000, 2000);
  
  for (int i = 0; i < 4; i++) {
    setMotorSpeed(i, motor_dps[i]);
    delay(5);
  }
}

void omniMoveDirection(int32_t wheelSpeed, float direction, float omega) {
  float directionRad = direction / RADIAN;
  float speedFactor = 0.7071f;
  float Vx = wheelSpeed * speedFactor * cos(directionRad);
  float Vy = wheelSpeed * speedFactor * sin(directionRad);
  omniMove(Vx, Vy, omega);
}

void rotate(float omega_dps) {
  for (int i = 0; i < 4; i++) {
    if (i > 1)
      setMotorSpeed(i, omega_dps);
    else
      setMotorSpeed(i, -omega_dps);
    delay(5);
  }
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

void drawMenuItem(const char* text, int index, bool selected) {
  uint8_t y = 16 + index * 10;
  if (selected) {
    drawRect(2, y - 2, 124, 10, true, true);
    drawString(6, y, text, false);
  } else {
    drawString(6, y, text, true);
  }
}

void drawMainMenu() {
  drawMenuFrame("OMNI PLATFORM v3.0");
  
  const char* menuItems[] = {
    "> Manual Control",
    "> Auto Mode (CAN 0x111)",
    "> Calibration",
    "> Test Sequence",
    "> Settings",
    "> Info"
  };
  
  for (int i = 0; i < 6; i++) {
    drawMenuItem(menuItems[i], i, (i == menuItemIndex));
  }
}

void drawManualControlScreen() {
  drawMenuFrame("MANUAL CONTROL");
  
  char buf[20];
  
  // Скорость
  sprintf(buf, "Speed: %s dps", speedLabels[selectedSpeedIndex]);
  drawString(4, 16, buf, true);
  
  // Направление
  sprintf(buf, "Dir: %s (%3.0f deg)", directionLabels[selectedDirIndex], directionOptions[selectedDirIndex]);
  drawString(4, 28, buf, true);
  
  // Текущее движение
  if (currentTask == TASK_MOVING_DIRECTION) {
    drawString(4, 40, "STATUS: MOVING", true);
    sprintf(buf, "V: %d dps", currentSpeed);
    drawString(4, 50, buf, true);
  } else {
    drawString(4, 40, "STATUS: STOPPED", true);
  }
  
  // Подсказки
  drawString(4, 58, "UP/DN:Speed  SEL:Move", false);
}

void drawAutoModeScreen() {
  drawMenuFrame("AUTO MODE (CAN 0x111)");
  
  char buf[30];
  
  // Проверяем, есть ли свежие данные (не старше 1 секунды)
  bool hasFreshData = (millis() - lastCommandTime < 1000);
  
  if (hasFreshData) {
    // Отображаем полученные данные
    sprintf(buf, "Distance: %3d", lastDistance);
    drawString(4, 16, buf, true);
    
    float originalAngle = lastAngleRaw / 10.0f;
    sprintf(buf, "Raw Angle: %5.1f deg", originalAngle);
    drawString(4, 26, buf, true);
    
    // Отображаем тип коррекции
    switch(correctionType) {
      case 1:
        sprintf(buf, "Correction: +90 deg");
        break;
      case 2:
        sprintf(buf, "Correction: -90 deg");
        break;
      default:
        sprintf(buf, "Correction: none (dist<4)");
        break;
    }
    drawString(4, 36, buf, true);
    
    // Отображаем итоговый угол
    sprintf(buf, "Final Angle: %5.1f deg", currentAutoAngle);
    drawString(4, 46, buf, true);
    
    // Статус движения
    if (currentTask == TASK_AUTO_TRACKING) {
      drawString(4, 56, "STATUS: TRACKING", true);
    } else {
      drawString(4, 56, "STATUS: WAITING", true);
    }
  } else {
    drawString(4, 16, "Waiting for CAN data...", true);
    drawString(4, 26, "ID: 0x111", true);
    drawString(4, 36, "Format: [Dist][AngleL][AngleH]", true);
    drawString(4, 46, "Angle: int16 (x10 = deg)", true);
    drawString(4, 56, "STATUS: NO SIGNAL", true);
  }
  
  drawString(4, 58, "SEL:Back", false);
}

void drawCalibrationScreen() {
  drawMenuFrame("CALIBRATION");
  
  drawString(4, 16, "Calibrating gyro...", true);
  drawString(4, 28, "Rotating for 10 sec", true);
  
  uint32_t elapsed = millis() - taskStartTime;
  uint8_t percent = constrain(map(elapsed, 0, 10000, 0, 100), 0, 100);
  drawProgressBar(14, 42, 100, 8, percent);
  
  char buf[20];
  sprintf(buf, "%d%%", percent);
  drawString(54, 54, buf, true);
}

void drawTestSequenceScreen() {
  drawMenuFrame("TEST SEQUENCE");
  
  const char* steps[] = {
    "Forward",
    "Right", 
    "Backward",
    "Left",
    "Rotate CW",
    "Rotate CCW",
    "Diagonal NE",
    "Diagonal NW",
    "Complete!"
  };
  
  char buf[30];
  if (testStep < 8) {
    sprintf(buf, "Step %d/8: %s", testStep + 1, steps[testStep]);
    drawString(4, 16, buf, true);
    
    uint8_t percent = map(testStep, 0, 8, 0, 100);
    drawProgressBar(14, 32, 100, 8, percent);
    sprintf(buf, "%d%% complete", percent);
    drawString(4, 46, buf, true);
  } else {
    drawString(4, 16, "TEST COMPLETE!", true);
    drawString(4, 32, "Press SEL for menu", true);
  }
  
  drawString(4, 58, "SEL:Stop/Back", false);
}

void drawSettingsScreen() {
  drawMenuFrame("SETTINGS");
  
  drawString(4, 16, "Max Speed: 2000 dps", true);
  drawString(4, 26, "CAN Speed: 1000 kbps", true);
  drawString(4, 36, "Robot Radius: 200 mm", true);
  drawString(4, 46, "Wheel Radius: 65 mm", true);
  
  drawString(4, 58, "SEL:Back", false);
}

void drawInfoScreen() {
  drawMenuFrame("INFO");
  
  drawString(4, 16, "OMNI Platform v3.0", true);
  drawString(4, 26, "MCP2515 CAN Bus", true);
  drawString(4, 36, "4x BLDC Motors", true);
  drawString(4, 46, "SSD1306 OLED", true);
  
  char buf[20];
  sprintf(buf, "Uptime: %lu s", millis() / 1000);
  drawString(4, 56, buf, true);
  
  drawString(4, 58, "SEL:Back", false);
}

void updateDisplay() {
  buffer_clear();
  
  switch (currentMenu) {
    case MENU_MAIN:
      drawMainMenu();
      break;
    case MENU_MANUAL:
      drawManualControlScreen();
      break;
    case MENU_AUTO:
      drawAutoModeScreen();
      break;
    case MENU_CALIBRATION:
      drawCalibrationScreen();
      break;
    case MENU_TEST_SEQUENCE:
      drawTestSequenceScreen();
      break;
    case MENU_SETTINGS:
      drawSettingsScreen();
      break;
    case MENU_INFO:
      drawInfoScreen();
      break;
  }
  
  display();
  redrawMenu = false;
}


// ============================================================================
//  УПРАВЛЕНИЕ КНОПКАМИ
// ============================================================================

void handleButtons() {
  bool upPressed = !digitalRead(BTN_UP);
  bool downPressed = !digitalRead(BTN_DOWN);
  bool selectPressed = !digitalRead(BTN_SELECT);
  bool backPressed = !digitalRead(BTN_BACK);
  
  static bool upProcessed = false;
  static bool downProcessed = false;
  static bool selectProcessed = false;
  static bool backProcessed = false;
  
  // Кнопка UP
  if (upPressed && !upProcessed) {
    upProcessed = true;
    
    switch (currentMenu) {
      case MENU_MAIN:
        menuItemIndex = (menuItemIndex > 0) ? menuItemIndex - 1 : 5;
        redrawMenu = true;
        break;
        
      case MENU_MANUAL:
        if (currentTask != TASK_MOVING_DIRECTION) {
          selectedSpeedIndex = (selectedSpeedIndex + 1) % 5;
          currentSpeed = speedOptions[selectedSpeedIndex];
          redrawMenu = true;
        }
        break;
        
      default:
        break;
    }
  }
  
  if (!upPressed) upProcessed = false;
  
  // Кнопка DOWN
  if (downPressed && !downProcessed) {
    downProcessed = true;
    
    switch (currentMenu) {
      case MENU_MAIN:
        menuItemIndex = (menuItemIndex + 1) % 6;
        redrawMenu = true;
        break;
        
      case MENU_MANUAL:
        if (currentTask != TASK_MOVING_DIRECTION) {
          selectedDirIndex = (selectedDirIndex + 1) % 8;
          currentDirection = directionOptions[selectedDirIndex];
          redrawMenu = true;
        }
        break;
        
      default:
        break;
    }
  }
  
  if (!downPressed) downProcessed = false;
  
  // Кнопка SELECT
  if (selectPressed && !selectProcessed) {
    selectProcessed = true;
    
    switch (currentMenu) {
      case MENU_MAIN:
        switch (menuItemIndex) {
          case 0: // Manual Control
            currentMenu = MENU_MANUAL;
            currentTask = TASK_IDLE;
            omniStop();
            redrawMenu = true;
            break;
          case 1: // Auto Mode
            currentMenu = MENU_AUTO;
            currentTask = TASK_AUTO_TRACKING;
            redrawMenu = true;
            break;
          case 2: // Calibration
            currentMenu = MENU_CALIBRATION;
            currentTask = TASK_CALIBRATING;
            taskStartTime = millis();
            redrawMenu = true;
            break;
          case 3: // Test Sequence
            currentMenu = MENU_TEST_SEQUENCE;
            currentTask = TASK_TEST_SEQUENCE;
            testStep = 0;
            taskStartTime = millis();
            redrawMenu = true;
            break;
          case 4: // Settings
            currentMenu = MENU_SETTINGS;
            redrawMenu = true;
            break;
          case 5: // Info
            currentMenu = MENU_INFO;
            redrawMenu = true;
            break;
        }
        break;
        
      case MENU_MANUAL:
        if (currentTask == TASK_IDLE) {
          currentTask = TASK_MOVING_DIRECTION;
          omniMoveDirection(currentSpeed, currentDirection, 0);
        } else {
          currentTask = TASK_IDLE;
          omniStop();
        }
        redrawMenu = true;
        break;
        
      case MENU_AUTO:
        currentMenu = MENU_MAIN;
        currentTask = TASK_IDLE;
        omniStop();
        redrawMenu = true;
        break;
        
      case MENU_CALIBRATION:
        if (currentTask == TASK_CALIBRATING) {
          currentTask = TASK_IDLE;
          omniStop();
        }
        currentMenu = MENU_MAIN;
        redrawMenu = true;
        break;
        
      case MENU_TEST_SEQUENCE:
        if (currentTask == TASK_TEST_SEQUENCE) {
          currentTask = TASK_IDLE;
          omniStop();
        }
        currentMenu = MENU_MAIN;
        redrawMenu = true;
        break;
        
      case MENU_SETTINGS:
        currentMenu = MENU_MAIN;
        redrawMenu = true;
        break;
        
      case MENU_INFO:
        currentMenu = MENU_MAIN;
        redrawMenu = true;
        break;
    }
  }
  
  if (!selectPressed) selectProcessed = false;
  
  // Кнопка BACK
  if (backPressed && !backProcessed) {
    backProcessed = true;
    
    if (currentMenu != MENU_MAIN) {
      currentMenu = MENU_MAIN;
      currentTask = TASK_IDLE;
      omniStop();
      redrawMenu = true;
    }
  }
  
  if (!backPressed) backProcessed = false;
}


// ============================================================================
//  ВЫПОЛНЕНИЕ ЗАДАЧ
// ============================================================================

void handleTasks() {
  switch (currentTask) {
    case TASK_CALIBRATING:
      {
        uint32_t elapsed = millis() - taskStartTime;
        
        if (elapsed < 10000) {
          rotate(130);
          if (millis() % 500 < 50) redrawMenu = true;
        } else {
          // Завершение калибровки
          byte calibrateStopCmd[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
          for (int i = 0; i < 10; i++) {
            send(0x110, calibrateStopCmd);
          }
          omniStop();
          currentTask = TASK_IDLE;
          redrawMenu = true;
        }
      }
      break;
      
    case TASK_MOVING_DIRECTION:
      omniMoveDirection(currentSpeed, currentDirection, 0);
      if (millis() % 500 < 50) redrawMenu = true;
      break;
      
    case TASK_TEST_SEQUENCE:
      {
        uint32_t stepDuration = 2000; // 2 секунды на шаг
        uint32_t elapsedInStep = millis() - taskStartTime;
        
        if (testStep < 8) {
          if (elapsedInStep > stepDuration) {
            testStep++;
            taskStartTime = millis();
            redrawMenu = true;
          } else {
            // Выполнение текущего шага
            switch (testStep) {
              case 0: omniMoveDirection(360, 0, 0); break;    // Forward
              case 1: omniMoveDirection(360, 90, 0); break;   // Right
              case 2: omniMoveDirection(360, 180, 0); break;  // Backward
              case 3: omniMoveDirection(360, 270, 0); break;  // Left
              case 4: omniMove(0, 0, 90); break;              // Rotate CW
              case 5: omniMove(0, 0, -90); break;             // Rotate CCW
              case 6: omniMoveDirection(360, 45, 0); break;   // Diagonal NE
              case 7: omniMoveDirection(360, 315, 0); break;  // Diagonal NW
            }
            if (millis() % 500 < 50) redrawMenu = true;
          }
        } else {
          // Тест завершен
          omniStop();
          if (millis() % 1000 < 50) redrawMenu = true;
        }
      }
      break;
      
    case TASK_AUTO_TRACKING:
      {
        // Автоматическое движение по данным с CAN
        if (hasNewCommand && (millis() - lastCommandTime < 1000)) {
          // Движемся по скорректированному углу
          omniMoveDirection(500, currentAutoAngle, 0);
          if (millis() % 500 < 50) redrawMenu = true;
          hasNewCommand = false;
        } else if (millis() - lastCommandTime >= 1000 && millis() - lastCommandTime < 1500) {
          // Нет свежих данных - останавливаемся
          omniStop();
        }
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
  Serial.println("\n========== OMNI PLATFORM v3.0 ==========");
  Serial.println("CAN ID: 0x111 | Format: [Dist][AngleL][AngleH]");
  Serial.println("Correction rules:");
  Serial.println("  - Distance >= 4 AND Angle >= 0: Angle + 90°");
  Serial.println("  - Distance >= 4 AND Angle < 0:  Angle - 90°");
  Serial.println("  - Distance < 4: No correction");
  Serial.println("=========================================");
  
  // --- Инициализация дисплея ---
  i2c_init();
  oled_init();
  Serial.println("[OK] OLED Display");
  
  // --- Инициализация CAN0 (основная шина) ---
  SPI.setMOSI(7);
  SPI.setMISO(4);
  SPI.setSCK(6);
  SPI.begin();
  
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("[OK] MCP2515 (CAN0) Initialized");
  } else {
    while(1) Serial.println("[ERROR] MCP2515 (CAN0) Init Failed!");
  }
  
  CAN0.setMode(MCP_NORMAL);
  pinMode(CAN0_INT, INPUT);
  
  // --- Инициализация кнопок ---
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_SELECT, INPUT_PULLUP);
  pinMode(BTN_BACK, INPUT_PULLUP);
  
  // --- Инициализация CAN для моторов ---
  SPI1.setMOSI(11);
  SPI1.setMISO(12);
  SPI1.setSCK(10);
  SPI1.begin();
  
  while (motorCANLine.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("[WAIT] Motor CAN init...");
    delay(500);
  }
  
  motorCANLine.setMode(MCP_NORMAL);
  Serial.println("[OK] Motor CAN Bus");
  
  // --- Включение моторов ---
  allMotorsOn();
  delay(500);
  
  Serial.println("[OK] System Ready");
  Serial.println("=========================================");
  
  // --- Стартовый экран ---
  redrawMenu = true;
  updateDisplay();
}


// ============================================================================
//  LOOP
// ============================================================================

void loop() {
  // Чтение CAN сообщений
  readCANMessages();
  
  // Обработка кнопок
  handleButtons();
  
  // Выполнение текущей задачи
  handleTasks();
  
  // Обновление дисплея при необходимости
  if (redrawMenu) {
    updateDisplay();
  }
  
  delay(10);
}