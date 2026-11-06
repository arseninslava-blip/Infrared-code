#include <SPI.h>
#include <mcp_can.h>
#include <Adafruit_NeoPixel.h>

#ifdef __AVR__
  #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

// ========== CAN PINS ==========
#define CAN_CS_PIN 13   // CS на GP17
#define CAN_INT     9   // CAN interrupt pin

// ========== LED STRIP PINS ==========
#define LED_PIN     7
#define LED_COUNT   16

// ========== SENSOR PINS ==========
#define TRIG_PIN    A1
#define SENSOR_PIN  A0

// ========== MULTIPLEXER PINS ==========
const int sig1 = 2;
const int sig2 = 3;
const int sig3 = 4;
const int sig4 = 5;

// ========== GLOBAL OBJECTS ==========
MCP_CAN CAN(&SPI1, CAN_CS_PIN);
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// ========== CAN VARIABLES ==========
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];
int minGreen[16] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023};

// ========== SETUP ==========
void setup() {
  // Initialize Serial
  Serial.begin(115200);

  // Initialize LED strip
  strip.begin();
  strip.show();            // Turn OFF all pixels
  strip.setBrightness(200);
  
  colorWipe(strip.Color(255, 0, 0), 50);  // Red - startup indicator

  unsigned long tmrCal = millis();
  while(millis() - tmrCal < 500){
    for(byte x = 0; x < 16; x++){
      minGreen[x] = min(minGreen[x], readADC(x, SENSOR_PIN));
    }
  }
}

void setup1(){
  // Initialize multiplexer pins
  int pins[4] = {sig1, sig2, sig3, sig4};
  for(int pin : pins) {
    pinMode(pin, OUTPUT);
  }

  // Configure SPI1 pins
  SPI1.setRX(12);   // MISO на GP16
  SPI1.setTX(11);   // MOSI на GP19
  SPI1.setSCK(10);  // SCK на GP18
  SPI1.begin();

  // Initialize CAN controller
  if(CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("MCP2515 Initialized Successfully!");
    CAN.setMode(MCP_NORMAL);   // Normal mode for transmitting messages

  } else {
    while(1) {
      Serial.println("Error Initializing MCP2515...");
    }
  }
  pinMode(CAN_INT, INPUT);
  Serial.println("dastish good");
}
// ========== MAIN LOOP ==========
bool package = 0;


byte information[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 1. bool 2-3 angle
byte operation[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte cmd = 0x0;
int number = 0;

constexpr float sensVecs[16][2] PROGMEM = {
  { 0.980785,  0.195090 },  //  11.25°
  { 0.923880,  0.382683 },  //  33.75° (было 22.5° + 11.25°)
  { 0.831470,  0.555570 },  //  56.25°
  { 0.555570,  0.831470 },  //  78.75°
  { 0.195090,  0.980785 },  // 101.25°
  {-0.195090,  0.980785 },  // 123.75°
  {-0.555570,  0.831470 },  // 146.25°
  {-0.831470,  0.555570 },  // 168.75°
  {-0.980785,  0.195090 },  // 191.25°
  {-0.980785, -0.195090 },  // 213.75°
  {-0.831470, -0.555570 },  // 236.25°
  {-0.555570, -0.831470 },  // 258.75°
  {-0.195090, -0.980785 },  // 281.25°
  { 0.195090, -0.980785 },  // 303.75°
  { 0.555570, -0.831470 },  // 326.25°
  { 0.831470, -0.555570 }   // 348.75°
};

float vx = 0, vy = 0;
float vxx = 0, vyy = 0;
int angle = 0;
byte line = 0;



void loop() {
  //colorWipe(strip.Color(255, 0, 0), 50);  // Red - indicate activity
  switch (cmd) {
    case 0xFF:
      for(int i = 0; i < 16; i++){
        minGreen[i] = 1023;
      }
      cmd = 0xFE;
      operation[0] = 0xFE;
    case 0xFE:
      for(uint8_t sensor = 0; sensor < 16; sensor++){
        number = readADC(sensor, SENSOR_PIN);
        Serial.print(number);
        Serial.print(" ");
        minGreen[sensor] = min(minGreen[sensor], number);     
      }
      Serial.println();
      break;
    case 0xFD: {
      vx = vy = 0;
      float totalWeight = 0;
      float vecX[16], vecY[16];
      for (uint8_t sensor = 0; sensor < 16; sensor++) {
        memcpy_P(&vecX[sensor], &sensVecs[sensor][0], sizeof(float));
        memcpy_P(&vecY[sensor], &sensVecs[sensor][1], sizeof(float));
      }
      // 1. Используем непрерывные значения, а не порог
      for (uint8_t sensor = 0; sensor < 16; sensor++) {
        uint16_t value = readADC(sensor, SENSOR_PIN);
        Serial.print(value);
        Serial.print(" ");
        
        // Чем меньше значение (темнее), тем больше вес
        if (value < minGreen[sensor]) {
          
          vx += sensVecs[sensor][0];
          vy += sensVecs[sensor][1];
        }
      }
      Serial.println();
      
      // 2. Определяем наличие линии
      if (totalWeight < 0.1) { // Порог уверенности
        line = 0;
        angle = 0;
      } else {
        line = 1;
        // Нормализуем вектор
        if (totalWeight > 0) {
          vx /= totalWeight;
          vy /= totalWeight;
        }
        angle = round(atan2(vy, vx) * 57.296);
        Serial.println(angle);
      }
      
      // 3. Пакет
      package = 1;
      
      break;
    }
    default:
      vx = vy = 0;
      bool W = 0;
      
      // 1. Используем непрерывные значения, а не порог
      for (uint8_t sensor = 0; sensor < 16; sensor++) {
        uint16_t value = readADC(sensor, SENSOR_PIN);
        
        // Чем меньше значение (темнее), тем больше вес
        if (value < minGreen[sensor] - 5) {

          vx += sensVecs[sensor][0];
          vy += sensVecs[sensor][1];
          W = 1;
        }
      }
      
      // 2. Определяем наличие линии
      if (!W) { // Порог уверенности
        line = 0;
        angle = 0;
      } else {
        line = 1;
        angle = round((atan2(vy, vx) - M_PI) * 57.296);
        Serial.println(angle);
      }
      
      // 3. Пакет
      package = 1;

      break;
  }
}

bool type = 0;
void loop1() {
    // Читаем все доступные CAN-сообщения
    if(!digitalRead(CAN_INT)) {
        unsigned long id;
        unsigned char len;
        unsigned char buf[8];
        if(CAN.readMsgBuf(&id, &len, buf) == CAN_OK) {
            // Нас интересуют только запросы с ID=1 и длиной >=2
            if(id == 1) {
                  type = (bool)buf[0];   // Тип 1 = мяч, 2 = линия
                  static uint32_t last_print = 0;
                  if(millis() - last_print > 200) {
                      last_print = millis();
                      Serial.printf("Request type=%d\n", type);
                  }
                }
            }
        }

    
    // Отправляем ответ НЕМЕДЛЕННО после получения запроса типа 2 (линия)
    if(type == 1) {
        // Свежие данные line и angle уже обновляются в loop() на core0
        information[0] = line;
        information[1] = angle & 0xFF;
        information[2] = (angle >> 8) & 0xFF;
        information[3] = 0;
        information[4] = 0;
        information[5] = 0;
        information[6] = 0;
        information[7] = 0;
        
        // Отправляем с ID = 110 (0x6E)
        if(CAN.sendMsgBuf(0x6E, 0, 8, information) == CAN_OK) {
            #if NO_SERIAL_DEBUG == 0
            static uint32_t last_sent = 0;
            if(millis() - last_sent > 200) {
                last_sent = millis();
                Serial.printf("Response sent: line=%d angle=%d\n", line, angle);
            }
            #endif
        }
      type = 0;
    }
}
// ========== CAN FUNCTIONS ==========
void send(uint16_t id, uint8_t *data) {
  CAN.sendMsgBuf(id, 0, 8, data);
  
  Serial.print("Отправлено сообщение ID: 0x");
  Serial.print(id, HEX);
  Serial.print(", Data: ");
  
  for (int i = 0; i < 8; i++) {
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

float fastSqrt(float x) {
  // Быстрый приближенный квадратный корень (Quake trick)
  union {
    float f;
    uint32_t i;
  } u = {x};
  u.i = 0x1FBD3E20 + (u.i >> 1);
  return u.f;
}
// Исправленная версия fastAtan2 (без переопределения PI)
int16_t fastAtan2(float y, float x) {
  // Быстрый приближенный atan2 (ошибка < 0.005 радиан)
  const float MY_PI = 3.14159265f;
  const float MY_PI_2 = 1.57079633f;
  
  float abs_y = fabsf(y);
  float angle;
  
  if (x == 0.0f) {
    angle = MY_PI_2;
  } else {
    float r = abs_y / fabsf(x);
    angle = 0.273f * r - 0.076f * r * r + 0.0246f * r * r * r;
    if (x < 0) angle = MY_PI - angle;
  }
  
  if (y < 0) angle = -angle;
  return round((angle - MY_PI) * 57.29578f);
}

void readCANMessages() {
  if(!digitalRead(CAN_INT)) {
    CAN.readMsgBuf(&rxId, &len, rxBuf);
    
    // Print message info
    if((rxId & 0x80000000) == 0x80000000) {
      Serial.printf("Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len);
    } else {
      Serial.printf("Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);
    }
  
    if((rxId & 0x40000000) == 0x40000000) {
      Serial.print(" REMOTE REQUEST FRAME");
    } else {
      for(byte i = 0; i < len; i++) {
        Serial.printf(" 0x%.2X", rxBuf[i]);
      }
      operation[0] = rxBuf[0];
      operation[1] = rxBuf[1];
      operation[2] = rxBuf[2];
      operation[3] = rxBuf[3];
      operation[4] = rxBuf[4];
      operation[5] = rxBuf[5];
      operation[6] = rxBuf[6];
      operation[7] = rxBuf[7];
      Serial.println("OK");
    }

    Serial.println();
  }
}

// ========== MULTIPLEXER FUNCTIONS ==========
void setChannel(byte channel) {
  digitalWrite(sig1, channel & 1);
  digitalWrite(sig2, (channel >> 1) & 1);
  digitalWrite(sig3, (channel >> 2) & 1);
  digitalWrite(sig4, (channel >> 3) & 1);
}

int readADC(byte channel, int pin) {
  static byte channelOld = 0;
  
  if(channel != channelOld) {
    setChannel(channel);
    channelOld = channel;
  }
  
  return analogRead(pin);
}

int readNADC(byte channel, int pin) {
  static byte channelOld = 0;
  
  if(channel != channelOld) {
    setChannel(channel);
    channelOld = channel;
  }
  return 1;
//  return map(analogRead(pin), normal[channel][0], normal[channel][1], 100, 900);
}

// ========== LED FUNCTIONS ==========
void colorWipe(uint32_t color, int wait) {
  for(int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, color);
    strip.show();
    delay(wait);
  }
}

void rainbow(int wait) {
  for(long firstPixelHue = 0; firstPixelHue < 5 * 65536; firstPixelHue += 256) {
    strip.rainbow(firstPixelHue);
    strip.show();
    delay(wait);
  }
}