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

// ========== SETUP ==========
void setup() {
  // Initialize LED strip
  strip.begin();
  strip.show();            // Turn OFF all pixels
  strip.setBrightness(255);
  
  colorWipe(strip.Color(255, 0, 0), 50);  // Red - startup indicator

  // Initialize Serial
  Serial.begin(115200);

  // Configure SPI1 pins
  SPI1.setRX(12);   // MISO на GP16
  SPI1.setTX(11);   // MOSI на GP19
  SPI1.setSCK(10);  // SCK на GP18
  SPI1.begin();

  // Initialize CAN controller
  if(CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("MCP2515 Initialized Successfully!");
  } else {
    while(1) {
      Serial.println("Error Initializing MCP2515...");
    }
  }

  CAN.setMode(MCP_NORMAL);   // Normal mode for transmitting messages
  pinMode(CAN_INT, INPUT);
}

void setup1(){
  // Initialize multiplexer pins
  int pins[4] = {sig1, sig2, sig3, sig4};
  for(int pin : pins) {
    pinMode(pin, OUTPUT);
  }
}
// ========== MAIN LOOP ==========
bool package = 0;

int minGreen[16] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023};

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
  colorWipe(strip.Color(255, 0, 0), 5);  // Red - indicate activity
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
        minGreen[sensor][0] = min(minGreen[sensor][0], number);     
      }
      Serial.println();
      break;
    case 0xFD:
      vx, vy = 0, 0;

      for(uint8_t sensor = 0; sensor < 16; sensor++){
        number = readADC(sensor, SENSOR_PIN);
        Serial.print(number);
        Serial.print(" ");
        if(number < minGreen[sensor]){
          vx += sensVecs[sensor][0];
          vy += sensVecs[sensor][1]; 
          //colorWipe(strip.Color(100, 0, 0), 5);  // Red - indicate activity    
        }
      }
      Serial.println();
      if(vx==0 && vy == 0){
        line = 0;
        angle = 0;
      } else{
        line = 1;

        angle = round(atan2(vy, vx) * 57.296);
        Serial.println(angle);
      }
      package = 1;
      information[0] = line;                  // Значение line (усекается до 1 байта)
      information[1] = angle & 0xFF;         // Младший байт угла
      information[2] = (angle >> 8) & 0xFF; // Старший байт угла
      information[3] = 0x00;              // Запасной байт 1
      information[4] = 0x00;              // Запасной байт 2
      information[5] = 0x00;              // Запасной байт 3
      information[6] = 0x00;              // Запасной байт 4
      information[7] = 0x00;              // Запасной байт 5
      break;
    default: break;
  }
}

void loop1(){
  readCANMessages();
  if (package){
    send(0x6D, information);
    package = 0;
  }
  cmd = operation[0];
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
  
  return map(analogRead(pin), 0, 1023, 0, 1023);
}

int readNADC(byte channel, int pin) {
  static byte channelOld = 0;
  
  if(channel != channelOld) {
    setChannel(channel);
    channelOld = channel;
  }
  
  return map(analogRead(pin), normal[channel][0], normal[channel][1], 100, 900);
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