#include <Arduino.h>
#include <Ticker.h>
Ticker kick_timer;

#include "config.h"
#include <SPI.h>
#include <mcp_can.h>
MCP_CAN motorCAN(CAN_CS_MOTOR);
#include <Wire.h>

volatile bool line_be = 0;
volatile int line_dir = 0;

volatile int16_t ball_dir = 0;
volatile int16_t gate_dir_1 = 0;
volatile int16_t gate_dir_2 = 0;
volatile int16_t cam_line_dir = 0;
volatile int16_t ball_dist = 0;

#include "structs.h"
BNOData imuData;

#include "imu.h"
#include "functions.h"

void setup() {
  Serial.begin(115200);
  Serial.println("Start 0 core");

  SPI.setMOSI(CAN_MOSI_MOTOR);
  SPI.setMISO(CAN_MISO_MOTOR);
  SPI.setSCK(CAN_SCK_MOTOR);
  SPI.begin();
  while (motorCAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) delay(100);
  motorCAN.setMode(MCP_NORMAL);

  pinMode(MODE_BTN, INPUT_PULLUP);
  pinMode(KICK, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  kick_timer.attach_ms(KICK_COOLDOWN_MS, armKick);
  allMotorsOn();
}

int f = 0;
bool flagB = 0;
void loop() {
  if (!digitalRead(MODE_BTN) && flagB == 0) {
    flagB = 1;
  }
  if (digitalRead(MODE_BTN) && flagB) {
    flagB = 0;
    f = !f;
  }
  //f=1;
  //digitalWrite(LED_BUILTIN, 1);
    //DefenseMode();
  //AtackMode();
  if (f == 1) {
    //digitalWrite(LED_BUILTIN, 1);
    //DefenseMode();
    AtackMode();
    //int d = millis() % 500 * 90;
    //readBNOData(imuData);
    //printBNOData(imuData);
    //int imU = -(imuData.euler[0] - 360 * (imuData.euler[0] > 180));
    //Serial.println(imU);
    //omniMoveDirection(1000, 90, 0);
  } else {
    //allMotorsStop();
    //omniMoveDirection(100, -90, 0);
    //digitalWrite(LED_BUILTIN, 0);
    //Serial.println(ball_dir);
  }
}

void setup1(){
  Serial.begin(115200);
  Serial.println("Start 1 core");
  Wire1.setSDA(SDA_PIN);
  Wire1.setSCL(SCL_PIN);
  Wire1.begin();
  Wire1.setClock(400000);
  Serial.println("Core I2C start");
  beginIMU();
}
void loop1(){
  cam_read();
  //ball_detect();
}
