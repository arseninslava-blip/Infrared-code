#include <Arduino.h>
#include "config.h"
#include <SPI.h>
#include <mcp_can.h>
MCP_CAN motorCAN(CAN_CS_MOTOR);
#include <Wire.h>

volatile bool line_be = 0;
volatile int line_dir = 0;

volatile int16_t ball_dir = 0;
volatile int16_t y_gate_dir = 0;
volatile int16_t b_gate_dir = 0;
volatile int16_t cam_line_dir = 0;
volatile uint16_t trigger = 0;
volatile int gate_dir = 0;
volatile int gate_dir2 = 0;

#include "functions.h"

void setup() {
  Serial.begin(115200);
  Serial.println("Start 0 core");

  Wire1.setSDA(SDA_PIN);
  Wire1.setSCL(SCL_PIN);
  Wire1.begin();
  Wire1.setClock(400000);
  Serial.println("Core I2C start");

  SPI.setMOSI(CAN_MOSI_MOTOR);
  SPI.setMISO(CAN_MISO_MOTOR);
  SPI.setSCK(CAN_SCK_MOTOR);
  SPI.begin();
  while (motorCAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) delay(100);
  motorCAN.setMode(MCP_NORMAL);

  pinMode(KICK, OUTPUT);
  pinMode(BTN_1, INPUT_PULLUP);
  pinMode(BTN_2, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  allMotorsOn();
}
int f = 0;
bool flagB = 0;
void loop() {
  defKick();

  if(!digitalRead(BTN_2) && flagB == 0){
    flagB = 1;
  } 
  if(digitalRead(BTN_2) && flagB){
    flagB = 0;
    f = !f;
  }

  if(f == 1) {
    //omniMoveDirection(2500, ball_dir, 35*(ball_dir));
    if (b_gate_dir < 1){
      kicker();
    }
    if(b_gate_dir!= -1){
      omniMoveDirection(10000, b_gate_dir, y_gate_dir * 15);
    } else{
      omniMoveDirection(10000, ball_dir * 1.5 - y_gate_dir, y_gate_dir * 15);
    }
    digitalWrite(LED_BUILTIN, 1);
  }
  else {
    allMotorsStop();
    digitalWrite(LED_BUILTIN, 0);
  }
  
}


void setup1(){
  Serial.begin(115200);
  Serial.println("Start 1 core");

}
void loop1(){
  cam_read();
}