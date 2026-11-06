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

// Время (millis) последнего УСПЕШНОГО чтения датчика; 0 = ещё ни разу.
// Пишет ядро 1, проверяет ядро 0 (см. camFresh/imuFresh в functions.h).
volatile uint32_t camStamp = 0;
volatile uint32_t lineStamp = 0;
volatile uint32_t imuStamp = 0;
volatile float imuYawRate = 0;   // град/с, знак как у heading; считает readBNOeuler (ядро 1)

// Согласованный снимок данных камеры между ядрами (seqlock, см. cam_read/camSnapshot):
// camSeqLock нечётный, пока ядро 1 пишет пакет. camFrameCount растёт, когда изменились
// ball_dir или gate_dir_2, camFrameMs - момент этого изменения.
volatile uint32_t camSeqLock = 0;
volatile uint32_t camFrameCount = 0;
volatile uint32_t camFrameMs = 0;

#include "structs.h"
BNOData imuData;

#include "imu.h"
#include "functions.h"
#include "buzzer.h"

void setup() {
  Serial.begin(115200);

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
int f2 = 0;
bool flagB2 = 0;
unsigned long tmrTICKS0 = 0;
unsigned long tmr2TICKS0 = 0;

void loop() {
  tmrTICKS0 = millis();

  if (!digitalRead(MODE_BTN) && flagB == 0) {
    flagB = 1;
  }
  if (digitalRead(MODE_BTN) && flagB) {
    flagB = 0;
    f = !f;
  }

  if (f == 1) {
    //digitalWrite(LED_BUILTIN, 1);

    AtackMode();

    //DefenseMode();

  } else {
    allMotorsStop();
  }
  //readMotorFeedback(); // нужен для одометрии (computeRobotVelocity / defense2)

  tmr2TICKS0 = millis() - tmrTICKS0;
  //Serial.println(tmr2TICKS0);
}

void setup1(){
  Serial.begin(115200);
  buzzerBegin();          // мелодия на пищалке: таймер создаётся здесь, на ядре 1
  delay(800);
  Wire1.setSDA(SDA_PIN);
  Wire1.setSCL(SCL_PIN);
  Wire1.begin();
  Wire1.setClock(400000);

  // BNO055 после подачи питания просыпается не мгновенно (~650-850мс POR).

  for (uint8_t attempt = 0; attempt < 5 && !beginIMU(); attempt++) {
    delay(200);
  }
}

unsigned long tmrTICKS = 0;
unsigned long tmr2TICKS = 0;
void loop1(){
  tmrTICKS = millis();
  if (!digitalRead(MODE_BTN2) && flagB2 == 0) {
    flagB2 = 1;
  }
  if (digitalRead(MODE_BTN2) && flagB2) {
    flagB2 = 0;
    f2 = !f2;
  }

  cam_read();
  readBNOeuler(imuData);
  readLine();
  buzzerLoop();           // переключение мелодии по Serial (1 / 2)
  buzzerEnable(*(volatile int *)&f2 == 0);   // f - кнопка на ядре 0: мелодия играет, пока робот стоит, и молчит, пока едет

  tmr2TICKS = millis() - tmrTICKS;
  //Serial.println(tmr2TICKS);
}