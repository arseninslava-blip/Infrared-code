#include <Arduino.h>
#include <Ticker.h>
#include <SPI.h>
#include <mcp_can.h>
#include <Wire.h>
#include "config.h"
#include "structs.h"
#include "imu.h"
#include <Servo.h>
#include "functions.h"

Servo dribble;

Ticker kick_timer;
MCP_CAN motorCAN(CAN_CS_MOTOR);

Motor motors[4];
CamData camData;
LineData lineData;
BNOData imuData;
volatile bool kickFlag = false;

bool defenseActive = false;
bool modeBtnLatched = false;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, 1);
    Serial.begin(115200);
    Serial.println("Start core 0");

    SPI.setMOSI(CAN_MOSI_MOTOR);
    SPI.setMISO(CAN_MISO_MOTOR);
    SPI.setSCK(CAN_SCK_MOTOR);
    SPI.begin();
    while (motorCAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
        delay(100);
    }
    motorCAN.setMode(MCP_NORMAL);

    pinMode(MODE_BTN, INPUT_PULLUP);
    pinMode(KICK, OUTPUT);
    pinMode(BALL_DETECT_PIN, INPUT);

    delay(1000);
    dribbleInit();

    kick_timer.attach_ms(KICK_COOLDOWN_MS, armKick);
    allMotorsOn();
    digitalWrite(LED_BUILTIN, 0);
    while (digitalRead(MODE_BTN)){}

}

void loop() {
    if (!digitalRead(MODE_BTN) && !modeBtnLatched) {
        modeBtnLatched = true;
    }
    if (digitalRead(MODE_BTN) && modeBtnLatched) {
        modeBtnLatched = false;
        defenseActive = !defenseActive;
    }

    if (defenseActive) {
        digitalWrite(LED_BUILTIN, 0);
        //DefenseMode();
        AtackMode();
        //omniMoveDirection(1000 , 0, 0, false);
    } else {
        allMotorsStop();
        digitalWrite(LED_BUILTIN, 0);
    }
    //kicker();
    readMotorEncoders();
    syncMotorsByEncoder();
    printAndResetEncoders();
}

void setup1() {
    Serial.println("Start core 1");
    Wire1.setSDA(SDA_PIN);
    Wire1.setSCL(SCL_PIN);
    Wire1.begin();
    Wire1.setClock(400000);
    beginIMU();
}

void loop1() {
    cam_read();
    readLine();
    readBNOData(imuData);
    //printAllSensors();
}
