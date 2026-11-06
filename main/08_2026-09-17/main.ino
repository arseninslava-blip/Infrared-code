#include <Arduino.h>
#include <math.h>

#include <Ticker.h>
Ticker kick_timer;
struct Vec2 {
  float pos_x;
  float pos_y;
};

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
  //Serial.println("Start 0 core");

  SPI.setMOSI(CAN_MOSI_MOTOR);
  SPI.setMISO(CAN_MISO_MOTOR);
  SPI.setSCK(CAN_SCK_MOTOR);
  SPI.begin();
  while (motorCAN.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) != CAN_OK) delay(100);
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
    DefenseMode();
    //defense2();
    //defenseMode2();
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
  //readMotorFeedback(); // нужен для одометрии (computeRobotVelocity / defense2)
}



// Известные точки, через которые проходит каждая прямая пеленга
// o1 — жёлтая метка, o2 — синяя метка
const float _RELOC_O1_X = -110.0f;
const float _RELOC_O1_Y = 0.0f;
const float _RELOC_O2_X = 110.0f;
const float _RELOC_O2_Y = 0.0f;

// "Глобальный" курс: BNO055 в NDOF при каждом перезапуске/переприкладывании
// питания даёт свой "0" (магнитометр ещё не откалибровался, наводки от
// моторов и т.п.), поэтому сырой imuData.euler[0] от запуска к запуску не
// совпадает для одной и той же физической ориентации робота. headingOffset
// фиксирует курс в момент первого валидного чтения IMU после старта, и
// дальше используется разница (euler[0] - headingOffset) - она не зависит
// от того, куда смотрел робот в момент включения.
float headingOffset = 0.0f;
bool headingCalibrated = false;

float normalizeDeg(float a) {
  a = fmod(a + 180.0f, 360.0f);
  if (a < 0) a += 360.0f;
  return a - 180.0f;
}

float globalHeading() {
  return normalizeDeg(imuData.euler[0] - headingOffset);
}

Vec2 reloc(float YgoalD, float BgoalD) {
  /*
    Вычисляет позицию (x, y) по двум угловым пеленгам.
    YgoalD — угол от жёлтой точки o1(-110, 0)
    BgoalD — угол от синей точки o2(110, 0)
  */

  // gate_dir_1/gate_dir_2 и глобальный курс - в градусах, а tan() ждёт радианы.
  float heading = globalHeading();
  YgoalD = PI - (YgoalD - heading) * DEG_TO_RAD;
  BgoalD = PI - (BgoalD - heading) * DEG_TO_RAD;

  // Каждый пеленг — прямая y = k*x + b, k = tg(угла), b находится из
  // известной точки на прямой (b = y0 - k*x0).
  float k1 = tan(YgoalD);
  float k2 = tan(BgoalD);

  Vec2 result;
  result.pos_x = 0.0f;
  result.pos_y = 0.0f;

  // Защита от деления на ноль (параллельные лучи — решения нет)
  if (fabs(k1 - k2) < 1e-9f) {
    return result; // (0, 0)
  }

  float b1 = _RELOC_O1_Y - k1 * _RELOC_O1_X;
  float b2 = _RELOC_O2_Y - k2 * _RELOC_O2_X;

  result.pos_x = (b2 - b1) / (k1 - k2);
  result.pos_y = k1 * result.pos_x + b1;

  return result;
}

// Буфер для sprintf (запас по размеру)
char reloc_buf[64];

// Вывод ограничен по времени константой SERIAL_INTERVAL (как в printCamData),
// иначе loop1() заваливает Serial сообщениями на каждой итерации.
uint32_t lastRelocPrint = 0;

void printRelocResult(float YgoalD, float BgoalD) {
  Vec2 p = reloc(YgoalD, BgoalD);

  if (millis() - lastRelocPrint < SERIAL_INTERVAL) return;
  lastRelocPrint = millis();

  // Форматируем строку: "X=123.45, Y=67.89"
  sprintf(reloc_buf, "X=%.2f, Y=%.2f, alfa=%.2f, beta=%.2f", p.pos_x, p.pos_y, YgoalD, BgoalD);

  Serial.println(reloc_buf);
}

void setup1(){
  Serial.begin(115200);
  //Serial.println("Start 1 core");
  Wire1.setSDA(SDA_PIN);
  Wire1.setSCL(SCL_PIN);
  Wire1.begin();
  Wire1.setClock(400000);
  //Serial.println("Core I2C start");

  // BNO055 после подачи питания просыпается не мгновенно (~650-850мс POR).
  // Если дёрнуть beginIMU() раньше, чтение chip ID проваливается на обоих
  // адресах, функция возвращает false и БОЛЬШЕ НИКОГДА не переключит сенсор
  // в NDOF за всю сессию - он так и останется в CONFIGMODE (euler всегда 0).
  // Даём время на POR, затем пробуем несколько раз с паузой, но не виснем
  // тут навечно, если сенсора реально нет/он сломан - cam_read() в loop1()
  // должен крутиться в любом случае.
  delay(800);
  for (uint8_t attempt = 0; attempt < 5 && !beginIMU(); attempt++) {
    delay(200);
  }
}
void loop1(){
  cam_read();
  readBNOData(imuData);
  if (!headingCalibrated) {
    headingOffset = imuData.euler[0];
    headingCalibrated = true;
  }
  //ball_detect();
  printRelocResult(gate_dir_2, gate_dir_1);
}
