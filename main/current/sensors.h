// ============================================================================
//  Датчики на шине Wire1 (опрашивает ядро 1)
// ============================================================================
// Шина Wire1 и её восстановление, камера OpenMV, датчик линии, гироскоп BNO055.

#pragma once
#include <Wire.h>
#include <pico/mutex.h>
#include "config.h"
#include "state.h"

// ============================================================================
//  Шина Wire1 (камера, IMU, датчик линии): запуск, таймаут, освобождение
// ============================================================================
// Если ведомый не отпускает шину (оборвалась посылка, и он держит SDA), каждое чтение
// ждёт до конца таймаута и молчат все три датчика сразу. По умолчанию таймаут Wire — 1 с,
// и цикл ядра 1 растягивался до 3 с. Поэтому:
//   - таймаут одной посылки I2C_TIMEOUT_MS; после него ядро Wire перезапускает свой I2C;
//   - если ни одно устройство не отвечает дольше I2C_STUCK_MS, шина освобождается вручную:
//     до 9 тактов SCL (ведомый дошлёт байт и отпустит SDA), STOP и Wire1 заново.
// Если SCL держит сама камера (растянула такт и не отпускает), отсюда это не лечится:
// камера через 0,5 с без обмена перезапускает свой I2C (I2C_RESET_MS в EXP-5.py).


volatile uint32_t i2cRecoveries = 0;   // сколько раз освобождали шину (пишет ядро 1)

void i2cBegin() {
  Wire1.setSDA(SDA_PIN);
  Wire1.setSCL(SCL_PIN);
  Wire1.begin();
  Wire1.setClock(I2C_CLOCK_HZ);
  Wire1.setTimeout(I2C_TIMEOUT_MS, true);   // true — после таймаута ядро Wire перезапускает I2C
}

// Линия шины как «открытый сток»: у вывода с INPUT_PULLUP digitalWrite(LOW) тянет к земле,
// digitalWrite(HIGH) отпускает на подтяжку (так устроен digitalWrite в arduino-pico)
static void i2cPulse(uint8_t pin, PinStatus level) {
  digitalWrite(pin, level);
  delayMicroseconds(5);   // ~100 кГц
}

void i2cRecover() {
  Wire1.end();
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
  delayMicroseconds(5);

  // Ведомый посреди байта держит SDA: тактуем, пока не отпустит (больше 9 тактов не нужно)
  for (uint8_t i = 0; i < 9 && digitalRead(SDA_PIN) == LOW; i++) {
    i2cPulse(SCL_PIN, LOW);
    i2cPulse(SCL_PIN, HIGH);
  }
  // STOP: SDA снизу вверх при отпущенной SCL — ведомые сбрасывают своё состояние
  i2cPulse(SCL_PIN, LOW);
  i2cPulse(SDA_PIN, LOW);
  i2cPulse(SCL_PIN, HIGH);
  i2cPulse(SDA_PIN, HIGH);

  i2cBegin();
  i2cRecoveries++;
}

// Ядро 1, в конце прохода loop1: anyOk — ответило ли хоть одно устройство.
// Никто не отвечает дольше I2C_STUCK_MS — шина зависла, освобождаем (не чаще раза в I2C_STUCK_MS).
void i2cWatch(bool anyOk) {
  static uint32_t lastOk = 0;
  if (anyOk) {
    lastOk = millis();
    return;
  }
  if (millis() - lastOk < I2C_STUCK_MS) return;
  i2cRecover();
  lastOk = millis();
}

// ============================================================================
//  Камера OpenMV — пакет 24 байта по I2C (адрес CAM_ADDR)
// ============================================================================
// cam_read() — ядро 1, каждый проход loop1; camTake() — ядро 0, начало loop().
// Пакет — 12 чисел int16: мяч°, ворота 1°, ворота 2°, от линии°, дистанция до мяча, захват,
// X×10, Y×10 (положение на поле), ось поля°, дистанция до линии, от дальней линии°, дистанция до неё.
// Формат должен совпадать с _PACKET_FMT в камере.


const size_t CAM_PACKET_LEN = 24;
const int16_t CAM_POS_NONE = INT16_MIN;   // положение неизвестно: в кадре не обе пары ворот

volatile uint32_t camErrors = 0;   // неудачных чтений камеры с запуска (пишет ядро 1)

bool camFresh() { return isFresh(camStamp, SENSOR_TIMEOUT); }

// Пакет камеры целиком. Ядро 1 кладёт его в camShared под мьютексом, ядро 0 забирает копию
// в начале loop(): раньше ядро 0 могло прочитать ball_dir из нового пакета, а ball_dist из старого.
struct CamPacket {
  int16_t ballDir, gate2, gate1, lineDir, ballDist;
  bool trig;
  int16_t posX10, posY10, fieldAxis;
  uint32_t seq;   // номер принятого пакета: камера шлёт один пакет на кадр
  int16_t outDist;
  int16_t lineDir2, outDist2;   // дальняя зона линий (LINE_ROI2_R в камере)
};
CamPacket camShared = { 0, 0, 0, 0, 0, false, CAM_POS_NONE, CAM_POS_NONE, 0, 0, 0, -1, -1 };
auto_init_mutex(camMutex);

// Фильтр координат поля по кадрам камеры (альфа-бета): предсказывает положение по скорости и
// поправляет его на долю FIELD_FILTER_ALPHA ошибки, скорость — на FIELD_FILTER_BETA. За счёт скорости
// он убирает шум почти без запаздывания. После перерыва фильтр начинает заново с нового значения.
void filterFieldPos(float x, float y, bool restart) {
  static float vx = 0, vy = 0;   // см/с
  static uint32_t lastMs = 0;
  uint32_t now = millis();
  float dt = (now - lastMs) / 1000.0f;
  lastMs = now;
  if (restart || dt <= 0 || dt > FIELD_FILTER_MAX_GAP) {
    field_x = x;
    field_y = y;
    vx = vy = 0;
    return;
  }
  float px = field_x + vx * dt, py = field_y + vy * dt;
  float ex = x - px, ey = y - py;
  field_x = px + FIELD_FILTER_ALPHA * ex;
  field_y = py + FIELD_FILTER_ALPHA * ey;
  vx += FIELD_FILTER_BETA * ex / dt;
  vy += FIELD_FILTER_BETA * ey / dt;
}

// true — пакет принят
bool cam_read() {
  uint8_t buffer[CAM_PACKET_LEN];
  size_t bytesRead = Wire1.requestFrom(CAM_ADDR, CAM_PACKET_LEN);

  if (bytesRead == CAM_PACKET_LEN) {
    for (size_t i = 0; i < CAM_PACKET_LEN; i++) {
      buffer[i] = Wire1.read();
    }

    // Распаковка углов (little-endian)
    CamPacket p;
    p.ballDir = buffer[0] | (buffer[1] << 8);   // угол — от центра зеркала, поправка не нужна
    p.gate1 = buffer[2] | (buffer[3] << 8);
    p.gate2 = buffer[4] | (buffer[5] << 8);
    p.lineDir = buffer[6] | (buffer[7] << 8);
    p.ballDist = buffer[8] | (buffer[9] << 8);
    p.outDist = buffer[18] | (buffer[19] << 8);
    p.trig = buffer[10] | (buffer[11] << 8);
    p.posX10 = buffer[12] | (buffer[13] << 8);   // X уже в см поля: камера пересчитывает по зеркалу
    p.posY10 = buffer[14] | (buffer[15] << 8);
    p.fieldAxis = buffer[16] | (buffer[17] << 8);
    p.lineDir2 = buffer[20] | (buffer[21] << 8);
    p.outDist2 = buffer[22] | (buffer[23] << 8);
    static uint32_t seq = 0;
    p.seq = ++seq;

    mutex_enter_blocking(&camMutex);
    camShared = p;
    mutex_exit(&camMutex);
    camStamp = millis();
    return true;
  }
  camErrors++;   // число сбоев за интервал — в строке состояния
  return false;
}

// Ядро 0, начало loop(): копия последнего пакета камеры в рабочие переменные
void camTake() {
  mutex_enter_blocking(&camMutex);
  CamPacket p = camShared;
  mutex_exit(&camMutex);

  ball_dir = p.ballDir;
  gate_dir_2 = p.gate2;
  gate_dir_1 = p.gate1;
  cam_line_dir = p.lineDir;
  ball_dist = p.ballDist;
  out_dist = p.outDist;
  cam_line_dir_2 = p.lineDir2;
  out_dist_2 = p.outDist2;
  trig = p.trig;

  // Фильтр — только по новому кадру: camTake() вызывается чаще, чем приходят пакеты
  static uint32_t lastSeq = 0;
  bool newFrame = p.seq != lastSeq;
  lastSeq = p.seq;

  bool wasValid = field_valid;
  field_valid = p.posX10 != CAM_POS_NONE;
  if (field_valid && newFrame) {
    filterFieldPos(p.posX10 / 10.0f, p.posY10 / 10.0f, !wasValid);
    field_axis = p.fieldAxis;
    fieldAxisYaw = imuData.euler[0];   // курс в момент кадра: между кадрами ось поправляет гироскоп
  }
}

// ============================================================================
//  Датчик линии (PicoLineFollower) — 3 байта по I2C (адрес LINE_ADDR)
// ============================================================================
// readLine() — ядро 1, каждый проход loop1.

volatile uint32_t lineErrors = 0;   // неудачных чтений датчика линии с запуска (пишет ядро 1)

// Датчик линии на связи: последний ответ не старше SENSOR_TIMEOUT
bool lineFresh() { return isFresh(lineStamp, SENSOR_TIMEOUT); }

bool readLine() {
  uint8_t buf[3];
  if (Wire1.requestFrom(LINE_ADDR, 3) != 3) {
    lineErrors++;   // число сбоев за интервал — в строке состояния
    // Нет ответа: не держим старое "линия видна" дольше LINE_TIMEOUT,
    // иначе робот будет вечно ехать "от линии".
    if (millis() - lineStamp > LINE_TIMEOUT) line_be = false;
    return false;
  }
  for (uint8_t i = 0; i < 3; i++) buf[i] = Wire1.read();
  // line_dir - одним присваиванием и раньше line_be: ядро 0 читает их без блокировок
  // и не должно увидеть промежуточное значение (до вычитания 180).
  line_dir = (int16_t)(buf[1] | (buf[2] << 8)) - 180;
  line_be = buf[0];
  lineStamp = millis();
  return true;
}

// ============================================================================
//  BNO055 (гироскоп/IMU) - читается по уже настроенному Wire1 (см. setup1())
// ============================================================================


// Реальный адрес определяется в beginIMU() (0x29 или 0x28 - зависит от COM3).
uint8_t bnoAddr = BNO_ADDR;

volatile uint32_t imuErrors = 0;   // неудачных чтений курса с запуска (пишет ядро 1)

// Режим работы BNO055 (регистр OPR_MODE): 0x0C — NDOF, курс считается. 0 — CONFIGMODE:
// так бывает после сброса гироскопа (просадка питания) — чтение проходит без ошибок,
// а курс всё время 0. 0xFF — режим ещё не читали или гироскоп не ответил.
const uint8_t BNO_MODE_NDOF = 0x0C;
volatile uint8_t imuMode = 0xFF;

bool bnoWriteReg(uint8_t reg, uint8_t val) {
    Wire1.beginTransmission(bnoAddr);
    Wire1.write(reg);
    Wire1.write(val);
    Wire1.endTransmission();
    delay(30);
    return true;
}

bool bnoReadRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
    Wire1.beginTransmission(bnoAddr);
    Wire1.write(reg);
    uint8_t txErr = Wire1.endTransmission(false);
    if (txErr != 0) return false; // нет ACK на этом адресе - устройства там нет
    if (Wire1.requestFrom((uint8_t)bnoAddr, len) != len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = Wire1.read();
    return true;
}

bool beginIMU() {
    uint8_t id;
    const uint8_t candidates[2] = { BNO_ADDR, BNO_ADDR_ALT };

    for (uint8_t c = 0; c < 2; c++) {
        bnoAddr = candidates[c];
        if (bnoReadRegs(BNO_CHIP_ID_REG, &id, 1)) {
            if (id == 0xA0) {
                //Serial.print("BNO055 найден на 0x"); Serial.println(bnoAddr, HEX);
                break;
            }
        } else {
            //Serial.print("нет ответа (NACK) по адресу 0x"); Serial.println(bnoAddr, HEX);
        }
        if (c == 1) {
            //Serial.println("не удалось считать гироскоп (BNO055 не найден ни на 0x28, ни на 0x29) - проверь питание, SDA/SCL и подтяжки (4.7к на 3.3В)");
            return false;
        }
    }

    bnoWriteReg(0x3D, 0x00); // CONFIGMODE
    bnoWriteReg(0x3F, 0x20); // system reset
    delay(700);
    bnoWriteReg(0x3B, 0x00); // units: m/s^2, dps, degrees, celsius
    bnoWriteReg(0x3D, 0x0C); // NDOF (sensor fusion)
    delay(100);
    return true;
}

bool readBNOeuler(BNOData &data){
  uint8_t buf[6];

  // При неудачном чтении buf не заполнен - data не трогаем, остаётся прошлое значение,
  // а imuStamp не обновляется (ядро 0 по нему поймёт, что курс устарел).
  if (!bnoReadRegs(0x1A, buf, 6)) {
    imuErrors++;   // число сбоев за интервал — в строке состояния
    return false;
  }

  for (int i = 0; i < 3; i++) data.euler[i] = (int16_t)((buf[i * 2 + 1] << 8) | buf[i * 2]) / 16.0f;
  // курс с учётом установки датчика (IMU_DIR, IMU_ZERO в config.h), 0..360
  float h = IMU_DIR * data.euler[0] + IMU_ZERO;
  h = fmodf(h, 360.0f);
  data.euler[0] = h < 0 ? h + 360.0f : h;
  imuStamp = millis();
  return true;
}

bool imuFresh() { return isFresh(imuStamp, SENSOR_TIMEOUT); }

// Ядро 1, не чаще раза в IMU_MODE_CHECK_MS: прочитать режим BNO055 (см. imuMode)
void checkIMUMode() {
  static uint32_t last = 0;
  if (millis() - last < IMU_MODE_CHECK_MS) return;
  last = millis();
  uint8_t m;
  imuMode = bnoReadRegs(0x3D, &m, 1) ? (m & 0x0F) : 0xFF;
}
