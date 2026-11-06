// ============================================================================
//  Режимы игры
// ============================================================================
// AttackMode() — атака, DefenseMode() — защита; выбор — RUN_MODE в config.h.

#pragma once
#include "state.h"
#include "sensors.h"
#include "hardware.h"
#include "geometry.h"

// ============================================================================
//  Режим атаки
// ============================================================================
// Вход — данные камеры и линии (state.h), выход — omniMoveDirection() и kicker().


int accel_via_distance() {
  return ACCEL_DISTANCE_SPEED_MULTIPLIER * ball_dist;
}

// Скорость объезда мяча: чем дальше мяч по углу от 0 (центра), тем быстрее едем.
int attackSpeed(int dir) {
  int a = constrain(abs(dir), 0, 180);
  return map(a, 0, 180, ATTACK_SPEED_MIN, ATTACK_SPEED_MAX);
}

float rot_speed = 0;
unsigned long tmrDebuf = 0;
void AttackMode() {
  if (out_dist < ball_dist) {omniMoveDirection(0, 0, 0);}
  else {digitalWrite(LED_BUILTIN, 0);}
  // Камера молчит дольше SENSOR_TIMEOUT — её углы устарели (при ошибке она перезапускается ~2 с).
  // С линии съезжаем по датчику линии, иначе плавно останавливаемся и не крутимся.
  if (!camFresh()) {
    if (line_be) omniMoveDirection(1600, line_dir, 0, true);
    else omniMoveDirection(0, 0, 0);
    return;
  }

  rot_speed = gate_dir_2 * 10;
  //rot_speed =  (norm180(gate_dir_1) - norm180(180 + (ball_dir))) * 10;
  if ((cam_line_dir > -1 || line_be)) {
    if (line_be) {
      tmrDebuf = millis();
      omniMoveDirection(1600, line_dir, rot_speed, true);
    } else {
      tmrDebuf = millis();
      omniMoveDirection(1600, VectorSum(cam_line_dir, ball_dir, 0.5, 1), rot_speed, true);
      //VectorSum(cam_line_dir, ball_dir, 2, 1)
    }
    return;
  }
  float k = 1;
  if (millis() - tmrDebuf < 1000) {k = 0.5;}
  else {k = 1;}
  if (!trig) {
    digitalWrite(LED_BUILTIN, 0);
    if (abs(ball_dir) < 10) {
      // Мяч перед роботом и не захвачен - едем прямо на него
      omniMoveDirection(5500*k, ball_dir, rot_speed);
      if (abs(gate_dir_1) < 15) {
        kicker();
      }
    } else {
      // Мяч не перед роботом - объезжаем, скорость растёт с углом
      if (ball_dist <= 20) {
        omniMoveDirection((attackSpeed(ball_dir) + accel_via_distance()) * k, (abs(ball_dir) + 90) * (ball_dir / abs(ball_dir)), rot_speed);
      } else {
        omniMoveDirection(5000*k, ball_dir, rot_speed);
      }
    }
  } else {
    digitalWrite(LED_BUILTIN, 1);
    // Мяч захвачен - целимся в ворота и бьём
    if (abs(gate_dir_1) < 15) {
      kicker();
    }
    omniMoveDirection(5000*k, 0, rot_speed);
  }
}

// ============================================================================
//  Режим защиты
// ============================================================================
// Включается RUN_MODE RUN_DEFENSE в config.h.


int degreesForDef = 0, speedForDef = 0;
void DefenseMode() {
  // ---------- параметры ----------
  const int   DIR_SIGN = 1;                       // -1, если защита уезжает ОТ линии мяч-ворота, а не к ней
  const float SPEED_MAX = 6000;                   // потолок скорости (на колесо это <= 3000 + rot, запас до MAX_MOTOR_SPEED есть)
  const float KS = 750.0f;                        // дальняя зона: скорость = KS*sqrt(ошибка): 10° -> ~2000, 20° -> ~3100, 40° -> ~4550
  const float KL = 300.0f;                        // ближняя зона: скорость = KL*ошибка (до ~6° сверх E_IN), у цели без бесконечного усиления sqrt
  const float KD = 8.0f;                          // ед. скорости на 1°/с изменения ошибки: тормозит заранее, гасит перелёт
  const float E_IN = 3.0f, E_OUT = 5.0f;          // гистерезис: |e| <= E_IN - встали, > E_OUT - поехали
  const float E_TAU = 0.02f, D_TAU = 0.05f;       // фильтры ошибки и её производной, с
  const float ACC = 24000, DECEL = 32000;         // разгон/торможение вектора скорости, ед/с
  const float BASE_TAU = 0.15f;                   // плавная смена курса 80° <-> 135°, с
  const int   BEHIND_IN = 170, BEHIND_OUT = 155;  // мяч строго сзади -> стоим
  const int   BACK_IN = 110, BACK_OUT = 95;       // мяч сзади-сбоку -> едем, но не быстрее BACK_SPEED_MAX
  const float BACK_SPEED_MAX = 1500;              // потолок скорости, пока мяч сзади (раньше до 6000 - назад уезжал слишком быстро)
  const int   SIDE_HOLD = 150;                    // |e| больше: сторону не переключаем
  const float ROT_K = 20, ROT_MAX = 600, ROT_DEAD = 1;   // стабилизация курса
  // рывок с ударом: мяч в "области ворот 1" - в секторе +-DASH_GATE вокруг направления на ворота соперника
  const int   DASH_GATE = 20;                     // полуширина области ворот, град (было 5, и мяч отдельно "спереди" +-15)
  const int   DASH_DIST = 35;                     // ball_dist не больше этого (в атаке "рядом" - 20; 100 - без ограничения)
  const int   KICK_DIST = 12;                     // ball_dist не больше этого (или trig) - мяч у пеналки, бьём
  const float DASH_SPEED = 5500;                  // скорость рывка
  const float DASH_ACC = 40000;                   // разгон на рывке, ед/с (обычный ACC 24000)
  const float DASH_ROT_K = 10, DASH_ROT_MAX = 600;    // на рывке доворачиваем корпус (пеналку) на мяч
  const uint32_t KICK_LATEST_MS = 450;            // мяч так и не дошёл до пеналки - бьём всё равно через столько мс
  const uint32_t DASH_FOLLOW_MS = 100;            // после удара ещё столько мс едем на мяч (дожимаем)
  const uint32_t DASH_COOL_MS = 800;              // пауза между рывками, мс
  const uint32_t DASH_HOLD_MS = BALL_FRONT_HOLD_MS;   // условие рывка должно держаться непрерывно столько мс (500)
  const float RETURN_SPEED = 1500;                // после рывка сдаём назад с такой скоростью, пока камера не увидит линию
  const uint32_t RETURN_MAX_MS = 2000;            // но не дольше этого

  // ---------- состояние между вызовами ----------
  static unsigned long lastUs = 0, lastPrintMs = 0;
  static int   lastSide = 1;
  static bool  moving = false, ballBehind = false, ballBack = false;
  static float eF = 0, prevEF = 0, dEF = 0;       // отфильтрованная ошибка и её производная, °/с
  static float vF = 0, vS = 0;                    // текущий вектор скорости: вперёд (cos) и вбок (sin)
  static float base = 80;                         // текущий "боковой" курс, °
  static uint32_t dashStart = 0, kickAt = 0, dashReadyAt = 0, returnUntil = 0;
  static uint32_t dashCondSince = 0;              // с какого момента условие рывка держится непрерывно; 0 - не держится
  static bool  dash = false, kicked = false;

  // dt по micros(): цикл бывает короче 1 мс, по millis() dt скакал бы 0/1/2
  unsigned long nowUs = micros();
  float dt = (nowUs - lastUs) * 1e-6f;
  lastUs = nowUs;
  bool fresh = dt > 0.1f;                         // давно не вызывали: начинаем заново
  if (dt > 0.05f)   dt = 0.05f;
  if (dt < 0.0001f) dt = 0.0001f;
  uint32_t nowMs = millis();

  // ---------- входные величины ----------
  float imU        = imuData.euler[0];
  if (imU > 180) imU -= 360;
  int e            = norm180(ball_dir - 180 - gate_dir_2);   // со знаком, с переходом через ±180
  bool farFromLine = (cam_line_dir < 0) && abs(gate_dir_2) > 135;

  if (fresh) {
    eF = e; prevEF = e; dEF = 0; vF = vS = 0; base = farFromLine ? 135 : 80;
    returnUntil = nowMs; dash = false; dashCondSince = 0;
  }

  // фильтр ошибки по времени, а не по числу вызовов (по кратчайшей дуге около ±180)
  float d = e - eF;
  if (d > 180) d -= 360; else if (d < -180) d += 360;
  eF += d * dt / (E_TAU + dt);
  if (eF > 180) eF -= 360; else if (eF < -180) eF += 360;

  float dd = eF - prevEF;
  if (dd > 180) dd -= 360; else if (dd < -180) dd += 360;
  prevEF = eF;
  dEF += (dd - dEF * dt) / (D_TAU + dt);          // сглаженная dE/dt

  // ---------- гистерезисы ----------
  if (fabsf(eF) > E_OUT) moving = true; else if (fabsf(eF) <= E_IN) moving = false;
  if (abs(ball_dir) > BEHIND_IN) ballBehind = true; else if (abs(ball_dir) < BEHIND_OUT) ballBehind = false;
  if (abs(ball_dir) > BACK_IN) ballBack = true; else if (abs(ball_dir) < BACK_OUT) ballBack = false;
  if (fabsf(eF) <= SIDE_HOLD && eF != 0) lastSide = (eF > 0) ? 1 : -1;

  // ---------- 1. желаемая боковая скорость ----------
  // Профиль: далеко sqrt (как торможение с постоянным замедлением - едем быстро),
  // у цели линейный: у sqrt около нуля усиление бесконечное, и с задержкой камеры
  // робот бы перелетал. KD сбрасывает скорость заранее,
  // пока робот ещё подъезжает к линии мяч-ворота (камера запаздывает).
  float vLat = 0;                                 // со знаком: > 0 - в сторону +base
  float mag  = fabsf(eF) - E_IN;
  if (moving && !ballBehind && mag > 0 && camFresh()) {   // камера молчит - стоим, а не едем по старым данным
    float p = fminf(KL * mag, KS * sqrtf(mag)) * lastSide;
    vLat = p + KD * dEF;
    if (vLat * p < 0) vLat = 0;                   // D только тормозит, назад не разворачивает
    // мяч сзади: ошибка огромная, без потолка робот на полной скорости уезжал назад/вбок
    float vMax = ballBack ? BACK_SPEED_MAX : SPEED_MAX;
    vLat = DIR_SIGN * constrain(vLat, -vMax, vMax);
  }

  // плавно меняем "боковой" курс 80° <-> 135°
  base += ((farFromLine ? 135 : 80) - base) * dt / (BASE_TAU + dt);
  float b  = base * DEG_TO_RAD;
  float tF = fabsf(vLat) * cosf(b);               // вперёд/назад
  float tS = vLat * sinf(b);                      // вбок

  // ---------- 1б. рывок с ударом ----------
  // Мяч в области ворот 1 (не дальше DASH_GATE от направления на них; ворота - в передней полусфере) -
  // тупо едем на мяч и бьём. Начатый рывок выполняется гарантированно: не прерывается, пока не будет удара,
  // а удар - как только мяч у пеналки, но не позже KICK_LATEST_MS. Стартуем, только когда пеналка заряжена
  // (kickFlag), иначе удар мог бы не случиться. Съезд с линии по датчику ниже всё равно главнее (безопасность).
  // Таймер перед рывком: мяч должен пробыть в области ворот (и рядом) непрерывно DASH_HOLD_MS - случайный
  // проскок мяча через область или дребезг камеры рывка не вызывают.
  if (!dash) {
    bool cond = camFresh() && !line_be && abs(gate_dir_1) < 90 &&
                abs(norm180(ball_dir - gate_dir_1)) <= DASH_GATE && ball_dist <= DASH_DIST;
    if (!cond)              dashCondSince = 0;
    else if (!dashCondSince) dashCondSince = nowMs ? nowMs : 1;
    if (cond && nowMs - dashCondSince >= DASH_HOLD_MS && kickFlag && (int32_t)(nowMs - dashReadyAt) >= 0) {
      dash = true; kicked = false; dashStart = nowMs;
      dashCondSince = 0;                          // следующий рывок - снова только после DASH_HOLD_MS
    }
  }
  if (dash) {
    if (!kicked && (trig || ball_dist <= KICK_DIST || nowMs - dashStart >= KICK_LATEST_MS)) {
      kicker();                                   // пеналка заряжена (проверили на старте) - удар будет
      kicked = true; kickAt = millis();           // millis(), а не nowMs: kicker() держит импульс KICK_PULSE_MS
    }
    if (kicked && millis() - kickAt >= DASH_FOLLOW_MS) {
      dash = false;                               // рывок закончен - возвращаемся назад к линии
      dashReadyAt = millis() + DASH_COOL_MS;
      returnUntil = millis() + RETURN_MAX_MS;
    }
  }
  if (dash) {
    float a = ball_dir * DEG_TO_RAD;              // тупо на мяч
    tF = DASH_SPEED * cosf(a);
    tS = DASH_SPEED * sinf(a);
  }
  if (cam_line_dir > -1 || line_be || ballBack) returnUntil = nowMs;   // линия видна / мяч сзади - возврат не нужен
  if (!dash && (int32_t)(returnUntil - nowMs) > 0 && tF > -RETURN_SPEED) tF = -RETURN_SPEED;

  // ---------- 2. реакция на линию ----------
  // Линия не заменяет боковое движение, а только срезает его составляющую "в линию":
  // робот скользит вдоль линии с полной боковой скоростью, а не скачет вперёд-назад.
  // line_dir и cam_line_dir - направление ОТ линии.
  bool onLine = line_be;
  if (onLine || cam_line_dir > -1) {
    float a    = (onLine ? line_dir : cam_line_dir) * DEG_TO_RAD;
    float nF   = cosf(a), nS = sinf(a);
    float away = tF * nF + tS * nS;               // < 0 - едем в линию
    if (away < 0) { tF -= away * nF; tS -= away * nS; away = 0; }
    if (onLine && away < DEF_LINE_ESCAPE_SPEED) { // на линии ещё и отталкиваемся, даже если стоим
      tF += (DEF_LINE_ESCAPE_SPEED - away) * nF;
      tS += (DEF_LINE_ESCAPE_SPEED - away) * nS;
    }
  }

  // ---------- 3. сглаживание ВЕКТОРА скорости ----------
  // Сглаживаем (vF, vS), а не угол: смена стороны - это плавный проход vS через 0,
  // а не разворот вектора через "вперёд" с выездом из ворот и срезом скорости.
  if (onLine) {
    vF = tF; vS = tS;                             // у линии рывок без ограничения разгона
  } else {
    float dF = tF - vF, dS = tS - vS;
    float dist = hypotf(dF, dS);
    float step = (dash ? DASH_ACC : (hypotf(tF, tS) > hypotf(vF, vS) ? ACC : DECEL)) * dt;
    if (dist > step) { vF += dF * step / dist; vS += dS * step / dist; }
    else             { vF = tF; vS = tS; }
  }
  float dir = atan2f(vS, vF) * RAD_TO_DEG;
  speedForDef = (int)lroundf(hypotf(vF, vS));
  if (speedForDef > 0) degreesForDef = (int)lroundf(dir);

  // ---------- курс ----------
  // нет свежего курса - не крутимся; мёртвая зона плавная: rot растёт от 0, а не прыгает 0 -> 60
  int rot = 0;
  float h = fabsf(imU) - ROT_DEAD;
  if (imuFresh() && h > 0) rot = (int)constrain(-ROT_K * h * (imU > 0 ? 1 : -1), -ROT_MAX, ROT_MAX);
  // на рывке разворачиваем пеналку на мяч (мяч в области ворот - значит, заодно и на ворота);
  // после рывка удержание курса по BNO само вернёт робота лицом вперёд
  if (dash) rot = (int)constrain(ball_dir * DASH_ROT_K, -DASH_ROT_MAX, DASH_ROT_MAX);

  // ---------- отладка и движение ----------
  // печать не чаще SERIAL_INTERVAL: Serial на каждом вызове тормозит цикл управления
  if (millis() - lastPrintMs >= SERIAL_INTERVAL) {
    lastPrintMs = millis();
    char buffer[80];
    // bk - мяч сзади (скорость ограничена), dash - идёт рывок, bg - угол мяча от ворот 1 (рывок при |bg| <= DASH_GATE)
    snprintf(buffer, sizeof(buffer), "def = %d, e = %d, spd = %d, bk = %d, dash = %d, bg = %d",
             degreesForDef, (int)eF, speedForDef, (int)ballBack, (int)dash, norm180(ball_dir - gate_dir_1));
    Serial.println(buffer);
  }
  // instant = true: разгон уже ограничен выше, рампа моторов добавила бы вторую задержку
  omniMoveDirection(speedForDef, dir, rot, true);
}
