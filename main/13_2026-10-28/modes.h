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
//  Движение в игре: у линии медленнее
// ============================================================================
// Режимы ездят через drive(): это omniMoveDirection(), но если камера видит линию (ближняя
// cam_line_dir или дальняя cam_line_dir_2), а робот едет к ней — в конусе ±LINE_SLOW_CONE°
// от направления на линию, — скорость LINE_SLOW_PERCENT %. От линии и вдоль неё — без замедления.
// driveTo() и испытания на поле едут через omniMoveVector() и не замедляются.


// Едет ли робот курсом dir к линии, которую видит камера
bool towardLine(float dir) {
  if (!camFresh()) return false;
  const int16_t away[2] = { cam_line_dir, cam_line_dir_2 };   // направления ОТ линии, -1 — нет
  for (int16_t a : away) {
    if (a >= 0 && abs(norm180((int)lroundf(dir) - (a + 180))) <= LINE_SLOW_CONE) return true;
  }
  return false;
}

void drive(int32_t speed, float dir, int32_t w, bool instant = false) {
  if (speed > 0 && towardLine(dir)) speed = speed * LINE_SLOW_PERCENT / 100;
  omniMoveDirection(speed, dir, w, instant);
}

// ============================================================================
//  Режим атаки
// ============================================================================
// Вход — данные камеры и линии (state.h), выход — drive() и kicker().


int accel_via_distance() {
  return ACCEL_DISTANCE_SPEED_MULTIPLIER * ball_dist;
}

// Скорость объезда мяча: чем дальше мяч по углу от 0 (центра), тем быстрее едем.
int attackSpeed(int dir) {
  int a = constrain(abs(dir), 0, 180);
  return map(a, 0, 180, ATTACK_SPEED_MIN, ATTACK_SPEED_MAX);
}

// "Застряли": мяч почти по центру (|ball_dir| < 15), но дистанция до него
// не уменьшается дольше STALL_TIMEOUT_MS - форсируем скорость, чтобы не
// зависнуть в дребезге на месте.
int16_t stall_best_dist = 32767;
uint32_t stall_since = 0;

int stallBoost() {
  if (ball_dist < stall_best_dist - STALL_PROGRESS_DELTA) {
    stall_best_dist = ball_dist;
    stall_since = millis();
    return 0;
  }
  if (millis() - stall_since > STALL_TIMEOUT_MS) return STALL_BOOST_SPEED;
  return 0;
}

void resetStall() {
  stall_best_dist = 32767;
  stall_since = millis();
}

float rot_speed = 0;
unsigned long tmrDebuf = 0;
bool APPTrig = 0;
int v = 0, d = 0, w = 0;
void AttackMode() {
  v = 0;
  d = 0;
  w = 0;
  float w_k = 10.0;
  static bool follow_ball = false;
  static uint32_t push_until = 0;   // до какого момента отталкиваемся
  static int push_dir = 0;          // запомненное направление от линии

  int line = cam_line_dir % 180;

  // ===== 1. Линия — наивысший приоритет =====
  if (line_be) {
    follow_ball = false;            // выходим из режима
    push_dir = line_dir;            // запоминаем, куда уезжать
    push_until = millis() + 250;    // подобрать время под робота
  }

  bool pushing = millis() < push_until;
  bool aL = 0;
  if (pushing) {
    v = 1600;                       // уезжаем быстро
    d = push_dir;
  }
  // ===== 2. Обычный режим =====
  else if (!follow_ball) {
    if ((APPTrig || ball_dist < 8) && (abs(ball_dir) < 8)) {
      d = 0;
    } else if (cam_line_dir > -1 || line_be) {
      if (line_be){
        d = line_be;
      } else {
        d = cam_line_dir;
      }
      aL = 1;
    } else if (ball_dist < 20) {
      int side = (ball_dir / max(abs(ball_dir), 1));
      d = (abs(ball_dir) + 75) * side;
    } else {
      d = ball_dir;
    }


    if(aL){
      v = 1600;
    }else{
      if ((abs(ball_dir) < 8)) {
        resetStall();
        v = 5000;
      } else if (!line_be &&  (abs(ball_dir) < 8 && (-15 <= gate_dir_1 && gate_dir_1 <= 15))){
          v = accel_via_distance() + stallBoost();;
      } if (ball_dist < 20) {
        resetStall();
        v = attackSpeed(ball_dir) + accel_via_distance();
      } else {
        resetStall();
        v = 5000;
      }
    }
  }
  // ===== 3. follow_ball =====
  else {
    if(!line_be){
    v = max(accel_via_distance() + stallBoost(), 500);
    
    int side = (ball_dir / max(abs(ball_dir), 1));
    d = (abs(ball_dir)) * side;
    }
    else {
      d = line_dir;
    }
  }

  // ===== Мяч в лунке (триггер) — бьём в любой ветке; пеналка сама ждёт перезарядки =====
  if (APPTrig || ball_dist < 4) {
    kicker();
  }

  // ===== Переключение режима (не во время отталкивания) =====
  if (!follow_ball) {
    if (abs(ball_dir) <= 110 && line >= 20 && line <= 150 && ball_dist < 20)
      follow_ball = true;
  } else {
    if (abs(ball_dir) > 120 || line < 10 || line > 160 || ball_dist > 21)
      follow_ball = false;
  }

  // ===== w =====
  if (follow_ball) {
    w = ball_dir * w_k;
    if(abs(w) < 55){
      w = 0;
    }
  } else {
    w = gate_dir_1 * w_k;
  }
  w = constrain(w, -1000, 1000);
  drive(v, d, w);
  Serial.println(follow_ball);
}


// ============================================================================
//  Режим защиты
// ============================================================================
// Включается RUN_MODE RUN_DEFENSE в config.h.


int degreesForDef = 0, speedForDef = 0;
void DefenseMode() {
  // ---------- параметры ----------
  const float SPEED_MAX = 6000;                   // потолок скорости (на колесо это <= 3000 + rot, запас до MAX_MOTOR_SPEED есть)
  const float KS = 750.0f;                        // дальняя зона: скорость = KS*sqrt(ошибка): 10° -> ~2000, 20° -> ~3100, 40° -> ~4550
  const float KL = 300.0f;                        // ближняя зона: скорость = KL*ошибка (до ~6° сверх E_IN), у цели без бесконечного усиления sqrt
  const float KD = 8.0f;                          // ед. скорости на 1°/с изменения ошибки: тормозит заранее, гасит перелёт
  const float E_IN = 3.0f, E_OUT = 5.0f;          // гистерезис: |e| <= E_IN - встали, > E_OUT - поехали
  const float E_TAU = 0.02f, D_TAU = 0.05f;       // фильтры ошибки и её производной, с
  const float ACC = 24000, DECEL = 32000;         // разгон/торможение вектора скорости, ед/с
  const int   BEHIND_IN = 170, BEHIND_OUT = 155;  // мяч строго сзади -> стоим
  const int   BACK_IN = 110, BACK_OUT = 95;       // мяч сзади-сбоку -> едем, но не быстрее BACK_SPEED_MAX
  const float BACK_SPEED_MAX = 1500;              // потолок скорости, пока мяч сзади (раньше до 6000 - назад уезжал слишком быстро)
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
  static bool  moving = false, ballBehind = false, ballBack = false;
  static float eF = 0, prevEF = 0, dEF = 0;       // отфильтрованная ошибка и её производная, °/с
  static float vF = 0, vS = 0;                    // текущий вектор скорости: вперёд (cos) и вбок (sin)
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
  // углы в осях поля (0 - к воротам соперника): корпус может быть повёрнут к мячу (DEF_FACE_BALL),
  // а «мяч сзади» и «ворота сзади» - про поле, а не про корпус
  int yaw          = imuFresh() ? (int)lroundf(imU) : 0;
  int ballF        = norm180(ball_dir + yaw);
  int goalF        = norm180(gate_dir_2 + yaw);
  bool farFromLine = (cam_line_dir < 0) && abs(goalF) > 135;

  if (fresh) {
    eF = e; prevEF = e; dEF = 0; vF = vS = 0;
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
  if (abs(ballF) > BEHIND_IN) ballBehind = true; else if (abs(ballF) < BEHIND_OUT) ballBehind = false;
  if (abs(ballF) > BACK_IN) ballBack = true; else if (abs(ballF) < BACK_OUT) ballBack = false;

  // ---------- 1. куда и как быстро ехать ----------
  // Цель - встать на прямую мяч-ворота, между ними: единичные векторы на мяч (b) и на свои
  // ворота (g) противоположны, b + g = 0. Сам b + g - биссектриса угла мяч-робот-ворота, она
  // смотрит на отрезок мяч-ворота: по ней и едем. Линии штрафной не видно (уехали вперёд) -
  // добавляем g: заодно возвращаемся к воротам.
  // Скорость - по ошибке e: далеко sqrt (как торможение с постоянным замедлением - едем быстро),
  // у цели линейно: у sqrt около нуля усиление бесконечное, и с задержкой камеры робот бы
  // перелетал. KD сбрасывает скорость заранее, пока робот ещё подъезжает к прямой мяч-ворота.
  Vec2 toBall = Vec2::polar(ball_dir), toGoal = Vec2::polar(gate_dir_2);
  Vec2 aim = toBall + toGoal;                     // к отрезку мяч-ворота; 0 - уже на нём
  if (farFromLine) aim = aim.normalized() + toGoal;
  float speed = 0;
  float mag   = fabsf(eF) - E_IN;
  if (moving && !ballBehind && mag > 0 && camFresh()) {   // камера молчит - стоим, а не едем по старым данным
    speed = fminf(KL * mag, KS * sqrtf(mag)) + KD * dEF * (eF > 0 ? 1 : -1);   // D только тормозит
    // мяч сзади: ошибка огромная, без потолка робот на полной скорости уезжал назад/вбок
    speed = constrain(speed, 0.0f, ballBack ? BACK_SPEED_MAX : SPEED_MAX);
  }
  Vec2 aimV = aim.withLength(speed);
  float tF = aimV.x;                              // вперёд/назад
  float tS = aimV.y;                              // вбок

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
  // DEF_FACE_BALL: пеналкой на мяч (вперёд робот ездит и тормозит лучше, чем вбок), но не дальше
  // DEF_FACE_MAX от курса на ворота соперника; иначе или камера молчит - курс 0.
  // Нет свежего курса - не крутимся; мёртвая зона плавная: rot растёт от 0, а не прыгает 0 -> 60
  int want = (DEF_FACE_BALL && camFresh()) ? constrain(ballF, -DEF_FACE_MAX, DEF_FACE_MAX) : 0;
  int herr = norm180(want - (int)lroundf(imU));
  int rot = 0;
  float h = abs(herr) - ROT_DEAD;
  if (imuFresh() && h > 0) rot = (int)constrain(ROT_K * h * (herr > 0 ? 1 : -1), -ROT_MAX, ROT_MAX);
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
  drive(speedForDef, dir, rot, true);
}
