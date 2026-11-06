// Мелодии на пьезопищалке для 2-го ядра (setup1 / loop1). Из sketch_sep24a.
// Звук: аппаратный ШИМ RP2040 напрямую через pico-sdk. Ноты переключает аппаратный таймер (alarm),
// созданный на ядре 1 (setup1), поэтому прерывание не мешает циклу моторов на ядре 0.
// В паузах ШИМ полностью выключается, пин переводится в обычный GPIO с постоянным уровнем - никакого фонового писка.
//
// Громкость = скважность. Максимум у пьезо при 50%: амплитуда основной гармоники ~ sin(pi * duty),
// отсюда duty = asin(VOLUME) / pi.
//
// Вызовы: buzzerBegin() - в setup1(), buzzerLoop() - в loop1() (переключение мелодии по Serial: 1 или 2).

#pragma once
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "pico/time.h"

#define BUZZER_ENABLED 1     // 0 - пищалка полностью выключена

#define BZ_REST  0
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_D5  587
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_D6  1175

namespace buzz {

// Выбор мелодии: 1 - Jingle Bells, 2 - Megalovania (Sans, Undertale)
int song = 2;

const uint PIN = 12;

// Громкость от 0.0 до 1.0 (1.0 = максимум пищалки). 0.75 = на 25% тише.
const float VOLUME = 0.75;

// true, если пищалка звучит при 0 на пине (подключена к 3.3V, а не к GND).
const bool ACTIVE_LOW = true;

// ---------- Jingle Bells ----------
const int Q  = 300;           // четверть, мс
const int H  = Q * 2;
const int W  = Q * 4;
const int E  = Q / 2;
const int DQ = Q + E;

const int jingle[][2] = {
  {NOTE_E5, Q}, {NOTE_E5, Q}, {NOTE_E5, H},
  {NOTE_E5, Q}, {NOTE_E5, Q}, {NOTE_E5, H},
  {NOTE_E5, Q}, {NOTE_G5, Q}, {NOTE_C5, DQ}, {NOTE_D5, E},
  {NOTE_E5, W},
  {NOTE_F5, Q}, {NOTE_F5, Q}, {NOTE_F5, DQ}, {NOTE_F5, E},
  {NOTE_F5, Q}, {NOTE_E5, Q}, {NOTE_E5, Q}, {NOTE_E5, E}, {NOTE_E5, E},
  {NOTE_E5, Q}, {NOTE_D5, Q}, {NOTE_D5, Q}, {NOTE_E5, Q},
  {NOTE_D5, H}, {NOTE_G5, H},

  {NOTE_E5, Q}, {NOTE_E5, Q}, {NOTE_E5, H},
  {NOTE_E5, Q}, {NOTE_E5, Q}, {NOTE_E5, H},
  {NOTE_E5, Q}, {NOTE_G5, Q}, {NOTE_C5, DQ}, {NOTE_D5, E},
  {NOTE_E5, W},
  {NOTE_F5, Q}, {NOTE_F5, Q}, {NOTE_F5, DQ}, {NOTE_F5, E},
  {NOTE_F5, Q}, {NOTE_E5, Q}, {NOTE_E5, Q}, {NOTE_E5, E}, {NOTE_E5, E},
  {NOTE_G5, Q}, {NOTE_G5, Q}, {NOTE_F5, Q}, {NOTE_D5, Q},
  {NOTE_C5, W},
};

// ---------- Megalovania ----------
const int S  = 125;           // шестнадцатая, мс (~120 BPM)
const int S2 = S * 2;         // восьмая
const int S3 = S * 3;         // восьмая с точкой

// Каждый такт: X X D6 A5 . G#5 G5 F5 D5 F5 G5, меняется только первая нота
#define MEGA_BAR(n) \
  {n, S}, {n, S}, {NOTE_D6, S2}, {NOTE_A5, S3}, {NOTE_GS5, S2}, \
  {NOTE_G5, S2}, {NOTE_F5, S2}, {NOTE_D5, S}, {NOTE_F5, S}, {NOTE_G5, S}

const int megalovania[][2] = {
  MEGA_BAR(NOTE_D5),
  MEGA_BAR(NOTE_C5),
  MEGA_BAR(NOTE_B4),
  MEGA_BAR(NOTE_AS4),
  MEGA_BAR(NOTE_D5),
  MEGA_BAR(NOTE_C5),
  MEGA_BAR(NOTE_B4),
  MEGA_BAR(NOTE_AS4),
};

// ---------- Звук на аппаратном ШИМ ----------

uint pwmSlice, pwmChan;
float dutyFrac;

void toneOn(uint freq) {
  uint32_t sys = clock_get_hz(clk_sys);
  // делитель подбираем так, чтобы wrap влез в 16 бит
  float div = (float)sys / (freq * 65536.0f);
  if (div < 1.0f) div = 1.0f;
  uint32_t wrap = (uint32_t)(sys / (div * freq)) - 1;
  uint32_t level = (uint32_t)((wrap + 1) * dutyFrac);

  pwm_set_enabled(pwmSlice, false);
  pwm_set_clkdiv(pwmSlice, div);
  pwm_set_wrap(pwmSlice, wrap);
  pwm_set_chan_level(pwmSlice, pwmChan, level);
  pwm_set_counter(pwmSlice, 0);
  gpio_set_function(PIN, GPIO_FUNC_PWM);
  pwm_set_enabled(pwmSlice, true);
}

void toneOff() {
  pwm_set_enabled(pwmSlice, false);
  gpio_set_function(PIN, GPIO_FUNC_SIO);   // обычный пин, без ШИМ
  gpio_put(PIN, ACTIVE_LOW ? 1 : 0);       // уровень тишины
}

// ---------- Плеер на таймере ----------

const int (*volatile curNotes)[2] = nullptr;
volatile int curCount = 0;
volatile int curIdx = 0;
volatile bool inGap = false;
alarm_pool_t *pool = nullptr;                 // пул таймера на ядре 1
alarm_id_t alarmId = 0;
bool playing = false;                         // мелодия сейчас играет (включается по кнопке, см. buzzerEnable)

// Вызывается аппаратным таймером. Возвращает задержку до следующего вызова в мкс.
int64_t playerTick(alarm_id_t, void *) {
  if (inGap) {                         // закончилась пауза между нотами
    inGap = false;
    curIdx++;
  } else if (curIdx < curCount) {      // закончилась нота -> короткая пауза
    toneOff();
    inGap = true;
    return (int64_t)curNotes[curIdx][1] * 100;       // 10% длительности
  }

  if (curIdx >= curCount) {            // мелодия кончилась -> 1.5 с тишины и заново
    toneOff();
    inGap = true;
    curIdx = -1;                       // после паузы станет 0
    return 1500000;
  }

  int freq = curNotes[curIdx][0];
  if (freq == BZ_REST) toneOff();
  else toneOn(freq);
  return (int64_t)curNotes[curIdx][1] * 900;         // 90% длительности
}

void startSong(int n) {
  if (alarmId > 0) alarm_pool_cancel_alarm(pool, alarmId);
  toneOff();
  if (n == 1) {
    curNotes = jingle;
    curCount = sizeof(jingle) / sizeof(jingle[0]);
  } else {
    curNotes = megalovania;
    curCount = sizeof(megalovania) / sizeof(megalovania[0]);
  }
  curIdx = -1;                         // стартуем "из паузы", tick сразу возьмёт ноту 0
  inGap = true;
  alarmId = alarm_pool_add_alarm_in_ms(pool, 10, playerTick, nullptr, true);
}

}  // namespace buzz

// Вызывать в setup1(): пул таймера создаётся на ядре 1, значит и прерывания нот идут на ядре 1.
void buzzerBegin() {
#if BUZZER_ENABLED
  gpio_init(buzz::PIN);
  gpio_set_dir(buzz::PIN, GPIO_OUT);
  buzz::pwmSlice = pwm_gpio_to_slice_num(buzz::PIN);
  buzz::pwmChan  = pwm_gpio_to_channel(buzz::PIN);
  // для ACTIVE_LOW инвертируем выход канала аппаратно
  pwm_set_output_polarity(buzz::pwmSlice,
                          buzz::pwmChan == PWM_CHAN_A && buzz::ACTIVE_LOW,
                          buzz::pwmChan == PWM_CHAN_B && buzz::ACTIVE_LOW);
  buzz::dutyFrac = asinf(buzz::VOLUME) / PI;
  buzz::toneOff();

  buzz::pool = alarm_pool_create_with_unused_hardware_alarm(4);
  if (!buzz::pool) buzz::pool = alarm_pool_get_default();   // свободного таймера нет - запасной вариант (ядро 0)
  buzz::startSong(buzz::song);                               // при включении играет, пока робот не запущен кнопкой
  buzz::playing = true;
#endif
}

// Включить / выключить мелодию (вызывать из loop1: по кнопке играет, пока робот стоит, и молчит, пока едет).
void buzzerEnable(bool on) {
#if BUZZER_ENABLED
  if (on == buzz::playing) return;
  buzz::playing = on;
  if (on) {
    buzz::startSong(buzz::song);
  } else {
    if (buzz::alarmId > 0) alarm_pool_cancel_alarm(buzz::pool, buzz::alarmId);
    buzz::alarmId = 0;
    buzz::toneOff();
  }
#endif
}

// Вызывать в loop1(): переключение мелодии по Serial (отправь 1 или 2 в мониторе порта).
void buzzerLoop() {
#if BUZZER_ENABLED
  while (Serial.available()) {
    char c = Serial.read();
    if ((c == '1' || c == '2') && c - '0' != buzz::song) {
      buzz::song = c - '0';
      if (buzz::playing) buzz::startSong(buzz::song);
    }
  }
#endif
}
