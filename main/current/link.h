// ============================================================================
//  Связь с пультом: команды через Bluetooth-модуль HC-05 на UART1
// ============================================================================
// Для платы HC-05 — обычный последовательный порт (Serial2, пины 8 и 9). linkPoll() забирает
// принятое и отправляет ответ по кусочкам, только сколько влезает в буфер UART, и сразу выходит:
// на 9600 бод ответ идёт больше 100 мс, и ждать его в loop() нельзя. Поэтому её можно звать
// в начале каждого прохода loop() рядом с остальными задачами.
//
// Команды — текстовые строки, заканчиваются переводом строки:
//   LED ON, LED OFF    — включить и выключить светодиод
//   STATUS             — ничего не менять, только ответить
// Ответ на команду:    OK LED=1 UP=12345 C0=678 C1=910  (UP — мс с запуска, C0 и C1 —
//                      счётчики проходов ядер), на неизвестную команду: ERR

#pragma once
#include <Arduino.h>
#include "config.h"

// Счётчики проходов: core0 считает linkPoll(), core1 увеличивает loop1() (если счётчик не трогать,
// в ответе будет 0). У каждого один писатель, поэтому блокировки не нужны.
volatile uint32_t linkPassesCore0 = 0;
volatile uint32_t linkPassesCore1 = 0;

#if !defined(ARDUINO_MYRP_2350B)

// Другая плата (симулятор): скетч тот же, вызовы ничего не делают
inline void linkBegin() {}
inline void linkPoll() {}

#else

#define LINK_LINE_MAX          32   // самая длинная строка команды, байт
#define LINK_REPLY_MAX         200
#define LINK_BYTES_PER_POLL    64   // за один проход, чтобы поток байт не задержал остальные задачи

bool linkLedOn = false;

void linkSetLed(bool on) {
  linkLedOn = on;
  digitalWrite(LINK_LED_PIN, on ? HIGH : LOW);
}

// Выполняет команду и пишет ответ, завершённый переводом строки
void linkExecute(const char *cmd, char *reply, size_t replySize) {
#ifdef LINK_APP
  bool known = remoteCommand(cmd);   // свои команды скетча, см. main/remote.h
#else
  bool known = false;
#endif
  if (known) {
  } else if (strcmp(cmd, "LED ON") == 0) {
    linkSetLed(true);
  } else if (strcmp(cmd, "LED OFF") == 0) {
    linkSetLed(false);
  } else if (strcmp(cmd, "STATUS") != 0) {
    snprintf(reply, replySize, "ERR\n");
    return;
  }
  int n = snprintf(reply, replySize, "OK LED=%d UP=%lu C0=%lu C1=%lu", linkLedOn ? 1 : 0,
                   (unsigned long)millis(), (unsigned long)linkPassesCore0, (unsigned long)linkPassesCore1);
#ifdef LINK_APP
  remoteStatus(reply + n, replySize - n - 1);
  n = strlen(reply);
#endif
  reply[n] = '\n';
  reply[n + 1] = 0;
}

// Ответ, который ещё не ушёл целиком
char linkTx[LINK_REPLY_MAX];
size_t linkTxLen = 0, linkTxPos = 0;

void linkPollRx() {
  static char line[LINK_LINE_MAX];
  static uint8_t len = 0;
  static bool tooLong = false;

  for (int i = 0; i < LINK_BYTES_PER_POLL; i++) {
    int c = LINK_SERIAL.read();
    if (c < 0) return;

    if (c != '\n' && c != '\r') {
      if (len < sizeof(line) - 1) line[len++] = c;
      else tooLong = true;
      continue;
    }

    if (len > 0 || tooLong) {   // пустые строки пропускаем: \r\n — один перевод строки, а не два
      line[len] = 0;
      char reply[LINK_REPLY_MAX];
      linkExecute(tooLong ? "" : line, reply, sizeof(reply));
      // Прошлый ответ ещё идёт — новый не шлём: команда выполнена, пульт спросит состояние снова
      if (linkTxPos >= linkTxLen) {
        strcpy(linkTx, reply);
        linkTxLen = strlen(reply);
        linkTxPos = 0;
      }
    }
    len = 0;
    tooLong = false;
  }
}

void linkPollTx() {
  while (linkTxPos < linkTxLen && LINK_SERIAL.availableForWrite() > 0) {
    LINK_SERIAL.write((uint8_t)linkTx[linkTxPos++]);
  }
}

// Запускает связь, зовётся один раз из setup()
void linkBegin() {
  pinMode(LINK_LED_PIN, OUTPUT);
  digitalWrite(LINK_LED_PIN, LOW);
  LINK_SERIAL.setTX(LINK_TX_PIN);
  LINK_SERIAL.setRX(LINK_RX_PIN);
  LINK_SERIAL.begin(LINK_BAUD);
}

// Забирает принятые команды и выполняет их, зовётся в каждом проходе loop()
void linkPoll() {
  linkPassesCore0++;
  linkPollRx();
  linkPollTx();
}

#endif  // ARDUINO_MYRP_2350B
