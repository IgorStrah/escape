/*
 * Т-ОБРАЗНАЯ ЛЕНТА — хрономаг, устройство «Огонь чернокнижника»
 * ESP32-C3 SuperMini / PlatformIO (framework = arduino)
 *
 * Одна физическая лента WS2812B на одном пине (GPIO3). Горизонталь и
 * вертикаль — два диапазона индексов внутри неё (H_START/H_COUNT,
 * V_START/V_COUNT), а не два отдельных пина/объекта. Итоговое число
 * диодов в каждом куске пока не известно — значения ниже placeholder,
 * поправить по факту монтажа. GPIO4 этим освобождён.
 *
 * + 2 кнопки (GPIO5 левая, GPIO6 правая).
 *
 * Игрок жмёт кнопку — от края летит луч (переливается, свой цвет пока
 * нет). Два луча встретились — зоны засветки (голова ±TOLERANCE)
 * пересеклись — середина приводится к ближайшей из 5 точек HIT_POINTS[]
 * (9/18/28/38/47 — см. ниже, почему не круглые 10/20/30/40/50).
 * Попал в допуск — цвет центральной точки — особый случай, белый и сразу
 * вверх). Не попал — промах: на easy лучи просто гаснут, на hard едут
 * наверх как обычно, но цвет "мусорный" (мигалка).
 *
 * Возвращающийся к центру луч, пересечённый любым летящим — "портится"
 * (SPOILED), едет и поднимается как обычно, но тоже кладёт мусор —
 * это отдельная механика, срабатывает на любой сложности.
 *
 * Накопитель — FIFO на 8 слотов по 5 диодов. Приход нового блока: старые
 * мгновенно сдвигаются на 5 диодов вверх, новый луч едет только в
 * освободившийся нижний слот (V0..V4) — короткий, видимый подъём.
 * Переполнение — верхний слот просто теряется.
 *
 * ЗАГЛУШКИ / ЧТО ПРОВЕРИТЬ НА МЕСТЕ:
 *   - WIFI_SSID/PASS — сейчас стенд (weasgley), для реальной сети Cisco
 *     вписать актуальные перед боевой заливкой (см. п.7 ТЗ).
 *   - H_START/H_COUNT, V_START/V_COUNT — реальные границы кусков на
 *     общей ленте, пока placeholder (60+40, встык).
 *   - VERTICAL_REVERSED — направление вертикального куска, см. ниже.
 *   - HIT_COLORS, GLOW_DIM_PCT, BRIGHTNESS — подобрать по месту (п.10 ТЗ).
 *   - BEAM_SPEED_MS=100 — если ±200мс слишком жёстко игрокам, поднимать.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <Adafruit_NeoPixel.h>
#include <PubSubClient.h>
#include <esp_system.h>

// ============================================================
// КОНФИГУРАЦИЯ — СЕТЬ
// ============================================================
#define WIFI_SSID     "weasgley"
#define WIFI_PASS     "weasgley123"

#define OTA_HOSTNAME  "chrono-strip"
#define OTA_PASSWORD  "chrono"

// 1 = статический IP (боевая сеть, п.7 ТЗ), 0 = DHCP (стенд weasgley).
#define USE_STATIC_IP  0

IPAddress local_IP(192, 168, 8, 187);
IPAddress gateway (192, 168, 8, 1);
IPAddress subnet  (255, 255, 255, 0);
IPAddress dns1    (192, 168, 8, 1);

#define WIFI_RETRY_MS   5000

#define MQTT_HOST         "quest-hub.local"
#define MQTT_PORT         1883
#define MQTT_CLIENT_ID    "chrono-strip"
#define MQTT_RECONNECT_MS 4000
#define MQTT_PUBLISH_MS   5000
#define MQTT_BUTTONS_REPUBLISH_MS (3UL * 60UL * 1000UL)

#define MQTT_TOPIC        "quest/chronomage/Strip"
#define MQTT_TOPIC_INFO      MQTT_TOPIC "/info"
#define MQTT_TOPIC_BUTTONS   MQTT_TOPIC "/buttons"
#define MQTT_TOPIC_STATE     MQTT_TOPIC "/state"
#define MQTT_TOPIC_RSSI      MQTT_TOPIC "/rssi"
#define MQTT_TOPIC_HEARTBEAT MQTT_TOPIC "/heartbeat"
#define MQTT_TOPIC_CMD       MQTT_TOPIC "/cmd"
#define MQTT_TOPIC_DONE      MQTT_TOPIC "/done"

// Кнопки хаба: метка:топик:payload, через "|". Парсер боевого хаба режет
// строго на 3 части по ":", поэтому payload здесь без двоеточий внутри
// (easy/hard, не difficulty:easy) — а в самом /cmd ниже принимаем ОБА
// варианта, вдруг реальный контроллер игры шлёт полный "difficulty:easy".
#define MQTT_BUTTONS_PAYLOAD \
  "reset:" MQTT_TOPIC_CMD ":reset|" \
  "easy:"  MQTT_TOPIC_CMD ":easy|" \
  "hard:"  MQTT_TOPIC_CMD ":hard|" \
  "test:"  MQTT_TOPIC_CMD ":test"

// ============================================================
// КОНФИГУРАЦИЯ — ЖЕЛЕЗО
// ============================================================
#define LED_PIN       3   // одна лента, оба куска на этом пине
#define BUTTON_L_PIN  6   // физически распаяно наоборот — пины поменяны местами
#define BUTTON_R_PIN  5

// Горизонталь и вертикаль — диапазоны внутри ОДНОЙ ленты. Числа ниже —
// placeholder (60+40, встык): сколько реально будет диодов в каждом
// куске пока не известно, поправить тут по факту монтажа.
#define H_START   0
#define H_COUNT  56
#define V_START  (H_START + H_COUNT)
#define V_COUNT  34

#define LED_TOTAL_COUNT (((V_START + V_COUNT) > (H_START + H_COUNT)) ? (V_START + V_COUNT) : (H_START + H_COUNT))

#define SEGMENT_SIZE   5
// Лишние диоды вертикали (не делятся нацело на SEGMENT_SIZE) остаются
// пустыми у самого низа, а не размазаны по слотам — верх накопителя
// всегда доходит до физического конца ленты.
#define V_GAP          (V_COUNT % SEGMENT_SIZE)   // 4
#define STORAGE_SLOTS  ((V_COUNT - V_GAP) / SEGMENT_SIZE)   // 6
#define TOLERANCE          1   // допуск для обычных точек, ±диодов
#define CENTER_TOLERANCE   3   // допуск для центра — шире, т.к. горизонталь
                                 // размечена не равными промежутками и точно
                                 // в середину лучи попадают не всегда
#define CENTER_INDEX   28   // см. HIT_POINTS ниже — 6 равных участков, остаток в центр

// Если данные на вертикальном куске заходят сверху — поставить 1.
#define VERTICAL_REVERSED 0

#define BRIGHTNESS   140

// ---- Тайминги ----
#define BEAM_SPEED_MS    100   // летящая точка по горизонтали
#define MERGED_SPEED_MS   40   // возврат к центру и подъём по вертикали

// ---- Кнопки ----
#define DEBOUNCE_COUNT     3
#define DEBOUNCE_POLL_MS   5

// ---- Отрисовка луча ----
#define GLOW_DIM_PCT   35   // яркость пред-/пост-свечения относительно головы, %
#define HINT_DIM_PCT   22   // яркость подсказки точек встречи, пока летит хоть один луч, %
#define SHIMMER_MS     30   // как часто сдвигать переливание в полёте

// ---- Тест-режим (команда "test") ----
#define TEST_CHASE_STEP_MS  15UL     // скорость пробега по всей длине (фаза 1)
#define TEST_MARK_HOLD_MS 8000UL     // держим старт/финиш/точки встречи (фаза 2) — подольше, чтобы успеть сверить с покраской на стене
#define TEST_FILL_HOLD_MS 3000UL     // держим 8 цветных блоков на вертикали (фаза 3)

// ---- Прочее ----
#define STATS_PRINT_MS 10000
#define MAX_BEAMS      8

// ============================================================
// ТОЧКИ СТОЛКНОВЕНИЯ И ЦВЕТА
// ============================================================
// 56 диодов / 6 равных участков = 9 с остатком 2 — оба лишних диода
// отданы двум центральным участкам (9,9,10,10,9,9 = 56), поэтому точки
// не на круглых 10/20/30/40/50, а здесь. H_COUNT/V_COUNT зафиксированы,
// пересчитывать не нужно.
const uint8_t HIT_POINTS[5] = {9, 18, 28, 38, 47};
#define CENTER_HIT_INDEX 2   // индекс центральной точки (28) внутри HIT_POINTS

struct ColorRGB { uint8_t r, g, b; };

// Бирюзовый — намеренно (0,255,180), не голубой: голубой на WS2812B
// на высокой яркости почти слипается с белым (см. п.10 ТЗ).
// Подобраны так, чтобы максимально отличаться друг от друга на WS2812
// (синий и бирюзовый, жёлтый и зелёный на дешёвых лентах часто сливаются
// из-за реального спектра диодов) — у каждого цвета доминирует свой
// канал почти без примеси соседних.
const ColorRGB HIT_COLORS[5] = {
  {255, 110,   0},   // точка 1 (9)  — жёлтый/янтарный (не зеленит)
  {  0,  10, 255},   // точка 2 (18) — синий, почти без зелёного
  {255, 255, 255},   // точка 3 (28) — белый (центр)
  {255,   0,   0},   // точка 4 (38) — красный
  {  0, 255,  40},   // точка 5 (47) — бирюзовый/зелёный, почти без синего
};

// 8 чисто визуально разных цветов для фазы 3 теста (залить вертикаль
// целиком 8 блоками) — не связано с игровыми HIT_COLORS, просто чтобы
// монтажник видел все 8 блоков по отдельности.
const ColorRGB TEST_PALETTE[8] = {
  {255,   0,   0}, {255, 120,   0}, {255, 255,   0}, {  0, 255,   0},
  {  0, 255, 255}, {  0,  80, 255}, {160,   0, 255}, {255, 255, 255},
};

const char *colorName(int8_t idx) {
  switch (idx) {
    case 0: return "yellow";
    case 1: return "blue";
    case 2: return "white";
    case 3: return "red";
    case 4: return "turquoise";
    default: return "trash";
  }
}

// ============================================================
// ВЫИГРЫШНАЯ КОМБИНАЦИЯ
// Цифры 1..5 = точки столкновения по порядку HIT_POINTS/HIT_COLORS:
// 1=точка1 жёлтый, 2=точка2 синий, 3=центр белый, 4=точка4 красный,
// 5=точка5 бирюзовый. Читается от низа накопителя (слот 0, V0) вверх.
// Чтобы поменять комбинацию — просто переписать цифры здесь.
const uint8_t WIN_COMBINATION[] = {1, 2, 3, 4, 5, 1};
#define WIN_COMBINATION_LEN (sizeof(WIN_COMBINATION) / sizeof(WIN_COMBINATION[0]))

// ============================================================
// ОБЪЕКТЫ
// ============================================================
Adafruit_NeoPixel strip(LED_TOTAL_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

WiFiClient   mqttNet;
PubSubClient mqttClient(mqttNet);

// ============================================================
// СОСТОЯНИЕ — ЛУЧИ
// ============================================================
enum BeamState : uint8_t { BM_FLYING, BM_RETURNING, BM_RISING };

struct Beam {
  bool used;
  BeamState state;
  int8_t  dir;          // +1 / -1 — текущее направление движения
  int16_t position;     // голова: H-индекс (FLYING/RETURNING) или V-индекс 0..4 (RISING), локальные, без смещения куска
  unsigned long lastStep;
  uint8_t huePhase;      // свой сдвиг переливания — лучи не выглядят клонами
  bool    spoiled;       // "испорчен" пересечением на возврате
  int8_t  colorIndex;    // 0..4 = HIT_COLORS, -1 = промах/мусор
};

Beam beams[MAX_BEAMS];

// ============================================================
// СОСТОЯНИЕ — НАКОПИТЕЛЬ
// ============================================================
struct StorageSlot { bool used; bool trash; int8_t colorIndex; };
StorageSlot storage[STORAGE_SLOTS];   // [0] = низ (V0-V4) ... [STORAGE_SLOTS-1] = верх

// ============================================================
// СОСТОЯНИЕ — ОБЩЕЕ
// ============================================================
enum Difficulty { DIFF_EASY, DIFF_HARD };
Difficulty difficulty = DIFF_EASY;

bool testModeActive = false;
unsigned long testModeStart = 0;

bool needRedraw = true;
bool winActive  = false;   // уже объявляли победу для текущего содержимого накопителя

unsigned long lastWifiTry        = 0;
unsigned long lastStatsTime      = 0;
unsigned long loopCount          = 0;

bool otaInProgress    = false;
bool wifiWasConnected = false;
bool otaStarted        = false;

unsigned long lastMqttReconnect   = 0;
unsigned long lastMqttPublish     = 0;
unsigned long lastHeartbeat       = 0;
unsigned long lastButtonsPublish  = 0;
bool mqttNeedInitialPublish = false;

char infoMsg[96] = "Т-лента: старт";

// ---- Кнопки (дебаунс) ----
bool pendingL = false, pendingR = false;
bool confirmedL = false, confirmedR = false;
uint8_t debounceCountL = 0, debounceCountR = 0;
unsigned long lastButtonPoll = 0;

// ============================================================
// ПРОТОТИПЫ
// ============================================================
void wifiConnect();
void wifiCheck();
void otaSetup();
void mqttCheck();
void mqttPublishPeriodic();
void mqttCallback(char *topic, uint8_t *payload, unsigned int length);
void publishInfo(const char *text);
void publishState();
void pushStorage(int8_t colorIdx, bool trash);
void resetAll();

// ============================================================
// МЕЛКИЕ ПОМОЩНИКИ
// ============================================================
uint8_t vIndex(uint8_t logical) {
  return VERTICAL_REVERSED ? (V_COUNT - 1 - logical) : logical;
}

// Классическое "колесо" радуги (как в примерах Adafruit_NeoPixel) — для
// переливания летящего луча, у которого своего цвета пока нет.
void wheel(uint8_t pos, uint8_t &r, uint8_t &g, uint8_t &b) {
  pos = 255 - pos;
  if (pos < 85) {
    r = 255 - pos * 3; g = 0; b = pos * 3;
  } else if (pos < 170) {
    pos -= 85;
    r = 0; g = pos * 3; b = 255 - pos * 3;
  } else {
    pos -= 170;
    r = pos * 3; g = 255 - pos * 3; b = 0;
  }
}

void randomFlickerColor(uint8_t &r, uint8_t &g, uint8_t &b) {
  r = random(60, 256);
  g = random(60, 256);
  b = random(60, 256);
}

// Цвет "головы" луча прямо сейчас — переливание в полёте, мусорная
// мигалка при промахе/порче, чистый цвет точки во всех остальных случаях.
void colorForBeam(const Beam &b, uint8_t &r, uint8_t &g, uint8_t &b2) {
  if (b.state == BM_FLYING) {
    uint8_t pos = (uint8_t)((millis() / SHIMMER_MS) + b.huePhase);
    wheel(pos, r, g, b2);
    return;
  }

  bool trash = b.spoiled || (b.colorIndex < 0);
  if (trash) {
    randomFlickerColor(r, g, b2);
    return;
  }

  r = HIT_COLORS[b.colorIndex].r;
  g = HIT_COLORS[b.colorIndex].g;
  b2 = HIT_COLORS[b.colorIndex].b;
}

// ============================================================
// ЛУЧИ — СПАВН / ШАГ / СТОЛКНОВЕНИЯ
// ============================================================
void spawnBeam(int8_t dir) {
  for (uint8_t i = 0; i < MAX_BEAMS; i++) {
    if (beams[i].used) continue;
    beams[i].used       = true;
    beams[i].state       = BM_FLYING;
    beams[i].dir         = dir;
    beams[i].position    = (dir > 0) ? 0 : (H_COUNT - 1);
    beams[i].lastStep    = millis();
    beams[i].huePhase    = (uint8_t)random(0, 256);
    beams[i].spoiled     = false;
    beams[i].colorIndex  = -1;
    needRedraw = true;
    return;
  }
  // Пул лучей занят до предела — новое нажатие просто игнорируется.
}

void pollButtons() {
  if (millis() - lastButtonPoll < DEBOUNCE_POLL_MS) return;
  lastButtonPoll = millis();

  bool rawL = (digitalRead(BUTTON_L_PIN) == HIGH);   // сенсорная кнопка — касание = HIGH
  if (rawL == pendingL) {
    if (debounceCountL < DEBOUNCE_COUNT) debounceCountL++;
  } else {
    pendingL = rawL;
    debounceCountL = 1;
  }
  if (debounceCountL >= DEBOUNCE_COUNT && confirmedL != pendingL) {
    confirmedL = pendingL;
    if (confirmedL) spawnBeam(+1);   // левая кнопка — луч летит H0 -> H(end)
  }

  bool rawR = (digitalRead(BUTTON_R_PIN) == HIGH);   // сенсорная кнопка — касание = HIGH
  if (rawR == pendingR) {
    if (debounceCountR < DEBOUNCE_COUNT) debounceCountR++;
  } else {
    pendingR = rawR;
    debounceCountR = 1;
  }
  if (debounceCountR >= DEBOUNCE_COUNT && confirmedR != pendingR) {
    confirmedR = pendingR;
    if (confirmedR) spawnBeam(-1);   // правая кнопка — луч летит H(end) -> H0
  }
}

// Переводит луч i из RETURNING/прямого центра в RISING: мгновенно двигает
// стопку накопителя на 1 слот вверх, новый луч едет только в
// освободившийся нижний слот (V0..V4).
void enterRising(uint8_t i, int8_t colorIdx, bool trash) {
  pushStorage(colorIdx, trash);
  beams[i].state    = BM_RISING;
  beams[i].position = 0;
  beams[i].lastStep = millis();
}

void handleCollision(uint8_t i, uint8_t j) {
  int sum = beams[i].position + beams[j].position;   // удвоенная середина

  int bestDiff = 9999, bestIdx = -1;
  for (uint8_t k = 0; k < 5; k++) {
    int diff = abs(sum - 2 * (int)HIT_POINTS[k]);
    if (diff < bestDiff) { bestDiff = diff; bestIdx = (int8_t)k; }
  }
  int tol = (bestIdx == CENTER_HIT_INDEX) ? CENTER_TOLERANCE : TOLERANCE;
  bool hit = (bestDiff <= 2 * tol);

  beams[j].used = false;   // второй луч больше не нужен — остаётся i

  if (!hit && difficulty == DIFF_EASY) {
    // Промах на easy — лучи просто гаснут, в накопитель ничего не идёт.
    beams[i].used = false;
    needRedraw = true;
    return;
  }

  int mergedPos  = sum / 2;
  int8_t colorIdx = hit ? bestIdx : -1;   // -1 = промах на hard -> мусор

  beams[i].spoiled    = false;
  beams[i].colorIndex = colorIdx;
  beams[i].position   = mergedPos;
  beams[i].lastStep    = millis();

  if (hit && bestIdx == CENTER_HIT_INDEX) {
    // Встреча в центре — белеет и сразу уходит вверх, без возврата.
    enterRising(i, colorIdx, false);
  } else {
    beams[i].state = BM_RETURNING;
  }
  needRedraw = true;
}

void checkCollisions() {
  for (uint8_t i = 0; i < MAX_BEAMS; i++) {
    if (!beams[i].used || beams[i].state != BM_FLYING || beams[i].dir <= 0) continue;
    for (uint8_t j = 0; j < MAX_BEAMS; j++) {
      if (!beams[j].used || beams[j].state != BM_FLYING || beams[j].dir >= 0) continue;
      if (abs((int)beams[i].position - (int)beams[j].position) <= 2 * TOLERANCE) {
        handleCollision(i, j);
        break;   // луч i уже обработан (или погашен) — к следующему i
      }
    }
  }

  // Порча возвращающегося луча любым летящим — отдельная проверка,
  // срабатывает при любой сложности.
  for (uint8_t i = 0; i < MAX_BEAMS; i++) {
    if (!beams[i].used || beams[i].state != BM_RETURNING || beams[i].spoiled) continue;
    for (uint8_t j = 0; j < MAX_BEAMS; j++) {
      if (!beams[j].used || beams[j].state != BM_FLYING) continue;
      if (abs((int)beams[i].position - (int)beams[j].position) <= 2 * TOLERANCE) {
        beams[i].spoiled = true;
        needRedraw = true;
        break;
      }
    }
  }
}

void updateBeams() {
  unsigned long now = millis();

  for (uint8_t i = 0; i < MAX_BEAMS; i++) {
    if (!beams[i].used) continue;
    Beam &b = beams[i];

    if (b.state == BM_FLYING) {
      if (now - b.lastStep < BEAM_SPEED_MS) continue;
      b.lastStep = now;
      b.position += b.dir;
      needRedraw = true;
      if (b.position < 0 || b.position >= H_COUNT) {
        // Прошёл насквозь, никого не встретив — гаснет.
        b.used = false;
      }
      continue;
    }

    if (b.state == BM_RETURNING) {
      if (now - b.lastStep < MERGED_SPEED_MS) continue;
      b.lastStep = now;
      if (b.position < CENTER_INDEX)      b.position++;
      else if (b.position > CENTER_INDEX) b.position--;
      needRedraw = true;
      if (b.position == CENTER_INDEX) {
        enterRising(i, b.colorIndex, b.spoiled || b.colorIndex < 0);
      }
      continue;
    }

    if (b.state == BM_RISING) {
      if (now - b.lastStep < MERGED_SPEED_MS) continue;
      b.lastStep = now;
      b.position++;
      needRedraw = true;
      if (b.position >= SEGMENT_SIZE - 1) {
        b.used = false;   // доехал на своё место, слот в storage уже выставлен
      }
      continue;
    }
  }

  checkCollisions();
}

// ============================================================
// НАКОПИТЕЛЬ
// ============================================================
// Сверяет текущий накопитель (снизу вверх) с WIN_COMBINATION. Если
// комбинация длиннее, чем реально есть слотов в накопителе — проверка
// невозможна физически, тихо пропускаем (см. STORAGE_SLOTS от V_COUNT).
bool checkWinCombo() {
  if (WIN_COMBINATION_LEN > STORAGE_SLOTS) return false;
  for (uint8_t s = 0; s < WIN_COMBINATION_LEN; s++) {
    if (!storage[s].used || storage[s].trash) return false;
    if (storage[s].colorIndex != (int8_t)(WIN_COMBINATION[s] - 1)) return false;
  }
  return true;
}

void pushStorage(int8_t colorIdx, bool trash) {
  for (int8_t s = STORAGE_SLOTS - 1; s > 0; s--) storage[s] = storage[s - 1];
  storage[0].used       = true;
  storage[0].trash      = trash;
  storage[0].colorIndex = trash ? -1 : colorIdx;
  publishState();

  bool win = checkWinCombo();
  if (win && !winActive) {
    winActive = true;
    publishInfo("выигрышная комбинация собрана!");
    if (mqttClient.connected()) mqttClient.publish(MQTT_TOPIC_DONE, "OK");
  } else if (!win) {
    winActive = false;
  }
}

void resetAll() {
  for (uint8_t i = 0; i < MAX_BEAMS; i++) beams[i].used = false;
  for (uint8_t s = 0; s < STORAGE_SLOTS; s++) storage[s].used = false;
  winActive  = false;
  needRedraw = true;
  publishState();
}

// ============================================================
// ОТРИСОВКА
// ============================================================
// Голова + пред-/пост-свечение, всё в ЛОКАЛЬНЫХ координатах куска
// (segStart/segCount) — глоу не вылезает за границу своего куска, даже
// если H и V на общей ленте стоят впритык друг к другу.
void drawGlow(int segStart, int segCount, int localPos, uint8_t r, uint8_t g, uint8_t b) {
  if (localPos >= 0 && localPos < segCount) {
    strip.setPixelColor(segStart + localPos, strip.Color(r, g, b));
  }

  uint8_t dr = (uint8_t)((uint16_t)r * GLOW_DIM_PCT / 100);
  uint8_t dg = (uint8_t)((uint16_t)g * GLOW_DIM_PCT / 100);
  uint8_t db = (uint8_t)((uint16_t)b * GLOW_DIM_PCT / 100);

  if (localPos - 1 >= 0 && localPos - 1 < segCount) {
    strip.setPixelColor(segStart + localPos - 1, strip.Color(dr, dg, db));
  }
  if (localPos + 1 >= 0 && localPos + 1 < segCount) {
    strip.setPixelColor(segStart + localPos + 1, strip.Color(dr, dg, db));
  }
}

// Ниже renderH()/renderV() только РИСУЮТ в общий буфер — clear()/show()
// делает renderLoop() один раз на весь кадр (общая лента на одном пине).
bool anyFlyingBeam() {
  for (uint8_t i = 0; i < MAX_BEAMS; i++) {
    if (beams[i].used && beams[i].state == BM_FLYING) return true;
  }
  return false;
}

void renderH() {
  // Пока летит хоть один луч — тускло подсвечиваем все 5 точек встречи
  // их будущим цветом (подсказка игрокам). Сам луч рисуется поверх, если
  // окажется ровно на точке — перекроет подсказку своей полной яркостью.
  if (anyFlyingBeam()) {
    for (uint8_t k = 0; k < 5; k++) {
      uint8_t r = (uint8_t)((uint16_t)HIT_COLORS[k].r * HINT_DIM_PCT / 100);
      uint8_t g = (uint8_t)((uint16_t)HIT_COLORS[k].g * HINT_DIM_PCT / 100);
      uint8_t b = (uint8_t)((uint16_t)HIT_COLORS[k].b * HINT_DIM_PCT / 100);
      strip.setPixelColor(H_START + HIT_POINTS[k], strip.Color(r, g, b));
    }
  }

  for (uint8_t i = 0; i < MAX_BEAMS; i++) {
    if (!beams[i].used) continue;
    if (beams[i].state == BM_RISING) continue;   // уже на вертикали
    uint8_t r, g, b;
    colorForBeam(beams[i], r, g, b);
    drawGlow(H_START, H_COUNT, beams[i].position, r, g, b);
  }
}

// Слоты все одного размера (SEGMENT_SIZE), но сдвинуты на V_GAP — лишние
// диоды остаются пустыми строго у низа (V0..V_GAP-1), верх накопителя
// (последний слот) доходит ровно до физического конца ленты.
void renderV() {
  for (uint8_t s = 0; s < STORAGE_SLOTS; s++) {
    if (!storage[s].used) continue;
    for (uint8_t p = 0; p < SEGMENT_SIZE; p++) {
      uint8_t logical = V_GAP + s * SEGMENT_SIZE + p;
      uint8_t r, g, b;
      if (storage[s].trash) randomFlickerColor(r, g, b);
      else {
        r = HIT_COLORS[storage[s].colorIndex].r;
        g = HIT_COLORS[storage[s].colorIndex].g;
        b = HIT_COLORS[storage[s].colorIndex].b;
      }
      strip.setPixelColor(V_START + vIndex(logical), strip.Color(r, g, b));
    }
  }

  for (uint8_t i = 0; i < MAX_BEAMS; i++) {
    if (!beams[i].used || beams[i].state != BM_RISING) continue;
    uint8_t r, g, b;
    colorForBeam(beams[i], r, g, b);
    drawGlow(V_START, V_COUNT, vIndex((uint8_t)(V_GAP + beams[i].position)), r, g, b);
  }
}

void renderLoop() {
  // Рисуем из актуального состояния каждый цикл, без условного пропуска —
  // так накопитель не может "потухнуть" в простое ни при каких условиях.
  strip.clear();
  renderH();
  renderV();
  strip.show();
  needRedraw = false;
}

// ============================================================
// ТЕСТ-РЕЖИМ (команда "test") — прогон ленты для проверки монтажа
// ============================================================
// Фаза 1 — один бегущий диод по всей длине общей ленты (проверка
// порядка/направления подключения).
void testPhase1(unsigned long elapsed) {
  uint16_t step = (uint16_t)(elapsed / TEST_CHASE_STEP_MS);
  strip.clear();
  if (step < LED_TOTAL_COUNT) strip.setPixelColor(step, strip.Color(0, 150, 255));
  strip.show();
}

// Фаза 2 — статично загораются начало/конец горизонтали и все 5 точек
// встречи (своими игровыми цветами) — проверка совпадения с покраской
// на стене.
void testPhase2() {
  strip.clear();
  strip.setPixelColor(H_START, strip.Color(255, 0, 255));
  strip.setPixelColor(H_START + H_COUNT - 1, strip.Color(255, 0, 255));
  for (uint8_t k = 0; k < 5; k++) {
    strip.setPixelColor(H_START + HIT_POINTS[k],
                         strip.Color(HIT_COLORS[k].r, HIT_COLORS[k].g, HIT_COLORS[k].b));
  }
  strip.show();
}

// Фаза 3 — вертикаль целиком залита 8 разными блоками (проверка
// направления/равномерности вертикального куска).
void testPhase3() {
  strip.clear();
  for (uint8_t i = 0; i < 8; i++) {
    uint16_t segStart = (uint16_t)((uint32_t)i * V_COUNT / 8);
    uint16_t segEnd    = (uint16_t)((uint32_t)(i + 1) * V_COUNT / 8);
    for (uint16_t p = segStart; p < segEnd; p++) {
      strip.setPixelColor(V_START + vIndex((uint8_t)p),
                           strip.Color(TEST_PALETTE[i].r, TEST_PALETTE[i].g, TEST_PALETTE[i].b));
    }
  }
  strip.show();
}

void runTestMode() {
  unsigned long elapsed    = millis() - testModeStart;
  unsigned long phase1Dur  = (unsigned long)LED_TOTAL_COUNT * TEST_CHASE_STEP_MS;
  unsigned long phase2End  = phase1Dur + TEST_MARK_HOLD_MS;
  unsigned long phase3End  = phase2End + TEST_FILL_HOLD_MS;

  if (elapsed < phase1Dur) {
    testPhase1(elapsed);
  } else if (elapsed < phase2End) {
    testPhase2();
  } else if (elapsed < phase3End) {
    testPhase3();
  } else {
    testModeActive = false;
    strip.clear();
    strip.show();
    publishInfo("тест завершён");
  }
}

// ============================================================
// WIFI
// ============================================================
void wifiConnect() {
  WiFi.mode(WIFI_STA);

  // Регулятор C3 SuperMini не тянет полную мощность радио — строго по ТЗ.
  WiFi.setTxPower(WIFI_POWER_11dBm);
  WiFi.setSleep(false);

#if USE_STATIC_IP
  if (!WiFi.config(local_IP, gateway, subnet, dns1)) {
    Serial.println("[WIFI] не удалось применить статический IP");
  }
#else
  Serial.println("[WIFI] режим DHCP (стенд)");
#endif

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("[WIFI] подключение к ");
  Serial.println(WIFI_SSID);
}

void wifiCheck() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiWasConnected) {
      wifiWasConnected = true;
      Serial.print("[WIFI] подключено, IP ");
      Serial.print(WiFi.localIP());
      Serial.print("  RSSI ");
      Serial.print(WiFi.RSSI());
      Serial.println(" dBm");

      if (!otaStarted) {
        otaSetup();
        otaStarted = true;
      }
    }
    return;
  }

  if (wifiWasConnected) {
    wifiWasConnected = false;
    Serial.println("[WIFI] соединение потеряно");
  }

  if (millis() - lastWifiTry > WIFI_RETRY_MS) {
    lastWifiTry = millis();
    Serial.println("[WIFI] переподключение...");
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }
}

// ============================================================
// OTA
// ============================================================
void otaSetup() {
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);

  ArduinoOTA.onStart([]() {
    otaInProgress = true;
    Serial.println("[OTA] начало заливки");
    strip.clear();
    strip.show();
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("[OTA] завершено, перезагрузка");
    for (uint16_t i = 0; i < LED_TOTAL_COUNT; i++) strip.setPixelColor(i, strip.Color(0, 255, 0));
    strip.show();
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    uint16_t lit = (uint32_t)progress * LED_TOTAL_COUNT / total;
    strip.clear();
    for (uint16_t i = 0; i < lit && i < LED_TOTAL_COUNT; i++) strip.setPixelColor(i, strip.Color(0, 0, 255));
    strip.show();
  });

  ArduinoOTA.onError([](ota_error_t error) {
    otaInProgress = false;
    Serial.print("[OTA] ОШИБКА ");
    Serial.println(error);
    for (uint16_t i = 0; i < LED_TOTAL_COUNT; i++) strip.setPixelColor(i, strip.Color(255, 0, 0));
    strip.show();
  });

  ArduinoOTA.begin();
  Serial.print("[OTA] готов, имя ");
  Serial.println(OTA_HOSTNAME);
}

// ============================================================
// MQTT
// ============================================================
void publishInfo(const char *text) {
  strncpy(infoMsg, text, sizeof(infoMsg) - 1);
  infoMsg[sizeof(infoMsg) - 1] = '\0';
  Serial.println(infoMsg);
  if (mqttClient.connected()) mqttClient.publish(MQTT_TOPIC_INFO, infoMsg, true);
}

void publishState() {
  if (!mqttClient.connected()) return;
  char buf[160];
  size_t pos = 0;
  for (uint8_t s = 0; s < STORAGE_SLOTS && pos < sizeof(buf) - 12; s++) {
    const char *name = storage[s].used ? (storage[s].trash ? "trash" : colorName(storage[s].colorIndex)) : "-";
    size_t n = snprintf(buf + pos, sizeof(buf) - pos, "%s%s", s == 0 ? "" : ",", name);
    pos += n;
  }
  buf[pos] = '\0';
  mqttClient.publish(MQTT_TOPIC_STATE, buf, true);
}

void mqttCallback(char *topic, uint8_t *payload, unsigned int length) {
  char buf[48] = {0};
  unsigned int n = length < sizeof(buf) - 1 ? length : sizeof(buf) - 1;
  memcpy(buf, payload, n);

  Serial.print("[MQTT cmd] ");
  Serial.println(buf);

  if (strcmp(buf, "reset") == 0) {
    resetAll();
    publishInfo("сброшено");
    return;
  }

  if (strcmp(buf, "easy") == 0 || strcmp(buf, "difficulty:easy") == 0) {
    difficulty = DIFF_EASY;
    publishInfo("сложность: easy");
    return;
  }
  if (strcmp(buf, "hard") == 0 || strcmp(buf, "difficulty:hard") == 0) {
    difficulty = DIFF_HARD;
    publishInfo("сложность: hard");
    return;
  }

  if (strcmp(buf, "test") == 0) {
    testModeActive = true;
    testModeStart  = millis();
    publishInfo("тест запущен");
    return;
  }

  if (strncmp(buf, "fill:", 5) == 0) {
    const char *name = buf + 5;
    int8_t idx = -1;
    if      (strcmp(name, "yellow")    == 0) idx = 0;
    else if (strcmp(name, "blue")      == 0) idx = 1;
    else if (strcmp(name, "white")     == 0) idx = 2;
    else if (strcmp(name, "red")       == 0) idx = 3;
    else if (strcmp(name, "turquoise") == 0) idx = 4;

    pushStorage(idx, idx < 0);   // неизвестное имя/"trash" -> мусорный блок
    needRedraw = true;

    char msg[64];
    snprintf(msg, sizeof(msg), "вручную положено: %s", idx < 0 ? "trash" : name);
    publishInfo(msg);
    return;
  }
}

void mqttCheck() {
  if (WiFi.status() != WL_CONNECTED) return;

  if (mqttClient.connected()) {
    mqttClient.loop();
    return;
  }

  mqttNeedInitialPublish = false;

  if (millis() - lastMqttReconnect < MQTT_RECONNECT_MS) return;
  lastMqttReconnect = millis();

  Serial.print("[MQTT] подключение к ");
  Serial.print(MQTT_HOST);
  Serial.print(" ... ");

  if (mqttClient.connect(MQTT_CLIENT_ID)) {
    Serial.println("ок");
    mqttClient.subscribe(MQTT_TOPIC_CMD);
    mqttNeedInitialPublish = true;
  } else {
    Serial.print("не вышло, state=");
    Serial.println(mqttClient.state());
  }
}

void mqttPublishPeriodic() {
  if (!mqttClient.connected()) return;

  if (mqttNeedInitialPublish) {
    mqttClient.publish(MQTT_TOPIC_INFO, infoMsg, true);
    mqttClient.publish(MQTT_TOPIC_BUTTONS, MQTT_BUTTONS_PAYLOAD, true);
    publishState();
    lastButtonsPublish = millis();
    mqttNeedInitialPublish = false;
  }

  if (millis() - lastMqttPublish >= MQTT_PUBLISH_MS) {
    lastMqttPublish = millis();
    mqttClient.publish(MQTT_TOPIC_RSSI, String(WiFi.RSSI()).c_str());
  }

  if (millis() - lastHeartbeat >= MQTT_PUBLISH_MS) {
    lastHeartbeat = millis();
    mqttClient.publish(MQTT_TOPIC_HEARTBEAT, "OK");
  }

  if (millis() - lastButtonsPublish >= MQTT_BUTTONS_REPUBLISH_MS) {
    lastButtonsPublish = millis();
    mqttClient.publish(MQTT_TOPIC_BUTTONS, MQTT_BUTTONS_PAYLOAD, true);
  }
}

// ============================================================
// SETUP / LOOP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(2000);

  randomSeed(esp_random());

  pinMode(BUTTON_L_PIN, INPUT);   // сенсорная кнопка сама держит уровень, подтяжка не нужна
  pinMode(BUTTON_R_PIN, INPUT);

  strip.begin();
  strip.setBrightness(BRIGHTNESS);
  strip.clear();
  strip.show();

  mqttNet.setTimeout(150);   // короче, чтобы недоступный сервер не мешал ArduinoOTA.handle() — OTA должна работать без сервера, достаточно общей сети
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(384);

  Serial.println();
  Serial.println("========================================");
  Serial.println("  Т-ОБРАЗНАЯ ЛЕНТА, старт");
  Serial.println("========================================");
  Serial.print("LED_PIN="); Serial.print(LED_PIN);
  Serial.print(" всего диодов="); Serial.println(LED_TOTAL_COUNT);
  Serial.print("H: start="); Serial.print(H_START); Serial.print(" count="); Serial.println(H_COUNT);
  Serial.print("V: start="); Serial.print(V_START); Serial.print(" count="); Serial.println(V_COUNT);

  wifiConnect();

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
    delay(30);
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiWasConnected = true;
    Serial.print("[WIFI] подключено, IP ");
    Serial.print(WiFi.localIP());
    Serial.print("  RSSI ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
    otaSetup();
    otaStarted = true;
  } else {
    Serial.println("[WIFI] не подключилось, работаем локально, реконнект в фоне");
  }

  Serial.println("[СТАРТ] готово, опрос запущен");
  Serial.println();
}

void loop() {
  ArduinoOTA.handle();
  if (otaInProgress) return;

  if (testModeActive) {
    runTestMode();
  } else {
    pollButtons();
    updateBeams();
    renderLoop();
  }

  loopCount++;

  wifiCheck();
  mqttCheck();
  mqttPublishPeriodic();

  if (millis() - lastStatsTime >= STATS_PRINT_MS) {
    lastStatsTime = millis();
    Serial.print("[СТАТ] оборотов: ");
    Serial.print(loopCount);
    Serial.print("  RSSI ");
    Serial.print(WiFi.RSSI());
    Serial.print(" dBm  difficulty=");
    Serial.println(difficulty == DIFF_EASY ? "easy" : "hard");
  }
}
