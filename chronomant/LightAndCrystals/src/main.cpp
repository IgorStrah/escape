/*
 * СВЕТ КОМНАТ + КРИСТАЛЛЫ — хрономаг, устройство «Огонь чернокнижника»
 * ESP32-C3 SuperMini  /  PlatformIO (framework = arduino)
 *
 * Два логических устройства на одной плате (как маски+свечи в CandleAndMask):
 *   quest/chronomage/roomlight/...  — свет комнат 1 и 2, 40 диодов на одном
 *                                     пине, поделены на зоны (сейчас 2, потом
 *                                     может стать 5-6 — см. roomZones[])
 *   quest/chronomage/crystals/...   — 11 диодов-подсказок на другом пине,
 *                                     показывают, сколько кристаллов сейчас
 *                                     нужно положить на весы (Scales)
 *
 * Пока нет связи с хабом (или хаб ещё не прислал ни одной реальной команды
 * для конкретного устройства) — на его ленте крутится демо-режим: каждый
 * диод независимо переливается синий -> фиолетовый -> белый -> обратно,
 * с собственным фазовым сдвигом.
 *
 * СВЕТ — 3 режима (кнопки на хабе room_start/room_stop/room_horror):
 *   ON     — зоны горят ровно, без эффектов затемнения
 *   OFF    — тёмная лента
 *   HORROR — как ON, но периодически одна случайная зона плавно гаснет
 *            в ноль, держится тёмной пару секунд и плавно возвращается
 *
 * КРИСТАЛЛЫ — синхронизированы с общим таймером цикла хаба
 * (quest/chronomage/cycle/start), либо запускаются вручную кнопкой
 * crystals_start (для теста, без ожидания реального цикла). На каждый
 * новый раунд — случайное число кристаллов 1..11 и случайные позиции на
 * 11 диодах (при малых числах — не рядом друг с другом). Горят розовым,
 * яркость дышит 20..220. crystals/state — голое число горящих кристаллов.
 *
 * ЗАГЛУШКИ, ТРЕБУЮЩИЕ УТОЧНЕНИЯ ПОСЛЕ МОНТАЖА:
 *   - LED_ROOM_PIN / LED_CRYSTAL_PIN — GPIO ещё не разведены на плате,
 *     выбраны как безопасные (не strapping GPIO8/9, не USB GPIO18/19,
 *     не "антенные" GPIO20/21). Перепроверить при монтаже.
 *   - roomZones[] — границы зон 10+30 предварительные, расширение до
 *     5-6 зон позже — просто добавить строки в таблицу.
 *   - Назначение устройства на вкладку "Chronomant" на хабе — делается
 *     руками через веб-Settings хаба, прошивка на это не влияет.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <Adafruit_NeoPixel.h>
#include <PubSubClient.h>
#include <esp_system.h>
#include <math.h>
#include <string.h>

// ============================================================
// КОНФИГУРАЦИЯ
// ============================================================

// ---- Сеть ----
#define WIFI_SSID     "weasgley"
#define WIFI_PASS     "weasgley123"

#define OTA_HOSTNAME  "chrono-lights"
#define OTA_PASSWORD  "chrono"

// 1 = статический IP (продакшн-сеть Quest Hub), 0 = DHCP (стенд).
#define USE_STATIC_IP  0

// Продакшн-адреса (сеть Quest Hub 192.168.8.x) — применяются при USE_STATIC_IP=1.
// .181 занят масками — проверить перед боевой заливкой, что .182 свободен.
IPAddress local_IP(192, 168, 8, 182);
IPAddress gateway (192, 168, 8, 1);
IPAddress subnet  (255, 255, 255, 0);
IPAddress dns1    (192, 168, 8, 1);

#define WIFI_RETRY_MS   5000

// ---- MQTT ----
#define MQTT_HOST         "quest-hub.local"
#define MQTT_PORT         1883
#define MQTT_CLIENT_ID    "chrono-lights"
#define MQTT_RECONNECT_MS 4000
#define MQTT_PUBLISH_MS   4000
#define MQTT_BUTTONS_REPUBLISH_MS (3UL * 60UL * 1000UL)

// Общий таймер квеста — хаб публикует сюда старт нового цикла (см. TestHub).
#define MQTT_TOPIC_CYCLE_START "quest/chronomage/cycle/start"

// -- Устройство 1: свет комнат --
#define MQTT_ROOM_TOPIC        "quest/chronomage/roomlight"
#define MQTT_TOPIC_ROOM_INFO   MQTT_ROOM_TOPIC "/info"
#define MQTT_TOPIC_ROOM_STATE  MQTT_ROOM_TOPIC "/state"
#define MQTT_TOPIC_ROOM_CMD    MQTT_ROOM_TOPIC "/cmd"
#define MQTT_TOPIC_ROOM_RSSI   MQTT_ROOM_TOPIC "/rssi"
#define MQTT_TOPIC_ROOM_BUTTONS MQTT_ROOM_TOPIC "/buttons"

// -- Устройство 2: кристаллы (живёт на той же плате, отдельное имя для хаба) --
#define MQTT_CRYSTALS_TOPIC       "quest/chronomage/crystals"
#define MQTT_TOPIC_CRYSTALS_INFO  MQTT_CRYSTALS_TOPIC "/info"
#define MQTT_TOPIC_CRYSTALS_STATE MQTT_CRYSTALS_TOPIC "/state"
#define MQTT_TOPIC_CRYSTALS_CMD   MQTT_CRYSTALS_TOPIC "/cmd"
#define MQTT_TOPIC_CRYSTALS_RSSI  MQTT_CRYSTALS_TOPIC "/rssi"   // только чтобы хаб не считал устройство оффлайн
#define MQTT_TOPIC_CRYSTALS_BUTTONS MQTT_CRYSTALS_TOPIC "/buttons"

// Кнопки хаба: метка:топик:payload, через "|". Payload без ":" внутри —
// парсер боевого хаба режет строго на 3 части. Каждое устройство публикует
// СВОИ кнопки в свой topic/buttons — иначе хаб вешает все кнопки на то
// устройство, с топика которого они пришли.
#define MQTT_ROOM_BUTTONS_PAYLOAD \
  "light_on:"     MQTT_TOPIC_ROOM_CMD ":ON|" \
  "light_off:"    MQTT_TOPIC_ROOM_CMD ":OFF|" \
  "light_horror:" MQTT_TOPIC_ROOM_CMD ":HORROR"

#define MQTT_CRYSTALS_BUTTONS_PAYLOAD \
  "crystals_start:" MQTT_TOPIC_CRYSTALS_CMD ":START|" \
  "crystals_stop:"  MQTT_TOPIC_CRYSTALS_CMD ":STOP"

// ---- Пины (безопасные для C3 SuperMini, ещё не подтверждены на железе) ----
#define LED_ROOM_PIN     2
#define LED_CRYSTAL_PIN  3

// ---- Лента "свет комнат" ----
#define ROOM_LED_COUNT     99

// ZONE_MAGIC — старый отдельный эффект (мягкая синяя пульсация), зона 1,
// трогать не просили. ZONE_FLAME — общий "холодный огонь" (ниже), один
// алгоритм на все мерцающие зоны, но у каждой свои cooling/spark/colorBias,
// поэтому визуально они не клоны друг друга.
enum RoomZoneStyle { ZONE_MAGIC, ZONE_FLAME };

struct RoomZone {
  uint8_t start;
  uint8_t count;
  RoomZoneStyle style;
  uint8_t cooling;      // ZONE_FLAME: выше -> пламя короче/гаснет быстрее
  uint8_t sparkChance;  // ZONE_FLAME: 0..255, шанс новой искры за кадр у основания
  uint8_t sparkMin;     // ZONE_FLAME: мин. "жар" новой искры
  uint8_t sparkMax;     // ZONE_FLAME: макс. "жар" новой искры
  int8_t  colorBias;    // ZONE_FLAME: сдвиг цветовой температуры зоны (в G-канал)
};

// Максимум диодов в одной ZONE_FLAME-зоне — под это выделен буфер "жара".
#define MAX_FLAME_ZONE_LEDS 32

// 6 зон, 10+31+25+11+11+11 = 99. Если физических полос станет больше —
// добавить сюда ещё строк (при MAX_FLAME_ZONE_LEDS >= их длины), остальной
// код трогать не нужно.
RoomZone roomZones[] = {
  // start, count, style,       cooling, sparkChance, sparkMin, sparkMax, colorBias
  {  0, 10, ZONE_MAGIC, 0,  0,   0,   0,   0 },  // как было — не трогаем
  { 10, 31, ZONE_FLAME, 55, 120, 160, 255,  15 },  // вставленная секция — живее, чуть голубее
  { 41, 25, ZONE_FLAME, 40, 90,  130, 220, -10 },  // бывшая "огневая" секция — чуть синее/фиолетовее
  { 66, 11, ZONE_FLAME, 70, 40,  60,  140,  25 },  // лёд #1 — спокойнее, белее
  { 77, 11, ZONE_FLAME, 65, 35,  50,  130,   0 },  // лёд #2 — нейтральный бело-синий
  { 88, 11, ZONE_FLAME, 75, 30,  50,  120, -20 },  // лёд #3 — спокойнее, холоднее/синее
};
#define ROOM_ZONE_COUNT (sizeof(roomZones) / sizeof(roomZones[0]))

// Буфер "жара" на каждую зону — отдельная история случайностей на зону,
// поэтому даже с одинаковыми параметрами они не идут синхронно.
uint8_t zoneHeat[ROOM_ZONE_COUNT][MAX_FLAME_ZONE_LEDS];

// ---- Лента "кристаллы" ----
#define CRYSTAL_LED_COUNT  11

// ---- Яркость (отдельно на каждую ленту) ----
#define ROOM_BRIGHTNESS     90
#define CRYSTAL_BRIGHTNESS  90   // дыхание само регулирует пик 20..220, ленте даём максимум

// ---- Демо-режим (нет связи с хабом или хаб ещё не прислал команду) ----
#define DEMO_PERIOD_MS   6000UL
#define DEMO_RENDER_MS     40UL

// ---- Рендер боевого режима света ----
#define ROOM_RENDER_MS      50

// ---- Хоррор: одна случайная зона периодически гаснет в ноль ----
#define HORROR_FADE_MS     1200UL   // плавный уход в 0 / возврат
#define HORROR_DARK_MS     2000UL   // сколько держать полную темноту
#define HORROR_MIN_GAP_MS  4000UL   // пауза между эпизодами — минимум
#define HORROR_MAX_GAP_MS 12000UL   // пауза между эпизодами — максимум

// ---- Кристаллы: дыхание яркости и цвет ----
#define CRYSTAL_RENDER_MS   30
#define CRYSTAL_MIN_BRIGHT   20
#define CRYSTAL_MAX_BRIGHT  220
// Розовый — пропорции каналов относительно пикового значения (R доминирует).
#define CRYSTAL_G_RATIO   0.35f
#define CRYSTAL_B_RATIO   0.63f

// ---- Прочее ----
#define STATS_PRINT_MS  10000

// ============================================================
// ОБЪЕКТЫ
// ============================================================
Adafruit_NeoPixel roomStrip(ROOM_LED_COUNT, LED_ROOM_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel crystalStrip(CRYSTAL_LED_COUNT, LED_CRYSTAL_PIN, NEO_GRB + NEO_KHZ800);

WiFiClient   mqttNet;
PubSubClient mqttClient(mqttNet);

// ============================================================
// СОСТОЯНИЕ — СВЕТ КОМНАТ
// ============================================================
enum RoomMode { ROOM_OFF, ROOM_ON, ROOM_HORROR };

bool     haveRoomState = false;   // true только после первой реальной команды с хаба
RoomMode roomMode      = ROOM_OFF;

enum HorrorPhase { HORROR_IDLE, HORROR_FADE_OUT, HORROR_HOLD_DARK, HORROR_FADE_IN };
HorrorPhase   horrorPhase        = HORROR_IDLE;
uint8_t       horrorZoneIndex    = 0;
unsigned long horrorPhaseStart   = 0;
unsigned long horrorNextTrigger  = 0;

unsigned long lastRoomRender = 0;

// ============================================================
// СОСТОЯНИЕ — КРИСТАЛЛЫ
// ============================================================
bool    haveCrystalsState             = false;  // true после первой команды/cycle-старта
bool    crystalsActive                = false;  // сейчас показываем раунд (не просто "0 и тишина")
uint8_t crystalCount                  = 0;
bool    crystalLit[CRYSTAL_LED_COUNT] = {false};

unsigned long lastCrystalRender = 0;

// ============================================================
// ОБЩЕЕ СОСТОЯНИЕ
// ============================================================
unsigned long lastWifiTry        = 0;
unsigned long lastStatsTime      = 0;
unsigned long loopCount          = 0;

bool otaInProgress    = false;
bool wifiWasConnected = false;
bool otaStarted        = false;

unsigned long lastMqttReconnect  = 0;
unsigned long lastMqttPublish    = 0;
unsigned long lastButtonsPublish = 0;
bool mqttNeedInitialPublish = false;

// На хабе DeviceState.info — всего char[64] (2 байта на кириллический
// символ), длинные строки там обрубаются посередине символа и превращаются
// в "�". Держим все info-сообщения короткими с запасом.
char roomInfoMsg[96]      = "свет: демо (нет связи)";
char roomStateMsg[16]     = "DEMO";
char crystalsInfoMsg[96]  = "кристаллы: демо (нет связи)";
char crystalsStateMsg[8]  = "0";

// ============================================================
// ПРОТОТИПЫ
// ============================================================
void wifiConnect();
void wifiCheck();
void otaSetup();
void mqttCheck();
void mqttPublishPeriodic();
void mqttCallback(char *topic, uint8_t *payload, unsigned int length);
void publishRoomInfo(const char *text);
void publishRoomState(const char *text);
void publishCrystalsInfo(const char *text);
void publishCrystalsState(const char *text);
void crystalsGenerateRound();
void crystalsStop();

// ============================================================
// ЦВЕТОВОЙ ГРАДИЕНТ ДЕМО: СИНИЙ -> ФИОЛЕТОВЫЙ -> БЕЛЫЙ -> ФИОЛЕТОВЫЙ -> ...
// ============================================================
void colorFromPhase(float phase, uint8_t &r, uint8_t &g, uint8_t &b) {
  static const uint8_t stops[4][3] = {
    {  0,   0, 255},
    {140,   0, 255},
    {255, 255, 255},
    {140,   0, 255}
  };

  float p = fmodf(phase, 4.0f);
  if (p < 0) p += 4.0f;

  int seg = (int)p;
  float t = p - seg;
  int segNext = (seg + 1) % 4;

  r = stops[seg][0] + (uint8_t)((stops[segNext][0] - stops[seg][0]) * t);
  g = stops[seg][1] + (uint8_t)((stops[segNext][1] - stops[seg][1]) * t);
  b = stops[seg][2] + (uint8_t)((stops[segNext][2] - stops[seg][2]) * t);
}

void renderDemo(Adafruit_NeoPixel &strip, uint16_t count) {
  float basePhase = fmodf(millis() / (float)DEMO_PERIOD_MS * 4.0f, 4.0f);

  for (uint16_t i = 0; i < count; i++) {
    float phase = basePhase + (4.0f * i) / count;
    uint8_t r, g, b;
    colorFromPhase(phase, r, g, b);
    strip.setPixelColor(i, strip.Color(r, g, b));
  }
  strip.show();
}

// ============================================================
// СВЕТ КОМНАТ — рендер зон
// ============================================================

// Холодный бело-синий "магический" свет — мягкая, медленная пульсация,
// у каждого диода свой фазовый сдвиг (не мигают все разом).
void renderZoneMagic(const RoomZone &zone, float darkness) {
  float t = millis() / 1000.0f;
  for (uint8_t i = 0; i < zone.count; i++) {
    float phase = t * 0.6f + i * 0.35f;
    float twinkle = 0.75f + 0.25f * sinf(phase);   // 0.5..1.0
    float k = twinkle * darkness;
    uint8_t r = (uint8_t)(120 * k);
    uint8_t g = (uint8_t)(170 * k);
    uint8_t b = (uint8_t)(255 * k);
    roomStrip.setPixelColor(zone.start + i, roomStrip.Color(r, g, b));
  }
}

// Насыщающее сложение (как qadd8 у FastLED) — чтобы "жар" не переворачивался
// через 255 обратно в 0.
uint8_t satAdd8(uint8_t a, uint8_t b) {
  uint16_t sum = (uint16_t)a + b;
  return sum > 255 ? 255 : (uint8_t)sum;
}

// "Жар" (0..255) -> цвет. Та же идея, что в классическом огне (Fire2012 /
// наработки AlexGyver по лампам-пламя): чёрный -> тёмный -> яркий -> белый,
// только вместо чёрный->красный->жёлтый->белый у нас чёрный->синий->
// голубой->белый. colorBias сдвигает зелёный канал — лёгкий уход в сторону
// бирюзы (+) или в сторону чистого синего/фиолетового (-), чтобы соседние
// зоны отличались по "цветовой температуре", а не были клонами.
void heatToColorCold(uint8_t heat, int8_t colorBias, uint8_t &r, uint8_t &g, uint8_t &b) {
  if (heat > 170) {
    uint8_t k = heat - 170;                 // 0..85 -> переход в белый
    uint8_t w = (uint16_t)k * 255 / 85;
    r = w;
    g = 220 + w / 5;
    b = 255;
  } else if (heat > 85) {
    uint8_t k = heat - 85;                  // 0..85 -> тёмно-синий -> голубой
    r = 0;
    g = 40  + (uint16_t)k * (180 - 40)  / 85;
    b = 140 + (uint16_t)k * (255 - 140) / 85;
  } else {
    uint8_t k = heat;                       // 0..85 -> чёрный -> тёмно-синий
    r = 0;
    g = 0;
    b = 10 + (uint16_t)k * (140 - 10) / 85;
  }

  int16_t gBiased = (int16_t)g + colorBias;
  g = gBiased < 0 ? 0 : (gBiased > 255 ? 255 : (uint8_t)gBiased);
}

// Один и тот же алгоритм "холодного огня" на все ZONE_FLAME-зоны, но у
// каждой свой буфер "жара" (zoneHeat[zoneIdx]) и свои cooling/spark/colorBias
// из таблицы roomZones[] — поэтому визуально они не повторяют друг друга.
void renderZoneFlame(uint8_t zoneIdx, const RoomZone &zone, float darkness) {
  uint8_t *heat = zoneHeat[zoneIdx];
  uint8_t n = zone.count;

  // 1) остывание — каждый пиксель теряет немного "жара".
  for (uint8_t i = 0; i < n; i++) {
    uint8_t cooldown = random(0, (zone.cooling * 10) / n + 2);
    heat[i] = (cooldown >= heat[i]) ? 0 : heat[i] - cooldown;
  }

  // 2) жар "поднимается" вдоль зоны (диффузия к большим индексам).
  for (uint8_t i = n - 1; i >= 2; i--) {
    heat[i] = (heat[i - 1] + heat[i - 2] + heat[i - 2]) / 3;
  }

  // 3) случайная "искра" у основания зоны.
  if ((uint8_t)random(0, 256) < zone.sparkChance) {
    uint8_t y = random(0, 2);
    heat[y] = satAdd8(heat[y], random(zone.sparkMin, zone.sparkMax + 1));
  }

  // 4) жар -> цвет, с учётом затемнения хоррора.
  for (uint8_t i = 0; i < n; i++) {
    uint8_t r, g, b;
    heatToColorCold(heat[i], zone.colorBias, r, g, b);
    r = (uint8_t)(r * darkness);
    g = (uint8_t)(g * darkness);
    b = (uint8_t)(b * darkness);
    roomStrip.setPixelColor(zone.start + i, roomStrip.Color(r, g, b));
  }
}

// ============================================================
// СВЕТ КОМНАТ — хоррор: одна случайная зона периодически уходит в ноль
// ============================================================
float horrorDarknessForZone(uint8_t zoneIndex) {
  if (roomMode != ROOM_HORROR || zoneIndex != horrorZoneIndex) return 1.0f;

  unsigned long now = millis();
  switch (horrorPhase) {
    case HORROR_FADE_OUT: {
      float t = (now - horrorPhaseStart) / (float)HORROR_FADE_MS;
      if (t > 1.0f) t = 1.0f;
      return 1.0f - t;
    }
    case HORROR_HOLD_DARK:
      return 0.0f;
    case HORROR_FADE_IN: {
      float t = (now - horrorPhaseStart) / (float)HORROR_FADE_MS;
      if (t > 1.0f) t = 1.0f;
      return t;
    }
    default:
      return 1.0f;
  }
}

void updateHorror() {
  if (roomMode != ROOM_HORROR) {
    horrorPhase = HORROR_IDLE;
    return;
  }

  unsigned long now = millis();
  switch (horrorPhase) {
    case HORROR_IDLE:
      if (now >= horrorNextTrigger) {
        horrorZoneIndex  = random(0, ROOM_ZONE_COUNT);
        horrorPhase      = HORROR_FADE_OUT;
        horrorPhaseStart = now;
      }
      break;

    case HORROR_FADE_OUT:
      if (now - horrorPhaseStart >= HORROR_FADE_MS) {
        horrorPhase = HORROR_HOLD_DARK;
        horrorPhaseStart = now;
      }
      break;

    case HORROR_HOLD_DARK:
      if (now - horrorPhaseStart >= HORROR_DARK_MS) {
        horrorPhase = HORROR_FADE_IN;
        horrorPhaseStart = now;
      }
      break;

    case HORROR_FADE_IN:
      if (now - horrorPhaseStart >= HORROR_FADE_MS) {
        horrorPhase = HORROR_IDLE;
        horrorNextTrigger = now + random(HORROR_MIN_GAP_MS, HORROR_MAX_GAP_MS);
      }
      break;
  }
}

void renderRoomZones() {
  for (uint8_t z = 0; z < ROOM_ZONE_COUNT; z++) {
    float darkness = horrorDarknessForZone(z);
    if (roomZones[z].style == ZONE_MAGIC) renderZoneMagic(roomZones[z], darkness);
    else                                  renderZoneFlame(z, roomZones[z], darkness);
  }
  roomStrip.show();
}

void renderRoomOff() {
  roomStrip.clear();
  roomStrip.show();
}

void roomLoop() {
  if (!mqttClient.connected() || !haveRoomState) {
    if (millis() - lastRoomRender >= DEMO_RENDER_MS) {
      lastRoomRender = millis();
      renderDemo(roomStrip, ROOM_LED_COUNT);
    }
    return;
  }

  if (roomMode == ROOM_OFF) {
    if (millis() - lastRoomRender >= ROOM_RENDER_MS) {
      lastRoomRender = millis();
      renderRoomOff();
    }
    return;
  }

  updateHorror();

  if (millis() - lastRoomRender >= ROOM_RENDER_MS) {
    lastRoomRender = millis();
    renderRoomZones();
  }
}

// ============================================================
// КРИСТАЛЛЫ — генерация раунда: случайное число 1..11, случайные позиции
// (при малых числах — стараемся не ставить рядом на ленте)
// ============================================================
void crystalsGenerateRound() {
  uint8_t n = (uint8_t)random(1, CRYSTAL_LED_COUNT + 1);   // 1..11

  bool placed = false;
  bool candidate[CRYSTAL_LED_COUNT];
  uint8_t order[CRYSTAL_LED_COUNT];

  // До 300 попыток случайно расставить N позиций так, чтобы никакие две
  // не были соседними на ленте (при большом N это может быть невозможно —
  // тогда падаем на простой случайный набор без этого ограничения).
  for (uint16_t attempt = 0; attempt < 300 && !placed; attempt++) {
    for (uint8_t i = 0; i < CRYSTAL_LED_COUNT; i++) { candidate[i] = false; order[i] = i; }

    for (int8_t i = CRYSTAL_LED_COUNT - 1; i > 0; i--) {
      int j = random(0, i + 1);
      uint8_t tmp = order[i]; order[i] = order[j]; order[j] = tmp;
    }

    uint8_t placedCount = 0;
    for (uint8_t k = 0; k < CRYSTAL_LED_COUNT && placedCount < n; k++) {
      uint8_t idx = order[k];
      bool ok = true;
      if (idx > 0 && candidate[idx - 1]) ok = false;
      if (idx < CRYSTAL_LED_COUNT - 1 && candidate[idx + 1]) ok = false;
      if (ok) { candidate[idx] = true; placedCount++; }
    }

    if (placedCount == n) placed = true;
  }

  if (placed) {
    memcpy(crystalLit, candidate, sizeof(candidate));
  } else {
    // N слишком велико для "не рядом" на 11 диодах — просто случайные N.
    for (uint8_t i = 0; i < CRYSTAL_LED_COUNT; i++) { crystalLit[i] = false; order[i] = i; }
    for (int8_t i = CRYSTAL_LED_COUNT - 1; i > 0; i--) {
      int j = random(0, i + 1);
      uint8_t tmp = order[i]; order[i] = order[j]; order[j] = tmp;
    }
    for (uint8_t k = 0; k < n; k++) crystalLit[order[k]] = true;
  }

  crystalCount     = n;
  crystalsActive   = true;
  haveCrystalsState = true;

  char msg[64];
  snprintf(msg, sizeof(msg), "кристаллов горит: %u", crystalCount);
  publishCrystalsInfo(msg);

  char stateBuf[8];
  snprintf(stateBuf, sizeof(stateBuf), "%u", crystalCount);
  publishCrystalsState(stateBuf);
}

void crystalsStop() {
  crystalsActive    = false;
  haveCrystalsState = true;
  crystalCount      = 0;
  for (uint8_t i = 0; i < CRYSTAL_LED_COUNT; i++) crystalLit[i] = false;

  publishCrystalsInfo("кристаллы: стоп");
  publishCrystalsState("0");
}

void crystalsLoop() {
  if (millis() - lastCrystalRender < CRYSTAL_RENDER_MS) return;
  lastCrystalRender = millis();

  if (!mqttClient.connected() || !haveCrystalsState) {
    renderDemo(crystalStrip, CRYSTAL_LED_COUNT);
    return;
  }

  if (!crystalsActive) {
    crystalStrip.clear();
    crystalStrip.show();
    return;
  }

  float t = millis() / 1000.0f;
  float breathe = 0.5f + 0.5f * sinf(t * 2.0f);   // 0..1
  uint8_t pulse = CRYSTAL_MIN_BRIGHT + (uint8_t)((CRYSTAL_MAX_BRIGHT - CRYSTAL_MIN_BRIGHT) * breathe);

  uint8_t r = pulse;
  uint8_t g = (uint8_t)(pulse * CRYSTAL_G_RATIO);
  uint8_t b = (uint8_t)(pulse * CRYSTAL_B_RATIO);

  for (uint8_t i = 0; i < CRYSTAL_LED_COUNT; i++) {
    crystalStrip.setPixelColor(i, crystalLit[i] ? crystalStrip.Color(r, g, b) : 0);
  }
  crystalStrip.show();
}

// ============================================================
// WIFI
// ============================================================
void wifiConnect() {
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_15dBm);

#if USE_STATIC_IP
  if (!WiFi.config(local_IP, gateway, subnet, dns1)) {
    Serial.println("[WIFI] не удалось применить статический IP");
  }
#else
  Serial.println("[WIFI] режим DHCP (стенд)");
#endif

  WiFi.setSleep(false);
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
    roomStrip.clear();
    roomStrip.show();
    crystalStrip.clear();
    crystalStrip.show();
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("[OTA] завершено, перезагрузка");
    for (uint16_t i = 0; i < ROOM_LED_COUNT; i++) roomStrip.setPixelColor(i, roomStrip.Color(0, 255, 0));
    roomStrip.show();
    for (uint16_t i = 0; i < CRYSTAL_LED_COUNT; i++) crystalStrip.setPixelColor(i, crystalStrip.Color(0, 255, 0));
    crystalStrip.show();
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    uint16_t lit = (uint32_t)progress * ROOM_LED_COUNT / total;
    roomStrip.clear();
    for (uint16_t i = 0; i < lit && i < ROOM_LED_COUNT; i++) {
      roomStrip.setPixelColor(i, roomStrip.Color(0, 0, 255));
    }
    roomStrip.show();
  });

  ArduinoOTA.onError([](ota_error_t error) {
    otaInProgress = false;
    Serial.print("[OTA] ОШИБКА ");
    Serial.println(error);
    for (uint16_t i = 0; i < ROOM_LED_COUNT; i++) roomStrip.setPixelColor(i, roomStrip.Color(255, 0, 0));
    roomStrip.show();
    for (uint16_t i = 0; i < CRYSTAL_LED_COUNT; i++) crystalStrip.setPixelColor(i, crystalStrip.Color(255, 0, 0));
    crystalStrip.show();
  });

  ArduinoOTA.begin();
  Serial.print("[OTA] готов, имя ");
  Serial.println(OTA_HOSTNAME);
}

// ============================================================
// MQTT
// ============================================================
void publishRoomInfo(const char *text) {
  strncpy(roomInfoMsg, text, sizeof(roomInfoMsg) - 1);
  roomInfoMsg[sizeof(roomInfoMsg) - 1] = '\0';
  Serial.println(roomInfoMsg);
  if (mqttClient.connected()) mqttClient.publish(MQTT_TOPIC_ROOM_INFO, roomInfoMsg, true);
}

void publishRoomState(const char *text) {
  strncpy(roomStateMsg, text, sizeof(roomStateMsg) - 1);
  roomStateMsg[sizeof(roomStateMsg) - 1] = '\0';
  if (mqttClient.connected()) mqttClient.publish(MQTT_TOPIC_ROOM_STATE, roomStateMsg, true);
}

void publishCrystalsInfo(const char *text) {
  strncpy(crystalsInfoMsg, text, sizeof(crystalsInfoMsg) - 1);
  crystalsInfoMsg[sizeof(crystalsInfoMsg) - 1] = '\0';
  Serial.println(crystalsInfoMsg);
  if (mqttClient.connected()) mqttClient.publish(MQTT_TOPIC_CRYSTALS_INFO, crystalsInfoMsg, true);
}

void publishCrystalsState(const char *text) {
  strncpy(crystalsStateMsg, text, sizeof(crystalsStateMsg) - 1);
  crystalsStateMsg[sizeof(crystalsStateMsg) - 1] = '\0';
  if (mqttClient.connected()) mqttClient.publish(MQTT_TOPIC_CRYSTALS_STATE, crystalsStateMsg, true);
}

void mqttCallback(char *topic, uint8_t *payload, unsigned int length) {
  char buf[32] = {0};
  unsigned int n = length < sizeof(buf) - 1 ? length : sizeof(buf) - 1;
  memcpy(buf, payload, n);

  Serial.print("[MQTT cmd] ");
  Serial.print(topic);
  Serial.print(" -> ");
  Serial.println(buf);

  if (strcmp(topic, MQTT_TOPIC_CYCLE_START) == 0) {
    // Общий таймер квеста — новый цикл = новый раунд кристаллов.
    crystalsGenerateRound();
    return;
  }

  if (strcmp(topic, MQTT_TOPIC_ROOM_CMD) == 0) {
    if (strcmp(buf, "ON") == 0) {
      haveRoomState = true;
      roomMode = ROOM_ON;
      publishRoomInfo("свет комнат: включён");
      publishRoomState("ON");
    } else if (strcmp(buf, "OFF") == 0) {
      haveRoomState = true;
      roomMode = ROOM_OFF;
      publishRoomInfo("свет комнат: выключен");
      publishRoomState("OFF");
    } else if (strcmp(buf, "HORROR") == 0) {
      haveRoomState = true;
      roomMode = ROOM_HORROR;
      horrorPhase = HORROR_IDLE;
      horrorNextTrigger = millis() + random(HORROR_MIN_GAP_MS, HORROR_MAX_GAP_MS);
      publishRoomInfo("свет комнат: хоррор");
      publishRoomState("HORROR");
    }
    return;
  }

  if (strcmp(topic, MQTT_TOPIC_CRYSTALS_CMD) == 0) {
    if (strcmp(buf, "START") == 0) {
      crystalsGenerateRound();
    } else if (strcmp(buf, "STOP") == 0) {
      crystalsStop();
    }
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
    mqttClient.subscribe(MQTT_TOPIC_ROOM_CMD);
    mqttClient.subscribe(MQTT_TOPIC_CRYSTALS_CMD);
    mqttClient.subscribe(MQTT_TOPIC_CYCLE_START);
    mqttNeedInitialPublish = true;
  } else {
    Serial.print("не вышло, state=");
    Serial.println(mqttClient.state());
  }
}

// После (пере)подключения — сразу публикуем текущее состояние (retain).
// rssi у ОБОИХ устройств раз в MQTT_PUBLISH_MS — иначе хаб считает
// "молчащее" устройство оффлайн через 10 сек (было замечено на кристаллах).
void mqttPublishPeriodic() {
  if (!mqttClient.connected()) return;

  if (mqttNeedInitialPublish) {
    mqttClient.publish(MQTT_TOPIC_ROOM_INFO, roomInfoMsg, true);
    mqttClient.publish(MQTT_TOPIC_ROOM_STATE, roomStateMsg, true);
    mqttClient.publish(MQTT_TOPIC_ROOM_BUTTONS, MQTT_ROOM_BUTTONS_PAYLOAD, true);
    mqttClient.publish(MQTT_TOPIC_CRYSTALS_INFO, crystalsInfoMsg, true);
    mqttClient.publish(MQTT_TOPIC_CRYSTALS_STATE, crystalsStateMsg, true);
    mqttClient.publish(MQTT_TOPIC_CRYSTALS_BUTTONS, MQTT_CRYSTALS_BUTTONS_PAYLOAD, true);
    lastButtonsPublish = millis();
    mqttNeedInitialPublish = false;
  }

  if (millis() - lastMqttPublish >= MQTT_PUBLISH_MS) {
    lastMqttPublish = millis();
    String rssi = String(WiFi.RSSI());
    mqttClient.publish(MQTT_TOPIC_ROOM_RSSI, rssi.c_str());
    mqttClient.publish(MQTT_TOPIC_CRYSTALS_RSSI, rssi.c_str());
  }

  if (millis() - lastButtonsPublish >= MQTT_BUTTONS_REPUBLISH_MS) {
    lastButtonsPublish = millis();
    mqttClient.publish(MQTT_TOPIC_ROOM_BUTTONS, MQTT_ROOM_BUTTONS_PAYLOAD, true);
    mqttClient.publish(MQTT_TOPIC_CRYSTALS_BUTTONS, MQTT_CRYSTALS_BUTTONS_PAYLOAD, true);
  }
}

// ============================================================
// SETUP / LOOP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(2000);

  randomSeed(esp_random());

  roomStrip.begin();
  roomStrip.setBrightness(ROOM_BRIGHTNESS);
  roomStrip.clear();
  roomStrip.show();

  crystalStrip.begin();
  crystalStrip.setBrightness(CRYSTAL_BRIGHTNESS);
  crystalStrip.clear();
  crystalStrip.show();

  mqttNet.setTimeout(150);   // короче, чтобы недоступный сервер не мешал ArduinoOTA.handle() — OTA должна работать без сервера, достаточно общей сети
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(384);

  Serial.println();
  Serial.println("========================================");
  Serial.println("  СВЕТ КОМНАТ + КРИСТАЛЛЫ, старт");
  Serial.println("========================================");
  Serial.print("ROOM: pin="); Serial.print(LED_ROOM_PIN);
  Serial.print(" leds="); Serial.print(ROOM_LED_COUNT);
  Serial.print(" zones="); Serial.println(ROOM_ZONE_COUNT);
  Serial.print("CRYSTALS: pin="); Serial.print(LED_CRYSTAL_PIN);
  Serial.print(" leds="); Serial.println(CRYSTAL_LED_COUNT);

  wifiConnect();

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
    renderDemo(roomStrip, ROOM_LED_COUNT);
    renderDemo(crystalStrip, CRYSTAL_LED_COUNT);
    delay(30);
  }
  Serial.println();

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
    Serial.println("[WIFI] не подключилось, работаем локально (демо), реконнект в фоне");
  }

  Serial.println("[СТАРТ] готово, демо-режим активен пока хаб не пришлёт состояние");
  Serial.println();
}

void loop() {
  ArduinoOTA.handle();
  if (otaInProgress) return;

  roomLoop();
  crystalsLoop();

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
    Serial.print(" dBm  room=");
    Serial.print(haveRoomState ? roomStateMsg : "DEMO");
    Serial.print("  crystals=");
    Serial.println(haveCrystalsState ? crystalsStateMsg : "DEMO");
  }
}
