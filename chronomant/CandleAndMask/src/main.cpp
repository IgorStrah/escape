/*
 * МАСКИ + СВЕЧИ — хрономаг, устройство «Огонь чернокнижника»
 * ESP32-C3 SuperMini  /  PlatformIO (framework = arduino)
 *
 * MUX1 (GPIO3) - цифра, 16 каналов, маски 1-16
 * MUX2 (GPIO4) - аналог, кан 0-2 маски 17-19, кан 3-10 свечи 1-8
 * WS2812 (GPIO1) - 38 диодов, по 2 на маску
 *
 * Через MQTT (TestHub на стенде, chrono-hub.local):
 *   quest/chronomage/masks/...   — маски: обычный режим, калибровка
 *                                  пар (CALIBRATE_START/SKIP/ABORT),
 *                                  тестовый прогон игры (START_MASKS_TEST),
 *                                  таймаут касания (SET_TIMEOUT:<мс>)
 *   quest/chronomage/candles/... — сырые ADC свечей (без сравнения с паттерном)
 *
 * Прошивка по воздуху: PlatformIO -> upload_port = chrono-masks.local
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <Adafruit_NeoPixel.h>
#include <PubSubClient.h>
#include <esp_system.h>

// ============================================================
// КОНФИГУРАЦИЯ
// ============================================================

// ---- Сеть ----
#define WIFI_SSID     "weasgley"
#define WIFI_PASS     "weasgley123"

#define OTA_HOSTNAME  "chrono-masks"
#define OTA_PASSWORD  "chrono"

// 1 = статический IP (продакшн-сеть Quest Hub), 0 = DHCP (стенд).
// На стенде сеть weasgley = 192.168.0.x, поэтому пока DHCP.
#define USE_STATIC_IP  0

// Продакшн-адреса (сеть Quest Hub 192.168.8.x) — применяются при USE_STATIC_IP=1
IPAddress local_IP(192, 168, 8, 181);
IPAddress gateway (192, 168, 8, 1);
IPAddress subnet  (255, 255, 255, 0);
IPAddress dns1    (192, 168, 8, 1);

#define WIFI_RETRY_MS   5000

// ---- MQTT (тестовый хаб TestHub, стенд) ----
// Хаб на DHCP, поэтому по имени через mDNS, а не по фиксированному IP.
#define MQTT_HOST         "chrono-hub.local"
#define MQTT_PORT         1883
#define MQTT_CLIENT_ID    "chrono-masks"
#define MQTT_DEVICE_TOPIC "quest/chronomage/masks"
#define MQTT_RECONNECT_MS 4000
#define MQTT_PUBLISH_MS   4000
#define MQTT_BUTTONS_REPUBLISH_MS (3UL * 60UL * 1000UL)

#define MQTT_TOPIC_INFO    MQTT_DEVICE_TOPIC "/info"
#define MQTT_TOPIC_STATE   MQTT_DEVICE_TOPIC "/state"
#define MQTT_TOPIC_RSSI    MQTT_DEVICE_TOPIC "/rssi"
#define MQTT_TOPIC_BUTTONS MQTT_DEVICE_TOPIC "/buttons"
#define MQTT_TOPIC_CMD     MQTT_DEVICE_TOPIC "/cmd"

// Свечи публикуются под отдельным именем устройства — по ТЗ это отдельное
// задание квеста, хоть и живёт физически на той же плате.
#define MQTT_CANDLES_TOPIC        "quest/chronomage/candles"
#define MQTT_TOPIC_CANDLES_INFO   MQTT_CANDLES_TOPIC "/info"
#define MQTT_TOPIC_CANDLES_STATE  MQTT_CANDLES_TOPIC "/state"

// Формат кнопок — как у боевых устройств комнаты: метка:топик:payload,
// через "|". Клик на хабе публикует именно topic/payload, а не всегда /cmd.
// start_masks — тестовая кнопка: сама генерирует случайный порядок и
// запускает игровую логику локально, без реального quest/.../cycle/start.
#define MQTT_BUTTONS_PAYLOAD \
  "calibrate:" MQTT_TOPIC_CMD ":CALIBRATE_START|" \
  "calib_skip:" MQTT_TOPIC_CMD ":CALIBRATE_SKIP|" \
  "calib_abort:" MQTT_TOPIC_CMD ":CALIBRATE_ABORT|" \
  "start_masks:" MQTT_TOPIC_CMD ":START_MASKS_TEST|" \
  "stop_masks:" MQTT_TOPIC_CMD ":STOP_MASKS_TEST"

// ---- Пины ----
#define MUX_S0    5
#define MUX_S1    6
#define MUX_S2    7
#define MUX_S3    8

#define MUX1_SIG  3    // цифровой вход
#define MUX2_SIG  4    // аналоговый вход (ADC1_CH4)

#define LED_PIN   1    // WS2812 DIN

// ---- Размеры ----
#define MASK_COUNT        19
#define LED_COUNT         38

#define MUX1_CHANNELS     16   // маски 1-16
#define MUX2_MASK_START    0
#define MUX2_MASK_COUNT    3   // маски 17-19
#define MUX2_CANDLE_START  3
#define MUX2_CANDLE_COUNT  8   // свечи 1-8

// ---- Тайминги опроса ----
#define SETTLE_DIGITAL_US   5
#define SETTLE_ANALOG_US  100   // проверено на живом железе
#define DEBOUNCE_COUNT      3   // подтверждений подряд

// ---- Пороги (калибруются на месте) ----
#define MASK_THRESHOLD  2000   // аналоговое чтение TTP223 на MUX2

// ---- Прочее ----
#define STATS_PRINT_MS  10000
#define LED_BRIGHTNESS  40

// ---- Тайминги игры "маски" ----
#define DEFAULT_MASK_TIMEOUT_MS 2000   // сколько ждать касание активной маски
#define MIN_MASK_TIMEOUT_MS      200
#define MAX_MASK_TIMEOUT_MS    20000

#define CANDLE_PUBLISH_MS  1000

// ---- Мерцание пламени / цепочка касаний в простое ----
#define FLICKER_UPDATE_MS  80
#define FLICKER_MIN_RED     5
#define FLICKER_MAX_RED    20   // "от 5 до 20" на красный канал

// ============================================================
// ТАБЛИЦА: МАСКА -> ПАРА СВЕТОДИОДОВ
// Индекс = номер маски минус 1. Значение = номер пары (0..18),
// пара N = диоды N*2 и N*2+1. Единственное место правки при монтаже.
// Получено калибровкой (CALIBRATE_START) на живом железе.
// ============================================================
uint8_t maskToLedPair[MASK_COUNT] = {
  11,   // маска 1
  12,   // маска 2
  13,   // маска 3
  14,   // маска 4
  16,   // маска 5
  15,   // маска 6
  17,   // маска 7
  18,   // маска 8
  10,   // маска 9
   9,   // маска 10
   8,   // маска 11
   7,   // маска 12
   6,   // маска 13
   5,   // маска 14
   4,   // маска 15
   3,   // маска 16
   2,   // маска 17 (MUX2 кан 0)
   1,   // маска 18 (MUX2 кан 1)
   0    // маска 19 (MUX2 кан 2)
};

// ============================================================
// ТАБЛИЦА: СВЕЧА -> КАНАЛ MUX2
// ============================================================
uint8_t candleToChannel[MUX2_CANDLE_COUNT] = {
  3, 4, 5, 6, 7, 8, 9, 10
};

// ==================== ЦВЕТА ====================
#define COL_OFF     0, 0, 0
#define COL_RED     255, 0, 0
#define COL_WHITE   255, 255, 255
#define COL_GREEN   0, 255, 0
#define COL_BLUE    0, 0, 255

// ==================== ОБЪЕКТЫ ====================
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

WiFiClient   mqttNet;
PubSubClient mqttClient(mqttNet);

// ==================== СОСТОЯНИЕ ====================
bool    mux1State[MUX1_CHANNELS];
bool    mux1Candidate[MUX1_CHANNELS];
uint8_t mux1Counter[MUX1_CHANNELS];

bool    mux2MaskState[MUX2_MASK_COUNT];
bool    mux2MaskCandidate[MUX2_MASK_COUNT];
uint8_t mux2MaskCounter[MUX2_MASK_COUNT];

int     candleRaw[MUX2_CANDLE_COUNT];
unsigned long lastCandlePublish = 0;

unsigned long lastStatsTime   = 0;
unsigned long lastWifiTry     = 0;
unsigned long loopCount       = 0;
unsigned long cycleTimeSum    = 0;
unsigned long cycleTimeCount  = 0;

bool otaInProgress   = false;
bool wifiWasConnected = false;
bool otaStarted       = false;

unsigned long lastMqttReconnect     = 0;
unsigned long lastMqttPublish       = 0;
unsigned long lastButtonsPublish    = 0;
bool mqttNeedInitialPublish = false;   // выставляется true при (пере)подключении

// info — подробный человекочитаемый текст (что произошло).
// state — короткое машинное значение (режим устройства).
char infoMsg[128] = "маски+свечи, ESP32-C3 (только кнопки/LED, без фотодатчиков)";
char stateMsg[48] = "IDLE";

// ==================== КАЛИБРОВКА МАСКА<->ПАРА ====================
bool    calibMode          = false;
int     calibPairIndex     = -1;     // текущая проверяемая пара (-1 = ещё не начали)
bool    calibWaiting       = false;  // ждём нажатия для текущей пары
unsigned long calibConfirmUntil = 0; // до какого millis() держим зелёное подтверждение
uint8_t calibAssignedPair[MASK_COUNT]; // индекс = maskNum-1, значение = найденная пара (0xFF = не найдена)

// ==================== ИГРА "МАСКИ" (тестовый прогон) ====================
unsigned long maskTimeoutMs = DEFAULT_MASK_TIMEOUT_MS;  // настраивается командой SET_TIMEOUT:<мс>
bool          gameActive    = false;
int           gameIndex     = -1;
uint8_t       gameOrder[MASK_COUNT];
unsigned long gameDeadline  = 0;

// ==================== ОГОНЬ: МЕРЦАНИЕ + СЦЕНАРИЙ КАСАНИЙ ====================
// Активно только вне калибровки и явного теста (start_masks).
// Сценарий — случайный порядок всех 19 масок, сам пересобирается раз в
// SCENARIO_REGEN_MS (не зависит от того, играет ли сейчас кто-то).
// Любое касание в дежурном режиме "вооружает" прохождение (само касание
// не засчитывается) — загорается синим ПЕРВАЯ маска текущего сценария.
// Коснулся именно её вовремя (SCENARIO_TOUCH_MS) -> белая, загорается
// синим следующая по сценарию. Не успел -> полный сброс на позицию 0,
// все маски обратно мерцают красным, ждём новое "вооружающее" касание.
#define VIS_FLICKER 0
#define VIS_TARGET  1
#define VIS_DONE    2
#define VIS_CELEBRATE 3   // весь сценарий пройден — держим зелёный до новой генерации
uint8_t maskVisualState[MASK_COUNT];   // индекс = maskNum-1

unsigned long lastFlickerUpdate = 0;

#define SCENARIO_REGEN_MS (3UL * 60UL * 1000UL)   // новый сценарий каждые 3 мин
#define SCENARIO_TOUCH_MS 3000                     // 3 сек на текущую синюю цель

uint8_t       scenarioOrder[MASK_COUNT];   // текущий случайный порядок (значения 1..19)
unsigned long lastScenarioGen = 0;

bool          chaseActive         = false;  // "вооружено", прохождение идёт
int           chaseIndex          = -1;     // позиция в scenarioOrder, -1 = не начато
unsigned long chaseTargetDeadline = 0;       // дедлайн на текущую синюю цель

// ==================== ПРОТОТИПЫ ====================
void setMuxChannel(uint8_t ch);
bool readMux1(uint8_t ch);
int  readMux2(uint8_t ch);
void setMaskColor(uint8_t maskNum, uint8_t r, uint8_t g, uint8_t b);
void setPairColor(uint8_t pair, uint8_t r, uint8_t g, uint8_t b);
void clearAllLeds();
void wifiConnect();
void wifiCheck();
void otaSetup();
void mqttCheck();
void mqttPublishPeriodic();
void mqttCallback(char *topic, uint8_t *payload, unsigned int length);
void publishInfo(const char *text);
void publishState(const char *text);
void handleMaskTransition(uint8_t maskNum, int channel, const char *muxLabel, bool pressed);
void scanMux1();
void scanMux2();
void dumpAll();
void calibStart();
void calibSkip();
void calibAbort();
void calibFinish();
void calibLoop();
void calibShowPair(int pair);
void calibHandlePress(uint8_t maskNum, int channel, const char *muxLabel);
void publishCandles();
void gameStart();
void gameStop();
void gameShowActive();
void gameHandleTouch(uint8_t maskNum);
void gameFinish();
void gameLoop();
void updateFlicker();
void scenarioGenerate();
void scenarioCheck();
void chaseArm();
void chaseHandleTouch(uint8_t maskNum);
void chaseTimeoutCheck();
void chaseFinish();
void chaseReset();
void chaseStop();

// ==================== МУЛЬТИПЛЕКСОР ====================
void setMuxChannel(uint8_t ch) {
  digitalWrite(MUX_S0, ch & 0x01);
  digitalWrite(MUX_S1, (ch >> 1) & 0x01);
  digitalWrite(MUX_S2, (ch >> 2) & 0x01);
  digitalWrite(MUX_S3, (ch >> 3) & 0x01);
}

bool readMux1(uint8_t ch) {
  setMuxChannel(ch);
  delayMicroseconds(SETTLE_DIGITAL_US);
  return digitalRead(MUX1_SIG);
}

int readMux2(uint8_t ch) {
  setMuxChannel(ch);
  delayMicroseconds(SETTLE_ANALOG_US);
  return analogRead(MUX2_SIG);
}

// ==================== СВЕТОДИОДЫ ====================
void setMaskColor(uint8_t maskNum, uint8_t r, uint8_t g, uint8_t b) {
  if (maskNum < 1 || maskNum > MASK_COUNT) return;

  uint8_t pair = maskToLedPair[maskNum - 1];
  uint16_t led1 = pair * 2;
  uint16_t led2 = pair * 2 + 1;

  if (led1 < LED_COUNT) strip.setPixelColor(led1, strip.Color(r, g, b));
  if (led2 < LED_COUNT) strip.setPixelColor(led2, strip.Color(r, g, b));
}

void setPairColor(uint8_t pair, uint8_t r, uint8_t g, uint8_t b) {
  uint16_t led1 = pair * 2;
  uint16_t led2 = pair * 2 + 1;

  if (led1 < LED_COUNT) strip.setPixelColor(led1, strip.Color(r, g, b));
  if (led2 < LED_COUNT) strip.setPixelColor(led2, strip.Color(r, g, b));
}

void clearAllLeds() {
  strip.clear();
  strip.show();
}


// ==================== WIFI ====================
void wifiConnect() {
  WiFi.mode(WIFI_STA);

  // Известная проблема C3 SuperMini: регулятор не тянет 19.5 dBm.
  // При RSSI ~-89 поднято до 15 dBm — ПРОВЕРИТЬ на железе: несколько раз
  // передёрнуть питание с подключёнными шлейфами, убедиться что нет ребутов
  // по просадке. Если ловит ребуты/download mode — вернуть WIFI_POWER_11dBm.
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

// ==================== OTA ====================
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
    for (uint8_t p = 0; p < MASK_COUNT; p++) setPairColor(p, COL_GREEN);
    strip.show();
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    uint8_t pairs = (progress * MASK_COUNT) / total;
    strip.clear();
    for (uint8_t p = 0; p < pairs && p < MASK_COUNT; p++) {
      setPairColor(p, COL_BLUE);
    }
    strip.show();
  });

  ArduinoOTA.onError([](ota_error_t error) {
    otaInProgress = false;
    Serial.print("[OTA] ОШИБКА ");
    Serial.println(error);
    for (uint8_t p = 0; p < MASK_COUNT; p++) setPairColor(p, COL_RED);
    strip.show();
  });

  ArduinoOTA.begin();
  Serial.print("[OTA] готов, имя ");
  Serial.println(OTA_HOSTNAME);
}

// ==================== MQTT (тестовый хаб) ====================
void publishInfo(const char *text) {
  strncpy(infoMsg, text, sizeof(infoMsg) - 1);
  infoMsg[sizeof(infoMsg) - 1] = '\0';
  Serial.println(infoMsg);
  if (mqttClient.connected()) mqttClient.publish(MQTT_TOPIC_INFO, infoMsg, true);
}

void publishState(const char *text) {
  strncpy(stateMsg, text, sizeof(stateMsg) - 1);
  stateMsg[sizeof(stateMsg) - 1] = '\0';
  if (mqttClient.connected()) mqttClient.publish(MQTT_TOPIC_STATE, stateMsg, true);
}

// Команды от хаба: quest/chronomage/masks/cmd
void mqttCallback(char *topic, uint8_t *payload, unsigned int length) {
  char buf[32] = {0};
  unsigned int n = length < sizeof(buf) - 1 ? length : sizeof(buf) - 1;
  memcpy(buf, payload, n);

  Serial.print("[MQTT cmd] ");
  Serial.println(buf);

  if      (strcmp(buf, "CALIBRATE_START")   == 0) calibStart();
  else if (strcmp(buf, "CALIBRATE_SKIP")    == 0) calibSkip();
  else if (strcmp(buf, "CALIBRATE_ABORT")   == 0) calibAbort();
  else if (strcmp(buf, "START_MASKS_TEST")  == 0) gameStart();
  else if (strcmp(buf, "STOP_MASKS_TEST")   == 0) { gameStop(); chaseStop(); }
  else if (strncmp(buf, "SET_TIMEOUT:", 12) == 0) {
    long ms = atol(buf + 12);
    if (ms < MIN_MASK_TIMEOUT_MS) ms = MIN_MASK_TIMEOUT_MS;
    if (ms > MAX_MASK_TIMEOUT_MS) ms = MAX_MASK_TIMEOUT_MS;
    maskTimeoutMs = (unsigned long)ms;

    char msg[48];
    snprintf(msg, sizeof(msg), "таймаут маски: %lu мс", maskTimeoutMs);
    publishInfo(msg);
  }
}

// Неблокирующее (пере)подключение — вызывать каждый оборот loop().
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

// После (пере)подключения — info/state/buttons сразу (retain). Дальше:
// rssi раз в MQTT_PUBLISH_MS, buttons повторно раз в MQTT_BUTTONS_REPUBLISH_MS
// (на случай если хаб перезапустился и не увидел retain).
void mqttPublishPeriodic() {
  if (!mqttClient.connected()) return;

  if (mqttNeedInitialPublish) {
    mqttClient.publish(MQTT_TOPIC_INFO, infoMsg, true);
    mqttClient.publish(MQTT_TOPIC_STATE, stateMsg, true);
    mqttClient.publish(MQTT_TOPIC_BUTTONS, MQTT_BUTTONS_PAYLOAD, true);
    mqttClient.publish(MQTT_TOPIC_CANDLES_STATE, "READING", true);
    lastButtonsPublish = millis();
    mqttNeedInitialPublish = false;
  }

  if (millis() - lastMqttPublish >= MQTT_PUBLISH_MS) {
    lastMqttPublish = millis();
    mqttClient.publish(MQTT_TOPIC_RSSI, String(WiFi.RSSI()).c_str());
  }

  if (millis() - lastButtonsPublish >= MQTT_BUTTONS_REPUBLISH_MS) {
    lastButtonsPublish = millis();
    mqttClient.publish(MQTT_TOPIC_BUTTONS, MQTT_BUTTONS_PAYLOAD, true);
  }

  if (millis() - lastCandlePublish >= CANDLE_PUBLISH_MS) {
    lastCandlePublish = millis();
    publishCandles();
  }
}

// Общая точка входа для любого подтверждённого изменения кнопки —
// решает, идёт ли это в калибровку, явный тест (start_masks) или в
// дежурную цепочку "огонь" (по умолчанию). Отпускание вне калибровки/игры
// визуально ничего не значит — цвет теперь определяется maskVisualState.
void handleMaskTransition(uint8_t maskNum, int channel, const char *muxLabel, bool pressed) {
  if (calibMode) {
    if (pressed) calibHandlePress(maskNum, channel, muxLabel);
    return;
  }
  if (gameActive) {
    if (pressed) gameHandleTouch(maskNum);
    return;
  }
  if (pressed) chaseHandleTouch(maskNum);
}

// Сырые ADC-значения фоторезисторов свечей — отдельное "устройство"
// quest/chronomage/candles на хабе, понятная строка для индикации.
void publishCandles() {
  char buf[128];
  int off = 0;
  for (uint8_t i = 0; i < MUX2_CANDLE_COUNT; i++) {
    off += snprintf(buf + off, sizeof(buf) - off, "C%u=%d ", i + 1, candleRaw[i]);
    if (off >= (int)sizeof(buf)) break;
  }
  Serial.print("[СВЕЧИ] ");
  Serial.println(buf);
  if (mqttClient.connected()) mqttClient.publish(MQTT_TOPIC_CANDLES_INFO, buf, true);
}

// ==================== КАЛИБРОВКА МАСКА<->ПАРА ====================
void calibShowPair(int pair) {
  strip.clear();
  setPairColor(pair, COL_WHITE);
  strip.show();

  char buf[80];
  snprintf(buf, sizeof(buf), "калибровка: пара %d/%d, жду нажатия...", pair + 1, MASK_COUNT);
  publishInfo(buf);
}

void calibStart() {
  if (gameActive) {
    publishInfo("нельзя начать калибровку: идёт тест масок");
    return;
  }

  chaseReset();   // ленту забирает калибровка — цепочку гасим до её конца

  calibMode      = true;
  calibPairIndex = -1;
  calibWaiting   = false;
  calibConfirmUntil = 0;   // уже "в прошлом" -> calibLoop() сразу покажет пару 0

  for (uint8_t i = 0; i < MASK_COUNT; i++) calibAssignedPair[i] = 0xFF;

  publishState("CALIBRATING");
  publishInfo("калибровка начата");
  Serial.println("[КАЛИБРОВКА] старт");
}

void calibSkip() {
  if (!calibMode || !calibWaiting) return;

  char buf[48];
  snprintf(buf, sizeof(buf), "пара %d пропущена", calibPairIndex + 1);
  publishInfo(buf);

  calibWaiting = false;
  calibConfirmUntil = millis();   // без задержки -> calibLoop() сразу пойдёт дальше
}

void calibAbort() {
  if (!calibMode) return;
  calibMode = false;
  clearAllLeds();
  publishState("IDLE");
  publishInfo("калибровка отменена, таблица не менялась");
  Serial.println("[КАЛИБРОВКА] отменена");
}

void calibHandlePress(uint8_t maskNum, int channel, const char *muxLabel) {
  if (!calibWaiting) return;   // сейчас не ждём (идёт подтверждение/переход)

  calibAssignedPair[maskNum - 1] = (uint8_t)calibPairIndex;

  char buf[80];
  snprintf(buf, sizeof(buf), "пара %d -> маска %u (%s кан %d)",
           calibPairIndex + 1, maskNum, muxLabel, channel);
  publishInfo(buf);

  setPairColor(calibPairIndex, COL_GREEN);
  strip.show();

  calibWaiting = false;
  calibConfirmUntil = millis() + 500;   // держим зелёный 500мс, потом дальше
}

void calibFinish() {
  calibMode = false;
  clearAllLeds();

  Serial.println();
  Serial.println("---------- РЕЗУЛЬТАТ КАЛИБРОВКИ (скопировать в maskToLedPair[]) ----------");
  Serial.println("uint8_t maskToLedPair[MASK_COUNT] = {");
  char csv[96] = {0};
  for (uint8_t i = 0; i < MASK_COUNT; i++) {
    uint8_t pair = calibAssignedPair[i];
    char line[48];
    if (pair == 0xFF) {
      snprintf(line, sizeof(line), "  255,  // маска %u НЕ НАЙДЕНА", i + 1);
    } else {
      snprintf(line, sizeof(line), "  %u,  // маска %u", pair, i + 1);
    }
    Serial.println(line);

    char num[8];
    snprintf(num, sizeof(num), "%s%u", i == 0 ? "" : ",", pair == 0xFF ? 255 : pair);
    strncat(csv, num, sizeof(csv) - strlen(csv) - 1);
  }
  Serial.println("};");
  Serial.println("---------------------------------------------------------------------------");
  Serial.println();

  publishState(csv);   // компактный список пар по порядку маска1..19 — виден на хабе
  publishInfo("калибровка завершена, таблица в Serial (скопируй в maskToLedPair[])");
}

// Вызывается каждый loop(), пока calibMode == true. Неблокирующий шаг.
void calibLoop() {
  if (!calibMode) return;
  if (calibWaiting) return;                       // ждём нажатия — ничего не делаем
  if (millis() < calibConfirmUntil) return;        // ещё показываем подтверждение

  calibPairIndex++;
  if (calibPairIndex >= MASK_COUNT) {
    calibFinish();
    return;
  }

  calibWaiting = true;
  calibShowPair(calibPairIndex);
}

// ==================== ОГОНЬ: МЕРЦАНИЕ + СЦЕНАРИЙ КАСАНИЙ ====================
// Дежурное мерцание: маски в состоянии VIS_FLICKER дрожат красным 0..15
// каждые FLICKER_UPDATE_MS. Не трогает калибровку/игру и маски, занятые
// сценарием (цель/поймана).
void updateFlicker() {
  if (calibMode || gameActive) return;
  if (millis() - lastFlickerUpdate < FLICKER_UPDATE_MS) return;
  lastFlickerUpdate = millis();

  bool changed = false;
  for (uint8_t maskNum = 1; maskNum <= MASK_COUNT; maskNum++) {
    if (maskVisualState[maskNum - 1] != VIS_FLICKER) continue;
    setMaskColor(maskNum, random(FLICKER_MIN_RED, FLICKER_MAX_RED + 1), 0, 0);
    changed = true;
  }
  if (changed) strip.show();
}

// Пересобирает случайный порядок всех 19 масок. Если в этот момент шло
// прохождение — тихо сбрасываем его (сценарий сменился, старый прогресс
// по нему больше не валиден).
void scenarioGenerate() {
  lastScenarioGen = millis();

  for (uint8_t i = 0; i < MASK_COUNT; i++) scenarioOrder[i] = i + 1;
  for (uint8_t i = MASK_COUNT - 1; i > 0; i--) {   // Fisher-Yates
    uint8_t j = random(i + 1);
    uint8_t tmp = scenarioOrder[i];
    scenarioOrder[i] = scenarioOrder[j];
    scenarioOrder[j] = tmp;
  }

  Serial.print("[СЦЕНАРИЙ] новый порядок: ");
  for (uint8_t i = 0; i < MASK_COUNT; i++) {
    Serial.print(scenarioOrder[i]);
    Serial.print(i + 1 < MASK_COUNT ? "," : "\n");
  }

  chaseReset();   // новый сценарий -> сбросить и прогресс, и зелёный "победный" фон
}

// Вызывать каждый loop() — раз в SCENARIO_REGEN_MS пересобирает сценарий.
void scenarioCheck() {
  if (millis() - lastScenarioGen >= SCENARIO_REGEN_MS) scenarioGenerate();
}

// "Вооружает" прохождение: показывает синим первую маску текущего
// сценария. Само касание, которое привело сюда, не засчитывается —
// даже если случайно попали в ту же маску, её всё равно нужно тронуть
// ещё раз, пока она синяя.
void chaseArm() {
  chaseActive = true;
  chaseIndex  = 0;

  uint8_t target = scenarioOrder[chaseIndex];
  maskVisualState[target - 1] = VIS_TARGET;
  setMaskColor(target, COL_BLUE);
  strip.show();

  chaseTargetDeadline = millis() + SCENARIO_TOUCH_MS;

  char buf[96];
  snprintf(buf, sizeof(buf), "огонь: старт сценария, цель — маска %u (1/%d)", target, MASK_COUNT);
  publishInfo(buf);
  publishState("CHASE_ACTIVE");
}

// Любое касание вне калибровки/игры идёт сюда.
void chaseHandleTouch(uint8_t maskNum) {
  if (!chaseActive) {
    chaseArm();   // касание только запускает прохождение, не засчитывается
    return;
  }

  uint8_t target = scenarioOrder[chaseIndex];
  if (maskNum != target) return;   // не та цель — игнор, ждём таймаут или верную

  maskVisualState[target - 1] = VIS_DONE;
  setMaskColor(target, COL_WHITE);

  chaseIndex++;
  if (chaseIndex >= MASK_COUNT) {
    chaseFinish();
    return;
  }

  uint8_t nextTarget = scenarioOrder[chaseIndex];
  maskVisualState[nextTarget - 1] = VIS_TARGET;
  setMaskColor(nextTarget, COL_BLUE);
  strip.show();

  chaseTargetDeadline = millis() + SCENARIO_TOUCH_MS;

  char buf[96];
  snprintf(buf, sizeof(buf), "огонь: маска %u поймана, цель — маска %u (%d/%d)",
           target, nextTarget, chaseIndex + 1, MASK_COUNT);
  publishInfo(buf);
}

// Вызывать каждый loop(), пока chaseActive == true. 3 сек не тронули
// текущую синюю цель — полный сброс на позицию 0, все маски мерцают.
void chaseTimeoutCheck() {
  if (!chaseActive) return;
  if (millis() < chaseTargetDeadline) return;

  uint8_t target = scenarioOrder[chaseIndex];
  chaseReset();

  char buf[80];
  snprintf(buf, sizeof(buf), "огонь: не успели маску %u — сброс на позицию 0", target);
  publishInfo(buf);
  publishState("IDLE");
}

// Держит всю ленту зелёной (COL_GREEN на общей LED_BRIGHTNESS=40 -> видимо
// ~40/255 ≈ 15%) до следующей генерации сценария — VIS_CELEBRATE не трогает
// updateFlicker(), так что мерцание не перерисует поверх.
void chaseFinish() {
  chaseActive = false;
  chaseIndex  = -1;
  for (uint8_t i = 0; i < MASK_COUNT; i++) maskVisualState[i] = VIS_CELEBRATE;
  for (uint8_t p = 0; p < MASK_COUNT; p++) setPairColor(p, COL_GREEN);
  strip.show();
  publishState("MASKS_DONE");
  publishInfo("огонь: сценарий пройден полностью, держим зелёный до новой генерации");
}

// Тихий сброс без сообщений — вызывается перед тем как калибровка/игра
// забирают ленту себе, и при смене сценария посреди прохождения.
void chaseReset() {
  chaseActive = false;
  chaseIndex  = -1;
  for (uint8_t i = 0; i < MASK_COUNT; i++) maskVisualState[i] = VIS_FLICKER;
}

// Ручная остановка (STOP_MASKS_TEST) — с сообщением на хаб.
void chaseStop() {
  if (!chaseActive) return;
  chaseReset();
  publishState("IDLE");
  publishInfo("огонь: сценарий остановлен вручную, дежурный режим");
}

// ==================== ИГРА "МАСКИ" (тестовый прогон, кнопка start_masks) ====================
// Зажигает текущую активную маску красным, перезапускает таймер ожидания.
void gameShowActive() {
  uint8_t maskNum = gameOrder[gameIndex];

  strip.clear();
  setMaskColor(maskNum, COL_RED);
  strip.show();

  gameDeadline = millis() + maskTimeoutMs;

  char buf[96];
  snprintf(buf, sizeof(buf), "тест масок: прогресс %d/%d, активна маска %u",
           gameIndex, MASK_COUNT, maskNum);
  publishInfo(buf);
}

// Кнопка start_masks: свой случайный порядок, без реального cycle/start.
void gameStart() {
  if (calibMode) {
    publishInfo("нельзя стартовать тест масок: идёт калибровка");
    return;
  }

  chaseReset();   // ленту забирает тест — цепочку гасим до его конца

  for (uint8_t i = 0; i < MASK_COUNT; i++) gameOrder[i] = i + 1;
  for (uint8_t i = MASK_COUNT - 1; i > 0; i--) {   // Fisher-Yates
    uint8_t j = random(i + 1);
    uint8_t tmp = gameOrder[i];
    gameOrder[i] = gameOrder[j];
    gameOrder[j] = tmp;
  }

  Serial.print("[ТЕСТ МАСОК] порядок: ");
  for (uint8_t i = 0; i < MASK_COUNT; i++) {
    Serial.print(gameOrder[i]);
    Serial.print(i + 1 < MASK_COUNT ? "," : "\n");
  }

  gameActive = true;
  gameIndex  = 0;
  publishState("GAME_RUNNING");
  gameShowActive();
}

void gameStop() {
  if (!gameActive) return;
  gameActive = false;
  clearAllLeds();
  publishState("IDLE");
  publishInfo("тест масок остановлен вручную");
}

void gameHandleTouch(uint8_t maskNum) {
  if (!gameActive) return;
  if (maskNum != gameOrder[gameIndex]) return;   // не та маска — просто игнор

  setMaskColor(maskNum, COL_WHITE);
  strip.show();

  gameIndex++;
  if (gameIndex >= MASK_COUNT) {
    gameFinish();
    return;
  }
  gameShowActive();
}

void gameFinish() {
  gameActive = false;
  for (uint8_t p = 0; p < MASK_COUNT; p++) setPairColor(p, COL_GREEN);
  strip.show();
  publishState("MASKS_DONE");
  publishInfo("тест масок пройден: все 19 по порядку");
}

// Вызывается каждый loop(), пока gameActive == true. Неблокирующий шаг.
void gameLoop() {
  if (!gameActive) return;
  if (millis() < gameDeadline) return;

  char buf[112];
  snprintf(buf, sizeof(buf), "не успели маску %u (таймаут %lu мс) — сброс на начало",
           gameOrder[gameIndex], maskTimeoutMs);
  publishInfo(buf);

  gameIndex = 0;
  gameShowActive();
}

// ==================== ОПРОС MUX1 (маски 1-16) ====================
void scanMux1() {
  for (uint8_t ch = 0; ch < MUX1_CHANNELS; ch++) {
    bool val = readMux1(ch);

    if (val == mux1State[ch]) {
      mux1Counter[ch] = 0;
      continue;
    }

    if (val == mux1Candidate[ch]) {
      mux1Counter[ch]++;
      if (mux1Counter[ch] >= DEBOUNCE_COUNT) {
        mux1State[ch] = val;
        mux1Counter[ch] = 0;

        uint8_t maskNum = ch + 1;
        handleMaskTransition(maskNum, ch, "MUX1", val);
      }
    } else {
      mux1Candidate[ch] = val;
      mux1Counter[ch] = 1;
    }
  }
}

// ==================== ОПРОС MUX2 (маски 17-19 + свечи) ====================
void scanMux2() {
  // маски 17-19 — аналоговое чтение с порогом
  for (uint8_t i = 0; i < MUX2_MASK_COUNT; i++) {
    uint8_t ch = MUX2_MASK_START + i;
    int raw = readMux2(ch);
    bool val = (raw > MASK_THRESHOLD);

    if (val == mux2MaskState[i]) {
      mux2MaskCounter[i] = 0;
      continue;
    }

    if (val == mux2MaskCandidate[i]) {
      mux2MaskCounter[i]++;
      if (mux2MaskCounter[i] >= DEBOUNCE_COUNT) {
        mux2MaskState[i] = val;
        mux2MaskCounter[i] = 0;

        uint8_t maskNum = 17 + i;
        handleMaskTransition(maskNum, ch, "MUX2", val);
      }
    } else {
      mux2MaskCandidate[i] = val;
      mux2MaskCounter[i] = 1;
    }
  }

  // Свечи — просто сохраняем сырые значения, логика сравнения с паттерном будет позже.
  for (uint8_t i = 0; i < MUX2_CANDLE_COUNT; i++) {
    candleRaw[i] = readMux2(candleToChannel[i]);
  }
}

// ==================== ДАМП ВСЕХ КАНАЛОВ (один раз при старте) ====================
void dumpAll() {
  Serial.println();
  Serial.println("---------- ДАМП ----------");

  Serial.println("MUX1 (цифра), маски 1-16:");
  for (uint8_t ch = 0; ch < MUX1_CHANNELS; ch++) {
    bool val = readMux1(ch);
    Serial.printf("  кан %2d / маска %2d / пара LED %2d : %s\n",
                  ch, ch + 1, maskToLedPair[ch], val ? "1 НАЖАТА" : "0");
  }

  Serial.println("MUX2 (аналог), маски 17-19:");
  for (uint8_t i = 0; i < MUX2_MASK_COUNT; i++) {
    uint8_t ch = MUX2_MASK_START + i;
    int raw = readMux2(ch);
    Serial.printf("  кан %d / маска %d / пара LED %d : raw=%d порог=%d -> %s\n",
                  ch, 17 + i, maskToLedPair[16 + i], raw, MASK_THRESHOLD,
                  raw > MASK_THRESHOLD ? "НАЖАТА" : "отпущена");
  }

  Serial.println("MUX2 (аналог), свечи 1-8:");
  for (uint8_t i = 0; i < MUX2_CANDLE_COUNT; i++) {
    uint8_t ch = candleToChannel[i];
    int raw = readMux2(ch);
    Serial.printf("  кан %2d / свеча %d : raw=%d\n", ch, i + 1, raw);
  }

  Serial.println("--------------------------");
  Serial.println();
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);
  pinMode(MUX1_SIG, INPUT);

  analogReadResolution(12);
  randomSeed(esp_random());

  strip.begin();
  strip.setBrightness(LED_BRIGHTNESS);
  strip.clear();
  strip.show();

  mqttNet.setTimeout(500);   // иначе connect() к недоступному хосту блокирует опрос на секунды
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(384);   // список кнопок вплотную к дефолтным 256 байт

  for (uint8_t i = 0; i < MUX1_CHANNELS; i++) {
    mux1State[i] = false;
    mux1Candidate[i] = false;
    mux1Counter[i] = 0;
  }
  for (uint8_t i = 0; i < MUX2_MASK_COUNT; i++) {
    mux2MaskState[i] = false;
    mux2MaskCandidate[i] = false;
    mux2MaskCounter[i] = 0;
  }
  for (uint8_t i = 0; i < MASK_COUNT; i++) maskVisualState[i] = VIS_FLICKER;
  scenarioGenerate();   // нужен валидный сценарий сразу с загрузки

  Serial.println();
  Serial.println("========================================");
  Serial.println("  ТЕСТ МУЛЬТИПЛЕКСОРОВ + WS2812 + OTA");
  Serial.println("========================================");
  Serial.println("S0=5 S1=6 S2=7 S3=8");
  Serial.println("MUX1_SIG=3 (цифра)  MUX2_SIG=4 (аналог)");
  Serial.println("WS2812=1, диодов 38, пар 19");

  wifiConnect();

  // Ждём подключения максимум 10 сек, дальше работаем и реконнектим в фоне
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
    delay(200);
    Serial.print(".");
    strip.clear();
    setPairColor(((millis() - t0) / 200) % MASK_COUNT, COL_BLUE);
    strip.show();
  }
  Serial.println();
  strip.clear();
  strip.show();

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

  // Прогон ленты — проверка, что все диоды живы
  Serial.println("[СТАРТ] прогон ленты...");
  for (uint8_t p = 0; p < MASK_COUNT; p++) {
    strip.clear();
    setPairColor(p, COL_RED);
    strip.show();
    delay(80);
  }
  strip.clear();
  strip.show();   // дальше дежурное мерцание подхватит сама updateFlicker() в loop()

  dumpAll();
  Serial.println("[СТАРТ] готово, опрос запущен");
  Serial.println();
}

// ==================== ОСНОВНОЙ ЦИКЛ ====================
void loop() {
  ArduinoOTA.handle();
  if (otaInProgress) return;   // во время заливки не мешаем

  unsigned long cycleStart = micros();

  scanMux1();
  scanMux2();
  calibLoop();
  gameLoop();
  updateFlicker();
  scenarioCheck();
  chaseTimeoutCheck();

  cycleTimeSum += micros() - cycleStart;
  cycleTimeCount++;
  loopCount++;

  wifiCheck();
  mqttCheck();
  mqttPublishPeriodic();

  if (millis() - lastStatsTime >= STATS_PRINT_MS) {
    lastStatsTime = millis();
    Serial.print("[СТАТ] оборотов: ");
    Serial.print(loopCount);
    Serial.print("  средний цикл: ");
    Serial.print(cycleTimeCount ? cycleTimeSum / cycleTimeCount : 0);
    Serial.print(" мкс  RSSI ");
    Serial.print(WiFi.RSSI());
    Serial.print(" dBm  TX ");
    Serial.print(WiFi.getTxPower());
    Serial.println();
    cycleTimeSum = 0;
    cycleTimeCount = 0;
  }
}
