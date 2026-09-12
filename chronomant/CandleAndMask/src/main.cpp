/*
 * ТЕСТ ЖЕЛЕЗА: МУЛЬТИПЛЕКСОРЫ + WS2812 + WiFi/OTA
 * ESP32-C3 SuperMini  /  PlatformIO (framework = arduino)
 *
 * MUX1 (GPIO3) - цифра, 16 каналов, маски 1-16
 * MUX2 (GPIO4) - аналог, кан 0-2 маски 17-19, кан 3-10 свечи 1-8
 * WS2812 (GPIO1) - 38 диодов, по 2 на маску
 *
 * Что делает:
 *   - постоянно опрашивает все входы, печатает события нажатий масок
 *   - раз в секунду печатает сырые значения фоторезисторов свечей
 *   - глаза нажатой маски загораются красным (проверка ленты)
 *   - поднимает WiFi (статический IP) и OTA
 *
 * Команд по UART НЕТ. MQTT будет добавлен позже.
 * Прошивка по воздуху: PlatformIO -> upload_port = chrono-masks.local
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <Adafruit_NeoPixel.h>

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
#define CANDLE_PRINT_MS 1000
#define STATS_PRINT_MS  10000
#define LED_BRIGHTNESS    40

// Бегущая строка при нажатии любой маски: каждая пара горит SHOW_STEP_MS,
// затем гаснет и загорается следующая. Всего проход = 19 * SHOW_STEP_MS.
#define SHOW_STEP_MS     1000

// ============================================================
// ТАБЛИЦА: МАСКА -> ПАРА СВЕТОДИОДОВ
// Индекс = номер маски минус 1. Значение = номер пары (0..18),
// пара N = диоды N*2 и N*2+1. Единственное место правки при монтаже.
// ============================================================
uint8_t maskToLedPair[MASK_COUNT] = {
   0,  1,  2,  3,  4,  5,  6,  7,  8,  9,
  10, 11, 12, 13, 14, 15,
  16,   // маска 17 (MUX2 кан 0)
  17,   // маска 18 (MUX2 кан 1)
  18    // маска 19 (MUX2 кан 2)
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
#define COL_SHOW    COL_WHITE   // цвет бегущей строки

// ==================== ОБЪЕКТЫ ====================
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// ==================== СОСТОЯНИЕ ====================
bool    mux1State[MUX1_CHANNELS];
bool    mux1Candidate[MUX1_CHANNELS];
uint8_t mux1Counter[MUX1_CHANNELS];

bool    mux2MaskState[MUX2_MASK_COUNT];
bool    mux2MaskCandidate[MUX2_MASK_COUNT];
uint8_t mux2MaskCounter[MUX2_MASK_COUNT];

int     candleRaw[MUX2_CANDLE_COUNT];

unsigned long lastCandlePrint = 0;
unsigned long lastStatsTime   = 0;
unsigned long lastWifiTry     = 0;
unsigned long loopCount       = 0;
unsigned long cycleTimeSum    = 0;
unsigned long cycleTimeCount  = 0;

bool otaInProgress   = false;
bool wifiWasConnected = false;
bool otaStarted       = false;

// Бегущая строка по нажатию
bool          showRunning = false;
uint8_t       showPair    = 0;
unsigned long showTime    = 0;

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
void scanMux1();
void scanMux2();
void startShow();
void handleShow();
void printCandles();
void dumpAll();

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

// ==================== БЕГУЩАЯ СТРОКА ПО НАЖАТИЮ ====================
// Запуск (или перезапуск, если уже идёт) с первой пары
void startShow() {
  showRunning = true;
  showPair = 0;
  showTime = 0;   // 0 -> первый шаг сработает сразу в handleShow()
}

// Неблокирующий шаг: раз в SHOW_STEP_MS двигаем горящую пару дальше
void handleShow() {
  if (!showRunning) return;
  if (showTime != 0 && millis() - showTime < SHOW_STEP_MS) return;

  showTime = millis();

  if (showPair >= MASK_COUNT) {   // дошли до конца — гасим и выходим
    showRunning = false;
    strip.clear();
    strip.show();
    return;
  }

  strip.clear();
  setPairColor(showPair, COL_SHOW);
  strip.show();
  showPair++;
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

        Serial.print("[МАСКА ");
        Serial.print(maskNum);
        Serial.print("] MUX1 кан ");
        Serial.print(ch);
        Serial.print(" -> ");
        Serial.println(val ? "НАЖАТА" : "отпущена");

        if (val) startShow();   // любое нажатие запускает бегущую строку
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

        Serial.print("[МАСКА ");
        Serial.print(maskNum);
        Serial.print("] MUX2 кан ");
        Serial.print(ch);
        Serial.print(" raw=");
        Serial.print(raw);
        Serial.print(" -> ");
        Serial.println(val ? "НАЖАТА" : "отпущена");

        if (val) startShow();   // любое нажатие запускает бегущую строку
      }
    } else {
      mux2MaskCandidate[i] = val;
      mux2MaskCounter[i] = 1;
    }
  }

  // свечи — просто сохраняем сырые значения
  for (uint8_t i = 0; i < MUX2_CANDLE_COUNT; i++) {
    candleRaw[i] = readMux2(candleToChannel[i]);
  }
}

// ==================== ВЫВОД СВЕЧЕЙ ====================
void printCandles() {
  Serial.print("[СВЕЧИ] ");
  for (uint8_t i = 0; i < MUX2_CANDLE_COUNT; i++) {
    Serial.printf("C%d:%4d   ", i + 1, candleRaw[i]);
  }
  Serial.println();
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

  strip.begin();
  strip.setBrightness(LED_BRIGHTNESS);
  strip.clear();
  strip.show();

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
  for (uint8_t i = 0; i < MUX2_CANDLE_COUNT; i++) {
    candleRaw[i] = 0;
  }

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
  strip.show();

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
  handleShow();

  cycleTimeSum += micros() - cycleStart;
  cycleTimeCount++;
  loopCount++;

  wifiCheck();

  if (millis() - lastCandlePrint >= CANDLE_PRINT_MS) {
    lastCandlePrint = millis();
    printCandles();
  }

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
