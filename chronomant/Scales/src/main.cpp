/*
 * ВЕСЫ (HX711) — хрономаг, задание «Весы + лампочки»
 * ESP32-C3 SuperMini  /  PlatformIO (framework = arduino)
 *
 * HX711: DT=GPIO4, SCK=GPIO5. Если весы стабильно читают одно и то же
 * (не реагируют на груз) — вероятно, DT/SCK перепутаны местами, поменять.
 *
 * Задача весов — не измерить массу, а понять СКОЛЬКО "кристаллов"
 * (одинаковых по весу предметов) сейчас лежит: 0, 1, 2 ... N. Граммы
 * не важны, важен только счёт штук.
 *
 * MQTT (TestHub на стенде, chrono-hub.local):
 *   quest/chronomage/scales/info    — человекочитаемый статус (сколько штук)
 *   quest/chronomage/scales/state   — короткое значение (целое число штук)
 *   quest/chronomage/scales/rssi    — уровень сигнала
 *   quest/chronomage/scales/buttons — кнопки хаба
 *   quest/chronomage/scales/cmd     — команды:
 *       TARE            — обнулить (весы должны быть пустые)
 *       CALIBRATE:<N>    — на весах лежит ровно N кристаллов,
 *                          посчитать и применить "вес одного кристалла"
 *       FORCE_DONE      — тестовая кнопка: объявить задание пройденным
 *                          без реальной проверки (пока нет паттерна от
 *                          квест-мастера)
 *   quest/chronomage/scales/done    — "OK" при FORCE_DONE (пока только тест)
 *
 * Калибровка (TARE и CALIBRATE) сразу пишется в NVS (флеш) — переживает
 * перезагрузку сама, перепрошивать не нужно.
 *
 * Порядок калибровки на месте (кнопками на хабе или вручную через cmd):
 *   1. Убрать всё с весов -> кнопка tare
 *   2. Положить ИЗВЕСТНОЕ количество кристаллов (лучше не 1, а несколько
 *      сразу — точнее посчитается вес одного за счёт усреднения)
 *   3. Кнопка calibrate_5 (для 5 шт) или в свободное текстовое поле ввести
 *      CALIBRATE:<N> для другого количества
 *
 * Прошивка по воздуху: PlatformIO -> upload_port = chrono-scales.local
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <HX711.h>
#include <math.h>
#include <Preferences.h>

// ============================================================
// КОНФИГУРАЦИЯ
// ============================================================

// ---- Сеть ----
#define WIFI_SSID     "weasgley"
#define WIFI_PASS     "weasgley123"

#define OTA_HOSTNAME  "chrono-scales"
#define OTA_PASSWORD  "chrono"

#define WIFI_RETRY_MS 5000

// ---- MQTT (тестовый хаб TestHub, стенд) ----
#define MQTT_HOST         "chrono-hub.local"
#define MQTT_PORT         1883
#define MQTT_CLIENT_ID    "chrono-scales"
#define MQTT_DEVICE_TOPIC "quest/chronomage/scales"
#define MQTT_RECONNECT_MS 4000
#define MQTT_PUBLISH_MS   500

#define MQTT_TOPIC_INFO    MQTT_DEVICE_TOPIC "/info"
#define MQTT_TOPIC_STATE   MQTT_DEVICE_TOPIC "/state"
#define MQTT_TOPIC_RSSI    MQTT_DEVICE_TOPIC "/rssi"
#define MQTT_TOPIC_BUTTONS MQTT_DEVICE_TOPIC "/buttons"
#define MQTT_TOPIC_CMD     MQTT_DEVICE_TOPIC "/cmd"
#define MQTT_TOPIC_DONE    MQTT_DEVICE_TOPIC "/done"

// tare — нулевая точка (весы должны быть пустые прямо сейчас).
// calibrate_5 — вторая точка: на весах должно лежать ровно 5 кристаллов.
// force_done — тестовая кнопка: принудительно объявить задание пройденным,
// не дожидаясь реальной логики сравнения с паттерном.
// tare/calibrate_5 сразу пишутся во флеш (NVS) — переживают перезагрузку.
#define MQTT_BUTTONS_PAYLOAD \
  "tare:" MQTT_TOPIC_CMD ":TARE|" \
  "calibrate_5:" MQTT_TOPIC_CMD ":CALIBRATE:5|" \
  "force_done:" MQTT_TOPIC_CMD ":FORCE_DONE"

// ---- HX711 ----
#define HX711_DT_PIN  4
#define HX711_SCK_PIN 5

// Результат калибровки — после калибровки вписать сюда посчитанные
// значения и перезалить, чтобы весы были готовы сразу после старта.
#define DEFAULT_TARE_OFFSET  0
#define DEFAULT_SCALE_FACTOR 1.0f   // (сырое - offset) / один кристалл

// Ниже этой дробной величины (в штуках) считаем, что на весах пусто —
// шум АЦП иначе может округлиться до "1 кристалл".
#define COUNT_DEADBAND 0.3f

// ==================== ОБЪЕКТЫ ====================
HX711 scale;
Preferences prefs;   // NVS: калибровка переживает перезагрузку

WiFiClient   mqttNet;
PubSubClient mqttClient(mqttNet);

// ==================== СОСТОЯНИЕ ====================
long  tareOffset  = DEFAULT_TARE_OFFSET;
float scaleFactor = DEFAULT_SCALE_FACTOR;

long  lastRaw       = 0;
float lastCountRaw  = 0;   // дробное количество штук до округления
int   lastCount     = 0;   // округлённое и зажатое к 0..N количество
bool  hasReading    = false;

bool otaInProgress    = false;
bool wifiWasConnected  = false;
bool otaStarted        = false;
unsigned long lastWifiTry = 0;

unsigned long lastMqttReconnect  = 0;
unsigned long lastMqttPublish    = 0;
unsigned long lastButtonsPublish = 0;
bool mqttNeedInitialPublish = false;

#define MQTT_BUTTONS_REPUBLISH_MS (3UL * 60UL * 1000UL)

char infoMsg[96]  = "весы HX711, ESP32-C3, не откалиброваны";
char stateMsg[32] = "0";

// ==================== ПРОТОТИПЫ ====================
void wifiConnect();
void wifiCheck();
void otaSetup();
void mqttCheck();
void mqttPublishPeriodic();
void mqttCallback(char *topic, uint8_t *payload, unsigned int length);
void publishInfo(const char *text);
void publishState(const char *text);
void readScale();
void cmdTare();
void cmdCalibrate(long knownCount);
void cmdForceDone();

// ==================== WIFI ====================
void wifiConnect() {
  WiFi.mode(WIFI_STA);
  // Та же проблема регулятора C3 SuperMini, что и у масок — пониженная TX power.
  WiFi.setTxPower(WIFI_POWER_15dBm);
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
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("[OTA] завершено, перезагрузка");
  });
  ArduinoOTA.onError([](ota_error_t error) {
    otaInProgress = false;
    Serial.print("[OTA] ОШИБКА ");
    Serial.println(error);
  });

  ArduinoOTA.begin();
  Serial.print("[OTA] готов, имя ");
  Serial.println(OTA_HOSTNAME);
}

// ==================== MQTT ====================
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

// Команды от хаба: quest/chronomage/scales/cmd
void mqttCallback(char *topic, uint8_t *payload, unsigned int length) {
  char buf[32] = {0};
  unsigned int n = length < sizeof(buf) - 1 ? length : sizeof(buf) - 1;
  memcpy(buf, payload, n);

  Serial.print("[MQTT cmd] ");
  Serial.println(buf);

  if (strcmp(buf, "TARE") == 0) {
    cmdTare();
  } else if (strncmp(buf, "CALIBRATE:", 10) == 0) {
    long count = atol(buf + 10);
    cmdCalibrate(count);
  } else if (strcmp(buf, "FORCE_DONE") == 0) {
    cmdForceDone();
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
    mqttClient.publish(MQTT_TOPIC_STATE, stateMsg, true);
    mqttClient.publish(MQTT_TOPIC_BUTTONS, MQTT_BUTTONS_PAYLOAD, true);
    lastButtonsPublish = millis();
    mqttNeedInitialPublish = false;
  }

  if (millis() - lastButtonsPublish >= MQTT_BUTTONS_REPUBLISH_MS) {
    lastButtonsPublish = millis();
    mqttClient.publish(MQTT_TOPIC_BUTTONS, MQTT_BUTTONS_PAYLOAD, true);
  }

  if (millis() - lastMqttPublish >= MQTT_PUBLISH_MS) {
    lastMqttPublish = millis();

    mqttClient.publish(MQTT_TOPIC_RSSI, String(WiFi.RSSI()).c_str());

    if (hasReading) {
      // Понятная строка для хаба — просто "сколько штук", без сырых ADC.
      // Знак у "точно" убираем: он определяется только полярностью
      // проводов датчика и смысловой нагрузки не несёт.
      char info[48];
      snprintf(info, sizeof(info), "на весах: %d шт (точно %.2f)", lastCount, fabsf(lastCountRaw));
      publishInfo(info);

      // Сырые значения — только в Serial, для калибровки/отладки.
      Serial.printf("[ВЕСЫ] сырое=%ld offset=%ld factor=%.4f\n", lastRaw, tareOffset, scaleFactor);

      char state[8];
      snprintf(state, sizeof(state), "%d", lastCount);
      publishState(state);
    }
  }
}

// ==================== HX711 ====================
// Неблокирующее чтение: is_ready() только проверяет пин, read() зовём
// лишь когда данные точно готовы — никаких ожиданий в основном цикле.
void readScale() {
  if (!scale.is_ready()) return;

  lastRaw      = scale.read();
  lastCountRaw = (float)(lastRaw - tareOffset) / scaleFactor;

  // Принимаем новое целое количество, только если сырое значение близко
  // к нему (в пределах COUNT_DEADBAND) — иначе (кто-то кладёт/снимает
  // кристалл прямо сейчас) оставляем прошлое стабильное значение.
  float rounded = roundf(lastCountRaw);
  if (fabsf(lastCountRaw - rounded) < COUNT_DEADBAND) {
    lastCount = (rounded < 0.0f) ? 0 : (int)rounded;
  }

  hasReading = true;
}

// Обнуление — весы должны быть пустые. Несколько выборок подряд для
// устойчивости; это разовая команда по запросу, а не часть опроса в
// loop(), поэтому короткая блокировка (десятки мс) здесь допустима.
void cmdTare() {
  tareOffset = scale.read_average(10);
  prefs.putLong("offset", tareOffset);

  char buf[80];
  snprintf(buf, sizeof(buf), "тарирование: offset=%ld (сохранено во флеш)", tareOffset);
  publishInfo(buf);
}

// На весах лежит ровно knownCount кристаллов — считаем вес одного.
void cmdCalibrate(long knownCount) {
  if (knownCount <= 0) {
    publishInfo("калибровка: количество должно быть больше 0 (CALIBRATE:<штук>)");
    return;
  }

  long raw = scale.read_average(10);
  scaleFactor = (float)(raw - tareOffset) / (float)knownCount;
  prefs.putFloat("factor", scaleFactor);

  char buf[128];
  snprintf(buf, sizeof(buf),
           "калибровка готова: factor=%.4f (offset=%ld, положено=%ld шт), сохранено во флеш",
           scaleFactor, tareOffset, knownCount);
  publishInfo(buf);

  Serial.println("---------- КАЛИБРОВКА ВЕСОВ (уже сохранено в NVS) ----------");
  Serial.printf("#define DEFAULT_TARE_OFFSET  %ld\n", tareOffset);
  Serial.printf("#define DEFAULT_SCALE_FACTOR %.4ff\n", scaleFactor);
  Serial.println("---------------------------------------");
}

// Тестовая кнопка — объявить задание пройденным без реальной проверки
// (пока не подключена настоящая логика "квест-мастер прислал паттерн").
void cmdForceDone() {
  publishState("DONE");
  publishInfo("принудительно отмечено как пройдено (тест)");
  if (mqttClient.connected()) mqttClient.publish(MQTT_TOPIC_DONE, "OK");
  Serial.println("[FORCE_DONE] отправлено");
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println();
  Serial.println("========================================");
  Serial.println("  ВЕСЫ HX711 + WiFi/OTA/MQTT");
  Serial.println("========================================");
  Serial.printf("DT=%d SCK=%d\n", HX711_DT_PIN, HX711_SCK_PIN);

  scale.begin(HX711_DT_PIN, HX711_SCK_PIN);

  prefs.begin("scale", false);
  tareOffset  = prefs.getLong("offset", DEFAULT_TARE_OFFSET);
  scaleFactor = prefs.getFloat("factor", DEFAULT_SCALE_FACTOR);
  Serial.printf("[NVS] загружено: offset=%ld factor=%.4f\n", tareOffset, scaleFactor);

  mqttNet.setTimeout(500);   // иначе connect() к недоступному хосту блокирует опрос
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  wifiConnect();

  // Ждём подключения максимум 10 сек, дальше работаем и реконнектим в фоне
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
    delay(200);
    Serial.print(".");
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
    Serial.println("[WIFI] не подключилось, работаем локально, реконнект в фоне");
  }

  Serial.println("[СТАРТ] готово, опрос весов запущен");
  Serial.println();
}

// ==================== ОСНОВНОЙ ЦИКЛ ====================
void loop() {
  ArduinoOTA.handle();
  if (otaInProgress) return;   // во время заливки не мешаем

  readScale();

  wifiCheck();
  mqttCheck();
  mqttPublishPeriodic();
}
