/*
 * ПЛАСТИНЫ ПЕЛЬТЬЕ — тестовый стенд, 1 пластина
 * ESP32-C3 / PlatformIO (framework = arduino)
 *
 * Отдельная плата без сети — только для проверки на живом железе, прежде
 * чем собирать полную логику на 8 пластин (та полная версия в итоге
 * переедет в прошивку LightAndCrystals, на общую C3 со светом+кристаллами).
 *
 * Что делает этот скетч:
 *   - при старте проверяет, что PCA9685 реально отвечает по I2C
 *   - одна пластина (RPWM/LPWM = каналы 0/1 на PCA9685) стартует с HOT:
 *     кик HOT_KICK_PCT на HOT_KICK_MS, потом HOT_HOLD_PCT на HOT_HOLD_MS
 *   - break-before-make переход в COLD: кик COLD_KICK_PCT на COLD_KICK_MS,
 *     средняя ступень COLD_MID_PCT на COLD_MID_MS, потом COLD_HOLD_PCT
 *     на COLD_HOLD_MS — и по кругу
 *
 * Break-before-make: перед сменой полярности оба канала обнуляются и
 * выдерживается пауза, только потом плавно поднимается целевой канал —
 * так RPWM и LPWM никогда не активны одновременно (иначе H-мост горит).
 *
 * Все ступени duty — целые проценты (0-100), без float, чтобы было
 * удобно крутить константы в конфиге не думая о синтаксисе.
 *
 * ЗАГЛУШКИ, ТРЕБУЮЩИЕ ПРОВЕРКИ НА ЖЕЛЕЗЕ:
 *   - I2C_SDA/I2C_SCL — пины ещё не подтверждены на конкретной плате,
 *     выбраны из безопасного диапазона для C3 SuperMini (не strapping,
 *     не USB, не антенна).
 *   - PWM_FREQ_HZ и все *_PCT/*_MS ступени — калибровать по факту
 *     нагрева/охлаждения на реальной пластине.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ============================================================
// КОНФИГУРАЦИЯ
// ============================================================

// I2C — пины ещё не разведены физически на этой тестовой плате.
#define I2C_SDA  4
#define I2C_SCL  5

#define PCA9685_ADDR 0x40

// PCA9685 работает на внутреннем осцилляторе ~25MHz — этого достаточно
// для низкочастотного ШИМ на H-мост.
#define PWM_FREQ_HZ 150

// Тестовая пластина №1 — 2 канала PCA9685 на один H-мост (BTS7960).
#define PLATE1_RPWM_CH 0   // нагрев
#define PLATE1_LPWM_CH 1   // охлаждение

// ---- Профиль НАГРЕВА (HOT) — целые числа, менять смело ----
#define HOT_KICK_PCT        65   // первая ступень, %
#define HOT_KICK_MS       3000UL // сколько держать первую ступень
#define HOT_HOLD_PCT         10  // дальше держим до разворота, %
#define HOT_HOLD_MS      10000UL

// ---- Профиль ОХЛАЖДЕНИЯ (COLD) — целые числа, менять смело ----
#define COLD_KICK_PCT       85   // первая ступень, %
#define COLD_KICK_MS      3000UL
#define COLD_MID_PCT         65  // вторая ступень, %
#define COLD_MID_MS      20000UL
#define COLD_HOLD_PCT         55  // дальше держим до разворота, %
#define COLD_HOLD_MS     20000UL

// Break-before-make: обнулить оба канала, выдержать паузу, потом поднимать цель.
#define BREAK_BEFORE_MAKE_MS  75UL

// ============================================================
// ОБЪЕКТЫ / СОСТОЯНИЕ
// ============================================================
Adafruit_PWMServoDriver pwm(PCA9685_ADDR);

enum PlateMode { PLATE_OFF, PLATE_HOT, PLATE_COLD };
PlateMode currentMode = PLATE_OFF;

// ============================================================
// PCA9685: duty в процентах (целое 0..100) -> 12-битное значение (0..4095)
// ============================================================
void applyDuty(uint8_t channel, uint8_t dutyPct) {
  if (dutyPct > 100) dutyPct = 100;
  uint16_t value = (uint16_t)((uint32_t)dutyPct * 4095UL / 100UL);
  pwm.setPWM(channel, 0, value);
}

// ============================================================
// Проверка присутствия PCA9685 на шине I2C
// ============================================================
bool pca9685Present() {
  Wire.beginTransmission(PCA9685_ADDR);
  return Wire.endTransmission() == 0;
}

// Обнулить ОБА канала перед любой сменой режима — RPWM и LPWM никогда не
// активны одновременно (иначе H-мост горит).
void breakBeforeMake() {
  applyDuty(PLATE1_RPWM_CH, 0);
  applyDuty(PLATE1_LPWM_CH, 0);
  delay(BREAK_BEFORE_MAKE_MS);
}

void logStage(const char *label, uint8_t pct) {
  Serial.print("[PLATE] ");
  Serial.print(label);
  Serial.print(" ");
  Serial.print(pct);
  Serial.println("%");
}

// ============================================================
// НАГРЕВ: кик HOT_KICK_PCT на HOT_KICK_MS, потом HOT_HOLD_PCT на HOT_HOLD_MS.
// ============================================================
void setPlateHot() {
  Serial.println("[PLATE] переход -> HOT");
  breakBeforeMake();
  currentMode = PLATE_HOT;

  logStage("кик", HOT_KICK_PCT);
  applyDuty(PLATE1_RPWM_CH, HOT_KICK_PCT);
  delay(HOT_KICK_MS);

  logStage("держим", HOT_HOLD_PCT);
  applyDuty(PLATE1_RPWM_CH, HOT_HOLD_PCT);
  delay(HOT_HOLD_MS);
}

// ============================================================
// ОХЛАЖДЕНИЕ: кик COLD_KICK_PCT на COLD_KICK_MS, средняя ступень
// COLD_MID_PCT на COLD_MID_MS, потом COLD_HOLD_PCT на COLD_HOLD_MS.
// ============================================================
void setPlateCold() {
  Serial.println("[PLATE] переход -> COLD");
  breakBeforeMake();
  currentMode = PLATE_COLD;

  logStage("кик", COLD_KICK_PCT);
  applyDuty(PLATE1_LPWM_CH, COLD_KICK_PCT);
  delay(COLD_KICK_MS);

  logStage("средняя ступень", COLD_MID_PCT);
  applyDuty(PLATE1_LPWM_CH, COLD_MID_PCT);
  delay(COLD_MID_MS);

  logStage("держим", COLD_HOLD_PCT);
  applyDuty(PLATE1_LPWM_CH, COLD_HOLD_PCT);
  delay(COLD_HOLD_MS);
}

// ============================================================
// SETUP / LOOP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println();
  Serial.println("========================================");
  Serial.println("  ТЕСТ ПЛАСТИНЫ ПЕЛЬТЬЕ (1 шт)");
  Serial.println("========================================");

  Wire.begin(I2C_SDA, I2C_SCL);

  Serial.print("[I2C] проверка PCA9685 по адресу 0x");
  Serial.print(PCA9685_ADDR, HEX);
  Serial.print(" ... ");

  while (!pca9685Present()) {
    Serial.println("НЕ ОТВЕЧАЕТ, проверь проводку SDA/SCL/питание");
    delay(1000);
    Serial.print("[I2C] повтор... ");
  }
  Serial.println("ок, модуль на связи");

  pwm.begin();
  pwm.setPWMFreq(PWM_FREQ_HZ);
  applyDuty(PLATE1_RPWM_CH, 0);
  applyDuty(PLATE1_LPWM_CH, 0);

  Serial.println("[СТАРТ] готово, старт с HOT");
  Serial.println();
}

void loop() {
  setPlateHot();
  setPlateCold();
}
