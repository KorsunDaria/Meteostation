/*
 * E-ink метеостанция — ПОБОЧНЫЙ (наружный) модуль (ESP32-C6)
 * Zigbee end device + BME280
 *
 * Arduino IDE (Tools):
 *   Board:            ESP32C6 Dev Module
 *   Zigbee mode:      Zigbee ED (end device)
 *   Partition Scheme: Zigbee 4MB with spiffs
 *   Erase All Flash Before Sketch Upload: Enabled (при первой прошивке)
 *
 * Библиотеки: Adafruit BME280, Adafruit Unified Sensor, Adafruit BusIO
 *
 * Модуль постоянно бодрствует и обновляет значения температуры/влажности.
 * Главный модуль сам опрашивает их с периодом, заданным в его настройках.
 * Удержание кнопки BOOT 3 секунды — сброс Zigbee-привязки (повторное сопряжение).
 *
 * ВАЖНО: подключение к сети НЕ блокирует запуск. Раньше код "зависал" в
 * setup(), ожидая подключения — если в этот момент сеть на главном модуле
 * была закрыта, кнопка не читалась вообще (цикл до loop() не доходил).
 * Теперь попытка подключения идёт в фоне, а кнопка и датчик работают сразу.
 */

#ifndef ZIGBEE_MODE_ED
#error "Выбери Tools -> Zigbee mode -> Zigbee ED (end device)"
#endif

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include "Zigbee.h"

#define PIN_I2C_SDA   18
#define PIN_I2C_SCL   19
#define PIN_BUTTON    9      // BOOT
#define SENSOR_EP     10
#define MEASURE_MS    10000UL

Adafruit_BME280 bme;
ZigbeeTempSensor zbSensor = ZigbeeTempSensor(SENSOR_EP);

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  if (!bme.begin(0x76, &Wire) && !bme.begin(0x77, &Wire)) {
    Serial.println("BME280 not found!");
  }

  zbSensor.setManufacturerAndModel("Meteo", "OutdoorBME280");
  zbSensor.setMinMaxValue(-40, 85);
  zbSensor.setTolerance(0.1);
  zbSensor.addHumiditySensor(0, 100, 1);
  Zigbee.addEndpoint(&zbSensor);

  if (!Zigbee.begin()) {               // по умолчанию — end device
    Serial.println("Zigbee failed to start, rebooting");
    delay(1000);
    ESP.restart();
  }
  Serial.println("Zigbee started. Joining network in the background...");
  // Дальше НЕ ждём здесь подключения — сразу идём в loop(), чтобы кнопка
  // и датчик работали, даже пока сеть ещё не найдена.
}

void loop() {
  static uint32_t lastMs = 0;
  static uint32_t downAt = 0;
  static bool wasConnected = false;
  static uint32_t lastWaitMsgMs = 0;

  bool connected = Zigbee.connected();
  if (connected != wasConnected) {
    Serial.println(connected ? "Connected to Zigbee network" : "Lost connection to Zigbee network");
    wasConnected = connected;
  } else if (!connected && millis() - lastWaitMsgMs >= 3000UL) {
    lastWaitMsgMs = millis();
    Serial.println("Waiting for Zigbee network...");
  }

  if (connected && millis() - lastMs >= MEASURE_MS) {
    lastMs = millis();
    float t = bme.readTemperature();
    float h = bme.readHumidity();
    float p = bme.readPressure() / 100.0f;   // пока только в Serial (см. примечание в ответе)
    if (!isnan(t)) zbSensor.setTemperature(t);
    if (!isnan(h)) zbSensor.setHumidity(h);
    Serial.printf("T=%.2f C  H=%.1f %%  P=%.1f hPa\n", t, h, p);
  }

  // Сброс Zigbee удержанием кнопки 3 с — работает ВСЕГДА, даже пока не подключились
  if (digitalRead(PIN_BUTTON) == LOW) {
    if (downAt == 0) downAt = millis();
    else if (millis() - downAt > 3000) {
      Serial.println("Factory reset Zigbee");
      Zigbee.factoryReset();              // перезагрузит плату
    }
  } else {
    downAt = 0;
  }

  delay(50);
}
