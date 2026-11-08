/*
 * МЕТЕОСТАНЦИЯ — НАРУЖНЫЙ МОДУЛЬ (ESP32-C6)
 * Комплектация по подтверждению пользователя: только BME280.
 * Питание на этапе настройки: USB Type-C от ноутбука.
 *
 * BME280 измеряет температуру, влажность и давление.
 * По Zigbee публикуются температура и влажность через стандартный
 * Zigbee Temperature Sensor endpoint, который принимает главный модуль
 * (ZigbeeThermostat). Давление пока остаётся локальным диагностическим
 * измерением: главный модуль не имеет обработчика Zigbee Pressure endpoint.
 *
 * I2C: SDA GPIO18, SCL GPIO19; BME280 address 0x76 или 0x77.
 * Кнопка BOOT GPIO9: удержание 3 секунды — сброс Zigbee-привязки.
 */

#ifndef ZIGBEE_MODE_ED
#error "Выбери Tools -> Zigbee mode -> Zigbee ED (end device)"
#endif

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include "Zigbee.h"

#define PIN_I2C_SDA 18
#define PIN_I2C_SCL 19
#define PIN_BUTTON  9
#define TEMP_SENSOR_ENDPOINT 10
#define MEASURE_INTERVAL_MS 10000UL
#define ERROR_LOG_INTERVAL_MS 5000UL
#define SENSOR_RETRY_INTERVAL_MS 30000UL

struct OutdoorReading {
  float temperatureC = NAN;
  float humidityPct = NAN;
  float pressureHpa = NAN;
  bool temperatureValid = false;
  bool humidityValid = false;
  bool pressureValid = false;
};

class MyZigbeeGateway {
private:
  ZigbeeTempSensor endpoint;
  bool wasConnected = false;
  bool hasConnectedOnce = false;
  uint32_t disconnectedSinceMs = 0;
  uint32_t lastWaitLogMs = 0;
  static constexpr uint32_t RECONNECT_GRACE_MS = 30000UL;

public:
  MyZigbeeGateway() : endpoint(TEMP_SENSOR_ENDPOINT) {}

  bool begin() {
    endpoint.setManufacturerAndModel("Meteo", "OutdoorBME280");
    endpoint.setMinMaxValue(-40, 85);
    endpoint.setTolerance(0.1f);
    endpoint.addHumiditySensor(0, 100, 1);
    // Источник питания не передаём в Zigbee: сейчас это USB от ноутбука,
    // постоянная схема питания наружного модуля ещё не определена.
    Zigbee.addEndpoint(&endpoint);

    if (!Zigbee.begin()) {
      Serial.println("Zigbee failed to start; restarting");
      delay(1000);
      ESP.restart();
    }
    Serial.println("Zigbee end device started; waiting for coordinator");
    return true;
  }

  bool connected() const { return Zigbee.connected(); }
  void update(uint32_t now) {
    bool current = connected();
    if (current) {
      if (!wasConnected) Serial.println("Zigbee connected");
      wasConnected = true;
      hasConnectedOnce = true;
      disconnectedSinceMs = 0;
    } else {
      if (wasConnected) {
        Serial.println("Zigbee disconnected; waiting for automatic rejoin");
        disconnectedSinceMs = now;
      }
      wasConnected = false;

      // Если ранее сеть была доступна, но стек не восстановил связь за 30 секунд,
      // перезапускаем плату без factory reset. Сохранённые Zigbee-параметры остаются.
      if (hasConnectedOnce && disconnectedSinceMs != 0 &&
          now - disconnectedSinceMs >= RECONNECT_GRACE_MS) {
        Serial.println("Zigbee rejoin timeout; restarting with saved network settings");
        delay(100);
        ESP.restart();
      }
    }
    if (!current && now - lastWaitLogMs >= 5000UL) {
      lastWaitLogMs = now;
      Serial.println("Waiting for Zigbee coordinator...");
    }
  }

  // Значения только записываются в атрибуты endpoint; периодичность отчётов задаёт
  // координатор (setTemperatureReporting/setHumidityReporting на главном модуле).
  void publish(const OutdoorReading &r) {
    if (!connected()) return;
    if (r.temperatureValid) endpoint.setTemperature(r.temperatureC);
    if (r.humidityValid) endpoint.setHumidity(r.humidityPct);
    Serial.printf("Outdoor BME280: T=%.2f C, H=%.1f %%, P=%.1f hPa\n",
                  r.temperatureValid ? r.temperatureC : NAN,
                  r.humidityValid ? r.humidityPct : NAN,
                  r.pressureValid ? r.pressureHpa : NAN);
  }

  void factoryReset() { Zigbee.factoryReset(true); }
};

class BindingResetController {
private:
  uint32_t pressedAtMs = 0;
  bool tracking = false;
  bool resetDone = false;

public:
  void update(uint32_t now, MyZigbeeGateway &zigbee) {
    if (digitalRead(PIN_BUTTON) == LOW) {
      if (!tracking) {
        tracking = true;
        pressedAtMs = now;
      } else if (!resetDone && now - pressedAtMs >= 3000UL) {
        resetDone = true;
        Serial.println("Resetting Zigbee network settings");
        zigbee.factoryReset();
      }
    } else {
      tracking = false;
      resetDone = false;
      pressedAtMs = 0;
    }
  }
};

class OutdoorModuleController {
private:
  Adafruit_BME280 bme;
  bool sensorReady = false;
  uint32_t lastMeasureMs = 0;
  MyZigbeeGateway communication;
  BindingResetController button;
  OutdoorReading latest;
  uint32_t lastSensorErrorMs = 0;
  uint32_t lastSensorRetryMs = 0;

  bool initializeSensor() {
    sensorReady = bme.begin(0x76, &Wire) || bme.begin(0x77, &Wire);
    if (sensorReady) {
      // Принудительный режим: датчик измеряет только по запросу и между замерами «спит» —
      // нет самонагрева, который завышает температуру в режиме NORMAL.
      bme.setSampling(Adafruit_BME280::MODE_FORCED,
                      Adafruit_BME280::SAMPLING_X1,    // температура
                      Adafruit_BME280::SAMPLING_X1,    // давление
                      Adafruit_BME280::SAMPLING_X1,    // влажность
                      Adafruit_BME280::FILTER_OFF);
      Serial.println("BME280 initialized (forced mode)");
    }
    return sensorReady;
  }

public:
  void begin() {
    Serial.begin(115200);
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    initializeSensor();
    lastSensorRetryMs = millis();
    if (!sensorReady) Serial.println("BME280 not found; retrying every 30 seconds");
    communication.begin();
  }

  void update() {
    uint32_t now = millis();
    communication.update(now);

    if (!sensorReady && now - lastSensorRetryMs >= SENSOR_RETRY_INTERVAL_MS) {
      lastSensorRetryMs = now;
      initializeSensor();
    }

    if (sensorReady && now - lastMeasureMs >= MEASURE_INTERVAL_MS) {
      lastMeasureMs = now;
      const bool measured = bme.takeForcedMeasurement();   // один замер перед чтением (раз в 10 с)
      latest.temperatureC = measured ? bme.readTemperature() : NAN;
      latest.humidityPct = measured ? bme.readHumidity() : NAN;
      latest.pressureHpa = measured ? bme.readPressure() / 100.0f : NAN;
      latest.temperatureValid = isfinite(latest.temperatureC) &&
                                latest.temperatureC >= -40.0f && latest.temperatureC <= 85.0f;
      latest.humidityValid = isfinite(latest.humidityPct) &&
                             latest.humidityPct >= 0.0f && latest.humidityPct <= 100.0f;
      latest.pressureValid = isfinite(latest.pressureHpa) &&
                             latest.pressureHpa >= 300.0f && latest.pressureHpa <= 1200.0f;

      if (!latest.temperatureValid && !latest.humidityValid && !latest.pressureValid) {
        sensorReady = false;
        lastSensorRetryMs = now;
        Serial.println("BME280 read failed; will retry initialization");
      } else {
        communication.publish(latest);          // внутри — проверка connected()
      }
    }
    if (!sensorReady && now - lastSensorErrorMs >= ERROR_LOG_INTERVAL_MS) {
      lastSensorErrorMs = now;
      Serial.println("BME280 unavailable; check SDA=18, SCL=19 and address 0x76/0x77");
    }

    button.update(now, communication);
    delay(20);
  }
};

OutdoorModuleController station;
void setup() { station.begin(); }
void loop() { station.update(); }
