
/*
 * E-ink метеостанция — ГЛАВНЫЙ модуль (ESP32-C6)
 * Zigbee-координатор + E-ink 7.5" + SHT45 + BME280 + SCD40 (CO2) + GPS Ublox NEO-6M
 *
 * НОВОЕ В ЭТОЙ ВЕРСИИ
 *   - Новый макет экрана (по образцу со скриншота): крупная наружная температура,
 *     "Feels like", сетка показателей, график за 24 часа.
 *   - Температура в °C, давление в мм рт. ст.
 *   - График за 24 часа: наружная температура — линия, влажность дома — столбики,
 *     давление — пунктир. История (шаг 15 мин) хранится во флеше и переживает перезагрузки.
 *   - Прогноз на 5 дней (MET Norway, без ключа) + город/район (OpenStreetMap Nominatim).
 *     Wi-Fi и Zigbee вместе не работают, поэтому раз в 4 часа плата перезагружается
 *     в короткий режим синхронизации (~15-30 с), скачивает данные, сохраняет их
 *     и возвращается в обычный режим. Пока прогноза нет — график крупнее,
 *     в шапке только время и дата. Заодно по NTP уточняется время.
 *   - Восход/закат считаются по координатам (GPS, либо последние сохранённые).
 *
 * Экран обновляется с интервалом из настроек (по умолчанию 5 минут днём,
 * 60 минут ночью). История измерений сохраняется с шагом 15 минут; при превышении 1 MiB CSV на SD компактизируется до последних 30 дней.
 *
 * Arduino IDE (Tools):
 *   Board:            ESP32C6 Dev Module
 *   Zigbee mode:      Zigbee ZCZR (coordinator/router)
 *   Partition Scheme: Zigbee ZCZR 8MB with spiffs
 *   Erase All Flash Before Sketch Upload: Enabled (при первой прошивке)
 *
 * Библиотеки: GxEPD2, Adafruit GFX, Adafruit SHT4x, Adafruit BME280,
 *             Adafruit Unified Sensor, Sensirion I2C SCD4x, TinyGPSPlus,
 *             ArduinoJson (версия 7.x)
 *
 * Кнопка (GPIO9 / BOOT):
 *   - одиночное короткое нажатие (<1 с) — сразу открыть Zigbee-сеть на 3 минуты для
 *     подключения побочного модуля. Без перезагрузки, без Wi-Fi — быстро.
 *   - двойное короткое нажатие — внепланово опросить датчики и обновить экран.
 *   - удержание >1 с — Wi-Fi-настройка как раньше: точка доступа EInk-Meteo
 *     + веб-страница на 5 минут (там же — часовой пояс и ключ геокодера).
 * Кроме того, пока побочный модуль не привязан, сеть и так переоткрывается
 * автоматически каждые 5 минут на 60 с (см. MyZigbeeGateway::update()) —
 * отдельно жать кнопку не обязательно.
 *
 * Время: GPS -> (если нет фикса) время от последней NTP-синхронизации ->
 * (если и её нет) запасное время, введённое на странице настройки (без даты,
 * без записи истории графика).
 */

#ifndef ZIGBEE_MODE_ZCZR
#error "Выбери Tools -> Zigbee mode -> Zigbee ZCZR (coordinator/router)"
#endif

#include <Arduino.h>
#include <ctype.h>
#include <stdarg.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#if defined(CONFIG_MBEDTLS_CERTIFICATE_BUNDLE)
#include "esp_crt_bundle.h"
#endif
#include <ArduinoJson.h>
#include <GxEPD2_BW.h>
#include <Fonts/FreeMonoBold24pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeMono9pt7b.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSans18pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>
#include <Adafruit_SHT4x.h>
#include <Adafruit_BME280.h>
#include <SensirionI2cScd4x.h>
#include <TinyGPSPlus.h>
#include "Zigbee.h"
#include "esp_crt_bundle.h"

class WiFiGateway;
class IndoorSensorsGateway;

// ===================== ПИНЫ (меняй под свою разводку) =====================
#define PIN_EPD_CS    10
#define PIN_EPD_DC    2
#define PIN_EPD_RST   3
#define PIN_EPD_BUSY  4
#define PIN_EPD_SCK   6
#define PIN_EPD_MOSI  7

// microSD использует ту же SPI-шину, что и E-Ink, но отдельный CS.
// Эти два GPIO нужно сверить с фактической разводкой платы перед подключением.
#define PIN_SD_CS     5
#define PIN_SD_MISO   20
#define SD_SPI_HZ     4000000UL

#define PIN_I2C_SDA   18   // SHT45 + BME280 + SCD40
#define PIN_I2C_SCL   19

#define PIN_GPS_RX    22   // сюда идёт TX GPS-модуля
#define PIN_GPS_TX    23   // отсюда идёт RX GPS-модуля

#define PIN_BUTTON    9    // кнопка на GND (на плате это BOOT)

// ===================== ПАРАМЕТРЫ =====================
#define AP_SSID       "EInk-Meteo"
#define AP_PASS       "meteo1234"          // минимум 8 символов
#define PORTAL_MS     (5UL * 60UL * 1000UL)
#define THERMOSTAT_EP 1

#define SYNC_PERIOD_MS (4UL * 60UL * 60UL * 1000UL)  // прогноз и Air Quality раз в 4 часа
#define FIRST_SYNC_MS  (120UL * 1000UL)              // первая попытка через 2 мин после старта
#define RETRY_MS       (120UL * 1000UL)              // повтор синхронизации через 2 мин после загрузки, если прошлая попытка не удалась
#define ZB_INTERVAL_MIN_SEC 20                        // не меньше макс. интервала отчёта Zigbee (60 с): 3*20+5 = 65 с > 60 с
#define FC_MIN_DAYS    3                             // меньше дней в прогнозе — считаем "нет интернета"

#define HN         96                  // точек истории: 96 * 15 мин = 24 ч
#define HSLOT_SEC  900UL
#define HMAGIC     0xE1C2
#define FCMAGIC    0xFC02

// Причины ошибки Wi-Fi (код сохраняется во флеш и расшифровывается внизу экрана)
#define WE_OK        0
#define WE_NO_NET    2     // сеть с таким именем не найдена
#define WE_BAD_PASS  3     // неверный пароль
#define WE_CONN_FAIL 4     // не удалось подключиться (другая причина)
#define WE_NO_INET   5     // к роутеру подключились, но интернета нет
#define WE_FC_FAIL   6     // интернет есть, но прогноз не получен
#define WE_NO_POS    7     // нет координат (ни GPS, ни по IP)
#define WE_UNKNOWN   255   // синхронизаций ещё не было

// ===================== ТИПЫ (все до первой функции — требование Arduino-препроцессора) =====================
enum Trend { TREND_UNKNOWN, TREND_IMPROVING, TREND_STABLE, TREND_WORSENING };
enum IconKind { K_SUN, K_PART, K_CLOUD, K_RAIN, K_SNOW, K_STORM, K_FOG, K_UNKNOWN };

struct AirQuality {
  int16_t europeanAqi = -1;
  bool available = false;
};

struct Coordinates {
  float latitude = NAN;
  float longitude = NAN;
  bool valid = false;
};

struct LT {                            // локальное время
  bool haveDate;
  int y, mo, d, h, mi, wd;             // wd: 0 = воскресенье
};

struct Hist {                          // кольцевой буфер истории за 24 ч
  uint16_t magic;
  uint16_t headIdx;                    // индекс самой свежей записи
  uint32_t headSlot;                   // номер 15-минутного слота самой свежей записи
  int16_t  t[HN];                      // наружная температура, десятые доли °C (INT16_MIN = нет)
  uint8_t  h[HN];                      // влажность дома, в половинках процента (255 = нет)
  uint16_t p[HN];                      // давление, десятые доли мм рт. ст. (0 = нет)
};

struct Forecast {                      // прогноз и место
  uint16_t magic;
  uint8_t  n;                          // сколько дней сохранено (0..5)
  uint32_t date[5];                    // YYYYMMDD
  uint8_t  code[5];                    // код погоды WMO
  int8_t   tmax[5];
  int8_t   tmin[5];
  char     city[28];
  char     dist[28];
  char     region[28];                 // область/регион (показываем, если города нет)
  uint32_t hourEpoch;                  // UTC время первой почасовой точки
  uint8_t  nHour;                      // сколько почасовых точек сохранено (0..6)
  int16_t  hourT[6];                   // температура, десятые доли °C
  uint8_t  hourH[6];                   // влажность, % (255 = нет)
};

// Сопоставленные пары «прогноз 4 часа назад / факт» для Error Bars.
#define ERRBAR_COUNT 6
#define EBMAGIC 0xEB02
struct ErrorBar {
  uint32_t epoch;
  int16_t forecast10;
  int16_t actual10;
};
struct ErrorBarHistory {
  uint16_t magic;
  uint8_t count;
  uint8_t next;
  ErrorBar items[ERRBAR_COUNT];
};

// Поток для ArduinoJson поверх TLS-клиента: терпеливо ждёт следующие байты (до tmo мс) и
// считает прочитанное. Без этого пауза в приходе данных превращалась в "IncompleteInput".
class WaitStream : public Stream {
 public:
  Client  &c;
  uint32_t tmo;
  uint32_t count = 0;
  WaitStream(Client &cl, uint32_t t) : c(cl), tmo(t) {}
  int available() override { return c.available(); }
  int peek() override { return c.peek(); }
  size_t write(uint8_t) override { return 0; }
  int read() override {
    uint32_t t0 = millis();
    while (!c.available()) {
      if (!c.connected()) return -1;            // сервер закрыл соединение, данных больше нет
      if (millis() - t0 > tmo) return -1;       // долго ничего не приходит
      delay(1);
    }
    count++;
    return c.read();
  }
  size_t readBytes(char *b, size_t n) override {
    size_t i = 0;
    while (i < n) { int ch = read(); if (ch < 0) break; b[i++] = (char)ch; }
    return i;
  }
};

// ===================== ЭКРАН =====================
GxEPD2_BW<GxEPD2_750_GDEY075T7, GxEPD2_750_GDEY075T7::HEIGHT> display(
  GxEPD2_750_GDEY075T7(PIN_EPD_CS, PIN_EPD_DC, PIN_EPD_RST, PIN_EPD_BUSY));

// ===================== ОБЪЕКТЫ =====================
Adafruit_SHT4x sht4;
Adafruit_BME280 bme;
SensirionI2cScd4x scd4x;
TinyGPSPlus gps;
uint16_t co2Val = 0;          // ppm, 0 = ещё не измерено
uint32_t co2LastMs = 0;
bool forceMeasurementRequested = false;
bool forceExternalSyncRequested = false;
bool forceDisplayRequested = false;
uint32_t clockBaseMillis = 0; // millis() в момент старта — точка отсчёта запасных софт-часов
ZigbeeThermostat zbThermostat = ZigbeeThermostat(THERMOSTAT_EP);
Preferences prefs;
WebServer server(80);

Hist     hist;                // история графика
Forecast fc;                  // прогноз + город/район
ErrorBarHistory errorBars;
int16_t airQuality = -1;     // European AQI, -1 = нет актуальных данных
float forecastCurrentTemp = NAN, forecastWindMs = NAN, forecastPrecipMm = NAN;
float forecastHumidity = NAN, forecastUv = NAN;
uint8_t forecastWmoCode = 255;
enum AlertGroup1 : uint8_t { GROUP1_NONE, GROUP1_ICE, GROUP1_WIND };
enum AlertGroup2 : uint8_t { GROUP2_NONE, GROUP2_SNOW, GROUP2_STORM };
enum TempAlert : uint8_t { TEMP_NONE, TEMP_FROST, TEMP_HEAT };
AlertGroup1 alertGroup1 = GROUP1_NONE; // гололёд или сильный ветер
AlertGroup2 alertGroup2 = GROUP2_NONE; // сильный снегопад или гроза
TempAlert tempAlert = TEMP_NONE;       // экстремальная температура: мороз или жара
float    homeLat = NAN, homeLon = NAN;   // последние известные координаты
uint32_t epochBase = 0;       // UTC-время (unix), соответствующее epochBaseMs; 0 = неизвестно
uint32_t epochBaseMs = 0;
bool     syncFailFlag = false;
uint8_t  syncFails = 0;               // подряд неудачных синхронизаций (для нарастающего интервала повтора)
bool     homeFromIp = false;          // true — homeLat/homeLon получены по IP, а не со спутников
bool     skipFirstDraw = false;       // после повторной неудачи с тем же кодом ошибки экран при загрузке не перерисовываем
uint8_t  wifiErr = WE_UNKNOWN;          // результат последней попытки Wi-Fi
volatile uint8_t lastDiscReason = 0;    // причина последнего разрыва Wi-Fi (из события)

char errDetail[80] = "";                // подробность последней ошибки синхронизации (показывается на экране)
void setErrDetail(const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(errDetail, sizeof(errDetail), fmt, ap);
  va_end(ap);
  Serial.printf("ERR: %s\n", errDetail);
}

// ===================== НАСТРОЙКИ =====================
struct Settings {
  String   ssid;
  String   pass;
  uint16_t refreshMin      = 5;     // период обновления экрана днём, мин
  uint8_t  nightStart      = 23;    // ночной режим: с (час)
  uint8_t  nightEnd        = 7;     // ... до (час)
  uint16_t nightRefreshMin = 60;    // период обновления ночью, мин
  uint16_t zbIntervalSec   = 60;    // база тайм-аута потери связи с наружным модулем, с (тайм-аут = 3*значение + 5 с)
  int8_t   tz              = 0;     // часовой пояс (UTC+N)
  uint16_t clockSetMin     = 480;   // запасное время (если нет GPS и NTP), мин. с полуночи

  // Независимые настройки отображения показателей.
  bool showAirQuality      = true;
  bool showFeelsLike       = false;
  bool showUvIndex         = false;
  bool showPressure        = true;
  bool showCo2             = true;
  bool showIndoorHumidity  = true;
  bool showOutdoorHumidity = true;

  // Приоритетный источник наружных данных: false — датчик (интернет как запасной), true — интернет (датчик как запасной).
  bool outdoorWebFirst     = false;
} cfg;

void loadSettings() {
  cfg.ssid            = prefs.getString("ssid", "");
  cfg.pass            = prefs.getString("pass", "");
  cfg.refreshMin      = prefs.getUShort("refr", 5);
  cfg.nightStart      = prefs.getUChar("ns", 23);
  cfg.nightEnd        = prefs.getUChar("ne", 7);
  cfg.nightRefreshMin = prefs.getUShort("nr", 60);
  cfg.zbIntervalSec   = prefs.getUShort("zbi", 60);
  if (cfg.zbIntervalSec < ZB_INTERVAL_MIN_SEC) cfg.zbIntervalSec = ZB_INTERVAL_MIN_SEC;
  cfg.tz              = prefs.getChar("tz", 0);
  if (cfg.tz < -12 || cfg.tz > 14) cfg.tz = 0; // защита от повреждённой настройки
  cfg.clockSetMin     = prefs.getUShort("cmin", 480);
  cfg.showAirQuality = prefs.getBool("v_aqi", true);
  cfg.showFeelsLike = prefs.getBool("v_feel", false);
  cfg.showUvIndex = prefs.getBool("v_uv", false);
  cfg.showPressure = prefs.getBool("v_press", true);
  cfg.showCo2 = prefs.getBool("v_co2", true);
  cfg.showIndoorHumidity = prefs.getBool("v_ihum", true);
  cfg.showOutdoorHumidity = prefs.getBool("v_ohum", true);
  cfg.outdoorWebFirst = prefs.getBool("o_web", false);
}

void saveSettings() {
  prefs.putString("ssid", cfg.ssid);
  prefs.putString("pass", cfg.pass);
  prefs.putUShort("refr", cfg.refreshMin);
  prefs.putUChar("ns", cfg.nightStart);
  prefs.putUChar("ne", cfg.nightEnd);
  prefs.putUShort("nr", cfg.nightRefreshMin);
  prefs.putUShort("zbi", cfg.zbIntervalSec);
  prefs.putChar("tz", cfg.tz);
  prefs.putUShort("cmin", cfg.clockSetMin);
  prefs.putBool("v_aqi", cfg.showAirQuality);
  prefs.putBool("v_feel", cfg.showFeelsLike);
  prefs.putBool("v_uv", cfg.showUvIndex);
  prefs.putBool("v_press", cfg.showPressure);
  prefs.putBool("v_co2", cfg.showCo2);
  prefs.putBool("v_ihum", cfg.showIndoorHumidity);
  prefs.putBool("v_ohum", cfg.showOutdoorHumidity);
  prefs.putBool("o_web", cfg.outdoorWebFirst);
}

// ===================== ДАННЫЕ НАРУЖНОГО МОДУЛЯ (Zigbee) =====================
float    outT = NAN, outH = NAN;
uint32_t outTempLastMs = 0;
uint32_t outHumLastMs = 0;

void onOutTemp(float t) {
  if (isfinite(t) && t >= -40.0f && t <= 85.0f) {
    outT = t;
    outTempLastMs = millis();
  }
}

void onOutHum(float h) {
  if (isfinite(h) && h >= 0.0f && h <= 100.0f) {
    outH = h;
    outHumLastMs = millis();
  }
}

bool zbLinkOk() {
  const uint32_t timeout = 3UL * cfg.zbIntervalSec * 1000UL + 5000UL;
  const uint32_t now = millis();
  return zbThermostat.bound() && outTempLastMs != 0 && outHumLastMs != 0 &&
         now - outTempLastMs < timeout && now - outHumLastMs < timeout;
}

enum OutdoorState {
  OUT_NOT_WAITING,  // 1. Не ждём подключения
  OUT_WAITING,      // 2. Сеть открыта, ждём модуль
  OUT_CONNECTED     // 3. Модуль подключён
};

OutdoorState outdoorState = OUT_NOT_WAITING;
uint32_t zbOpenUntilMs = 0;
bool zbOpenRequested = false;

// Стартовый тест экрана: первый показ — все иконки, затем 5 секунд тестового вида.
bool startupAllIcons = false;
uint32_t startupAllIconsUntilMs = 0;

// ===================== ДАВЛЕНИЕ / ТРЕНД ДЛЯ ИКОНКИ =====================
#define PH_N 6                         // 6 точек с интервалом 1 минута = 5 минут
#define PH_INTERVAL_MS (60UL * 1000UL)
#define TREND_REFRESH_MS (3UL * 60UL * 60UL * 1000UL)
float    pHist[PH_N];
float    tHist[PH_N];
uint8_t  pCount = 0;
uint32_t pLastMs = 0;
Trend    cachedTrend = TREND_UNKNOWN;
uint32_t trendCalculatedAt = 0;

void pushTrendSample(float pressureHpa, float temperatureC) {
  if (!isfinite(pressureHpa) || pressureHpa < 300.0f || pressureHpa > 1200.0f) return;
  if (pCount < PH_N) {
    pHist[pCount] = pressureHpa;
    tHist[pCount] = temperatureC;
    pCount++;
  } else {
    memmove(pHist, pHist + 1, (PH_N - 1) * sizeof(float));
    memmove(tHist, tHist + 1, (PH_N - 1) * sizeof(float));
    pHist[PH_N - 1] = pressureHpa;
    tHist[PH_N - 1] = temperatureC;
  }
}

Trend forecastTrend() {
  // Первый расчёт после накопления 5 минут измерений; затем обновление раз в 3 часа.
  if (cachedTrend != TREND_UNKNOWN && millis() - trendCalculatedAt < TREND_REFRESH_MS)
    return cachedTrend;
  if (pCount < PH_N) return TREND_UNKNOWN;

  const float pressureDeltaMmHg = (pHist[PH_N - 1] - pHist[0]) * 0.750062f;
  const float temperatureDelta = tHist[PH_N - 1] - tHist[0];
  int score = 0;

  // Рост давления — положительный признак, падение — отрицательный.
  if (pressureDeltaMmHg >= 0.3f) score += 2;
  else if (pressureDeltaMmHg <= -0.3f) score -= 2;

  // Температура используется как дополнительный, более слабый признак.
  if (isfinite(tHist[0]) && isfinite(tHist[PH_N - 1])) {
    if (temperatureDelta >= 0.5f) score += 1;
    else if (temperatureDelta <= -0.5f) score -= 1;
  }

  if (score >= 2) cachedTrend = TREND_IMPROVING;
  else if (score <= -2) cachedTrend = TREND_WORSENING;
  else cachedTrend = TREND_STABLE;

  trendCalculatedAt = millis();
  return cachedTrend;
}

// ===================== КАЛЕНДАРЬ / СОЛНЦЕ / ОЩУЩАЕМАЯ ТЕМПЕРАТУРА =====================
// ---- PURE-BEGIN (чистая математика, без железа)
static int32_t daysFromCivil(int y, unsigned m, unsigned d) {
  y -= (m <= 2);
  const int era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = (unsigned)(y - era * 400);
  const unsigned doy = (153 * (m > 2 ? m - 3 : m + 9) + 2) / 5 + d - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + (int)doe - 719468;
}

static void civilFromDays(int32_t z, int &y, int &m, int &d) {
  z += 719468;
  const int era = (z >= 0 ? z : z - 146096) / 146097;
  const unsigned doe = (unsigned)(z - era * 146097);
  const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  y = (int)yoe + era * 400;
  const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  const unsigned mp = (5 * doy + 2) / 153;
  d = (int)(doy - (153 * mp + 2) / 5 + 1);
  m = (int)(mp < 10 ? mp + 3 : mp - 9);
  if (m <= 2) y++;
}

// Восход (rise=true) / закат в минутах от местной полуночи (часовой пояс tz — целые часы).
// false — солнце не восходит/не заходит в этот день (полярные широты).
static bool sunEvent(bool rise, double lat, double lon, int y, int m, int d, int tz, int &minutesOut) {
  const double rad = 3.14159265358979 / 180.0;
  int N = daysFromCivil(y, m, d) - daysFromCivil(y, 1, 1) + 1;
  double lngHour = lon / 15.0;
  double t = N + ((rise ? 6.0 : 18.0) - lngHour) / 24.0;
  double M = 0.9856 * t - 3.289;
  double L = M + 1.916 * sin(M * rad) + 0.020 * sin(2.0 * M * rad) + 282.634;
  L = fmod(L, 360.0); if (L < 0) L += 360.0;
  double RA = atan(0.91764 * tan(L * rad)) / rad;
  RA = fmod(RA + 360.0, 360.0);
  RA += (floor(L / 90.0) * 90.0) - (floor(RA / 90.0) * 90.0);
  RA /= 15.0;
  double sinDec = 0.39782 * sin(L * rad);
  double cosDec = cos(asin(sinDec));
  double cosH = (cos(90.833 * rad) - sinDec * sin(lat * rad)) / (cosDec * cos(lat * rad));
  if (cosH > 1.0 || cosH < -1.0) return false;
  double H = rise ? 360.0 - acos(cosH) / rad : acos(cosH) / rad;
  H /= 15.0;
  double T = H + RA - 0.06571 * t - 6.622;
  double UT = fmod(T - lngHour + 48.0, 24.0);
  double local = fmod(UT + tz + 48.0, 24.0);
  minutesOut = ((int)(local * 60.0 + 0.5)) % 1440;
  return true;
}

// ---- PURE-END

const char *WD[]  = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
const char *WD3[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
const char *MONTHS[] = {"January", "February", "March", "April", "May", "June", "July",
                        "August", "September", "October", "November", "December"};

// ===================== ВРЕМЯ =====================
uint32_t gpsEpoch() {
  if (!gps.time.isValid() || !gps.date.isValid() || gps.date.year() < 2020) return 0;
  if (gps.time.age() > 10000) return 0;
  int32_t days = daysFromCivil(gps.date.year(), gps.date.month(), gps.date.day());
  return (uint32_t)days * 86400UL + gps.time.hour() * 3600UL + gps.time.minute() * 60UL + gps.time.second();
}

// UTC-время (unix) или 0, если источников нет
uint32_t epochNow() {
  uint32_t g = gpsEpoch();
  if (g) { epochBase = g; epochBaseMs = millis(); return g; }
  if (epochBase) return epochBase + (millis() - epochBaseMs) / 1000UL;
  return 0;
}

// Перед ЛЮБЫМ плановым рестартом запоминаем время, чтобы после загрузки не терять его
// (флаг epok сбрасывается сразу после чтения — после сбоя питания время считается неизвестным)
void saveEpochForRestart() {
  uint32_t e = epochNow();
  if (e) {
    prefs.putUInt("ep", e + 3);        // +3 с на сам перезапуск
    prefs.putBool("epok", true);
  }
}

LT localNow() {
  LT t;
  memset(&t, 0, sizeof(t));
  uint32_t e = epochNow();
  if (e) {
    int64_t le = (int64_t)e + (int64_t)cfg.tz * 3600;
    int32_t days = (int32_t)(le / 86400);
    int32_t sec  = (int32_t)(le % 86400);
    civilFromDays(days, t.y, t.mo, t.d);
    t.h  = sec / 3600;
    t.mi = (sec % 3600) / 60;
    t.wd = (days + 4) % 7;
    t.haveDate = true;
  } else {
    uint32_t elapsedMin = (millis() - clockBaseMillis) / 60000UL;
    uint16_t m = (uint16_t)((cfg.clockSetMin + elapsedMin) % 1440);
    t.h = m / 60;
    t.mi = m % 60;
  }
  return t;
}

uint16_t currentMinutes() {
  LT t = localNow();
  return (uint16_t)(t.h * 60 + t.mi);
}

bool isNight() {
  uint16_t cur = currentMinutes();
  uint16_t s = (uint16_t)cfg.nightStart * 60, e = (uint16_t)cfg.nightEnd * 60;
  if (s == e) return false;
  return (s < e) ? (cur >= s && cur < e) : (cur >= s || cur < e);
}

bool isNightIcon() {
  LT t = localNow();
  double la = NAN, lo = NAN;
  if (gps.location.isValid() && gps.location.age() < 60000) { la = gps.location.lat(); lo = gps.location.lng(); }
  else if (!isnan(homeLat)) { la = homeLat; lo = homeLon; }
  if (t.haveDate && !isnan(la)) {
    int rise, set;
    if (sunEvent(true, la, lo, t.y, t.mo, t.d, cfg.tz, rise) &&
        sunEvent(false, la, lo, t.y, t.mo, t.d, cfg.tz, set) && rise != set) {
      uint16_t cur = (uint16_t)(t.h * 60 + t.mi);
      return (rise < set) ? (cur < rise || cur >= set)     // обычный случай: день внутри суток
                          : (cur >= set && cur < rise);    // день переходит через полночь
    }
  }
  return isNight();
}


// Запоминаем координаты (для восхода/заката и прогноза, когда GPS временно без фикса)
void updateHome() {
  static uint32_t last = 0;
  if (millis() - last < 60000UL) return;
  last = millis();
  if (!gps.location.isValid() || gps.location.age() > 10000) return;
  float la = (float)gps.location.lat(), lo = (float)gps.location.lng();
  // Если сохранённые координаты были получены по IP, а теперь есть фикс GPS — всегда заменяем их спутниковыми.
  if (homeFromIp || isnan(homeLat) || fabsf(la - homeLat) > 0.05f || fabsf(lo - homeLon) > 0.05f) {
    homeLat = la; homeLon = lo;
    prefs.putFloat("lat", la);
    prefs.putFloat("lon", lo);
    if (homeFromIp) { homeFromIp = false; prefs.putBool("latip", false); }
  }
}

// ===================== ИСТОРИЯ ГРАФИКА =====================
void histInvalidate(int i) { hist.t[i] = INT16_MIN; hist.h[i] = 255; hist.p[i] = 0; }

void histAdd(uint32_t slot, float t, float h, float pmm) {
  if (hist.magic != HMAGIC) {
    memset(&hist, 0, sizeof(hist));
    for (int i = 0; i < HN; i++) histInvalidate(i);
    hist.magic = HMAGIC;
    hist.headIdx = 0;
    hist.headSlot = slot;
  } else if (slot > hist.headSlot) {
    uint32_t gap = slot - hist.headSlot;
    if (gap >= HN) {
      for (int i = 0; i < HN; i++) histInvalidate(i);
      hist.headIdx = 0;
    } else {
      for (uint32_t k = 1; k < gap; k++) histInvalidate((hist.headIdx + k) % HN);
      hist.headIdx = (hist.headIdx + gap) % HN;
    }
    hist.headSlot = slot;
  } else {
    return;                            // слот уже записан (или часы ушли назад)
  }
  int i = hist.headIdx;
  histInvalidate(i);
  if (!isnan(t))   hist.t[i] = (int16_t)lroundf(t * 10.0f);
  if (!isnan(h))   hist.h[i] = (uint8_t)constrain((int)lroundf(h * 2.0f), 0, 200);
  if (!isnan(pmm)) hist.p[i] = (uint16_t)lroundf(pmm * 10.0f);
}

void histSave() { prefs.putBytes("hist", &hist, sizeof(hist)); }

void errorBarsSave() {
  errorBars.magic = EBMAGIC;
  prefs.putBytes("ebars", &errorBars, sizeof(errorBars));
}

// Сверяем точку, спрогнозированную 4 часа назад (old — прогноз ДО замены), с фактом.
// Факт: наружный датчик из 15-минутной истории; если последняя запись истории была взята из интернета
// (или датчика нет) — берём «сейчас» из только что полученного прогноза (fc).
void captureForecastError(const Forecast &old) {
  if (old.magic != FCMAGIC || old.nHour < 5) return;
  const uint32_t target = old.hourEpoch + 4UL * 3600UL;
  const uint32_t nowE = epochNow();
  if (nowE && target > nowE + 1800UL) return;          // точка прогноза ещё в будущем
  if (errorBars.magic != EBMAGIC) memset(&errorBars, 0, sizeof(errorBars));
  for (int k = 0; k < errorBars.count && k < ERRBAR_COUNT; ++k)
    if (errorBars.items[k].epoch == target) return;    // такая отметка уже есть — дубль не пишем

  const int16_t predicted = old.hourT[4];
  int16_t actual = INT16_MIN;
  if (!prefs.getBool("hsrc", false) && hist.magic == HMAGIC) {   // история писалась с датчика
    const int32_t targetSlot = (int32_t)(target / HSLOT_SEC);
    int bestIdx = -1;
    int32_t bestDelta = 3;                             // допускаем не более 2 слотов = 30 минут
    for (int i = 0; i < HN; ++i) {
      int32_t slot = (int32_t)hist.headSlot - ((int)hist.headIdx - i + HN) % HN;
      int32_t delta = abs(slot - targetSlot);
      if (delta < bestDelta && hist.t[i] != INT16_MIN) { bestDelta = delta; bestIdx = i; }
    }
    if (bestIdx >= 0) actual = hist.t[bestIdx];
  }
  if (actual == INT16_MIN && fc.magic == FCMAGIC && fc.hourEpoch && fc.hourEpoch <= target) {
    const uint32_t off = target - fc.hourEpoch;        // факт из интернета: почасовая точка нового прогноза
    if (off % 3600UL == 0 && off / 3600UL < fc.nHour) actual = fc.hourT[off / 3600UL];
  }
  if (actual == INT16_MIN) return;

  ErrorBar &b = errorBars.items[errorBars.next];
  b.epoch = target;
  b.forecast10 = predicted;
  b.actual10 = actual;
  errorBars.next = (errorBars.next + 1) % ERRBAR_COUNT;
  if (errorBars.count < ERRBAR_COUNT) errorBars.count++;
  errorBarsSave();
}

// SD-архив хранит время только в UTC Unix seconds. Часовой пояс применяется
// исключительно при отображении, поэтому смена TZ/DST не сдвигает старые записи.
bool sdReady = false;
bool sht4Ready = false;
bool bmeReady = false;
bool scd4Ready = false;
uint32_t bmeRetryAtMs = 0;
uint32_t sht4RetryAtMs = 0;
uint32_t scd4RetryAtMs = 0;
static constexpr uint32_t SENSOR_RETRY_INTERVAL_MS = 30000UL;
void sdBeginAndRestoreHistory();
void sdRecoverArchiveSwap();
bool sdRestoreForecastBackup();
void sdAppendWeather(uint32_t epoch, float indoorT, float indoorH, float outdoorT, float outdoorH, float pressureMmHg, uint16_t co2);

class SDCardController {
public:
  void begin() { sdBeginAndRestoreHistory(); }
  void append(uint32_t epoch, float indoorT, float indoorH, float outdoorT,
              float outdoorH, float pressureMmHg, uint16_t co2) {
    sdAppendWeather(epoch, indoorT, indoorH, outdoorT, outdoorH, pressureMmHg, co2);
  }
};

SDCardController sdCardController;

// Температура/влажность "сейчас" из последнего прогноза (почасовые точки, линейная интерполяция).
// Используется, когда наружный модуль недоступен. Годится ~6 часов после синхронизации.
bool webOutdoor(float &t, float &h) {
  t = NAN; h = NAN;
  if (fc.magic != FCMAGIC || fc.nHour == 0) return false;
  uint32_t e = epochNow();
  if (!e) return false;
  int64_t dt = (int64_t)e - (int64_t)fc.hourEpoch;
  if (dt < -3600) return false;                          // часы ушли назад
  if (dt < 0) dt = 0;
  int idx = (int)(dt / 3600);
  if (idx >= fc.nHour) return false;                     // данные устарели
  float fr = (dt % 3600) / 3600.0f;
  if (idx + 1 < fc.nHour) {
    t = (fc.hourT[idx] * (1.0f - fr) + fc.hourT[idx + 1] * fr) / 10.0f;
    if (fc.hourH[idx] != 255 && fc.hourH[idx + 1] != 255) h = fc.hourH[idx] * (1.0f - fr) + fc.hourH[idx + 1] * fr;
  } else {
    t = fc.hourT[idx] / 10.0f;
    if (fc.hourH[idx] != 255) h = fc.hourH[idx];
  }
  return true;
}

// Выбор источника наружных данных. Приоритет задаёт cfg.outdoorWebFirst (веб-страница настройки),
// второй источник — запасной. fromWeb = true, если значения взяты из интернета.
bool getOutdoor(float &t, float &h, bool &fromWeb) {
  t = NAN; h = NAN; fromWeb = false;
  float wt, wh;
  const bool sensorOk = zbLinkOk();
  const bool webOk = webOutdoor(wt, wh);
  const bool useWeb = cfg.outdoorWebFirst ? webOk : (!sensorOk && webOk);
  if (useWeb) { t = wt; h = wh; fromWeb = true; return true; }
  if (sensorOk) { t = outT; h = outH; return true; }
  return false;
}

// Низкоуровневый шлюз локальных датчиков главного модуля.
// Использует уже инициализированные SHT45 и BME280, не меняя их настройки.
class IndoorSensorsGateway {
public:
  void read(float &temperature, float &humidity, float &pressureHpa) {
    sensors_event_t hum, temp;
    temperature = NAN;
    humidity = NAN;
    const uint32_t now = millis();
    if (!sht4Ready && now - sht4RetryAtMs >= SENSOR_RETRY_INTERVAL_MS) {
      sht4RetryAtMs = now;
      sht4Ready = sht4.begin(&Wire);
      if (sht4Ready) {
        sht4.setPrecision(SHT4X_HIGH_PRECISION);
        Serial.println("SHT45 recovered");
      }
    }
    if (sht4Ready && sht4.getEvent(&hum, &temp)) {
      if (isfinite(temp.temperature) && temp.temperature >= -40.0f && temp.temperature <= 85.0f)
        temperature = temp.temperature;
      if (isfinite(hum.relative_humidity) && hum.relative_humidity >= 0.0f && hum.relative_humidity <= 100.0f)
        humidity = hum.relative_humidity;
    } else if (sht4Ready) {
      sht4Ready = false;
      sht4RetryAtMs = now;
    }
    pressureHpa = readPressure();
  }

  float readPressure() {
    const uint32_t now = millis();
    if (!bmeReady && now - bmeRetryAtMs >= SENSOR_RETRY_INTERVAL_MS) {
      bmeRetryAtMs = now;
      bmeReady = bme.begin(0x76, &Wire) || bme.begin(0x77, &Wire);
      if (bmeReady) Serial.println("BME280 recovered");
    }
    if (!bmeReady) return NAN;
    const float pressureHpa = bme.readPressure() / 100.0f;
    if (!isfinite(pressureHpa) || pressureHpa < 300.0f || pressureHpa > 1200.0f) {
      bmeReady = false;
      bmeRetryAtMs = now;
      Serial.println("BME280 pressure read invalid; will retry initialization");
      return NAN;
    }
    return pressureHpa;
  }
};

void sampleHistory(IndoorSensorsGateway &indoorSensors) {
  static uint32_t lastChk = 0;
  uint32_t now = millis();
  if (now - lastChk < 5000UL) return;
  lastChk = now;

  // SD: если карта не примонтировалась при старте (или была вынута) — повторная попытка раз в 45 с.
  static uint32_t sdRetryMs = 0;
  if (!sdReady && now - sdRetryMs >= 45000UL) {
    sdRetryMs = now;
    SD.end();
    if (SD.begin(PIN_SD_CS, SPI, SD_SPI_HZ)) {
      sdReady = true;
      Serial.println("SD: remounted");
      sdRecoverArchiveSwap();
    }
  }
  uint32_t e = epochNow();
  if (!e) return;                      // без времени историю не пишем
  uint32_t slot = e / HSLOT_SEC;
  if (hist.magic == HMAGIC && slot <= hist.headSlot) return;

  float inT, inH, pHpa;
  indoorSensors.read(inT, inH, pHpa);
  float histOut = NAN, histOutH = NAN;
  bool histWeb = false;
  getOutdoor(histOut, histOutH, histWeb);                // источник по приоритету из настроек
  float pressureMmHg = isnan(pHpa) ? NAN : pHpa * 0.750062f;
  histAdd(slot, histOut, inH, pressureMmHg);
  histSave();
  prefs.putBool("hsrc", histWeb);                        // откуда взята последняя точка истории (для Error Bars)
  sdCardController.append(e, inT, inH, histOut, histOutH, pressureMmHg, co2Val);
}

// Инициализация SD и восстановление кольцевой истории из CSV.
// Неполная последняя строка после внезапного отключения просто пропускается.
static constexpr uint32_t SD_ARCHIVE_RETENTION_SEC = 30UL * 24UL * 60UL * 60UL;
static constexpr uint32_t SD_ARCHIVE_MAX_BYTES = 1024UL * 1024UL;

// Recover an interrupted archive swap. A backup is kept until the new archive
// has been renamed successfully, so a power loss cannot destroy the old CSV.
void sdRecoverArchiveSwap() {
  const bool hasMain = SD.exists("/weather.csv");
  const bool hasBackup = SD.exists("/weather.csv.bak");
  if (!hasMain && hasBackup) SD.rename("/weather.csv.bak", "/weather.csv");
  else if (hasMain && hasBackup) SD.remove("/weather.csv.bak");
  if (SD.exists("/weather.csv.tmp")) SD.remove("/weather.csv.tmp");
}

void sdCompactArchiveIfNeeded(uint32_t currentEpoch) {
  if (!sdReady || !SD.exists("/weather.csv")) return;
  File sizeCheck = SD.open("/weather.csv", FILE_READ);
  if (!sizeCheck) return;
  const uint32_t currentSize = (uint32_t)sizeCheck.size();
  sizeCheck.close();
  if (currentSize <= SD_ARCHIVE_MAX_BYTES || currentEpoch < SD_ARCHIVE_RETENTION_SEC) return;

  if (SD.exists("/weather.csv.tmp")) SD.remove("/weather.csv.tmp");
  File source = SD.open("/weather.csv", FILE_READ);
  File target = SD.open("/weather.csv.tmp", FILE_WRITE);
  if (!source || !target) {
    if (source) source.close();
    if (target) target.close();
    SD.remove("/weather.csv.tmp");
    Serial.println("SD: archive compaction could not open files");
    return;
  }

  target.println("timestamp_utc,indoor_temperature_c,indoor_humidity_pct,outdoor_temperature_c,outdoor_humidity_pct,pressure_mmhg,co2_ppm");
  const uint32_t cutoff = currentEpoch - SD_ARCHIVE_RETENTION_SEC;
  uint32_t kept = 0;
  while (source.available()) {
    String line = source.readStringUntil('\n');
    line.trim();
    if (!line.length() || line.startsWith("timestamp_utc")) continue;
    unsigned long epoch = 0;
    if (sscanf(line.c_str(), "%lu", &epoch) == 1 && epoch >= cutoff && epoch <= currentEpoch + 86400UL) {
      target.println(line);
      ++kept;
    }
  }
  source.close();
  target.flush();
  target.close();

  File verify = SD.open("/weather.csv.tmp", FILE_READ);
  const bool validTemp = verify && verify.size() > 80 && kept > 0;
  if (verify) verify.close();
  if (!validTemp) {
    SD.remove("/weather.csv.tmp");
    Serial.println("SD: archive compaction aborted; original archive retained");
    return;
  }

  if (SD.exists("/weather.csv.bak")) SD.remove("/weather.csv.bak");
  if (!SD.rename("/weather.csv", "/weather.csv.bak")) {
    SD.remove("/weather.csv.tmp");
    Serial.println("SD: archive backup rename failed; original retained");
    return;
  }
  if (!SD.rename("/weather.csv.tmp", "/weather.csv")) {
    SD.rename("/weather.csv.bak", "/weather.csv");
    Serial.println("SD: compacted archive install failed; original restored");
    return;
  }
  SD.remove("/weather.csv.bak");
  Serial.printf("SD: archive compacted; retained %lu records from last 30 days\n", (unsigned long)kept);
}

void sdBeginAndRestoreHistory() {
  pinMode(PIN_EPD_CS, OUTPUT);
  digitalWrite(PIN_EPD_CS, HIGH);
  pinMode(PIN_SD_CS, OUTPUT);
  digitalWrite(PIN_SD_CS, HIGH);

  if (!SD.begin(PIN_SD_CS, SPI, SD_SPI_HZ)) {
    sdReady = false;
    Serial.println("SD: mount failed; continue with Preferences only");
    return;
  }
  sdReady = true;
  Serial.printf("SD: mounted, size=%llu MB\n", SD.cardSize() / (1024ULL * 1024ULL));
  sdRecoverArchiveSwap();

  // Завершаем отложенную очистку архива после factory reset.
  // Флаг остаётся во Flash, если SD-карта отсутствовала при сбросе.
  if (prefs.getBool("sdreset", false)) {
    SD.remove("/weather.csv");
    SD.remove("/forecast.json");
    SD.remove("/forecast.json.tmp");
    prefs.putBool("sdreset", false);
    Serial.println("SD: user data removed after factory reset");
  }

  sdRestoreForecastBackup();

  File f = SD.open("/weather.csv", FILE_READ);
  if (!f) {
    Serial.println("SD: no weather.csv yet");
    return;
  }

  Hist restored;
  memset(&restored, 0, sizeof(restored));
  for (int i = 0; i < HN; ++i) {
    restored.t[i] = INT16_MIN;
    restored.h[i] = 255;
    restored.p[i] = 0;
  }
  Hist previous = hist;
  hist = restored;
  uint32_t restoredCount = 0;
  bool headerSkipped = false;

  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;
    if (!headerSkipped) {
      headerSkipped = true;
      if (line.startsWith("timestamp_utc")) continue;
    }

    unsigned long epoch = 0;
    float inT = NAN, inH = NAN, t = NAN, outH = NAN, p = NAN;
    unsigned int co2 = 0;
    if (sscanf(line.c_str(), "%lu,%f,%f,%f,%f,%f,%u", &epoch, &inT, &inH, &t, &outH, &p, &co2) != 7) continue;
    if (epoch < 1700000000UL || epoch > 4102444800UL) continue;
    histAdd(epoch / HSLOT_SEC, t, inH, p);
    ++restoredCount;
  }
  f.close();

  if (restoredCount > 0 && hist.magic == HMAGIC) {
    histSave();
    Serial.printf("SD: restored %lu history records\n", (unsigned long)restoredCount);
  } else {
    hist = previous; // если CSV пуст/повреждён — оставляем резерв из Preferences
    Serial.println("SD: no valid records; keeping Preferences history");
  }
}

void sdAppendWeather(uint32_t epoch, float indoorT, float indoorH, float outdoorT, float outdoorH, float pressureMmHg, uint16_t co2) {
  if (!sdReady || epoch < 1700000000UL) return; // без достоверного UTC не пишем дату
  sdCompactArchiveIfNeeded(epoch);
  bool needHeader = !SD.exists("/weather.csv");
  if (!needHeader) { File check = SD.open("/weather.csv", FILE_READ); needHeader = check && check.size() == 0; if (check) check.close(); }
  File f = SD.open("/weather.csv", FILE_APPEND);
  if (!f) {
    Serial.println("SD: cannot open weather.csv for append");
    return;
  }
  if (needHeader) f.println("timestamp_utc,indoor_temperature_c,indoor_humidity_pct,outdoor_temperature_c,outdoor_humidity_pct,pressure_mmhg,co2_ppm");
  f.print(epoch);
  f.print(','); f.print(isfinite(indoorT) ? indoorT : NAN, 2);
  f.print(','); f.print(isfinite(indoorH) ? indoorH : NAN, 1);
  f.print(','); f.print(isfinite(outdoorT) ? outdoorT : NAN, 2);
  f.print(','); f.print(isfinite(outdoorH) ? outdoorH : NAN, 1);
  f.print(','); f.print(isfinite(pressureMmHg) ? pressureMmHg : NAN, 2);
  f.print(','); f.print(co2);
  f.println();
  f.flush();
  f.close();
}

// ===================== РИСОВАНИЕ: ВСПОМОГАТЕЛЬНОЕ =====================
int textW(const char *s) {
  int16_t x1, y1; uint16_t w, h;
  display.getTextBounds(s, 0, 0, &x1, &y1, &w, &h);
  return (int)w;
}

void printRight(int xr, int y, const char *s) {
  display.setCursor(xr - textW(s), y);
  display.print(s);
}

// "Novosibirsk Oblast" -> "Novosibirsk obl" (работает на копии строки)
void shortenRegion(char *s) {
  char *p = strstr(s, "Oblast");
  if (!p) p = strstr(s, "oblast");
  if (p) { memcpy(p, "obl", 3); memmove(p + 3, p + 6, strlen(p + 6) + 1); }
}

// печать с выравниванием вправо; если текст шире maxW — обрезаем и ставим точку в конце
void printRightFit(int xr, int y, const char *s, int maxW) {
  char b[40];
  strlcpy(b, s, sizeof(b));
  int n = (int)strlen(b);
  bool cut = false;
  while (n > 3 && textW(b) > maxW) { b[--n] = 0; cut = true; }
  if (cut) b[n - 1] = '.';
  printRight(xr, y, b);
}

void printCenter(int xc, int y, const char *s) {
  display.setCursor(xc - textW(s) / 2, y);
  display.print(s);
}

// число + значок градуса + "C" (шрифт и размер уже выбраны). rad/dy — радиус и высота кружка градуса
void printTempC(int x, int y, float t, int decimals, int rad, int dy) {
  display.setCursor(x, y);
  if (isnan(t)) display.print("--"); else display.print(t, decimals);
  int cx = display.getCursorX();
  display.drawCircle(cx + rad + 2, y - dy, rad, GxEPD_BLACK);
  display.setCursor(cx + 2 * rad + 6, y);
  display.print("C");
}

// Печать так, чтобы ЛЕВЫЙ край самих букв ("чернил") оказался в inkLeft; вокруг белая кайма halo px,
// чтобы текст не сливался с тем, что нарисовано под ним (иконкой). Возвращает правый край чернил.
static int printInkHalo(int inkLeft, int y, const char *s, int halo) {
  int16_t bx, by; uint16_t bw, bh;
  display.getTextBounds(s, 0, 0, &bx, &by, &bw, &bh);
  int cx = inkLeft - bx;
  display.setTextColor(GxEPD_WHITE);
  for (int dy = -halo; dy <= halo; dy++)
    for (int dx = -halo; dx <= halo; dx++) {
      if (dx == 0 && dy == 0) continue;
      if (dx * dx + dy * dy > halo * halo + 1) continue;
      display.setCursor(cx + dx, y + dy);
      display.print(s);
    }
  display.setTextColor(GxEPD_BLACK);
  display.setCursor(cx, y);
  display.print(s);
  return inkLeft + (int)bw;
}

// Крупная наружная температура справа от иконки (без наложения). Целая часть крупно (размер 2),
// десятые мелко вплотную, потом "°C". cy — вертикальный центр цифр, inkLeft — левый край цифр.
void drawBigTemp(int inkLeft, int cy, float t) {
  display.setFont(&FreeSansBold24pt7b);
  display.setTextSize(2);
  int16_t bx, by; uint16_t bw, bh;
  display.getTextBounds("0", 0, 0, &bx, &by, &bw, &bh);
  int y = cy - by - (int)bh / 2;
  int top = y + by;
  if (isnan(t)) {
    printInkHalo(inkLeft, y, "--", 0);
    display.setTextSize(1);
    return;
  }
  int tenths = (int)lroundf(t * 10.0f);
  bool neg = tenths < 0;
  int a = abs(tenths);
  char ip[8];
  snprintf(ip, sizeof(ip), "%s%d", neg ? "-" : "", a / 10);
  int xr = printInkHalo(inkLeft, y, ip, 0);
  display.setTextSize(1);
  display.setFont(&FreeSansBold18pt7b);
  char fp[3] = { '.', (char)('0' + a % 10), 0 };
  xr = printInkHalo(xr + 4, y, fp, 0);
  int ccx = xr + 10, ccy = top + 8;                         // значок градуса
  display.drawCircle(ccx, ccy, 6, GxEPD_BLACK);
  display.drawCircle(ccx, ccy, 5, GxEPD_BLACK);
  display.setFont(&FreeSans18pt7b);
  printInkHalo(xr + 22, top + 32, "C", 0);
}

void cell(int x, int y, const char *label, const char *val) {
  display.setFont(&FreeSans9pt7b);
  display.setCursor(x, y);
  display.print(label);
  display.setFont(&FreeSansBold12pt7b);
  display.setCursor(x, y + 22);
  display.print(val);
}

// Давление: значение тем же обычным шрифтом, что и подпись Pressure.
void cellPressure(int x, int y, int ic, const char *val) {
  drawSmallIcon(ic, x + 11, y + 9);

  // Pressure — обычный шрифт
  display.setFont(&FreeSans9pt7b);
  display.setCursor(x + 36, y);
  display.print("Pressure");

  // Отделяем число от mmHg
  char number[16];
  const char *unit = strstr(val, " mmHg");
  size_t len = unit ? (size_t)(unit - val) : strlen(val);
  if (len >= sizeof(number)) len = sizeof(number) - 1;
  memcpy(number, val, len);
  number[len] = '\0';

  // Число — жирный шрифт
  display.setFont(&FreeSansBold12pt7b);

  int16_t bx, by;
  uint16_t bw, bh;
  display.getTextBounds(number, x + 36, y + 22, &bx, &by, &bw, &bh);

  display.setCursor(x + 36, y + 22);
  display.print(number);

  // mmHg — обычный шрифт
  display.setFont(&FreeSans9pt7b);
  display.setCursor(x + 36 + bw + 4, y + 22);
  display.print("mmHg");
}

void cellTemp(int x, int y, const char *label, float t) {
  display.setFont(&FreeSans9pt7b);
  display.setCursor(x, y);
  display.print(label);
  display.setFont(&FreeSansBold12pt7b);
  printTempC(x, y + 22, t, 1, 3, 14);
}

// ===================== ИКОНКИ (контурные, "полые" внутри) =====================
// Все иконки рисуются контуром толщиной th пикселей. Облако — объединение скруглённого
// прямоугольника и трёх кругов: для каждого пикселя считаем "глубину" внутри фигуры;
// у границы (глубина < th) — чёрный, глубже — белый (заодно закрывает то, что за облаком).
static float depthCircle(float px, float py, float cx, float cy, float r) {
  float dx = px - cx, dy = py - cy;
  return r - sqrtf(dx * dx + dy * dy);
}

static float depthRRect(float px, float py, float cx, float cy, float hx, float hy, float rr) {
  float qx = fabsf(px - cx) - (hx - rr), qy = fabsf(py - cy) - (hy - rr);
  float ox = fmaxf(qx, 0.0f), oy = fmaxf(qy, 0.0f);
  return -(sqrtf(ox * ox + oy * oy) + fminf(fmaxf(qx, qy), 0.0f) - rr);
}

static void uiLine(int x1, int y1, int x2, int y2, int th) {
  display.drawLine(x1, y1, x2, y2, GxEPD_BLACK);
  if (th >= 2) {
    display.drawLine(x1 + 1, y1, x2 + 1, y2, GxEPD_BLACK);
    display.drawLine(x1, y1 + 1, x2, y2 + 1, GxEPD_BLACK);
  }
  if (th >= 3) {
    display.drawLine(x1 + 1, y1 + 1, x2 + 1, y2 + 1, GxEPD_BLACK);
    display.drawLine(x1 - 1, y1, x2 - 1, y2, GxEPD_BLACK);
  }
}

static void uiRing(int cx, int cy, int r, int th) {
  for (int k = 0; k < th; k++) display.drawCircle(cx, cy, r - k, GxEPD_BLACK);
}

// дуга окружности от угла a0 до a1 (градусы, по часовой от направления вправо; ось Y вниз)
static void uiArc(int cx, int cy, float r, int a0, int a1, int th) {
  for (int a = a0; a <= a1; a += 2) {
    float rad = a * 0.0174533f;
    for (int k = 0; k < th; k++) {
      float rr = r - k;
      display.drawPixel(cx + (int)lroundf(rr * cosf(rad)), cy + (int)lroundf(rr * sinf(rad)), GxEPD_BLACK);
    }
  }
}

// облако: (cx, cy) — центр "якоря", s — масштаб
static void cloudShape(int cx, int cy, float s, float th) {
  int x0 = (int)(cx - 1.25f * s), x1 = (int)(cx + 1.65f * s);
  int y0 = (int)(cy - 1.15f * s), y1 = (int)(cy + 0.90f * s);
  float hh = 0.4f * s;
  for (int y = y0; y <= y1; y++) {
    for (int x = x0; x <= x1; x++) {
      float px = x + 0.5f, py = y + 0.5f;
      float d = depthRRect(px, py, (float)cx, cy + hh, s, hh, hh);
      float d2 = depthCircle(px, py, cx - 0.5f * s, (float)cy, 0.65f * s);        if (d2 > d) d = d2;
      d2 = depthCircle(px, py, cx + s / 6.0f, cy - 0.25f * s, 0.80f * s);         if (d2 > d) d = d2;
      d2 = depthCircle(px, py, (float)cx + s, (float)cy, 0.55f * s);              if (d2 > d) d = d2;
      if (d > 0.0f) display.drawPixel(x, y, d < th ? GxEPD_BLACK : GxEPD_WHITE);
    }
  }
}

// облако, нарисованное так, чтобы его ВИДИМЫЙ центр оказался в (cx, cy)
static void cloudAt(int cx, int cy, float s, float th) {
  cloudShape(cx - (int)lroundf(0.2f * s), cy + (int)lroundf(0.125f * s), s, th);
}

static void sunShape(int cx, int cy, float r, int th) {
  uiRing(cx, cy, (int)lroundf(r), th);
  for (int a = 0; a < 360; a += 45) {
    float rad = a * 0.0174533f;
    int x1 = cx + (int)lroundf(cosf(rad) * r * 1.55f), y1 = cy + (int)lroundf(sinf(rad) * r * 1.55f);
    int x2 = cx + (int)lroundf(cosf(rad) * r * 2.35f), y2 = cy + (int)lroundf(sinf(rad) * r * 2.35f);
    uiLine(x1, y1, x2, y2, th);
  }
}

static void moonShape(int cx, int cy, float r, float th) {
  float bx = cx + r * 0.5f, by = cy - r / 3.0f, br = 0.85f * r;
  int R = (int)ceilf(r) + 1;
  for (int y = cy - R; y <= cy + R; y++) {
    for (int x = cx - R; x <= cx + R; x++) {
      float px = x + 0.5f, py = y + 0.5f;
      float dA = sqrtf((px - cx) * (px - cx) + (py - cy) * (py - cy));
      float dB = sqrtf((px - bx) * (px - bx) + (py - by) * (py - by));
      if (dA <= r && dB >= br && ((r - dA) < th || (dB - br) < th)) display.drawPixel(x, y, GxEPD_BLACK);
    }
  }
}

// Универсальная иконка погоды. s — масштаб (прогноз: 14, большая иконка слева: 30).
void drawIconKind(int cx, int cy, float s, IconKind k, bool night) {
  float th = (s >= 36.0f) ? 4.0f : ((s >= 20.0f) ? 3.0f : ((s >= 16.0f) ? 2.5f : 2.0f));
  int ith = (int)th;
  switch (k) {
    case K_SUN:
      if (night) moonShape(cx, cy, s * 0.95f, th); else sunShape(cx, cy, s * 0.55f, ith);
      break;
    case K_PART: {
      int sx = cx - (int)lroundf(0.55f * s), sy = cy - (int)lroundf(0.50f * s);
      if (night) moonShape(sx, sy, s * 0.62f, th); else sunShape(sx, sy, s * 0.42f, ith);
      cloudAt(cx + (int)lroundf(0.25f * s), cy + (int)lroundf(0.20f * s), 0.8f * s, th);
      break;
    }
    case K_CLOUD:
      if (night) {
        int sx = cx - (int)lroundf(0.55f * s);
        int sy = cy - (int)lroundf(0.50f * s);

    // Сначала рисуем луну
        moonShape(
          sx,
          sy,
          s * 0.70f,
          th
        );

    // Затем облако — оно частично перекрывает луну
        cloudAt(
          cx + (int)lroundf(0.20f * s),
          cy + (int)lroundf(0.20f * s),
          0.80f * s,
          th
        );
      } else {
        cloudAt(cx, cy, s, th);
      }
      break;
    case K_RAIN:
      cloudAt(cx, cy - (int)lroundf(0.35f * s), 0.85f * s, th);
      for (int j = -1; j <= 1; j++) {
        int x = cx + (int)lroundf(j * 0.55f * s);
        uiLine(x, cy + (int)lroundf(0.55f * s), x - (int)lroundf(0.2f * s), cy + (int)lroundf(1.0f * s), ith);
      }
      break;
    case K_SNOW: {
      cloudAt(cx, cy - (int)lroundf(0.35f * s), 0.85f * s, th);
      int rr = (int)lroundf(0.14f * s); if (rr < 2) rr = 2;
      for (int j = -1; j <= 1; j++) display.drawCircle(cx + (int)lroundf(j * 0.55f * s), cy + (int)lroundf(0.8f * s), rr, GxEPD_BLACK);
      break;
    }
    case K_STORM: {
      cloudAt(cx, cy - (int)lroundf(0.35f * s), 0.85f * s, th);
      int bx = cx, by = cy + (int)lroundf(0.5f * s);
      uiLine(bx + (int)lroundf(0.15f * s), by, bx - (int)lroundf(0.2f * s), by + (int)lroundf(0.4f * s), ith);
      uiLine(bx - (int)lroundf(0.2f * s), by + (int)lroundf(0.4f * s), bx + (int)lroundf(0.2f * s), by + (int)lroundf(0.4f * s), ith);
      uiLine(bx + (int)lroundf(0.2f * s), by + (int)lroundf(0.4f * s), bx - (int)lroundf(0.15f * s), by + (int)lroundf(0.9f * s), ith);
      break;
    }
    case K_FOG:
      for (int i = 0; i < 3; i++) {
        int y = cy - (int)lroundf(0.5f * s) + (int)lroundf(i * 0.5f * s);
        int w = (int)lroundf((1.2f - 0.3f * i) * s);
        uiLine(cx - w, y, cx + w, y, ith);
      }
      break;
    default:
      uiRing(cx, cy, (int)lroundf(0.8f * s), ith);
      break;
  }
}

IconKind wmoKind(int code) {
  if (code == 0) return K_SUN;
  if (code <= 2) return K_PART;
  if (code == 3) return K_CLOUD;
  if (code == 45 || code == 48) return K_FOG;
  if ((code >= 51 && code <= 67) || (code >= 80 && code <= 82)) return K_RAIN;
  if ((code >= 71 && code <= 77) || code == 85 || code == 86) return K_SNOW;
  if (code >= 95) return K_STORM;
  return K_CLOUD;
}

// Большая иконка по тренду давления (слева вверху). (x, y) — левый верхний угол области ~150x140
void drawWeatherIcon(int x, int y, Trend trend, bool night, int wmo) {
  int cx = x + 77, cy = y + 71;
  if (wmo >= 0) {                                    // есть свежие данные из интернета — текущая погода
    drawIconKind(cx, cy, 48.0f, wmoKind(wmo), night);
    return;
  }
  switch (trend) {
    case TREND_IMPROVING: drawIconKind(cx, cy, 48.0f, K_SUN, night);  break;
    case TREND_STABLE:    drawIconKind(cx, cy, 48.0f, K_PART, night); break;
    case TREND_WORSENING: drawIconKind(cx, cy, 48.0f, K_RAIN, night); break;
    default: {                                         // пока нет данных: просто большой "?" по центру
      display.setFont(&FreeSansBold24pt7b);
      display.setTextSize(3);
      int16_t bx, by; uint16_t bw, bh;
      display.getTextBounds("?", 0, 0, &bx, &by, &bw, &bh);
      display.setCursor(cx - (int)bw / 2 - bx, cy - (int)bh / 2 - by);
      display.print("?");
      display.setTextSize(1);
      break;
    }
  }
}

// Маленькая иконка по коду погоды WMO (прогноз на 5 дней), центр (cx, cy)
void drawWmo(int cx, int cy, int code) {
  drawIconKind(cx, cy, 18.0f, wmoKind(code), false);
}

// ---- маленькие иконки для сетки показателей (~24x24, центр (cx, cy))
#define IC_SUNRISE 0
#define IC_SUNSET  1
#define IC_DROP    2
#define IC_GAUGE   3
#define IC_THERMO  4
#define IC_HOUSE   5
#define IC_CO2     6
#define IC_GPS     7
#define IC_SIGNAL  8
#define IC_CLOCK   9
#define IC_GLOBE   10
#define IC_OUTDOOR 11

static void uiEllipse(int cx, int cy, float rx, float ry) {
  for (int a = 0; a < 360; a += 3) {
    float rad = a * 0.0174533f;
    display.drawPixel(cx + (int)lroundf(rx * cosf(rad)), cy + (int)lroundf(ry * sinf(rad)), GxEPD_BLACK);
  }
}

static void icDrop(int cx, int cy, float r, int th) {
  float yc = cy + r * 0.667f;
  uiArc(cx, (int)lroundf(yc), r, -27, 207, th);
  int ax = cx, ay = (int)lroundf(cy - r * 1.5f);
  int tx = (int)lroundf(r * 0.887f), ty = (int)lroundf(yc - r * 0.462f);
  uiLine(ax, ay, cx + tx, ty, th);
  uiLine(ax, ay, cx - tx, ty, th);
}

static void icSun(int cx, int cy, bool rise) {
  display.drawFastHLine(cx - 11, cy + 7, 23, GxEPD_BLACK);
  display.drawFastHLine(cx - 11, cy + 8, 23, GxEPD_BLACK);
  uiRing(cx, cy + 7, 6, 2);
  display.fillRect(cx - 8, cy + 9, 17, 6, GxEPD_WHITE);              // стираем нижнюю половину круга
  uiLine(cx - 6, cy + 1, cx - 9, cy - 2, 2);                          // лучи
  uiLine(cx + 6, cy + 1, cx + 9, cy - 2, 2);
  uiLine(cx, cy - 8, cx, cy - 1, 2);                                  // стрелка
  if (rise) { uiLine(cx, cy - 9, cx - 3, cy - 6, 2); uiLine(cx, cy - 9, cx + 3, cy - 6, 2); }
  else      { uiLine(cx, cy - 1, cx - 3, cy - 4, 2); uiLine(cx, cy - 1, cx + 3, cy - 4, 2); }
}

static void drawSmallIcon(int id, int cx, int cy) {
  switch (id) {
    case IC_SUNRISE: icSun(cx, cy, true);  break;
    case IC_SUNSET:  icSun(cx, cy, false); break;
    case IC_DROP:    icDrop(cx, cy, 6.0f, 2); break;
    case IC_GAUGE:
      uiRing(cx, cy, 10, 2);
      uiLine(cx, cy, cx + 5, cy - 6, 2);
      display.fillCircle(cx, cy, 2, GxEPD_BLACK);
      for (int a = -150; a <= -30; a += 60) {
        float rad = a * 0.0174533f;
        display.drawLine(cx + (int)lroundf(6.0f * cosf(rad)), cy + (int)lroundf(6.0f * sinf(rad)),
                         cx + (int)lroundf(8.0f * cosf(rad)), cy + (int)lroundf(8.0f * sinf(rad)), GxEPD_BLACK);
      }
      break;
    case IC_THERMO:
      display.drawRoundRect(cx - 3, cy - 11, 7, 15, 3, GxEPD_BLACK);
      display.drawRoundRect(cx - 4, cy - 12, 9, 17, 4, GxEPD_BLACK);
      uiRing(cx, cy + 8, 5, 2);
      display.drawFastHLine(cx - 2, cy + 4, 5, GxEPD_WHITE);
      display.drawFastHLine(cx - 2, cy + 5, 5, GxEPD_WHITE);
      display.fillRect(cx - 1, cy - 5, 3, 13, GxEPD_BLACK);
      display.fillCircle(cx, cy + 8, 2, GxEPD_BLACK);
      break;
    case IC_HOUSE:
      uiLine(cx - 11, cy - 1, cx, cy - 11, 2);
      uiLine(cx, cy - 11, cx + 11, cy - 1, 2);
      display.drawRect(cx - 8, cy - 1, 17, 12, GxEPD_BLACK);
      display.drawRect(cx - 7, cy, 15, 10, GxEPD_BLACK);
      icDrop(cx, cy + 5, 3.0f, 1);
      break;
    case IC_CO2:
      cloudAt(cx, cy - 2, 6.0f, 2.0f);
      display.drawCircle(cx - 6, cy + 9, 1, GxEPD_BLACK);
      display.drawCircle(cx, cy + 10, 1, GxEPD_BLACK);
      display.drawCircle(cx + 6, cy + 9, 1, GxEPD_BLACK);
      break;
    case IC_GPS:
      uiRing(cx, cy, 7, 2);
      display.fillCircle(cx, cy, 2, GxEPD_BLACK);
      uiLine(cx - 11, cy, cx - 6, cy, 2);
      uiLine(cx + 6, cy, cx + 11, cy, 2);
      uiLine(cx, cy - 11, cx, cy - 6, 2);
      uiLine(cx, cy + 6, cx, cy + 11, 2);
      break;
    case IC_SIGNAL:
      display.fillCircle(cx, cy + 7, 2, GxEPD_BLACK);
      uiArc(cx, cy + 7, 6.0f, -135, -45, 2);
      uiArc(cx, cy + 7, 10.0f, -135, -45, 2);
      uiArc(cx, cy + 7, 14.0f, -135, -45, 2);
      break;
    case IC_CLOCK:
      uiRing(cx, cy, 10, 2);
      uiLine(cx, cy, cx, cy - 6, 2);
      uiLine(cx, cy, cx + 4, cy + 2, 2);
      break;
    case IC_GLOBE:                                    // значок "интернет"
      uiRing(cx, cy, 10, 2);
      display.drawFastHLine(cx - 10, cy, 21, GxEPD_BLACK);
      display.drawFastVLine(cx, cy - 10, 21, GxEPD_BLACK);
      uiEllipse(cx, cy, 5.0f, 10.0f);
      break;
    case IC_OUTDOOR:                                  // значок "наружный модуль на связи"
      drawSmallIcon(IC_THERMO, cx - 6, cy);
      uiArc(cx - 1, cy, 7.0f, -50, 50, 2);
      uiArc(cx - 1, cy, 12.0f, -50, 50, 2);
      break;
    default: break;
  }
}

void drawOutdoorStateIcon(int cx, int cy, bool forceTest = false) {

  drawSmallIcon(IC_OUTDOOR, cx, cy);

  if (!forceTest && outdoorState == OUT_NOT_WAITING) {
    // Перечёркнутый значок — нет связи с наружным модулем: белая кайма + жирная чёрная линия (~3 px)
    for (int o = -2; o <= 2; o++) display.drawLine(cx - 10 + o, cy - 12, cx + 10 + o, cy + 12, GxEPD_WHITE);
    for (int o = -1; o <= 1; o++) display.drawLine(cx - 10 + o, cy - 12, cx + 10 + o, cy + 12, GxEPD_BLACK);
  }

  else if (!forceTest && outdoorState == OUT_WAITING) {
    // Маленькие часы — ожидаем подключения
    display.drawCircle(
      cx + 10, cy + 9, 5,
      GxEPD_BLACK
    );

    display.drawLine(
      cx + 10, cy + 9,
      cx + 10, cy + 6,
      GxEPD_BLACK
    );

    display.drawLine(
      cx + 10, cy + 9,
      cx + 13, cy + 9,
      GxEPD_BLACK
    );
  }

  // OUT_CONNECTED — обычный значок без дополнительных отметок
}

// Толстая линия для маленьких погодных пиктограмм.
void drawAlertBoldLine(int x1, int y1, int x2, int y2) {
  int ox = 0, oy = 0;
  if (abs(y2 - y1) > abs(x2 - x1)) ox = 1;
  else oy = 1;
  display.drawLine(x1, y1, x2, y2, GxEPD_BLACK);
  display.drawLine(x1 + ox, y1 + oy, x2 + ox, y2 + oy, GxEPD_BLACK);
  display.drawLine(x1 - ox, y1 - oy, x2 - ox, y2 - oy, GxEPD_BLACK);
}

// Расстояние от точки до отрезка (для рисования значков с ровной толщиной под любым углом).
static float uiSegDist(float px, float py, float x1, float y1, float x2, float y2) {
  float dx = x2 - x1, dy = y2 - y1;
  float t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy);
  if (t < 0.0f) t = 0.0f; else if (t > 1.0f) t = 1.0f;
  float qx = x1 + t * dx - px, qy = y1 + t * dy - py;
  return sqrtf(qx * qx + qy * qy);
}

// Снежинка ~23x23: шесть лучей толщиной 2 px и маленькие "ёлочки" у концов лучей.
// Рисуется попиксельно по расстоянию до отрезков, поэтому все шесть лучей одинаковы.
void drawSnowflakeIcon(int cx, int cy) {
  const float R  = 11.0f;                  // длина луча
  const float D  = 7.8f;                   // где на луче начинается веточка
  const float L  = 3.5f;                   // длина веточки
  const float HW = 1.0f;                   // половина толщины линии (итого 2 px)
  const float BA = 50.0f * 0.0174533f;     // угол веточки к лучу

  float x1[18], y1[18], x2[18], y2[18];
  int n = 0;
  for (int arm = 0; arm < 6; ++arm) {
    const float t  = -1.5707963f + arm * 1.0471976f;
    const float ux = cosf(t), uy = sinf(t);
    x1[n] = 0; y1[n] = 0; x2[n] = ux * R; y2[n] = uy * R; ++n;
    const float bx = ux * D, by = uy * D;
    for (int side = -1; side <= 1; side += 2) {
      const float ba = t + side * BA;
      x1[n] = bx; y1[n] = by; x2[n] = bx + cosf(ba) * L; y2[n] = by + sinf(ba) * L; ++n;
    }
  }
  for (int y = -13; y <= 13; ++y) {
    for (int x = -13; x <= 13; ++x) {
      const float px = x + 0.5f, py = y + 0.5f;
      for (int i = 0; i < n; ++i) {
        if (uiSegDist(px, py, x1[i], y1[i], x2[i], y2[i]) <= HW) {
          display.drawPixel(cx + x, cy + y, GxEPD_BLACK);
          break;
        }
      }
    }
  }
}

// Предупреждение об экстремальной температуре: треугольник с восклицательным знаком.
// Оно визуально не повторяет термометр наружного Zigbee-модуля.
void drawExtremeTempIcon(int cx, int cy, TempAlert kind) {
  drawAlertBoldLine(cx, cy - 12, cx - 11, cy + 9);
  drawAlertBoldLine(cx - 11, cy + 9, cx + 11, cy + 9);
  drawAlertBoldLine(cx + 11, cy + 9, cx, cy - 12);
  display.drawFastVLine(cx, cy - 4, 8, GxEPD_BLACK);
  display.drawFastVLine(cx + 1, cy - 4, 8, GxEPD_BLACK);

}

// Гололёд: отдельная пиктограмма ледяной поверхности с выраженной трещиной.
void drawIceIcon(int cx, int cy) {
  drawAlertBoldLine(cx - 11, cy - 5, cx - 5, cy - 11);
  drawAlertBoldLine(cx - 5, cy - 11, cx + 7, cy - 11);
  drawAlertBoldLine(cx + 7, cy - 11, cx + 11, cy - 5);
  drawAlertBoldLine(cx + 11, cy - 5, cx + 8, cy + 7);
  drawAlertBoldLine(cx + 8, cy + 7, cx - 8, cy + 7);
  drawAlertBoldLine(cx - 8, cy + 7, cx - 11, cy - 5);
  // Ломаная трещина — визуально отличает лёд от снежинки.
  drawAlertBoldLine(cx - 1, cy - 8, cx - 3, cy - 2);
  drawAlertBoldLine(cx - 3, cy - 2, cx + 3, cy + 1);
  drawAlertBoldLine(cx + 3, cy + 1, cx + 1, cy + 5);
}

// До трёх предупреждений: гололёд/ветер, снег/гроза, мороз/жара.
void drawWeatherAlerts(int firstX, int cy, bool forceTest = false) {
  const int step = 28;
  int x = firstX;
  AlertGroup1 shownGroup1 = forceTest && alertGroup1 == GROUP1_NONE ? GROUP1_ICE : alertGroup1;
  AlertGroup2 shownGroup2 = forceTest && alertGroup2 == GROUP2_NONE ? GROUP2_SNOW : alertGroup2;
  TempAlert shownTempAlert = forceTest && tempAlert == TEMP_NONE ? TEMP_FROST : tempAlert;

  if (shownGroup1 == GROUP1_ICE) {
    drawIceIcon(x, cy);
  } else if (shownGroup1 == GROUP1_WIND) {
    uiArc(x - 3, cy - 4, 8.0f, -35, 35, 2);
    display.drawFastHLine(x - 11, cy + 1, 21, GxEPD_BLACK);
    display.drawFastHLine(x - 7, cy + 6, 15, GxEPD_BLACK);
  }
  if (shownGroup1 != GROUP1_NONE) x += step;

  if (shownGroup2 == GROUP2_SNOW) {
    drawSnowflakeIcon(x, cy);
  } else if (shownGroup2 == GROUP2_STORM) {
    drawAlertBoldLine(x + 2, cy - 12, x - 5, cy + 1);
    drawAlertBoldLine(x - 5, cy + 1, x + 1, cy + 1);
    drawAlertBoldLine(x + 1, cy + 1, x - 2, cy + 12);
  }
  if (shownGroup2 != GROUP2_NONE) x += step;

  if (shownTempAlert != TEMP_NONE) {
    drawExtremeTempIcon(x, cy, shownTempAlert);
  }
}

// ячейка сетки с иконкой слева
void cellI(int x, int y, int ic, const char *label, const char *val) {
  drawSmallIcon(ic, x + 11, y + 9);
  cell(x + 36, y, label, val);
}

void cellITemp(int x, int y, int ic, const char *label, float t) {
  drawSmallIcon(ic, x + 11, y + 9);
  cellTemp(x + 36, y, label, t);
}

// "макс°|мин°" одной строкой по центру cx (пропорциональный шрифт, градус — кружком)
void printTempPair(int cx, int y, int hi, int lo) {
  char a[8], b[8];
  snprintf(a, sizeof(a), "%d", hi);
  snprintf(b, sizeof(b), "%d", lo);
  display.setFont(&FreeSans9pt7b);
  const int deg = 11, gap = 3;
  int wa = textW(a), wb = textW(b), wbar = textW("|");
  int total = wa + deg + gap + wbar + gap + wb + deg;
  int x = cx - total / 2;
  display.setCursor(x, y);                 display.print(a);
  display.drawCircle(x + wa + 6, y - 9, 2, GxEPD_BLACK);
  int xbar = x + wa + deg + gap;
  display.setCursor(xbar, y);              display.print("|");
  int xb = xbar + wbar + gap;
  display.setCursor(xb, y);                display.print(b);
  display.drawCircle(xb + wb + 6, y - 9, 2, GxEPD_BLACK);
}

// ===================== ГРАФИК ЗА 24 ЧАСА =====================
void ditherBar(int x, int y0, int w, int h) {   // "серый" столбик шахматкой; y0 — нижняя граница
  for (int yy = y0 - h; yy < y0; yy++)
    for (int xx = x; xx < x + w; xx++)
      if (((xx + yy) & 1) == 0) display.drawPixel(xx, yy, GxEPD_BLACK);
}

void grayVLine(int x, int y, int h) {   // серая вертикальная линия шахматкой
  for (int i = 0; i < h; i++)
    if (((x + y + i) & 1) == 1) display.drawPixel(x, y + i, GxEPD_BLACK);
}

void drawGraph(int gx0, int gy0, int gx1, int gy1) {
  uint32_t e = epochNow();
  uint32_t nowSlot = e ? e / HSLOT_SEC : hist.headSlot;

  float tv[HN], pv[HN];
  int   hv[HN];
  bool anyData = false;
  for (int i = 0; i < HN; i++) {
    tv[i] = NAN; pv[i] = NAN; hv[i] = -1;
    if (hist.magic != HMAGIC) continue;
    int32_t s = (int32_t)nowSlot - (HN - 1 - i);
    int32_t k = (int32_t)hist.headSlot - s;
    if (k < 0 || k >= HN) continue;
    int idx = ((int)hist.headIdx - (int)k + HN) % HN;
    if (hist.t[idx] != INT16_MIN) { tv[i] = hist.t[idx] / 10.0f; anyData = true; }
    if (hist.h[idx] != 255)       { hv[i] = hist.h[idx] / 2; anyData = true; }
    if (hist.p[idx])              { pv[i] = hist.p[idx] / 10.0f; anyData = true; }
  }

  auto X = [&](int i) { return gx0 + (int)((long)i * (gx1 - gx0) / (HN - 1)); };

  // --- шкала температуры
  float tmn = 1e9f, tmx = -1e9f;
  for (int i = 0; i < HN; i++) if (!isnan(tv[i])) { tmn = fminf(tmn, tv[i]); tmx = fmaxf(tmx, tv[i]); }
  bool haveT = tmn <= tmx;
  float rtmn = tmn, rtmx = tmx;                         // реальный диапазон для легенды
  // Отметки Error Bars тоже входят в шкалу, иначе они выходят за рамки графика.
  if (errorBars.magic == EBMAGIC) {
    for (int j = 0; j < errorBars.count; ++j) {
      const ErrorBar &eb = errorBars.items[j];
      if (!eb.epoch) continue;
      int32_t ageS = (int32_t)nowSlot - (int32_t)(eb.epoch / HSLOT_SEC);
      if (ageS < 0 || ageS >= HN) continue;
      const float e10 = fabsf((float)eb.forecast10 - (float)eb.actual10);
      tmn = fminf(tmn, ((float)eb.actual10 - e10) / 10.0f);
      tmx = fmaxf(tmx, ((float)eb.actual10 + e10) / 10.0f);
    }
  }
  int hmn = 1000, hmx = -1;
  for (int i = 0; i < HN; i++) if (hv[i] >= 0) { if (hv[i] < hmn) hmn = hv[i]; if (hv[i] > hmx) hmx = hv[i]; }
  bool haveHum = hmx >= 0;
  if (tmn > tmx) { tmn = 0; tmx = 10; }
  int step = (tmx - tmn > 30.0f) ? 10 : 5;
  int lo = (int)floorf(tmn / step) * step;
  int hi = (int)ceilf(tmx / step) * step;
  // Нулевая отметка всегда входит в шкалу. Временная ось Ox остаётся внизу,
  // а отрицательные значения подписываются на Oy и располагаются выше -30, но ниже -15.
  if (lo > 0) lo = 0;
  if (hi < 0) hi = 0;
  if (hi - lo < 2 * step) {
    if (hi == 0) lo = hi - 2 * step;
    else hi = lo + 2 * step;
  }
  auto YT = [&](float v) { return gy1 - (int)((v - lo) * (gy1 - gy0) / (float)(hi - lo)); };

  // --- сетка и подписи слева (температура)
  display.setFont(&FreeSans9pt7b);
  char buf[24];
  for (int v = lo; v <= hi; v += step) {
    int y = YT(v);
    for (int x = gx0; x < gx1; x += 6) display.drawFastHLine(x, y, 2, GxEPD_BLACK);
    snprintf(buf, sizeof(buf), "%d", v);
    printRight(gx0 - 13, y + 4, buf);
    display.drawCircle(gx0 - 9, y - 3, 2, GxEPD_BLACK);               // знак градуса
  }
  // --- подписи справа (влажность, 0-100 %)
  display.setCursor(gx1 + 6, gy1 + 4);              display.print("0%");
  display.setCursor(gx1 + 6, (gy0 + gy1) / 2 + 4);  display.print("50%");
  display.setCursor(gx1 + 6, gy0 + 4);              display.print("100%");

  // --- столбики влажности дома
  int bw = (gx1 - gx0) / (HN - 1) - 1;
  if (bw < 1) bw = 1;
  for (int i = 0; i < HN; i++)
    if (hv[i] >= 0) ditherBar(X(i) - bw / 2, gy1, bw, hv[i] * (gy1 - gy0) / 100);

  // --- давление: пунктир, своя шкала (диапазон не уже 6 мм рт. ст.)
  float pmn = 1e9f, pmx = -1e9f;
  for (int i = 0; i < HN; i++) if (!isnan(pv[i])) { pmn = fminf(pmn, pv[i]); pmx = fmaxf(pmx, pv[i]); }
  bool havePress = pmn <= pmx;
  if (havePress) {
    float pc = (pmn + pmx) / 2.0f;
    float span = fmaxf(pmx - pmn, 6.0f);
    float plo = pc - span * 0.55f, phi = pc + span * 0.55f;
    auto YP = [&](float v) { return gy1 - (int)((v - plo) * (gy1 - gy0) / (phi - plo)); };
    for (int i = 1; i < HN; i++)
      if (!isnan(pv[i - 1]) && !isnan(pv[i]) && (i % 2 == 0))
        display.drawLine(X(i - 1), YP(pv[i - 1]), X(i), YP(pv[i]), GxEPD_BLACK);
  }

  // --- Error Bars: вертикальный отрезок от старого прогноза до факта.
  // Отображаем только точки, попадающие в последние 24 часа графика.
  if (errorBars.magic == EBMAGIC) {
    for (int j = 0; j < errorBars.count; ++j) {
      const ErrorBar &b = errorBars.items[j];
      if (!b.epoch || b.epoch / HSLOT_SEC > nowSlot) continue;
      int32_t ageSlots = (int32_t)nowSlot - (int32_t)(b.epoch / HSLOT_SEC);
      if (ageSlots < 0 || ageSlots >= HN) continue;
      int xi = HN - 1 - ageSlots;
      int x = X(xi);
      const float e = fabsf((float)b.forecast10 - (float)b.actual10) / 10.0f;
      const float a = b.actual10 / 10.0f;
      const int yc0 = YT(a);                                  // факт без ограничения рамкой
      int halfPx = abs(YT(a + e) - yc0);                      // реальная ошибка в пикселях
      if (halfPx < 5) halfPx = 5;                           // минимум 10 px в каждую сторону
      int yc  = constrain(yc0,          gy0, gy1);            // факт (центр)
      int yHi = constrain(yc0 - halfPx, gy0, gy1);            // верхний ус
      int yLo = constrain(yc0 + halfPx, gy0, gy1);                       // факт - ошибка
      int yTop = min(yHi, yLo), hh = abs(yLo - yHi) + 1;
      grayVLine(x, yTop, hh);
      grayVLine(x + 1, yTop, hh);
      display.drawFastHLine(x - 3, yHi, 8, GxEPD_BLACK);
      display.drawFastHLine(x - 3, yLo, 8, GxEPD_BLACK);
      display.fillRect(x - 1, yc - 1, 4, 3, GxEPD_BLACK);     // центр: факт   // метка факта
    }
  }

  // --- наружная температура: сплошная линия (2 px) + точка на последнем значении
  int lastI = -1;
  for (int i = 0; i < HN; i++) {
    if (isnan(tv[i])) continue;
    lastI = i;
    if (i > 0 && !isnan(tv[i - 1])) {
      display.drawLine(X(i - 1), YT(tv[i - 1]),     X(i), YT(tv[i]),     GxEPD_BLACK);
      display.drawLine(X(i - 1), YT(tv[i - 1]) + 1, X(i), YT(tv[i]) + 1, GxEPD_BLACK);
    }
  }
  if (lastI >= 0) display.fillCircle(X(lastI), YT(tv[lastI]), 3, GxEPD_BLACK);

  // --- ось времени
  display.drawFastHLine(gx0, gy1, gx1 - gx0, GxEPD_BLACK);
  if (e) {
    for (int i = 0; i < HN; i++) {
      uint32_t ce = (nowSlot - (HN - 1 - i)) * HSLOT_SEC;
      int64_t le = (int64_t)ce + (int64_t)cfg.tz * 3600;
      int hr = (int)((le / 3600) % 24);
      int mn = (int)((le / 60) % 60);
      if (mn == 0 && hr % 4 == 0) {
        display.drawFastVLine(X(i), gy1, 5, GxEPD_BLACK);
        snprintf(buf, sizeof(buf), "%02d", hr);
        printCenter(X(i), gy1 + 19, buf);
      }
    }
  }
  if (!anyData) {
    printCenter((gx0 + gx1) / 2, (gy0 + gy1) / 2, e ? "Collecting history..." : "No time source yet");
  }

  // --- легенда: у каждой линии свой диапазон значений за 24 часа
  // мелкий встроенный шрифт 6x8 — легенда почти незаметна
  display.setFont(NULL);
  display.setTextSize(1);
  int ly = gy1 + 44;                                    // нижняя линия текста; у встроенного шрифта курсор — верх символа
  int lx = gx0 - 30;
  display.drawFastHLine(lx, ly - 4, 12, GxEPD_BLACK);
  if (haveT) snprintf(buf, sizeof(buf), "Out %d..%d", (int)lroundf(rtmn), (int)lroundf(rtmx));
  else       snprintf(buf, sizeof(buf), "Out temp C");
  display.setCursor(lx + 16, ly - 7); display.print(buf);
  lx += 16 + textW(buf) + 12;
  ditherBar(lx, ly - 1, 10, 8);
  if (haveHum) snprintf(buf, sizeof(buf), "In %d-%d%%", hmn, hmx);
  else         snprintf(buf, sizeof(buf), "In hum %%");
  display.setCursor(lx + 14, ly - 7); display.print(buf);
  lx += 14 + textW(buf) + 12;
  for (int k = 0; k < 12; k += 4) display.drawFastHLine(lx + k, ly - 4, 2, GxEPD_BLACK);
  if (havePress) snprintf(buf, sizeof(buf), "mmHg %d-%d", (int)lroundf(pmn), (int)lroundf(pmx));
  else           snprintf(buf, sizeof(buf), "mmHg");
  display.setCursor(lx + 16, ly - 7); display.print(buf);
}

// ===================== ГЛАВНЫЙ ЭКРАН =====================
void drawMain(float inT, float inH, float prHpa) {
  LT lt = localNow();

  // --- прогноз: берём дни начиная с сегодняшнего
  int fcFirst = 0, fcCnt = 0;
  if (fc.magic == FCMAGIC && fc.n > 0) {
    uint32_t today = lt.haveDate ? (uint32_t)(lt.y * 10000 + lt.mo * 100 + lt.d) : 0;
    while (fcFirst < fc.n && fc.date[fcFirst] < today) fcFirst++;
    fcCnt = fc.n - fcFirst;
    if (fcCnt > 5) fcCnt = 5;
  }
  bool online = fcCnt >= FC_MIN_DAYS;

  // --- строки
  char sTime[8], sDate[40], sRise[8], sSet[8], sPress[16], sOutH[12], sInH[12], sCo2[16], sSat[8];
  snprintf(sTime, sizeof(sTime), "%02d:%02d", lt.h, lt.mi);
  if (lt.haveDate) snprintf(sDate, sizeof(sDate), "%s, %d %s", WD[lt.wd], lt.d, MONTHS[lt.mo - 1]);
  else             sDate[0] = 0;

  double la = NAN, lo = NAN;
  bool gpsFix = false;
  if (gps.location.isValid() && gps.location.age() < 60000) { la = gps.location.lat(); lo = gps.location.lng(); gpsFix = true; }
  else if (!isnan(homeLat)) { la = homeLat; lo = homeLon; }
  strcpy(sRise, "--:--"); strcpy(sSet, "--:--");
  if (lt.haveDate && !isnan(la)) {
    int m1, m2;
    if (sunEvent(true,  la, lo, lt.y, lt.mo, lt.d, cfg.tz, m1)) snprintf(sRise, sizeof(sRise), "%02d:%02d", m1 / 60, m1 % 60);
    if (sunEvent(false, la, lo, lt.y, lt.mo, lt.d, cfg.tz, m2)) snprintf(sSet,  sizeof(sSet),  "%02d:%02d", m2 / 60, m2 % 60);
  }

  if (isnan(prHpa)) strcpy(sPress, "--- mmHg");
  else              snprintf(sPress, sizeof(sPress), "%.0f mmHg", prHpa * 0.750062f);
  // наружная температура/влажность: с модуля, а если его нет на связи — из интернета (прогноз "сейчас")
  float dispT = NAN, dispH = NAN;
  bool  fromWeb = false;
  getOutdoor(dispT, dispH, fromWeb);   // датчик или интернет — по приоритету из настроек
  if (isnan(dispH)) strcpy(sOutH, "-- %"); else snprintf(sOutH, sizeof(sOutH), "%.0f %%", dispH);
  if (isnan(inH))  strcpy(sInH,  "-- %"); else snprintf(sInH,  sizeof(sInH),  "%.0f %%", inH);
  if (co2Val == 0) strcpy(sCo2, "--- ppm"); else snprintf(sCo2, sizeof(sCo2), "%u ppm", co2Val);
  // "!" после числа спутников — координаты для API/восхода взяты по IP, а не со спутников
  snprintf(sSat, sizeof(sSat), "%u%s", (unsigned)gps.satellites.value(),
           (!gpsFix && homeFromIp && !isnan(homeLat)) ? "!" : "");

  // Первые 5 секунд после старта внизу показываем все значки состояния.
  bool showAllIcons = startupAllIcons;

  bool night = isNightIcon();
  Trend trend = forecastTrend();

  // Иконка текущей погоды из интернета: только если прогноз свежий (не старше 6 ч) и код погоды известен.
  int webWmo = -1;
  if (online && fc.magic == FCMAGIC && forecastWmoCode != 255 && fc.hourEpoch) {
    uint32_t en = epochNow();
    if (en) {
      int64_t age = (int64_t)en - (int64_t)fc.hourEpoch;
      if (age >= -3600 && age <= 6 * 3600) webWmo = forecastWmoCode;
    } else if (wifiErr == WE_OK) {
      webWmo = forecastWmoCode;
    }
  }
  const char *sLink = !zbThermostat.bound() ? "UNPAIRED" : (zbLinkOk() ? "OK" : "LOST");

  // --- строка состояния Wi-Fi (почему нет прогноза)
  char sWifi[100], ss[24];
  copyAscii(ss, sizeof(ss), cfg.ssid.c_str());
  if (cfg.ssid.length() == 0)        snprintf(sWifi, sizeof(sWifi), "Wi-Fi: not set up (hold button 1 s)");
  else switch (wifiErr) {
    case WE_OK:        snprintf(sWifi, sizeof(sWifi), "Wi-Fi: OK (%s)", ss); break;
    case WE_NO_NET:    snprintf(sWifi, sizeof(sWifi), "Wi-Fi: network '%s' not found (2.4 GHz?)", ss); break;
    case WE_BAD_PASS:  snprintf(sWifi, sizeof(sWifi), "Wi-Fi: wrong password for '%s'", ss); break;
    case WE_CONN_FAIL: snprintf(sWifi, sizeof(sWifi), "Wi-Fi: cannot connect to '%s'", ss); break;
    case WE_NO_INET:   snprintf(sWifi, sizeof(sWifi), "Wi-Fi: '%s' connected, but no internet", ss); break;
    case WE_FC_FAIL:   snprintf(sWifi, sizeof(sWifi), "Wi-Fi: connected, forecast request failed"); break;
    case WE_NO_POS:    snprintf(sWifi, sizeof(sWifi), "Wi-Fi: connected, but position unknown (no GPS fix)"); break;
    default:           snprintf(sWifi, sizeof(sWifi), "Wi-Fi: first sync in a few minutes"); break;
  }
  // подробная причина ошибки (то, что реально вернул сервер / стек), одна строка ~68 символов
  if (cfg.ssid.length() && errDetail[0] && wifiErr != WE_OK && wifiErr != WE_UNKNOWN) {
    char det[sizeof(errDetail)];
    copyAscii(det, sizeof(det), errDetail);
    if (wifiErr == WE_FC_FAIL || wifiErr == WE_NO_INET || wifiErr == WE_NO_POS) {
      snprintf(sWifi, sizeof(sWifi), "Forecast error: %s", det);
    } else {
      char base[sizeof(sWifi)];
      strlcpy(base, sWifi, sizeof(base));
      snprintf(sWifi, sizeof(sWifi), "%s [%s]", base, det);
    }
    sWifi[68] = 0;
  }

  display.init(115200, true, 50, false);
  display.setRotation(0);              // 800x480
  display.setFullWindow();
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setTextColor(GxEPD_BLACK);
    display.setTextSize(1);

    // ---- слева вверху: иконка, температура и дополнительные показатели
    drawWeatherIcon(-9, 1, trend, night, webWmo);

    drawBigTemp(142, 75, dispT);

    display.setFont(&FreeSans12pt7b);
    int infoY = 150;
    if (cfg.showFeelsLike) {
      display.setCursor(144, infoY);
      if (isfinite(dispT) && isfinite(dispH)) {
        float feels = dispT;
        if (dispT >= 27.0f && dispH >= 40.0f) {
          float h = dispH, t = dispT;
          feels = -8.784695f + 1.61139411f*t + 2.338549f*h - 0.14611605f*t*h
            - 0.012308094f*t*t - 0.016424828f*h*h + 0.002211732f*t*t*h
            + 0.00072546f*t*h*h - 0.000003582f*t*t*h*h;
        } else if (dispT <= 10.0f && isfinite(forecastWindMs) && forecastWindMs > 1.3f) {
          float v = forecastWindMs * 3.6f;
          feels = 13.12f + 0.6215f*dispT - 11.37f*powf(v,0.16f) + 0.3965f*dispT*powf(v,0.16f);
        }
        display.printf("Feels like: %.1f C", feels);
      } else display.print("Feels like: --");
      infoY += 27;
    }
    if (cfg.showAirQuality) {
      display.setCursor(144, infoY);
      if (airQuality >= 0) display.printf("Air quality: %d", (int)airQuality);
      else display.print("Air quality: --");
      infoY += 27;
    }
    if (cfg.showUvIndex) {
      display.setCursor(144, infoY);
      if (isfinite(forecastUv)) display.printf("UV index: %.1f", forecastUv);
      else display.print("UV index: --");
    }

    // ---- шапка справа
    if (online) {
      // "город, район" крупно, под ним дата (больших часов нет — время внизу справа)
      char pc[sizeof(fc.city)], pd[sizeof(fc.dist)], pr[sizeof(fc.region)], loc[80];
      strlcpy(pc, fc.city, sizeof(pc));
      strlcpy(pd, fc.dist, sizeof(pd));
      strlcpy(pr, fc.region, sizeof(pr));
      shortenRegion(pr);
      shortenRegion(pd);
      const char *pl1 = pc[0] ? pc : pr;
      const char *pl2 = pd[0] ? pd : (pc[0] ? pr : "");
      snprintf(loc, sizeof(loc), "%s%s%s", pl1, (pl1[0] && pl2[0]) ? ", " : "", pl2);
      if (loc[0]) {
        display.setFont(&FreeSansBold18pt7b);
        if (textW(loc) > 410) display.setFont(&FreeSansBold12pt7b);
        printRightFit(790, 36, loc, 410);
      }
      if (sDate[0]) {
        display.setFont(&FreeSans12pt7b);
        printRight(790, 66, sDate);
      }

      // ---- прогноз на 5 дней
      for (int i = 0; i < fcCnt; i++) {
        int cx = 414 + i * 84;
        display.setFont(&FreeSans12pt7b);
        int k = fcFirst + i;
        int32_t days = daysFromCivil((int)(fc.date[k] / 10000), (unsigned)((fc.date[k] / 100) % 100), (unsigned)(fc.date[k] % 100));
        printCenter(cx, 112, WD3[(days + 4) % 7]);
        drawWmo(cx, 152, fc.code[k]);
        printTempPair(cx, 200, (int)fc.tmax[k], (int)fc.tmin[k]);
      }
    } else {
      // нет интернета: только время и дата, крупно
      display.setFont(&FreeSansBold24pt7b);
      display.setTextSize(2);
      printRight(790, 80, sTime);
      display.setTextSize(1);
      display.setFont(&FreeSans12pt7b);
      if (sDate[0]) printRight(790, 116, sDate);
    }

    // ---- сетка показателей (2 колонки x 4 строки)
    const int c1 = 12, c2 = 170;
    cellI(c1, 212, IC_SUNRISE, "Sunrise", sRise);
    cellI(c2, 212, IC_SUNSET,  "Sunset", sSet);
    if (cfg.showIndoorHumidity) cellI(c1, 268, IC_HOUSE, "In humidity", sInH);
    if (cfg.showOutdoorHumidity) cellI(c2, 268, IC_DROP, "Out hum.", sOutH);
    cellITemp(c1, 324, IC_THERMO, "In temp", inT);
    if (cfg.showPressure) cellPressure(c2, 324, IC_GAUGE, sPress);
    if (cfg.showCo2) cellI(c1, 380, IC_CO2, "CO2", sCo2);
    cellI(c2, 380, IC_GPS, "GPS sats", sSat);

    // ---- значки состояния слева внизу: показываем только то, что сейчас работает
    //      (первые 5 секунд после старта — все значки сразу)
    //      порядок слева направо: время, интернет, наружный модуль, погодные предупреждения
    if (online && lt.haveDate) {
      display.setFont(&FreeSans9pt7b);
      display.setCursor(12, 456);
      display.print(sTime);
    }
    if (showAllIcons || (cfg.ssid.length() && wifiErr == WE_OK)) drawSmallIcon(IC_GLOBE,   88, 451);
    drawOutdoorStateIcon(126, 449, showAllIcons);
    drawWeatherAlerts(160, 449, showAllIcons);

    // ---- состояние Wi-Fi внизу: только пока не настроено или при ошибке.
    // Если всё хорошо (ssid задан и последняя синхронизация успешна) — строку не рисуем совсем.
    bool showWifi = (cfg.ssid.length() == 0) || (wifiErr != WE_OK);
    if (showWifi) {
      // Мелкий встроенный шрифт 6x8 — такой же, как у легенды графика.
      display.setFont(NULL);
      display.setTextSize(1);
      display.setCursor(12, 466);          // у встроенного шрифта курсор — верх символа
      display.print(sWifi);
    }

    // ---- график
    if (online) drawGraph(365, 235, 740, 410);
    else        drawGraph(365, 175, 740, 410);
  } while (display.nextPage());
  display.hibernate();
}

void drawConfigScreen(IPAddress ip) {
  String sIp = String("Open: http://") + ip.toString();
  display.init(115200, true, 50, false);
  display.setRotation(0);
  display.setFullWindow();
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setTextColor(GxEPD_BLACK);
    display.setFont(&FreeMonoBold24pt7b);
    display.setCursor(40, 90);  display.print("SETUP MODE");
    display.setFont(&FreeMonoBold12pt7b);
    display.setCursor(40, 170); display.print("1. Connect to Wi-Fi:");
    display.setCursor(80, 205); display.print(AP_SSID);
    display.setCursor(40, 250); display.print("2. Password:");
    display.setCursor(80, 285); display.print(AP_PASS);
    display.setCursor(40, 330); display.print(sIp);
    display.setCursor(40, 390); display.print("Closes in 5 minutes.");
    display.setCursor(40, 425); display.print("Zigbee is paused meanwhile.");
  } while (display.nextPage());
  display.hibernate();
}

// ===================== СИНХРОНИЗАЦИЯ ЧЕРЕЗ WI-FI (прогноз, город, NTP) =====================
void copyAscii(char *dst, size_t n, const char *src) {   // шрифты только ASCII — остальное отбрасываем
  size_t j = 0;
  for (size_t i = 0; src && src[i] && j + 1 < n; i++) {
    unsigned char c = (unsigned char)src[i];
    if (c >= 0x20 && c < 0x7F) dst[j++] = (char)c;
  }
  dst[j] = 0;
}

// ---- MET Norway Locationforecast (бесплатно, без ключа, коммерческое использование разрешено)
// ОБЯЗАТЕЛЬНО замени контакт на свою настоящую почту или ссылку на проект:
// без осмысленного User-Agent сервер отвечает 403.
#define API_USER_AGENT "EInkMeteo/1.0 daria.korsun.a@gmail.com"

// TLS trust configuration. With the ESP-IDF certificate bundle enabled,
// every HTTPS request validates the server chain. Older Arduino cores without
// the bundle retain the previous compatibility mode and emit a visible warning.
void configureTlsClient(WiFiClientSecure &client) {
#if defined(CONFIG_MBEDTLS_CERTIFICATE_BUNDLE)
  client.setInsecure();
#else
  Serial.println("WARNING: TLS CA bundle is unavailable; HTTPS certificate validation is disabled");
  client.setInsecure();
#endif
}

// symbol_code MET Norway ("partlycloudy_day", "lightrainshowers_night", ...) -> код WMO для drawWmo()
static uint8_t metSymbolToWmo(const char *s) {
  if (!s || !s[0])                return 3;
  if (strstr(s, "thunder"))       return 95;
  if (strstr(s, "sleet"))         return 67;
  if (strstr(s, "snow"))          return 73;
  if (strstr(s, "rain"))          return 61;
  if (strstr(s, "fog"))           return 45;
  if (strstr(s, "partlycloudy"))  return 2;
  if (strstr(s, "cloudy"))        return 3;
  if (strstr(s, "fair"))          return 0;
  if (strstr(s, "clearsky"))      return 0;
  return 3;
}

struct DayAgg {
  uint32_t date;     // YYYYMMDD (местная дата)
  float    tmax, tmin;
  int      bestDiff; // минут до полудня у выбранной точки для иконки
  uint8_t  code;
};

bool fetchForecast(float la, float lo) {
  char url[160];
  snprintf(url, sizeof(url),
           "https://api.met.no/weatherapi/locationforecast/2.0/compact?lat=%.4f&lon=%.4f", la, lo);

  WiFiClientSecure client;
  configureTlsClient(client);
  HTTPClient http;
  http.useHTTP10(true);                // без chunked-кодирования, чтобы читать поток напрямую
  http.setTimeout(15000);
  http.setUserAgent(API_USER_AGENT);
  if (!http.begin(client, url)) {
    setErrDetail("http.begin failed");
    return false;
  }
  Serial.printf("fetchForecast: GET %s (free heap=%u)\n", url, ESP.getFreeHeap());
  int code = http.GET();
  if (code != HTTP_CODE_OK && code != 203) {
    if (code < 0) {
      char tls[96] = {0};
      int tlsErr = client.lastError(tls, sizeof(tls));
      setErrDetail("%d %s, tls %d %s", code, HTTPClient::errorToString(code).c_str(), tlsErr, tls);
    } else {
      setErrDetail("HTTP %d", code);
    }
    http.end();
    return false;
  }

  // Ответ ~100 КБ: читаем потоком и оставляем только время, температуру и символ погоды
  JsonDocument filter;
  JsonObject f = filter["properties"]["timeseries"][0].to<JsonObject>();
  f["time"] = true;
  f["data"]["instant"]["details"]["air_temperature"] = true;
  f["data"]["instant"]["details"]["relative_humidity"] = true;
  f["data"]["instant"]["details"]["wind_speed"] = true;
  f["data"]["instant"]["details"]["ultraviolet_index"] = true;
  f["data"]["next_1_hours"]["details"]["precipitation_amount"] = true;
  f["data"]["next_1_hours"]["summary"]["symbol_code"] = true;
  f["data"]["next_6_hours"]["summary"]["symbol_code"] = true;
  f["data"]["next_12_hours"]["summary"]["symbol_code"] = true;

  int contentLen = http.getSize();                       // -1, если сервер не сообщил длину
  WaitStream ws(http.getStream(), 15000);
  JsonDocument doc;
  DeserializationError je = deserializeJson(doc, ws, DeserializationOption::Filter(filter));
  http.end();
  if (je) {
    setErrDetail("JSON: %s, got %u of %d bytes", je.c_str(), (unsigned)ws.count, contentLen);
    return false;
  }
  JsonArray ts = doc["properties"]["timeseries"];
  if (ts.isNull() || ts.size() == 0) {
    setErrDetail("reply has no timeseries");
    return false;
  }

  // Сводим точки в дни по МЕСТНОЙ дате (cfg.tz): max/min температуры + символ ближе к полудню
  DayAgg days[5];
  int nd = 0;
  // почасовые точки (температура/влажность "сейчас" из интернета, если наружный модуль недоступен)
  uint32_t hEpoch = 0; int16_t hT[6]; uint8_t hH[6]; int nh = 0; bool hDone = false;
  float curT = NAN, curH = NAN, curWind = NAN, curPrecip = NAN, curUv = NAN;
  uint8_t curCode = 255;
  for (JsonObject e : ts) {
    const char *tm = e["time"] | "";
    int y, mo, d, h, mi;
    if (sscanf(tm, "%d-%d-%dT%d:%d", &y, &mo, &d, &h, &mi) != 5) continue;
    float t = e["data"]["instant"]["details"]["air_temperature"] | NAN;
    if (!isfinite(curT) && isfinite(t)) {
      curT = t;
      curH = e["data"]["instant"]["details"]["relative_humidity"] | NAN;
      curWind = e["data"]["instant"]["details"]["wind_speed"] | NAN;
      curUv = e["data"]["instant"]["details"]["ultraviolet_index"] | NAN;
      curPrecip = e["data"]["next_1_hours"]["details"]["precipitation_amount"] | 0.0f;
      const char *currentSym = e["data"]["next_1_hours"]["summary"]["symbol_code"] | "";
      if (!currentSym[0]) currentSym = e["data"]["next_6_hours"]["summary"]["symbol_code"] | "";
      curCode = currentSym[0] ? (uint8_t)metSymbolToWmo(currentSym) : 255;
    }
    if (!hDone && !isnan(t)) {
      uint32_t ue = (uint32_t)((int64_t)daysFromCivil(y, mo, d) * 86400LL + h * 3600 + mi * 60);
      if (nh == 0 || ue == hEpoch + (uint32_t)nh * 3600UL) {
        if (nh == 0) hEpoch = ue;
        float hum = e["data"]["instant"]["details"]["relative_humidity"] | NAN;
        hT[nh] = (int16_t)lroundf(t * 10.0f);
        hH[nh] = isnan(hum) ? 255 : (uint8_t)constrain((int)lroundf(hum), 0, 100);
        if (++nh >= 6) hDone = true;
      } else hDone = true;                               // дальше точки не почасовые
    }

    int64_t le = (int64_t)daysFromCivil(y, mo, d) * 86400LL + h * 3600 + mi * 60 + (int64_t)cfg.tz * 3600;
    int32_t ldays = (int32_t)(le / 86400);
    int lh = (int)((le % 86400) / 3600);
    int ly, lm, ld;
    civilFromDays(ldays, ly, lm, ld);
    uint32_t key = (uint32_t)(ly * 10000 + lm * 100 + ld);

    int idx;
    if (nd > 0 && days[nd - 1].date == key) idx = nd - 1;
    else if (nd < 5) { idx = nd++; days[idx] = { key, -1000.0f, 1000.0f, 100000, 3 }; }
    else break;                                          // дальше 5 дней не нужно

    if (!isnan(t)) {
      if (t > days[idx].tmax) days[idx].tmax = t;
      if (t < days[idx].tmin) days[idx].tmin = t;
    }
    const char *sym = e["data"]["next_6_hours"]["summary"]["symbol_code"] | "";
    if (!sym[0]) sym = e["data"]["next_12_hours"]["summary"]["symbol_code"] | "";
    if (!sym[0]) sym = e["data"]["next_1_hours"]["summary"]["symbol_code"] | "";
    int diff = abs(lh * 60 + mi - 12 * 60);
    if (sym[0] && diff < days[idx].bestDiff) {
      days[idx].bestDiff = diff;
      days[idx].code = metSymbolToWmo(sym);
    }
  }

  char keepCity[sizeof(fc.city)], keepDist[sizeof(fc.dist)], keepReg[sizeof(fc.region)];
  memcpy(keepCity, fc.city, sizeof(keepCity));
  memcpy(keepDist, fc.dist, sizeof(keepDist));
  memcpy(keepReg,  fc.region, sizeof(keepReg));
  memset(&fc, 0, sizeof(fc));
  memcpy(fc.city, keepCity, sizeof(keepCity));
  memcpy(fc.dist, keepDist, sizeof(keepDist));
  memcpy(fc.region, keepReg, sizeof(keepReg));
  fc.hourEpoch = hEpoch;
  forecastCurrentTemp = curT;
  forecastHumidity = curH;
  forecastWindMs = curWind;
  forecastPrecipMm = curPrecip;
  if (isfinite(curUv)) forecastUv = curUv;
  forecastWmoCode = curCode;
  alertGroup1 = GROUP1_NONE;
  alertGroup2 = GROUP2_NONE;

  // Группа 1: гололёд имеет приоритет над сильным ветром.
  // Гололёд учитывает и температуру, и наличие влаги/осадков.
  if (isfinite(curT) && curT <= 1.0f && curT >= -8.0f &&
      ((isfinite(curPrecip) && curPrecip >= 0.1f) ||
       (isfinite(curH) && curH >= 90.0f))) {
    alertGroup1 = GROUP1_ICE;
  } else if (isfinite(curWind) && curWind > 15.0f) {
    alertGroup1 = GROUP1_WIND;
  }

  // Группа 2: гроза имеет приоритет над сильным снегопадом.
  if (curCode >= 95 && curCode <= 99) {
    alertGroup2 = GROUP2_STORM;
  } else if (((curCode >= 71 && curCode <= 77) || curCode == 85 || curCode == 86) &&
             isfinite(curPrecip) && curPrecip >= 2.0f) {
    alertGroup2 = GROUP2_SNOW;
  }

  tempAlert = TEMP_NONE;
  if (isfinite(curT) && curT < -30.0f) tempAlert = TEMP_FROST;
  else if (isfinite(curT) && curT > 30.0f) tempAlert = TEMP_HEAT;
  fc.nHour = (uint8_t)nh;
  for (int i = 0; i < nh; i++) { fc.hourT[i] = hT[i]; fc.hourH[i] = hH[i]; }

  int n = 0;
  for (int i = 0; i < nd; i++) {
    if (days[i].tmax < -999.0f) continue;                // в этом дне не было температур
    fc.date[n] = days[i].date;
    fc.code[n] = days[i].code;
    fc.tmax[n] = (int8_t)lroundf(days[i].tmax);
    fc.tmin[n] = (int8_t)lroundf(days[i].tmin);
    n++;
  }
  Serial.printf("fetchForecast: получено дней: %d\n", n);
  if (n == 0) { setErrDetail("no days parsed"); return false; }
  fc.n = (uint8_t)n;
  fc.magic = FCMAGIC;
  return true;
}

// Open-Meteo Air Quality: European AQI по последней доступной часовой точке.
bool fetchUvIndex(float la, float lo) {
  char url[260];
  snprintf(url, sizeof(url),
    "https://api.open-meteo.com/v1/forecast?latitude=%.4f&longitude=%.4f&current=uv_index",
    la, lo);
  WiFiClientSecure client; configureTlsClient(client);
  HTTPClient http; http.useHTTP10(true); http.setTimeout(12000);   // без chunked, иначе JSON не разбирается
  if (!http.begin(client, url)) return false;
  int code = http.GET();
  if (code != HTTP_CODE_OK) { Serial.printf("UV HTTP status: %d\n", code); http.end(); return false; }
  JsonDocument doc;
  WaitStream ws(http.getStream(), 12000);
  DeserializationError err = deserializeJson(doc, ws);
  http.end();
  if (err) { Serial.printf("UV JSON ERROR: %s (got %u bytes)\n", err.c_str(), (unsigned)ws.count); return false; }
  float uv = doc["current"]["uv_index"] | NAN;
  if (!isfinite(uv) || uv < 0 || uv > 30) return false;
  forecastUv = uv;
  prefs.putFloat("uv", forecastUv);
  return true;
}

bool fetchAirQuality(float la, float lo) {
  Serial.println();
  Serial.println("========== AIR QUALITY SYNC ==========");

  if (!isfinite(la) || !isfinite(lo) || la < -90.0f || la > 90.0f ||
      lo < -180.0f || lo > 180.0f) {
    Serial.println("AQI ERROR: invalid coordinates; GPS/IP position is unavailable");
    return false;
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.printf("AQI ERROR: Wi-Fi is not connected (status=%d)\n", (int)WiFi.status());
    return false;
  }

  char url[300];
  // domains не задаём: API автоматически выбирает доступную модель для координат.
  snprintf(url, sizeof(url),
    "https://air-quality-api.open-meteo.com/v1/air-quality?latitude=%.4f&longitude=%.4f&current=european_aqi",
    la, lo);
  Serial.printf("AQI coordinates: %.5f, %.5f\n", la, lo);
  Serial.printf("AQI request: %s\n", url);

  WiFiClientSecure client;
  configureTlsClient(client);
  HTTPClient http;
  http.useHTTP10(true);                // без chunked-кодирования: иначе JSON читается с ошибкой
  http.setTimeout(15000);
  if (!http.begin(client, url)) {
    Serial.println("AQI ERROR: HTTPClient.begin() failed");
    return false;
  }

  int code = http.GET();
  Serial.printf("AQI HTTP status: %d\n", code);
  if (code < 0) Serial.printf("AQI network error: %s\n", HTTPClient::errorToString(code).c_str());
  if (code != HTTP_CODE_OK) {
    String body = http.getString();
    Serial.printf("AQI HTTP error body: %.300s\n", body.c_str());
    http.end();
    return false;
  }

  JsonDocument doc;
  WaitStream ws(http.getStream(), 15000);
  DeserializationError jsonError = deserializeJson(doc, ws);
  http.end();
  if (jsonError) {
    Serial.printf("AQI JSON ERROR: %s (got %u bytes)\n", jsonError.c_str(), (unsigned)ws.count);
    return false;
  }
  if (!doc["current"].is<JsonObject>()) {
    Serial.println("AQI ERROR: response has no 'current' object");
    serializeJson(doc, Serial);
    Serial.println();
    return false;
  }
  if (doc["current"]["european_aqi"].isNull()) {
    Serial.println("AQI ERROR: 'current.european_aqi' is missing/null");
    serializeJson(doc, Serial);
    Serial.println();
    return false;
  }

  float value = doc["current"]["european_aqi"].as<float>();
  if (!isfinite(value) || value < 0.0f || value > 500.0f) {
    Serial.printf("AQI ERROR: invalid AQI value %.2f\n", value);
    return false;
  }

  airQuality = (int16_t)lroundf(value);
  prefs.putShort("eaqi", airQuality);
  Serial.printf("AQI SUCCESS: European AQI = %d\n", (int)airQuality);
  Serial.println("======================================");
  return true;
}

// Транслитерация кириллицы (UTF-8) в латиницу — шрифты на экране умеют только
// ASCII, а геокодер может отдать город/район по-русски. "Москва" -> "Moskva".
void translitRu(char *dst, size_t n, const char *src) {
  static const char *T[32] = {
    "A","B","V","G","D","E","Zh","Z","I","Y","K","L","M","N","O","P",
    "R","S","T","U","F","Kh","Ts","Ch","Sh","Sch","","Y","","E","Yu","Ya"
  };
  size_t j = 0;
  for (size_t i = 0; src && src[i] && j + 1 < n; ) {
    unsigned char c0 = (unsigned char)src[i];
    uint32_t cp; int len;
    if (c0 < 0x80)                       { cp = c0; len = 1; }
    else if ((c0 & 0xE0) == 0xC0 && src[i + 1]) { cp = ((c0 & 0x1F) << 6) | ((unsigned char)src[i + 1] & 0x3F); len = 2; }
    else                                  { i++; continue; }   // 3-4-байтные символы (emoji и т.п.) пропускаем
    i += len;
    if (cp == 0x0401 || cp == 0x0451) {              // Ё / ё
      dst[j++] = (cp == 0x0401) ? 'E' : 'e';
    } else if (cp >= 0x0410 && cp <= 0x042F) {       // А-Я
      const char *s = T[cp - 0x0410];
      bool first = true;
      while (*s && j + 1 < n) { dst[j++] = first ? *s : (char)tolower((unsigned char)*s); first = false; s++; }
    } else if (cp >= 0x0430 && cp <= 0x044F) {       // а-я
      const char *s = T[cp - 0x0430];
      while (*s && j + 1 < n) dst[j++] = (char)tolower((unsigned char)*s), s++;
    } else if (cp >= 0x20 && cp < 0x7F) {
      dst[j++] = (char)cp;
    }
  }
  dst[j] = 0;
}

// Название города/района по координатам: OpenStreetMap Nominatim (бесплатно, без ключа).
// Правила сервиса: осмысленный User-Agent (используем API_USER_AGENT) и не чаще 1 запроса в секунду.
// Просим названия на английском (accept-language=en); если английского названия нет, придёт кириллица —
// translitRu() превратит её в латиницу.
bool fetchPlace(float la, float lo) {
  char url[200];
  snprintf(url, sizeof(url),
           "https://nominatim.openstreetmap.org/reverse?format=jsonv2&zoom=14&addressdetails=1&accept-language=en&lat=%.5f&lon=%.5f",
           la, lo);
  WiFiClientSecure client;
  configureTlsClient(client);
  HTTPClient http;
  http.setTimeout(10000);
  http.setUserAgent(API_USER_AGENT);
  if (!http.begin(client, url)) { Serial.println("fetchPlace: begin failed"); return false; }
  http.addHeader("Accept", "application/json");
  int code = http.GET();
  String resp;
  if (code == HTTP_CODE_OK) resp = http.getString();
  http.end();
  if (code != HTTP_CODE_OK) { Serial.printf("fetchPlace: HTTP %d\n", code); return false; }

  JsonDocument doc;
  DeserializationError je = deserializeJson(doc, resp);
  if (je) { Serial.printf("fetchPlace: JSON %s\n", je.c_str()); return false; }
  JsonObject ad = doc["address"];
  if (ad.isNull()) { Serial.println("fetchPlace: no address in reply"); return false; }

  const char *city = ad["city"] | "";
  if (!city[0]) city = ad["town"] | "";
  if (!city[0]) city = ad["village"] | "";
  if (!city[0]) city = ad["municipality"] | "";
  if (!city[0]) city = "";
  const char *dist = ad["city_district"] | "";
  if (!dist[0]) dist = ad["suburb"] | "";
  if (!dist[0]) dist = ad["borough"] | "";
  if (!dist[0]) dist = ad["quarter"] | "";
  if (!dist[0]) dist = ad["neighbourhood"] | "";
  const char *region = ad["state"] | "";
  if (!region[0]) region = ad["region"] | "";
  if (!region[0]) region = ad["state_district"] | "";

  translitRu(fc.city, sizeof(fc.city), city);
  translitRu(fc.dist, sizeof(fc.dist), dist);
  translitRu(fc.region, sizeof(fc.region), region);
  Serial.printf("fetchPlace: city='%s' district='%s' region='%s'\n", fc.city, fc.dist, fc.region);
  return fc.city[0] || fc.region[0];
}

// Координаты по IP — запасной вариант, если GPS ещё не поймал спутники (точность: город)
bool fetchIpPosition() {
  WiFiClient client;
  HTTPClient http;
  http.setTimeout(8000);
  if (!http.begin(client, "http://ip-api.com/json/?fields=status,lat,lon")) { setErrDetail("ip-api: begin failed"); return false; }
  int code = http.GET();
  String body;
  if (code == HTTP_CODE_OK) body = http.getString();
  http.end();
  if (code != HTTP_CODE_OK) { setErrDetail("no GPS fix; ip-api HTTP %d %s", code, code < 0 ? HTTPClient::errorToString(code).c_str() : ""); return false; }
  JsonDocument doc;
  if (deserializeJson(doc, body)) { setErrDetail("no GPS fix; ip-api bad JSON"); return false; }
  if (strcmp(doc["status"] | "", "success") != 0) { setErrDetail("no GPS fix; ip-api status fail"); return false; }
  const float receivedLat = doc["lat"].as<float>();
  const float receivedLon = doc["lon"].as<float>();
  if (!isfinite(receivedLat) || !isfinite(receivedLon) ||
      receivedLat < -90.0f || receivedLat > 90.0f ||
      receivedLon < -180.0f || receivedLon > 180.0f) {
    setErrDetail("ip-api returned invalid coordinates");
    return false;
  }
  homeFromIp = true;
  prefs.putBool("latip", true);
  homeLat = receivedLat;
  homeLon = receivedLon;
  prefs.putFloat("lat", homeLat);
  prefs.putFloat("lon", homeLon);
  return true;
}

void onWifiEvent(arduino_event_t *e) {
  if (e->event_id == ARDUINO_EVENT_WIFI_STA_DISCONNECTED)
    lastDiscReason = e->event_info.wifi_sta_disconnected.reason;
}

// Расшифровка причины неудачного подключения (коды причин ESP-IDF)
uint8_t classifyWifiFail(uint8_t r) {
  switch (r) {
    case 201: case 210: case 211: case 212:           // AP не найден
      return WE_NO_NET;
    case 2: case 14: case 15: case 202: case 204:     // авторизация / 4-way handshake
      return WE_BAD_PASS;
    case 0: {                                         // события ещё не было — смотрим статус
      wl_status_t st = WiFi.status();
      if (st == WL_NO_SSID_AVAIL) return WE_NO_NET;
      if (st == WL_CONNECT_FAILED) return WE_BAD_PASS;
      return WE_CONN_FAIL;
    }
    default:
      return WE_CONN_FAIL;
  }
}

void sdSaveForecastBackup() {
  if (!sdReady || fc.magic != FCMAGIC) return;
  if (SD.exists("/forecast.json.tmp")) SD.remove("/forecast.json.tmp");
  File f = SD.open("/forecast.json.tmp", FILE_WRITE);
  if (!f) { Serial.println("SD: cannot create forecast backup"); return; }
  JsonDocument doc;
  doc["format_version"] = 2;
  doc["saved_at_utc"] = epochNow();
  doc["timezone_offset_hours"] = cfg.tz;
  doc["city"] = fc.city;
  doc["district"] = fc.dist;
  doc["region"] = fc.region;
  doc["hour_epoch_utc"] = fc.hourEpoch;
  JsonArray days = doc["days"].to<JsonArray>();
  for (uint8_t i = 0; i < fc.n && i < 5; ++i) {
    JsonObject d = days.add<JsonObject>();
    d["date"] = fc.date[i]; d["code"] = fc.code[i];
    d["tmax"] = fc.tmax[i]; d["tmin"] = fc.tmin[i];
  }
  JsonArray hours = doc["hours"].to<JsonArray>();
  for (uint8_t i = 0; i < fc.nHour && i < 6; ++i) {
    JsonObject h = hours.add<JsonObject>();
    h["temperature_c"] = fc.hourT[i] / 10.0f;
    if (fc.hourH[i] != 255) h["humidity_pct"] = fc.hourH[i];
  }
  serializeJson(doc, f);
  f.flush();
  f.close();
  if (SD.exists("/forecast.json")) SD.remove("/forecast.json");
  if (!SD.rename("/forecast.json.tmp", "/forecast.json"))
    Serial.println("SD: forecast backup rename failed");
}

// Flash Preferences остаётся основным быстрым хранилищем. JSON на SD — резерв,
// который восстанавливается только если в Flash нет корректного прогноза.
bool sdRestoreForecastBackup() {
  if (!sdReady || fc.magic == FCMAGIC || !SD.exists("/forecast.json")) return false;
  File f = SD.open("/forecast.json", FILE_READ);
  if (!f) return false;

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, f);
  f.close();
  if (error || (doc["format_version"] | 0) < 1) {
    Serial.println("SD: forecast backup is invalid");
    return false;
  }

  Forecast restored = {};
  strlcpy(restored.city, doc["city"] | "", sizeof(restored.city));
  strlcpy(restored.dist, doc["district"] | "", sizeof(restored.dist));
  strlcpy(restored.region, doc["region"] | "", sizeof(restored.region));

  JsonArrayConst days = doc["days"].as<JsonArrayConst>();
  for (JsonObjectConst d : days) {
    if (restored.n >= 5) break;
    uint32_t date = d["date"] | 0UL;
    int code = d["code"] | 0;
    int tmax = d["tmax"] | 0;
    int tmin = d["tmin"] | 0;
    if (date < 20000101UL || date > 20991231UL ||
        tmax < -100 || tmax > 100 || tmin < -100 || tmin > 100 ||
        tmin > tmax) continue;
    restored.date[restored.n] = date;
    restored.code[restored.n] = (uint8_t)constrain(code, 0, 255);
    restored.tmax[restored.n] = (int8_t)tmax;
    restored.tmin[restored.n] = (int8_t)tmin;
    restored.n++;
  }

  JsonArrayConst hours = doc["hours"].as<JsonArrayConst>();
  uint32_t hourEpoch = doc["hour_epoch_utc"] | 0UL;
  if (hourEpoch >= 1700000000UL && hourEpoch <= 4102444800UL) {
    restored.hourEpoch = hourEpoch;
    for (JsonObjectConst h : hours) {
      if (restored.nHour >= 6) break;
      float temp = h["temperature_c"] | NAN;
      if (!isfinite(temp) || temp < -100.0f || temp > 100.0f) continue;
      uint8_t i = restored.nHour++;
      restored.hourT[i] = (int16_t)lroundf(temp * 10.0f);
      float humidity = h["humidity_pct"] | NAN;
      restored.hourH[i] = (isfinite(humidity) && humidity >= 0 && humidity <= 100)
                            ? (uint8_t)lroundf(humidity) : 255;
    }
  }

  if (restored.n == 0) {
    Serial.println("SD: forecast backup contains no valid daily data");
    return false;
  }
  restored.magic = FCMAGIC;
  fc = restored;
  prefs.putBytes("fc", &fc, sizeof(fc));
  Serial.printf("SD: restored forecast (%u days, %u hourly points)\n", fc.n, fc.nHour);
  return true;
}

class ForecastServiceGateway {
public:
  bool getForecast(const Coordinates &coordinates) {
    return coordinates.valid && fetchForecast(coordinates.latitude, coordinates.longitude);
  }
  bool getAirQuality(const Coordinates &coordinates) {
    return coordinates.valid && fetchAirQuality(coordinates.latitude, coordinates.longitude);
  }
  bool getUvIndex(const Coordinates &coordinates) {
    return coordinates.valid && fetchUvIndex(coordinates.latitude, coordinates.longitude);
  }
  bool getPlace(const Coordinates &coordinates) {
    return coordinates.valid && fetchPlace(coordinates.latitude, coordinates.longitude);
  }
  bool getIpPosition() { return fetchIpPosition(); }
};

class ExternalForecastController {
private:
  ForecastServiceGateway service;
public:
  bool getForecast(const Coordinates &coordinates) { return service.getForecast(coordinates); }
  AirQuality getAirQuality(const Coordinates &coordinates) {
    AirQuality result;
    result.available = service.getAirQuality(coordinates);
    result.europeanAqi = airQuality;
    return result;
  }
  bool getUvIndex(const Coordinates &coordinates) { return service.getUvIndex(coordinates); }
  bool getPlace(const Coordinates &coordinates) { return service.getPlace(coordinates); }
  bool getIpPosition() { return service.getIpPosition(); }
};

ExternalForecastController externalForecastController;

class WiFiGateway {
public:
  bool connected() const { return WiFi.status() == WL_CONNECTED; }

  // Подключение к сети из cfg: те же вызовы и таймауты, что были в runSyncMode().
  bool connect() {
    lastDiscReason = 0;
    WiFi.mode(WIFI_STA);
    WiFi.onEvent(onWifiEvent);
    WiFi.begin(cfg.ssid.c_str(), cfg.pass.c_str());
    uint32_t t0 = millis();
    while (!connected() && millis() - t0 < 20000UL) {
      delay(200);
      // "сеть не найдена" ясно уже через ~8 с, дольше ждать незачем
      if (millis() - t0 > 8000UL && classifyWifiFail(lastDiscReason) == WE_NO_NET) break;
    }
    return connected();
  }

  void disconnect() {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
  }
};

void runSyncMode(WiFiGateway &wifi) {
  uint8_t err = WE_UNKNOWN;
  const uint8_t prevErr = wifiErr;     // результат предыдущей попытки (загружен при старте)
  errDetail[0] = 0;
  Serial.println("Sync mode: connecting to Wi-Fi");
  const bool wifiOk = wifi.connect();

  if (wifiOk) {
    configTime(0, 0, "pool.ntp.org", "time.google.com");
    uint32_t t0 = millis();
    time_t nt = 0;
    while (millis() - t0 < 8000UL) {
      nt = time(nullptr);
      if (nt > 1700000000) break;
      delay(200);
    }
    bool ntpOk = nt > 1700000000;
    if (ntpOk) { epochBase = (uint32_t)nt; epochBaseMs = millis(); }

    if (isnan(homeLat)) externalForecastController.getIpPosition();   // GPS ещё не поймал — берём примерно по IP
    if (isnan(homeLat)) {
      Serial.println("AQI SKIPPED: no valid GPS fix and IP geolocation failed");
      err = WE_NO_POS;
      if (!errDetail[0]) setErrDetail("no GPS fix and no IP position");
    } else {
      Coordinates coordinates;
      coordinates.latitude = homeLat;
      coordinates.longitude = homeLon;
      coordinates.valid = true;
      externalForecastController.getPlace(coordinates);   // город/район — необязательно
      // AQI и UV — независимые запросы: выполняем их, даже если погодный прогноз упадёт.
      AirQuality quality = externalForecastController.getAirQuality(coordinates);
      if (!quality.available)
        Serial.println("AQI RESULT: failed; previous saved value (if any) is kept");
      if (!externalForecastController.getUvIndex(coordinates))
        Serial.println("UV RESULT: failed; previous saved value (if any) is kept");

      Forecast oldFc = fc;   // прогноз до замены — нужен для отметок Error Bars
      if (externalForecastController.getForecast(coordinates)) {
        captureForecastError(oldFc);                    // сравниваем старый прогноз с фактом
        prefs.putBytes("fc", &fc, sizeof(fc));
        sdSaveForecastBackup();
        prefs.putUChar("alert_g1", (uint8_t)alertGroup1);
        prefs.putUChar("alert_g2", (uint8_t)alertGroup2);
        prefs.putUChar("tempwarn", (uint8_t)tempAlert);
        prefs.putFloat("fc_temp", forecastCurrentTemp);
        prefs.putFloat("fc_wind", forecastWindMs);
        prefs.putFloat("fc_precip", forecastPrecipMm);
        prefs.putFloat("fc_hum", forecastHumidity);
        prefs.putUChar("fc_wmo", forecastWmoCode);
        err = WE_OK;
      } else {
        if (fc.magic != FCMAGIC) fc = oldFc;            // неудачный разбор не должен стирать прежний прогноз
        err = ntpOk ? WE_FC_FAIL : WE_NO_INET;
        if (!ntpOk) {                                   // добавим, что и NTP не ответил
          char t[sizeof(errDetail)];
          strlcpy(t, errDetail, sizeof(t));
          setErrDetail("%s; no NTP", t);
        }
      }
    }
  } else {
    Serial.printf("AQI SKIPPED: Wi-Fi connection failed (status=%d, disconnect reason=%u)\n",
                  (int)WiFi.status(), (unsigned)lastDiscReason);
    err = classifyWifiFail(lastDiscReason);
    setErrDetail("disconnect reason %u, status %d", (unsigned)lastDiscReason, (int)WiFi.status());
  }

  Serial.printf("Sync result: code=%u, last disconnect reason=%u\n", err, lastDiscReason);
  prefs.putUChar("werr", err);
  prefs.putString("wdet", err == WE_OK ? "" : errDetail);
  // Нарастающий интервал повтора: счётчик подряд неудач хранится во флеше (см. retryMsFor).
  if (err == WE_OK) syncFails = 0; else if (syncFails < 250) syncFails++;
  prefs.putUChar("syncfails", syncFails);
  // Код ошибки не изменился — при следующей загрузке не перерисовываем e-ink зря.
  prefs.putBool("skipdraw", err != WE_OK && err == prevErr);
  prefs.putBool("syncfail", err != WE_OK);          // при неудаче (Wi-Fi или прогноз) повтор через retryMsFor(syncFails)
  wifi.disconnect();
  saveEpochForRestart();
  prefs.putBool("notest", true); 
  delay(200);
  ESP.restart();
}

// Интервал повтора после неудачной синхронизации: 2, 5, 15, 30, 60 мин и дальше 60.
uint32_t retryMsFor(uint8_t fails) {
  static const uint16_t MINUTES[] = {2, 5, 15, 30, 60};
  uint8_t i = (fails < 1) ? 0 : (uint8_t)(fails - 1);
  if (i > 4) i = 4;
  return (uint32_t)MINUTES[i] * 60UL * 1000UL;
}

void syncCheck(uint32_t now) {
  if (cfg.ssid.length() == 0) return;

  bool due = false;

  if (syncFailFlag) {
    // Если предыдущая попытка не удалась — повтор с нарастающим интервалом (2, 5, 15, 30, 60 мин)
    due = now > retryMsFor(syncFails);
  }
  else if (wifiErr == WE_UNKNOWN) {
    // Первая синхронизация — через 2 минуты,
    // даже если старый прогноз уже сохранён
    due = now > FIRST_SYNC_MS;
  }
  else {
    // Последующие синхронизации — раз в 4 часа
    due = now > SYNC_PERIOD_MS;
  }

  if (!due) return;

  Serial.println("Rebooting into sync mode (Wi-Fi)");

  prefs.putBool("sync", true);
  saveEpochForRestart();
  delay(100);
  ESP.restart();
}

// ===================== ВЕБ-СЕРВЕР НАСТРОЙКИ =====================
bool portalDone = false;
bool factoryResetRequested = false;

class ResetForm {
public:
  String confirmationForm() const {
    return String("<h3>Factory reset</h3><form method='post' action='/reset' onsubmit=\"return confirm('") +
      "Erase all settings, history and Zigbee pairing?')\"><button type='submit'>Reset to factory settings</button></form>";
  }
};


String esc(const String &s) {
  String o;
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '&') o += "&amp;";
    else if (c == '<') o += "&lt;";
    else if (c == '>') o += "&gt;";
    else if (c == '"') o += "&quot;";
    else if (c == '\'') o += "&#39;";
    else o += c;
  }
  return o;
}

String field(const char *label, const char *name, const String &val,
             const char *type = "text", const char *extra = "") {
  String s = "<label>";
  s += label;
  s += "<input type='";  s += type;
  s += "' name='";       s += name;
  s += "' value='";      s += esc(val);
  s += "' ";             s += extra;
  s += "></label>";
  return s;
}

String pageHead() {
  return F("<!doctype html><html><head><meta charset='utf-8'>"
           "<meta name='viewport' content='width=device-width,initial-scale=1'>"
           "<title>E-ink Meteo</title><style>"
           "body{font-family:sans-serif;max-width:440px;margin:1em auto;padding:0 1em}"
           "label{display:block;margin-top:.8em;font-size:.9em}"
           "input{width:100%;padding:.5em;box-sizing:border-box;font-size:1em}"
           "button{margin-top:1.2em;padding:.7em 1.2em;font-size:1em}"
           "h3{margin-top:1.5em}</style></head><body><h2>E-ink Meteo</h2>");
}

String msgPage(const String &msg) {
  return pageHead() + "<p>" + msg + "</p></body></html>";
}

void handleRoot() {
  String h = pageHead();
  h += "<form method='post' action='/save'><h3>Wi-Fi (forecast, city, time sync)</h3>";
  h += field("Network name (SSID)", "ssid", cfg.ssid);
  h += field("Wi-Fi password", "pass", cfg.pass, "password", "autocomplete='off' autocapitalize='none'");
  h += "<h3>Display</h3>";
  h += field("Refresh period, minutes (day)", "refr", String(cfg.refreshMin), "number", "min=1 max=1440");
  h += field("Night mode starts at (hour 0-23)", "ns", String(cfg.nightStart), "number", "min=0 max=23");
  h += field("Night mode ends at (hour 0-23)", "ne", String(cfg.nightEnd), "number", "min=0 max=23");
  h += field("Refresh period at night, minutes", "nr", String(cfg.nightRefreshMin), "number", "min=1 max=1440");
  h += field("Time zone (UTC+N)", "tz", String(cfg.tz), "number", "min=-12 max=14");
  h += field("Fallback time (HH:MM), used only if no GPS and no Wi-Fi time", "time", "", "time", "");
  h += "<h3>Screen data</h3>";
  auto check = [&](const char *label, const char *name, bool value) {
    h += "<label style='display:flex;gap:.6em;align-items:center'><input style='width:auto' type='checkbox' name='";
    h += name; h += "' value='1'"; if (value) h += " checked"; h += ">"; h += label; h += "</label>";
  };
  check("Air quality (European AQI)", "vaqi", cfg.showAirQuality);
  check("Feels like temperature", "vfeel", cfg.showFeelsLike);
  check("UV index", "vuv", cfg.showUvIndex);
  check("Pressure", "vpress", cfg.showPressure);
  check("CO2", "vco2", cfg.showCo2);
  check("Indoor humidity", "vihum", cfg.showIndoorHumidity);
  check("Outdoor humidity", "vohum", cfg.showOutdoorHumidity);
  h += "<p><small>Selected indicators are displayed on separate lines.</small></p>";
  h += "<h3>Outdoor module</h3>";
  h += "<label>Outdoor data priority<select name='osrc' style='width:100%;padding:.5em;font-size:1em'>";
  h += "<option value='sensor'"; if (!cfg.outdoorWebFirst) h += " selected"; h += ">Outdoor sensor first (internet as fallback)</option>";
  h += "<option value='web'"; if (cfg.outdoorWebFirst) h += " selected"; h += ">Internet first (sensor as fallback)</option>";
  h += "</select></label>";
  h += field("Outdoor module link-loss timeout base, seconds (link is lost after 3x this + 5 s without data)", "zbi", String(cfg.zbIntervalSec), "number", "min=20 max=3600");
  h += "<button type='submit'>Save and exit</button></form>";
  h += "<form method='post' action='/exit'><button type='submit'>Exit without saving</button></form>";
  h += ResetForm().confirmationForm();
  h += "</body></html>";
  server.send(200, "text/html", h);
}

void handleSave() {
  if (server.hasArg("ssid")) { cfg.ssid = server.arg("ssid"); cfg.ssid.trim(); }
  if (server.hasArg("pass")) cfg.pass = server.arg("pass");
  cfg.refreshMin      = constrain(server.arg("refr").toInt(), 1, 1440);
  cfg.nightStart      = constrain(server.arg("ns").toInt(), 0, 23);
  cfg.nightEnd        = constrain(server.arg("ne").toInt(), 0, 23);
  cfg.nightRefreshMin = constrain(server.arg("nr").toInt(), 1, 1440);
  cfg.zbIntervalSec   = constrain(server.arg("zbi").toInt(), ZB_INTERVAL_MIN_SEC, 3600);
  cfg.showAirQuality = server.hasArg("vaqi");
  cfg.showFeelsLike = server.hasArg("vfeel");
  cfg.showUvIndex = server.hasArg("vuv");
  cfg.showPressure = server.hasArg("vpress");
  cfg.showCo2 = server.hasArg("vco2");
  cfg.showIndoorHumidity = server.hasArg("vihum");
  cfg.showOutdoorHumidity = server.hasArg("vohum");
  cfg.outdoorWebFirst = (server.arg("osrc") == "web");
  cfg.tz              = constrain(server.arg("tz").toInt(), -12, 14);
  if (server.arg("time").length() >= 4) {
    int hh = 0, mm = 0;
    if (sscanf(server.arg("time").c_str(), "%d:%d", &hh, &mm) == 2) {
      cfg.clockSetMin = (uint16_t)constrain(hh, 0, 23) * 60 + (uint16_t)constrain(mm, 0, 59);
    }
  }
  saveSettings();
  prefs.putBool("syncfail", false);    // новые Wi-Fi данные — можно пробовать синхронизацию сразу
  syncFails = 0;
  prefs.putUChar("syncfails", 0);
  prefs.putUChar("werr", WE_UNKNOWN);
  server.send(200, "text/html", msgPage("Saved. The device is restarting into normal mode."));
  portalDone = true;
}

void handleExit() {
  server.send(200, "text/html", msgPage("Exiting setup mode. The device is restarting."));
  portalDone = true;
}

void handleReset() {
  prefs.clear();
  prefs.putBool("zbreset", true);     // Zigbee-сеть сбросим при следующей загрузке
  prefs.putBool("sdreset", true);     // SD-архив очистится при доступности карты
  factoryResetRequested = true;
  server.send(200, "text/html", msgPage("Factory reset done. The device is restarting."));
  portalDone = true;
}

void runConfigMode() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(300);
  IPAddress ip = WiFi.softAPIP();

  server.on("/", HTTP_GET, handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/exit", HTTP_POST, handleExit);
  server.on("/reset", HTTP_POST, handleReset);
  server.onNotFound(handleRoot);
  server.begin();

  Serial.printf("Config portal: SSID=%s -> http://%s (password configured)\n", AP_SSID, ip.toString().c_str());
  drawConfigScreen(ip);

  uint32_t t0 = millis();
  while (!portalDone && millis() - t0 < PORTAL_MS) {
    server.handleClient();
    delay(2);
  }
  delay(1500);                         // дать браузеру получить ответ
  Serial.println("Config portal closed, restarting");
  if (!factoryResetRequested) {
    saveEpochForRestart();             // при обычном выходе сохраняем время; после сброса — нет
  }
  ESP.restart();
}

class MeasurementRequest {
public:
  void request() { forceMeasurementRequested = true; forceDisplayRequested = true; }
  void requestFullSync() {
    // Полная внешняя синхронизация:
    // прогноз, AQI, UV, координаты/место и NTP.
    forceExternalSyncRequested = true;
    forceMeasurementRequested = true;
    forceDisplayRequested = true;
  }
};

MeasurementRequest measurementRequest;

// ===================== КНОПКА =====================
// Один короткий клик — открыть Zigbee-сеть; двойной короткий — внепланово
// опросить датчики и обновить экран; удержание >1 с — открыть Wi-Fi-настройку.
void checkButton() {
  static uint32_t downAt = 0;
  static uint32_t pendingShortAt = 0;
  static bool longFired = false;
  const uint32_t now = millis();
  const bool pressed = (digitalRead(PIN_BUTTON) == LOW);

  if (pressed) {
    if (downAt == 0) downAt = now;
    if (!longFired && now - downAt > 1000UL) {
      longFired = true;
      pendingShortAt = 0;
      prefs.putBool("cfg", true);
      saveEpochForRestart();
      Serial.println("Button: rebooting into config mode");
      delay(100);
      ESP.restart();
    }
  } else {
    if (downAt != 0 && !longFired) {
      const uint32_t held = now - downAt;
      if (held >= 50UL && held < 1000UL) {
        if (pendingShortAt != 0 && now - pendingShortAt <= 500UL) {
          pendingShortAt = 0;
          measurementRequest.requestFullSync();
          Serial.println("Button: full update requested (sensors + external data + forecast)");
        } else {
          pendingShortAt = now;
        }
      }
    }
    downAt = 0;
    longFired = false;
  }

  // Один короткий клик — Zigbee pairing; двойной короткий — внеплановый опрос.
  if (pendingShortAt != 0 && now - pendingShortAt > 500UL) {
    pendingShortAt = 0;
    Serial.println("Button: opening Zigbee network for 3 min");
  zbOpenRequested = true;
  forceDisplayRequested = true;
  }
}

// ===================== ОБЪЕКТНАЯ АРХИТЕКТУРА =====================
// Классы-обёртки ниже — слой фасадов поверх рабочего кода: они не меняют поведение при рефакторинге.
// Интерфейсы описывают точки полиморфизма. Конкретные контроллеры и шлюзы ниже
// реализуют их, а MainModuleController связывает объекты композицией.
// Алгоритмы измерений и форматы Preferences/CSV сохранены.

class IUpdatable {
public:
  virtual void update(uint32_t now) = 0;
  virtual ~IUpdatable() = default;
};

class IStorage {
public:
  virtual void begin() = 0;
  virtual ~IStorage() = default;
};

class IDisplay {
public:
  virtual void showMain(float insideTemperature, float insideHumidity,
                        float pressureHpa) = 0;
  virtual ~IDisplay() = default;
};

class IClock {
public:
  virtual uint32_t now() = 0;
  virtual LT local() = 0;
  virtual bool night() = 0;
  virtual ~IClock() = default;
};

class SettingsController {
public:
  void begin() { loadSettings(); }
};

class StorageManager : public IStorage {
public:
  void begin() override {
    syncFailFlag = prefs.getBool("syncfail", false);
    wifiErr = prefs.getUChar("werr", WE_UNKNOWN);
    String detail = prefs.getString("wdet", "");
    detail.toCharArray(errDetail, sizeof(errDetail));
    homeLat = prefs.getFloat("lat", NAN);
    homeLon = prefs.getFloat("lon", NAN);
    homeFromIp = prefs.getBool("latip", false);
    syncFails = prefs.getUChar("syncfails", 0);

    if (prefs.getBool("epok", false)) {
      epochBase = prefs.getUInt("ep", 0);
      epochBaseMs = millis();
      prefs.putBool("epok", false);
    }

    memset(&hist, 0, sizeof(hist));
    if (prefs.getBytes("hist", &hist, sizeof(hist)) != sizeof(hist) ||
        hist.magic != HMAGIC) memset(&hist, 0, sizeof(hist));

    memset(&fc, 0, sizeof(fc));
    if (prefs.getBytes("fc", &fc, sizeof(fc)) != sizeof(fc) ||
        fc.magic != FCMAGIC) memset(&fc, 0, sizeof(fc));

    memset(&errorBars, 0, sizeof(errorBars));
    if (prefs.getBytes("ebars", &errorBars, sizeof(errorBars)) != sizeof(errorBars) ||
        errorBars.magic != EBMAGIC || errorBars.count > ERRBAR_COUNT || errorBars.next >= ERRBAR_COUNT)
      memset(&errorBars, 0, sizeof(errorBars));
  }

};

class TimeManager : public IClock, public IUpdatable {
public:
  uint32_t now() override { return epochNow(); }
  LT local() override { return localNow(); }
  bool night() override { return isNight(); }
  void updateLocation() { updateHome(); }
  void update(uint32_t) override { updateLocation(); }
};

class HistoryManager : public IUpdatable {
public:
  void update(uint32_t) override {
    IndoorSensorsGateway indoorSensors;   // шлюз без состояния
    sampleHistory(indoorSensors);
  }
};

class EInkDisplayGateway : public IDisplay {
public:
  void showMain(float insideTemperature, float insideHumidity,
                float pressureHpa) override {
    drawMain(insideTemperature, insideHumidity, pressureHpa);
  }
};

class UpdateScheduler {
public:
  void update(uint32_t now) { syncCheck(now); }
};

class ConnectionController : public IUpdatable {
private:
  WiFiGateway wifi;
  UpdateScheduler scheduler;
public:
  void update(uint32_t now) override { scheduler.update(now); }
  void synchronize() { runSyncMode(wifi); }
};

class ConfigPortal {
public:
  void run() { runConfigMode(); }
};

class ButtonController : public IUpdatable {
public:
  void update(uint32_t) override { checkButton(); }
};

// GPS-шлюз отделяет чтение UART от логики измерений.
class GPSGateway {
public:
  void update() {
    while (Serial1.available()) gps.encode(Serial1.read());
  }
};

class MeasurementController : public IUpdatable {
private:
  IndoorSensorsGateway indoorSensors;
  GPSGateway gpsGateway;

public:
  // GPS обрабатывается до обновления времени, как и в исходном цикле.
  void update(uint32_t) override {
    gpsGateway.update();
  }

  // Эти измерения вызываются после Zigbee update(), сохраняя прежний порядок.
  void updateMeasurements(uint32_t now) {
    const bool force = forceMeasurementRequested;
    if (force) {
      forceMeasurementRequested = false;
      forceDisplayRequested = true;
    }
    updateCo2(now, force);
    updatePressure(now, force);
  }

  void updateCo2(uint32_t now, bool force = false) {
    if (!scd4Ready && now - scd4RetryAtMs >= SENSOR_RETRY_INTERVAL_MS) {
      scd4RetryAtMs = now;
      scd4x.begin(Wire, SCD41_I2C_ADDR_62);
      scd4x.stopPeriodicMeasurement();
      delay(500);
      scd4Ready = scd4x.startPeriodicMeasurement() == 0;
      if (scd4Ready) Serial.println("SCD4x recovered");
    }
    if (!scd4Ready) return;
    if (!force && now - co2LastMs < 5000UL) return;
    co2LastMs = now;
    bool ready = false;
    if (scd4x.getDataReadyStatus(ready) != 0) {
      scd4Ready = false;
      scd4RetryAtMs = now;
      return;
    }
    if (ready) {
      uint16_t co2; float t, h;
      if (scd4x.readMeasurement(co2, t, h) == 0 && co2 >= 300 && co2 <= 10000) co2Val = co2;
      else {
        scd4Ready = false;
        scd4RetryAtMs = now;
      }
    }
  }

  void updatePressure(uint32_t now, bool force = false) {
    if (force || pCount == 0 || now - pLastMs >= PH_INTERVAL_MS) {
      pLastMs = now;
      float pressureHpa = indoorSensors.readPressure();
      float temperatureC = NAN;
      if (zbLinkOk()) temperatureC = outT;
      else {
        float webT, webH;
        if (webOutdoor(webT, webH)) temperatureC = webT;
      }
      pushTrendSample(pressureHpa, temperatureC);
    }
  }

  void readIndoor(float &temperature, float &humidity, float &pressureHpa) {
    indoorSensors.read(temperature, humidity, pressureHpa);
  }
};

class MyZigbeeGateway : public IUpdatable {
private:
  bool reportingConfigured = false;
  uint32_t lastOpenMs = 0;
  bool linkLost = false;      // связь была потеряна после того, как была (для повторной настройки отчётов)
  bool prevLinkOk = false;
  bool pairedSaved = false;   // признак «модуль уже привязан» сохранён во флеш

public:
  void begin() {
    zbThermostat.onTempReceive(onOutTemp);
    zbThermostat.onHumidityReceive(onOutHum);

    zbThermostat.setManufacturerAndModel(
      "Meteo", "MainModule"
    );

    Zigbee.addEndpoint(&zbThermostat);

    // Если наружный модуль уже привязывался — сеть при загрузке не открываем (он перепривяжется сам).
    pairedSaved = prefs.getBool("zbpaired", false);
    if (!pairedSaved) Zigbee.setRebootOpenNetwork(180);

    if (!Zigbee.begin(ZIGBEE_COORDINATOR)) {
      Serial.println("Zigbee failed to start, rebooting");
      delay(1000);
      ESP.restart();
    }
    // После запуска Zigbee сеть открыта для подключения на 3 минуты
    uint32_t startedAt = millis();

    lastOpenMs = startedAt;
    if (pairedSaved) {
      zbOpenUntilMs = 0;
      outdoorState = OUT_NOT_WAITING;
    } else {
      zbOpenUntilMs = startedAt + 180000UL;
      outdoorState = OUT_WAITING;
    }

    Serial.println("Zigbee: waiting for outdoor module");
  }

  void update(uint32_t now) override {

    // Запрос на открытие сети кнопкой
    if (zbOpenRequested) {
      zbOpenRequested = false;

      Zigbee.openNetwork(180);
      zbOpenUntilMs = now + 180000UL;
      lastOpenMs = now;

      Serial.println("Zigbee: network open for 180 sec");
    }

    // Автоматическое повторное открытие,
    // если наружный модуль ещё не привязан
    if (!zbThermostat.bound() &&
        now - lastOpenMs >= 300000UL) {

      Zigbee.openNetwork(60);
      zbOpenUntilMs = now + 60000UL;
      lastOpenMs = now;

      Serial.println("Zigbee: reopening network for 60 sec");
    }

    // Сброс флага настройки отчётов: потеря привязки или восстановление связи после потери
    const bool boundNow = zbThermostat.bound();
    const bool linkNow = zbLinkOk();
    if (!boundNow) reportingConfigured = false;
    if (prevLinkOk && !linkNow) linkLost = true;
    if (linkNow && linkLost) {
      reportingConfigured = false;
      linkLost = false;
    }
    prevLinkOk = linkNow;
    if (boundNow && !pairedSaved) {      // запоминаем, что модуль привязан
      pairedSaved = true;
      prefs.putBool("zbpaired", true);
    }

    // Настройка передачи измерений
    if (zbThermostat.bound() && !reportingConfigured) {
      zbThermostat.setTemperatureReporting(10, 60, 0.2f);
      zbThermostat.setHumidityReporting(10, 60, 1.0f);

      reportingConfigured = true;

      Serial.println("Zigbee: reporting configured");
    }

    // Определение состояния
    const OutdoorState prevState = outdoorState;
    if (zbLinkOk()) {
      outdoorState = OUT_CONNECTED;
    }
    else if ((int32_t)(zbOpenUntilMs - now) > 0) {
      outdoorState = OUT_WAITING;
    }
    else {
      outdoorState = OUT_NOT_WAITING;
    }

    // Наружный модуль только что подключился — сразу перерисовать экран (температура с датчика)
    if (outdoorState == OUT_CONNECTED && prevState != OUT_CONNECTED) forceDisplayRequested = true;
  }
};
// Контроллер дисплея использует существующие функции отрисовки.
// Настройки отображения остаются в cfg и читаются исходными функциями.
class DisplayController {
private:
  EInkDisplayGateway display;
public:
  void showMain(float temperature, float humidity, float pressure) {
    IDisplay &displayApi = display;
    displayApi.showMain(temperature, humidity, pressure);
  }
};

// ===================== SETUP / LOOP =====================
// Главный класс приложения. Он владеет подсистемами и задаёт порядок их
// работы. Низкоуровневые драйверы остаются общими объектами файла, чтобы
// структурный рефакторинг не менял их жизненный цикл и порядок инициализации.
class MainModuleController {
private:
  SettingsController settings;
  StorageManager storage;
  TimeManager timeManager;
  HistoryManager history;
  ConnectionController network;
  ConfigPortal portal;
  ButtonController button;
  MeasurementController sensors;
  MyZigbeeGateway zigbee;
  DisplayController displayController;

public:
  void begin();
  void update();
};

void MainModuleController::begin() {
  startupAllIcons = false;
  startupAllIconsUntilMs = 0;
  Serial.begin(115200);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  prefs.begin("meteo", false);
  startupAllIcons = !prefs.getBool("skipStartupTest", false);
  prefs.putBool("skipStartupTest", false);
  airQuality = prefs.getShort("eaqi", -1);
  forecastUv = prefs.getFloat("uv", NAN);
  alertGroup1 = (AlertGroup1)prefs.getUChar("alert_g1", GROUP1_NONE);
  alertGroup2 = (AlertGroup2)prefs.getUChar("alert_g2", GROUP2_NONE);
  tempAlert = (TempAlert)prefs.getUChar("tempwarn", TEMP_NONE);
  forecastCurrentTemp = prefs.getFloat("fc_temp", NAN);
  forecastWindMs = prefs.getFloat("fc_wind", NAN);
  forecastPrecipMm = prefs.getFloat("fc_precip", NAN);
  forecastHumidity = prefs.getFloat("fc_hum", NAN);
  forecastWmoCode = prefs.getUChar("fc_wmo", 255);
  settings.begin();
  bool wantConfig  = prefs.getBool("cfg", false);
  bool wantZbReset = prefs.getBool("zbreset", false);
  bool wantSync    = prefs.getBool("sync", false);
  IStorage &storageApi = storage;
  storageApi.begin();              // флаги и ошибки синхронизации, координаты, время, история и прогноз из Flash
  skipFirstDraw = prefs.getBool("skipdraw", false);
  if (skipFirstDraw) {             // повторная неудача синхронизации с тем же кодом — экран не трогаем
    prefs.putBool("skipdraw", false);
    startupAllIcons = false;
  }
  if (prefs.getBool("notest", false)) {   // загрузка после синхронизации
    prefs.putBool("notest", false);
    startupAllIcons = false;
  }

  SPI.begin(PIN_EPD_SCK, PIN_SD_MISO, PIN_EPD_MOSI, PIN_EPD_CS);
  sdCardController.begin(); // SD-архив восстанавливает последние 24 часа, если карта доступна

  if (wantConfig) {
    prefs.putBool("cfg", false);       // чтобы не зациклиться при сбое
    portal.run();                      // не возвращается
  }

  if (wantSync) {
    prefs.putBool("sync", false);      // чтобы не зациклиться при сбое
    network.synchronize();             // не возвращается (экран не трогаем — картинка остаётся)
  }

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  sht4Ready = sht4.begin(&Wire);
  sht4RetryAtMs = millis();
  if (!sht4Ready) Serial.println("SHT45 not found; retrying every 30 seconds");
  else sht4.setPrecision(SHT4X_HIGH_PRECISION);
  bmeReady = bme.begin(0x76, &Wire) || bme.begin(0x77, &Wire);
  bmeRetryAtMs = millis();
  if (!bmeReady) Serial.println("BME280 not found; retrying every 30 seconds");

  scd4x.begin(Wire, SCD41_I2C_ADDR_62);
  scd4x.stopPeriodicMeasurement();     // на случай, если уже была запущена раньше
  delay(500);
  scd4Ready = scd4x.startPeriodicMeasurement() == 0;
  scd4RetryAtMs = millis();
  if (!scd4Ready) Serial.println("SCD4x not found; retrying every 30 seconds");

  Serial1.setRxBufferSize(1024);       // больше буфер UART — NMEA не теряется во время долгих операций
  Serial1.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  clockBaseMillis = millis();

  zigbee.begin();                    // регистрация наружного Zigbee-термостата

  if (wantZbReset) {
    prefs.putBool("zbreset", false);
    Serial.println("Zigbee factory reset");
    Zigbee.factoryReset(true);         // очистить Zigbee NVRAM и перезагрузить плату
  }
}

void MainModuleController::update() {
  static uint32_t lastDrawMs = 0;
  static bool firstDraw = !skipFirstDraw;
  uint32_t now = millis();
  
  // Полиморфный вызов сервисов. Порядок сохранён: GPS -> кнопка -> время ->
  // история -> сеть -> Zigbee. Все подсистемы принадлежат MainModuleController.
  IUpdatable *services[] = {
    &sensors, &button, &timeManager, &history, &network, &zigbee
  };
  for (IUpdatable *service : services) service->update(now);

  // CO2 и давление опрашиваются после Zigbee, как в исходном цикле.
  sensors.updateMeasurements(now);

  if (forceExternalSyncRequested) {
    forceExternalSyncRequested = false;

    if (cfg.ssid.length() > 0) {
      Serial.println("Button: starting full external synchronization");

      prefs.putBool("skipStartupTest", true);

      prefs.putBool("sync", true);
      saveEpochForRestart();
      delay(100);
      ESP.restart();
    } else {
      Serial.println("Button: full sync requested, but Wi-Fi is not configured");
    }
  }
  // Обновление экрана (ночью — реже)
  IClock &clockApi = timeManager;
  uint32_t periodMs = (clockApi.night() ? cfg.nightRefreshMin : cfg.refreshMin) * 60UL * 1000UL;
  if (firstDraw || forceDisplayRequested || now - lastDrawMs >= periodMs) {
    forceDisplayRequested = false;
    firstDraw = false;
    lastDrawMs = now;

    float inT, inH, pr;
    sensors.readIndoor(inT, inH, pr);
    Serial.printf("IN %.1fC %.1f%% | %.1f hPa (%.0f mmHg) | OUT %.1fC %.1f%% | zb=%d\n",
                  inT, inH, pr, pr * 0.750062f, outT, outH, zbLinkOk());
    displayController.showMain(inT, inH, pr);

    // Отсчитываем 5 секунд после фактического вывода экрана со всеми значками.
    if (startupAllIcons && startupAllIconsUntilMs == 0) {
      startupAllIconsUntilMs = millis() + 5000UL;
    }
  }

  if (startupAllIcons && startupAllIconsUntilMs != 0 &&
      (int32_t)(now - startupAllIconsUntilMs) >= 0) {
    startupAllIcons = false;
    startupAllIconsUntilMs = 0;
    forceDisplayRequested = true; // сразу перерисовать экран: значки только по реальным состояниям
  }

  delay(10);
}


// Единственная точка входа Arduino.
MainModuleController station;

void setup() {
  station.begin();
}

void loop() {
  station.update();
}
