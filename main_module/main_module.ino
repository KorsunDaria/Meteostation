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
 *   - Прогноз на 5 дней (Open-Meteo) + город/район (OpenStreetMap Nominatim).
 *     Wi-Fi и Zigbee вместе не работают, поэтому раз в 3 часа плата перезагружается
 *     в короткий режим синхронизации (~15-30 с), скачивает данные, сохраняет их
 *     и возвращается в обычный режим. Пока прогноза нет — график крупнее,
 *     в шапке только время и дата. Заодно по NTP уточняется время.
 *   - Восход/закат считаются по координатам (GPS, либо последние сохранённые).
 *
 * РЕЖИМ ТЕСТИРОВАНИЯ: экран обновляется раз в минуту, история давления для
 * иконки погоды копится каждые 30 с. Для реальной эксплуатации см. пометки
 * "// TESTING" ниже — верни там нормальные интервалы.
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
 *   - короткое нажатие (<1 с) — сразу открыть Zigbee-сеть на 3 минуты для
 *     подключения побочного модуля. Без перезагрузки, без Wi-Fi — быстро.
 *   - удержание >1 с — Wi-Fi-настройка как раньше: точка доступа EInk-Meteo
 *     + веб-страница на 5 минут (там же — часовой пояс и ключ геокодера).
 * Кроме того, пока побочный модуль не привязан, сеть и так переоткрывается
 * автоматически каждую минуту (см. loop()) — отдельно жать кнопку не обязательно.
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
#include <Wire.h>
#include <SPI.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <GxEPD2_BW.h>
#include <Fonts/FreeMonoBold24pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeMono9pt7b.h>
#include <Adafruit_SHT4x.h>
#include <Adafruit_BME280.h>
#include <SensirionI2cScd4x.h>
#include <TinyGPSPlus.h>
#include "Zigbee.h"

// ===================== ПИНЫ (меняй под свою разводку) =====================
#define PIN_EPD_CS    10
#define PIN_EPD_DC    2
#define PIN_EPD_RST   3
#define PIN_EPD_BUSY  4
#define PIN_EPD_SCK   6
#define PIN_EPD_MOSI  7

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

#define SYNC_PERIOD_MS (3UL * 60UL * 60UL * 1000UL)  // прогноз раз в 3 часа
#define FIRST_SYNC_MS  (120UL * 1000UL)              // первая попытка через 2 мин после старта
#define FC_MIN_DAYS    3                             // меньше дней в прогнозе — считаем "нет интернета"

#define HN         96                  // точек истории: 96 * 15 мин = 24 ч
#define HSLOT_SEC  900UL
#define HMAGIC     0xE1C2
#define FCMAGIC    0xFC01

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
uint32_t clockBaseMillis = 0; // millis() в момент старта — точка отсчёта запасных софт-часов
ZigbeeThermostat zbThermostat = ZigbeeThermostat(THERMOSTAT_EP);
Preferences prefs;
WebServer server(80);

Hist     hist;                // история графика
Forecast fc;                  // прогноз + город/район
float    homeLat = NAN, homeLon = NAN;   // последние известные координаты
uint32_t epochBase = 0;       // UTC-время (unix), соответствующее epochBaseMs; 0 = неизвестно
uint32_t epochBaseMs = 0;
bool     syncFailFlag = false;
uint8_t  wifiErr = WE_UNKNOWN;          // результат последней попытки Wi-Fi
volatile uint8_t lastDiscReason = 0;    // причина последнего разрыва Wi-Fi (из события)

// ===================== НАСТРОЙКИ =====================
struct Settings {
  String   ssid;
  String   pass;
  uint16_t refreshMin      = 1;     // TESTING: период обновления экрана днём, мин (в бою поставь 5-15)
  uint8_t  nightStart      = 23;    // ночной режим: с (час)
  uint8_t  nightEnd        = 7;     // ... до (час)
  uint16_t nightRefreshMin = 1;     // TESTING: период обновления ночью, мин (в бою поставь 30-60)
  uint16_t zbIntervalSec   = 60;    // как часто опрашиваем наружный модуль, с
  int8_t   tz              = 0;     // часовой пояс (UTC+N)
  uint16_t clockSetMin     = 480;   // запасное время (если нет GPS и NTP), мин. с полуночи
  String   geoKey;                  // API-ключ Dadata для определения города/района (dadata.ru)
} cfg;

void loadSettings() {
  cfg.ssid            = prefs.getString("ssid", "");
  cfg.pass            = prefs.getString("pass", "");
  cfg.refreshMin      = prefs.getUShort("refr", 5);
  cfg.nightStart      = prefs.getUChar("ns", 23);
  cfg.nightEnd        = prefs.getUChar("ne", 7);
  cfg.nightRefreshMin = prefs.getUShort("nr", 60);
  cfg.zbIntervalSec   = prefs.getUShort("zbi", 60);
  cfg.tz              = prefs.getChar("tz", 0);
  cfg.clockSetMin     = prefs.getUShort("cmin", 480);
  cfg.geoKey          = prefs.getString("geokey", "");
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
  prefs.putString("geokey", cfg.geoKey);
}

// ===================== ДАННЫЕ НАРУЖНОГО МОДУЛЯ (Zigbee) =====================
float    outT = NAN, outH = NAN;
uint32_t outLastMs = 0;

void onOutTemp(float t)  { outT = t; outLastMs = millis(); }
void onOutHum(float h)   { outH = h; outLastMs = millis(); }

bool zbLinkOk() {
  return zbThermostat.bound() && outLastMs != 0 &&
         (millis() - outLastMs) < (3UL * cfg.zbIntervalSec * 1000UL + 5000UL);
}

// ===================== ДАВЛЕНИЕ / ТРЕНД ДЛЯ ИКОНКИ =====================
#define PH_N 12                        // TESTING: 12 точек по 30с = 6 минут (в бою поставь 15 мин, будет 3 часа)
#define PH_INTERVAL_MS (30UL * 1000UL) // TESTING: в бою поставь 15UL*60UL*1000UL
float    pHist[PH_N];
uint8_t  pCount = 0;
uint32_t pLastMs = 0;

void pushPressure(float p) {
  if (isnan(p)) return;
  if (pCount < PH_N) {
    pHist[pCount++] = p;
  } else {
    memmove(pHist, pHist + 1, (PH_N - 1) * sizeof(float));
    pHist[PH_N - 1] = p;
  }
}

Trend forecastTrend() {
  if (pCount < 4) return TREND_UNKNOWN;
  float d = pHist[pCount - 1] - pHist[0];
  if (d >  1.5f) return TREND_IMPROVING;
  if (d < -1.5f) return TREND_WORSENING;
  return TREND_STABLE;
}

const char *trendText(Trend t) {
  switch (t) {
    case TREND_IMPROVING: return "Improving";
    case TREND_WORSENING: return "Worsening";
    case TREND_STABLE:    return "Steady";
    default:              return "No data";
  }
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

// Ощущаемая температура (формула Стедмана без ветра: у станции нет анемометра)
static float feelsLike(float t, float h) {
  float e = (h / 100.0f) * 6.105f * expf(17.27f * t / (237.7f + t));
  return t + 0.33f * e - 4.0f;
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

// Запоминаем координаты (для восхода/заката и прогноза, когда GPS временно без фикса)
void updateHome() {
  static uint32_t last = 0;
  if (millis() - last < 60000UL) return;
  last = millis();
  if (!gps.location.isValid() || gps.location.age() > 10000) return;
  float la = (float)gps.location.lat(), lo = (float)gps.location.lng();
  if (isnan(homeLat) || fabsf(la - homeLat) > 0.05f || fabsf(lo - homeLon) > 0.05f) {
    homeLat = la; homeLon = lo;
    prefs.putFloat("lat", la);
    prefs.putFloat("lon", lo);
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

void sampleHistory() {
  static uint32_t lastChk = 0;
  uint32_t now = millis();
  if (now - lastChk < 5000UL) return;
  lastChk = now;
  uint32_t e = epochNow();
  if (!e) return;                      // без времени историю не пишем
  uint32_t slot = e / HSLOT_SEC;
  if (hist.magic == HMAGIC && slot <= hist.headSlot) return;

  sensors_event_t hum, temp;
  float inH = NAN;
  if (sht4.getEvent(&hum, &temp)) inH = hum.relative_humidity;
  float pHpa = bme.readPressure() / 100.0f;
  histAdd(slot, zbLinkOk() ? outT : NAN, inH, isnan(pHpa) ? NAN : pHpa * 0.750062f);
  histSave();
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

// Крупная наружная температура: целая часть х2, десятые обычным размером
void drawBigTemp(int x, int y, float t) {
  display.setFont(&FreeMonoBold24pt7b);
  if (isnan(t)) {
    display.setTextSize(2);
    display.setCursor(x, y);
    display.print("--");
    display.setTextSize(1);
    return;
  }
  int tenths = (int)lroundf(t * 10.0f);
  bool neg = tenths < 0;
  int a = abs(tenths);
  char ip[10], fp[4];
  snprintf(ip, sizeof(ip), "%s%d", neg ? "-" : "", a / 10);
  snprintf(fp, sizeof(fp), ".%d", a % 10);
  display.setTextSize(2);
  display.setCursor(x, y);
  display.print(ip);
  int cx = display.getCursorX();
  display.setTextSize(1);
  display.setCursor(cx, y);
  display.print(fp);
  cx = display.getCursorX();
  display.drawCircle(cx + 9, y - 50, 6, GxEPD_BLACK);
  display.setCursor(cx + 22, y - 28);
  display.print("C");
}

void cell(int x, int y, const char *label, const char *val) {
  display.setFont(&FreeMono9pt7b);
  display.setCursor(x, y);
  display.print(label);
  display.setFont(&FreeMonoBold12pt7b);
  display.setCursor(x, y + 20);
  display.print(val);
}

void cellTemp(int x, int y, const char *label, float t) {
  display.setFont(&FreeMono9pt7b);
  display.setCursor(x, y);
  display.print(label);
  display.setFont(&FreeMonoBold12pt7b);
  printTempC(x, y + 20, t, 1, 2, 12);
}

// ===================== ИКОНКИ ПОГОДЫ =====================
void iconSun(int cx, int cy, int r) {
  display.fillCircle(cx, cy, r, GxEPD_BLACK);
  for (int a = 0; a < 360; a += 45) {
    float rad = a * 3.14159f / 180.0f;
    int x1 = cx + (int)(cos(rad) * (r + 5));
    int y1 = cy + (int)(sin(rad) * (r + 5));
    int x2 = cx + (int)(cos(rad) * (r + 14));
    int y2 = cy + (int)(sin(rad) * (r + 14));
    display.drawLine(x1, y1, x2, y2, GxEPD_BLACK);
  }
}

void iconMoon(int cx, int cy, int r) {
  display.fillCircle(cx, cy, r, GxEPD_BLACK);
  display.fillCircle(cx + r / 2, cy - r / 3, (int)(r * 0.85f), GxEPD_WHITE);
}

void iconCloud(int cx, int cy, int scale) {
  display.fillRoundRect(cx - scale, cy, scale * 2, (int)(scale * 0.8f), scale / 2, GxEPD_BLACK);
  display.fillCircle(cx - scale / 2, cy, (int)(scale * 0.65f), GxEPD_BLACK);
  display.fillCircle(cx + scale / 6, cy - scale / 4, (int)(scale * 0.8f), GxEPD_BLACK);
  display.fillCircle(cx + scale, cy, (int)(scale * 0.55f), GxEPD_BLACK);
}

// Большая иконка по тренду давления (слева вверху). (x, y) — левый верхний угол области ~90x70
void drawWeatherIcon(int x, int y, Trend trend, bool night) {
  int cx = x + 45, cy = y + 35;
  switch (trend) {
    case TREND_IMPROVING:
      if (night) iconMoon(cx, cy, 20); else iconSun(cx, cy, 18);
      break;
    case TREND_WORSENING:
      iconCloud(cx, cy - 5, 22);
      display.drawLine(cx - 15, cy + 25, cx - 22, cy + 38, GxEPD_BLACK);
      display.drawLine(cx,      cy + 25, cx - 7,  cy + 38, GxEPD_BLACK);
      display.drawLine(cx + 15, cy + 25, cx + 8,  cy + 38, GxEPD_BLACK);
      break;
    case TREND_STABLE:
      if (night) iconMoon(cx - 18, cy - 10, 13); else iconSun(cx - 18, cy - 10, 12);
      iconCloud(cx + 12, cy + 8, 20);
      break;
    default:
      display.drawCircle(cx, cy, 20, GxEPD_BLACK);
      display.setFont(&FreeMonoBold12pt7b);
      display.setCursor(cx - 7, cy + 7);
      display.print("?");
      break;
  }
}

// Маленькая иконка по коду погоды WMO (для прогноза на 5 дней), центр (cx, cy), размер ~40x40
void drawWmo(int cx, int cy, int code) {
  if (code == 0) {
    iconSun(cx, cy, 7);
  } else if (code <= 2) {
    iconSun(cx - 10, cy - 9, 5);
    iconCloud(cx + 5, cy + 5, 11);
  } else if (code == 3) {
    iconCloud(cx - 2, cy, 14);
  } else if (code == 45 || code == 48) {
    for (int k = 0; k < 3; k++) display.drawFastHLine(cx - 16 + k * 3, cy - 10 + k * 9, 32 - k * 6, GxEPD_BLACK);
  } else if ((code >= 51 && code <= 67) || (code >= 80 && code <= 82)) {
    iconCloud(cx - 2, cy - 6, 12);
    for (int k = -1; k <= 1; k++) display.drawLine(cx + k * 10, cy + 10, cx + k * 10 - 4, cy + 19, GxEPD_BLACK);
  } else if ((code >= 71 && code <= 77) || code == 85 || code == 86) {
    iconCloud(cx - 2, cy - 6, 12);
    for (int k = -1; k <= 1; k++) display.fillCircle(cx + k * 10, cy + 15, 2, GxEPD_BLACK);
  } else if (code >= 95) {
    iconCloud(cx - 2, cy - 6, 12);
    display.drawLine(cx + 2, cy + 8, cx - 4, cy + 15, GxEPD_BLACK);
    display.drawLine(cx - 4, cy + 15, cx + 3, cy + 15, GxEPD_BLACK);
    display.drawLine(cx + 3, cy + 15, cx - 3, cy + 23, GxEPD_BLACK);
  } else {
    iconCloud(cx - 2, cy, 14);
  }
}

// ===================== ГРАФИК ЗА 24 ЧАСА =====================
void ditherBar(int x, int y0, int w, int h) {   // "серый" столбик шахматкой; y0 — нижняя граница
  for (int yy = y0 - h; yy < y0; yy++)
    for (int xx = x; xx < x + w; xx++)
      if (((xx + yy) & 1) == 0) display.drawPixel(xx, yy, GxEPD_BLACK);
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
  if (tmn > tmx) { tmn = 0; tmx = 10; }
  int step = (tmx - tmn > 30.0f) ? 10 : 5;
  int lo = (int)floorf(tmn / step) * step;
  int hi = (int)ceilf(tmx / step) * step;
  if (hi - lo < 2 * step) hi = lo + 2 * step;
  auto YT = [&](float v) { return gy1 - (int)((v - lo) * (gy1 - gy0) / (float)(hi - lo)); };

  // --- сетка и подписи слева (температура)
  display.setFont(&FreeMono9pt7b);
  char buf[24];
  for (int v = lo; v <= hi; v += step) {
    int y = YT(v);
    for (int x = gx0; x < gx1; x += 6) display.drawFastHLine(x, y, 2, GxEPD_BLACK);
    snprintf(buf, sizeof(buf), "%d", v);
    printRight(gx0 - 6, y + 4, buf);
  }
  // --- подписи справа (влажность, 0-100 %)
  display.setCursor(gx1 + 6, gy1 + 4);              display.print("0");
  display.setCursor(gx1 + 6, (gy0 + gy1) / 2 + 4);  display.print("50");
  display.setCursor(gx1 + 6, gy0 + 4);              display.print("100");

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

  // --- легенда
  int ly = gy1 + 44;
  int lx = gx0 - 20;
  display.drawFastHLine(lx, ly - 4, 16, GxEPD_BLACK);
  display.drawFastHLine(lx, ly - 3, 16, GxEPD_BLACK);
  display.setCursor(lx + 22, ly); display.print("Out temp C");
  lx += 22 + textW("Out temp C") + 14;
  ditherBar(lx, ly, 14, 10);
  display.setCursor(lx + 20, ly); display.print("In hum %");
  lx += 20 + textW("In hum %") + 14;
  for (int k = 0; k < 16; k += 6) display.drawFastHLine(lx + k, ly - 4, 3, GxEPD_BLACK);
  if (havePress) snprintf(buf, sizeof(buf), "mmHg %d-%d", (int)lroundf(pmn), (int)lroundf(pmx));
  else           snprintf(buf, sizeof(buf), "mmHg");
  display.setCursor(lx + 22, ly); display.print(buf);
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
  if (gps.location.isValid() && gps.location.age() < 60000) { la = gps.location.lat(); lo = gps.location.lng(); }
  else if (!isnan(homeLat)) { la = homeLat; lo = homeLon; }
  strcpy(sRise, "--:--"); strcpy(sSet, "--:--");
  if (lt.haveDate && !isnan(la)) {
    int m1, m2;
    if (sunEvent(true,  la, lo, lt.y, lt.mo, lt.d, cfg.tz, m1)) snprintf(sRise, sizeof(sRise), "%02d:%02d", m1 / 60, m1 % 60);
    if (sunEvent(false, la, lo, lt.y, lt.mo, lt.d, cfg.tz, m2)) snprintf(sSet,  sizeof(sSet),  "%02d:%02d", m2 / 60, m2 % 60);
  }

  if (isnan(prHpa)) strcpy(sPress, "--- mmHg");
  else              snprintf(sPress, sizeof(sPress), "%.0f mmHg", prHpa * 0.750062f);
  if (isnan(outH)) strcpy(sOutH, "-- %"); else snprintf(sOutH, sizeof(sOutH), "%.0f %%", outH);
  if (isnan(inH))  strcpy(sInH,  "-- %"); else snprintf(sInH,  sizeof(sInH),  "%.0f %%", inH);
  if (co2Val == 0) strcpy(sCo2, "--- ppm"); else snprintf(sCo2, sizeof(sCo2), "%u ppm", co2Val);
  snprintf(sSat, sizeof(sSat), "%u", (unsigned)gps.satellites.value());

  bool night = isNight();
  Trend trend = forecastTrend();
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

  display.init(115200, true, 50, false);
  display.setRotation(0);              // 800x480
  display.setFullWindow();
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setTextColor(GxEPD_BLACK);
    display.setTextSize(1);

    // ---- слева вверху: иконка тренда + крупная наружная температура + "Feels like"
    drawWeatherIcon(25, 25, trend, night);
    display.setFont(&FreeMono9pt7b);
    printCenter(70, 128, trendText(trend));

    drawBigTemp(150, 108, outT);

    display.setFont(&FreeMonoBold12pt7b);
    display.setCursor(150, 142);
    display.print("Feels like ");
    if (!isnan(outT) && !isnan(outH)) printTempC(display.getCursorX(), 142, feelsLike(outT, outH), 1, 2, 12);
    else display.print("--");

    // ---- шапка справа
    if (online) {
      display.setFont(&FreeMonoBold12pt7b);
      if (fc.city[0]) printRight(790, 30, fc.city);
      display.setFont(&FreeMono9pt7b);
      if (fc.dist[0]) printRight(790, 50, fc.dist);
      display.setFont(&FreeMonoBold12pt7b);
      if (sDate[0]) printRight(790, 78, sDate);
      display.setFont(&FreeMonoBold24pt7b);
      printRight(790, 112, sTime);

      // ---- прогноз на 5 дней
      for (int i = 0; i < fcCnt; i++) {
        int k = fcFirst + i;
        int cx = 509 + i * 62;
        int32_t days = daysFromCivil((int)(fc.date[k] / 10000), (unsigned)((fc.date[k] / 100) % 100), (unsigned)(fc.date[k] % 100));
        display.setFont(&FreeMono9pt7b);
        printCenter(cx, 148, WD3[(days + 4) % 7]);
        drawWmo(cx, 175, fc.code[k]);
        char b[8];
        display.setFont(&FreeMonoBold12pt7b);
        snprintf(b, sizeof(b), "%d", (int)fc.tmax[k]);
        printCenter(cx, 212, b);
        display.setFont(&FreeMono9pt7b);
        snprintf(b, sizeof(b), "%d", (int)fc.tmin[k]);
        printCenter(cx, 230, b);
      }
    } else {
      // нет интернета: только время и дата, крупно
      display.setFont(&FreeMonoBold24pt7b);
      display.setTextSize(2);
      printRight(790, 88, sTime);
      display.setTextSize(1);
      display.setFont(&FreeMonoBold12pt7b);
      if (sDate[0]) printRight(790, 120, sDate);
    }

    // ---- сетка показателей (2 колонки x 5 строк)
    const int c1 = 15, c2 = 165;
    cell(c1, 250, "Sunrise", sRise);
    cell(c2, 250, "Sunset", sSet);
    cell(c1, 293, "Out humidity", sOutH);
    cell(c2, 293, "Pressure", sPress);
    cellTemp(c1, 336, "In temp", inT);
    cell(c2, 336, "In humidity", sInH);
    cell(c1, 379, "CO2", sCo2);
    cell(c2, 379, "GPS sats", sSat);
    cell(c1, 422, "Outdoor link", sLink);
    cell(c2, 422, "Updated", sTime);

    // ---- состояние Wi-Fi внизу: только пока не настроено или при ошибке.
    // Если всё хорошо (ssid задан и последняя синхронизация успешна) — строку не рисуем совсем.
    bool showWifi = (cfg.ssid.length() == 0) || (wifiErr != WE_OK);
    if (showWifi) {
      display.setFont(&FreeMono9pt7b);
      display.setCursor(15, 474);
      display.print(sWifi);
    }

    // ---- график
    if (online) drawGraph(350, 255, 745, 412);
    else        drawGraph(350, 175, 745, 412);
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
bool httpsGet(const String &url, String &body) {
  // --- диагностика: DNS, голый TCP, затем TLS ---
  String host = url.substring(url.indexOf("://") + 3);
  host = host.substring(0, host.indexOf('/'));

  IPAddress ip;
  bool dnsOk = WiFi.hostByName(host.c_str(), ip);
  Serial.printf("httpsGet: DNS %s -> %s\n", host.c_str(), dnsOk ? ip.toString().c_str() : "FAIL");

  {
    WiFiClient t;
    uint32_t t0 = millis();
    bool a = dnsOk && t.connect(ip, 443, 5000);
    Serial.printf("httpsGet: TCP %s: %d errno=%d %ums\n", host.c_str(), a, errno, (unsigned)(millis() - t0));
    t.stop();

    t0 = millis();
    bool b = t.connect("google.com", 443, 5000);
    Serial.printf("httpsGet: TCP google.com: %d errno=%d %ums\n", b, errno, (unsigned)(millis() - t0));
    t.stop();
  }

  WiFiClientSecure client;
  client.setInsecure();                // без проверки сертификата (данные не секретные)

  if (!client.connect(host.c_str(), 443, 8000)) {
    char e[100] = {0};
    int le = client.lastError(e, sizeof(e));
    Serial.printf("httpsGet: TLS connect FAIL, lastError=%d (%s)\n", le, e);
  } else {
    Serial.println("httpsGet: TLS connect OK");
  }
  client.stop();

  // --- сам запрос ---
  HTTPClient http;
  http.setTimeout(10000);
  http.setUserAgent("EInkMeteo/1.0 (ESP32 weather station)");
  if (!http.begin(client, url)) {
    Serial.println("httpsGet: http.begin() failed (плохой URL?)");
    return false;
  }
  Serial.printf("httpsGet: GET %s  (free heap=%u)\n", url.c_str(), ESP.getFreeHeap());
  int code = http.GET();
  if (code < 0) {
    Serial.printf("httpsGet: ошибка соединения, код=%d (%s)\n", code, HTTPClient::errorToString(code).c_str());
    http.end();
    return false;
  }
  if (code == HTTP_CODE_OK) {
    body = http.getString();
  } else {
    Serial.printf("httpsGet: сервер ответил HTTP %d, тело: %s\n", code, http.getString().c_str());
  }
  http.end();
  return code == HTTP_CODE_OK;
}

void copyAscii(char *dst, size_t n, const char *src) {   // шрифты только ASCII — остальное отбрасываем
  size_t j = 0;
  for (size_t i = 0; src && src[i] && j + 1 < n; i++) {
    unsigned char c = (unsigned char)src[i];
    if (c >= 0x20 && c < 0x7F) dst[j++] = (char)c;
  }
  dst[j] = 0;
}

bool fetchForecast(float la, float lo) {
  char url[256];
  snprintf(url, sizeof(url),
           "https://api.open-meteo.com/v1/forecast?latitude=%.4f&longitude=%.4f"
           "&daily=weather_code,temperature_2m_max,temperature_2m_min&timezone=auto&forecast_days=5",
           la, lo);
  String body;
  if (!httpsGet(url, body)) return false;
  JsonDocument doc;
  DeserializationError je = deserializeJson(doc, body);
  if (je) {
    Serial.printf("fetchForecast: ошибка разбора JSON: %s\n", je.c_str());
    Serial.printf("fetchForecast: тело ответа (первые 300 символов): %.300s\n", body.c_str());
    return false;
  }
  JsonArray tm = doc["daily"]["time"];
  JsonArray wc = doc["daily"]["weather_code"];
  JsonArray hi = doc["daily"]["temperature_2m_max"];
  JsonArray lw = doc["daily"]["temperature_2m_min"];
  if (tm.isNull() || wc.isNull() || hi.isNull() || lw.isNull()) {
    Serial.println("fetchForecast: JSON разобрался, но нет ожидаемых полей daily.*");
    Serial.printf("fetchForecast: тело ответа (первые 300 символов): %.300s\n", body.c_str());
    return false;
  }

  char keepCity[sizeof(fc.city)], keepDist[sizeof(fc.dist)];
  memcpy(keepCity, fc.city, sizeof(keepCity));
  memcpy(keepDist, fc.dist, sizeof(keepDist));
  memset(&fc, 0, sizeof(fc));
  memcpy(fc.city, keepCity, sizeof(keepCity));
  memcpy(fc.dist, keepDist, sizeof(keepDist));

  int n = 0;
  for (int i = 0; i < 5 && i < (int)tm.size(); i++) {
    int y = 0, m = 0, d = 0;
    const char *s = tm[i];
    if (!s || sscanf(s, "%d-%d-%d", &y, &m, &d) != 3) continue;
    fc.date[n] = (uint32_t)(y * 10000 + m * 100 + d);
    fc.code[n] = (uint8_t)wc[i].as<int>();
    fc.tmax[n] = (int8_t)lroundf(hi[i].as<float>());
    fc.tmin[n] = (int8_t)lroundf(lw[i].as<float>());
    n++;
  }
  if (n == 0) return false;
  fc.n = (uint8_t)n;
  fc.magic = FCMAGIC;
  return true;
}

// Транслитерация кириллицы (UTF-8) в латиницу — шрифты на экране умеют только
// ASCII, а Dadata отдаёт город/район по-русски. "Москва" -> "Moskva".
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

// Геокодирование через Dadata (российский сервис, нужен бесплатный ключ с dadata.ru,
// раздел "Подбор по координатам" / Geolocate API). Без ключа — просто не показываем город.
bool fetchPlace(float la, float lo) {
  if (cfg.geoKey.length() == 0) return false;

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setTimeout(8000);
  if (!http.begin(client, "https://suggestions.dadata.ru/suggestions/api/4_1/rs/geolocate/address")) return false;
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Accept", "application/json");
  http.addHeader("Authorization", String("Token ") + cfg.geoKey);

  char body[96];
  snprintf(body, sizeof(body), "{\"lat\":%.5f,\"lon\":%.5f,\"radius_meters\":100}", la, lo);
  int code = http.POST((uint8_t *)body, strlen(body));
  String resp;
  if (code == HTTP_CODE_OK) resp = http.getString();
  http.end();
  if (code != HTTP_CODE_OK) return false;

  JsonDocument doc;
  if (deserializeJson(doc, resp)) return false;
  JsonArray sug = doc["suggestions"];
  if (sug.isNull() || sug.size() == 0) return false;
  JsonObject d = sug[0]["data"];

  const char *city = d["city"] | "";
  if (!city[0]) city = d["settlement"] | "";
  const char *dist = d["city_district"] | "";
  if (!dist[0]) dist = d["area"] | "";

  translitRu(fc.city, sizeof(fc.city), city);
  translitRu(fc.dist, sizeof(fc.dist), dist);
  return fc.city[0] != 0;
}

// Координаты по IP — запасной вариант, если GPS ещё не поймал спутники (точность: город)
bool fetchIpPosition() {
  WiFiClient client;
  HTTPClient http;
  http.setTimeout(8000);
  if (!http.begin(client, "http://ip-api.com/json/?fields=status,lat,lon")) return false;
  int code = http.GET();
  String body;
  if (code == HTTP_CODE_OK) body = http.getString();
  http.end();
  if (code != HTTP_CODE_OK) return false;
  JsonDocument doc;
  if (deserializeJson(doc, body)) return false;
  if (strcmp(doc["status"] | "", "success") != 0) return false;
  homeLat = doc["lat"].as<float>();
  homeLon = doc["lon"].as<float>();
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

void runSyncMode() {
  uint8_t err = WE_UNKNOWN;
  Serial.println("Sync mode: connecting to Wi-Fi");
  lastDiscReason = 0;
  WiFi.mode(WIFI_STA);
  WiFi.onEvent(onWifiEvent);
  WiFi.begin(cfg.ssid.c_str(), cfg.pass.c_str());
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000UL) {
    delay(200);
    // "сеть не найдена" ясно уже через ~8 с, дольше ждать незачем
    if (millis() - t0 > 8000UL && classifyWifiFail(lastDiscReason) == WE_NO_NET) break;
  }

  if (WiFi.status() == WL_CONNECTED) {
    configTime(0, 0, "pool.ntp.org", "time.google.com");
    t0 = millis();
    time_t nt = 0;
    while (millis() - t0 < 8000UL) {
      nt = time(nullptr);
      if (nt > 1700000000) break;
      delay(200);
    }
    bool ntpOk = nt > 1700000000;
    if (ntpOk) { epochBase = (uint32_t)nt; epochBaseMs = millis(); }

    if (isnan(homeLat)) fetchIpPosition();          // GPS ещё не поймал — берём примерно по IP
    if (isnan(homeLat)) {
      err = WE_NO_POS;
    } else {
      fetchPlace(homeLat, homeLon);                 // город/район — необязательно
      if (fetchForecast(homeLat, homeLon)) {
        prefs.putBytes("fc", &fc, sizeof(fc));
        err = WE_OK;
      } else {
        err = ntpOk ? WE_FC_FAIL : WE_NO_INET;
      }
    }
  } else {
    err = classifyWifiFail(lastDiscReason);
  }

  Serial.printf("Sync result: code=%u, last disconnect reason=%u\n", err, lastDiscReason);
  prefs.putUChar("werr", err);
  prefs.putBool("syncfail", err != WE_OK);          // при неудаче следующая попытка только через 3 часа
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  saveEpochForRestart();
  delay(200);
  ESP.restart();
}

void syncCheck(uint32_t now) {
  if (cfg.ssid.length() == 0) return;
  bool due = now > SYNC_PERIOD_MS || (fc.n == 0 && !syncFailFlag && now > FIRST_SYNC_MS);
  if (!due) return;
  Serial.println("Rebooting into sync mode (Wi-Fi)");
  prefs.putBool("sync", true);
  saveEpochForRestart();
  delay(100);
  ESP.restart();
}

// ===================== ВЕБ-СЕРВЕР НАСТРОЙКИ =====================
bool portalDone = false;

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
  h += field("Password (shown as typed)", "pass", cfg.pass, "text", "autocomplete='off' autocapitalize='none'");
  h += "<h3>Display</h3>";
  h += field("Refresh period, minutes (day)", "refr", String(cfg.refreshMin), "number", "min=1 max=1440");
  h += field("Night mode starts at (hour 0-23)", "ns", String(cfg.nightStart), "number", "min=0 max=23");
  h += field("Night mode ends at (hour 0-23)", "ne", String(cfg.nightEnd), "number", "min=0 max=23");
  h += field("Refresh period at night, minutes", "nr", String(cfg.nightRefreshMin), "number", "min=1 max=1440");
  h += field("Time zone (UTC+N)", "tz", String(cfg.tz), "number", "min=-12 max=14");
  h += field("Fallback time (HH:MM), used only if no GPS and no Wi-Fi time", "time", "", "time", "");
  h += "<h3>City / district (Dadata)</h3>";
  h += "<p style='font-size:.85em'>Free API key from dadata.ru -&gt; Geolocate API. Leave empty to not show city.</p>";
  h += field("Dadata API key", "geokey", cfg.geoKey, "text", "autocomplete='off'");
  h += "<h3>Outdoor module</h3>";
  h += field("Poll outdoor module every, seconds", "zbi", String(cfg.zbIntervalSec), "number", "min=10 max=3600");
  h += "<button type='submit'>Save and exit</button></form>";
  h += "<form method='post' action='/exit'><button type='submit'>Exit without saving</button></form>";
  h += "<h3>Factory reset</h3>"
       "<form method='post' action='/reset' onsubmit=\"return confirm('Erase all settings, history and Zigbee pairing?')\">"
       "<button type='submit'>Reset to factory settings</button></form>";
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
  cfg.zbIntervalSec   = constrain(server.arg("zbi").toInt(), 10, 3600);
  cfg.tz              = constrain(server.arg("tz").toInt(), -12, 14);
  if (server.hasArg("geokey")) { cfg.geoKey = server.arg("geokey"); cfg.geoKey.trim(); }
  if (server.arg("time").length() >= 4) {
    int hh = 0, mm = 0;
    if (sscanf(server.arg("time").c_str(), "%d:%d", &hh, &mm) == 2) {
      cfg.clockSetMin = (uint16_t)constrain(hh, 0, 23) * 60 + (uint16_t)constrain(mm, 0, 59);
    }
  }
  saveSettings();
  prefs.putBool("syncfail", false);    // новые Wi-Fi данные — можно пробовать синхронизацию сразу
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
  prefs.putBool("zbreset", true);      // Zigbee-сеть сбросим при следующей загрузке
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

  Serial.printf("Config portal: %s / %s  ->  http://%s\n", AP_SSID, AP_PASS, ip.toString().c_str());
  drawConfigScreen(ip);

  uint32_t t0 = millis();
  while (!portalDone && millis() - t0 < PORTAL_MS) {
    server.handleClient();
    delay(2);
  }
  delay(1500);                         // дать браузеру получить ответ
  Serial.println("Config portal closed, restarting");
  saveEpochForRestart();               // (после factory reset prefs очищены — время просто станет неизвестным)
  ESP.restart();
}

// ===================== КНОПКА =====================
// Короткое нажатие (<1 с) — сразу открыть Zigbee-сеть на 3 минуты, БЕЗ
// перезагрузки и БЕЗ захода в Wi-Fi настройку (раньше единственным способом
// было зайти в настройку, а пока заполняешь форму — окно для подключения
// успевало закрыться). Долгое удержание (>1 с) — как раньше, Wi-Fi настройка.
void checkButton() {
  static uint32_t downAt = 0;
  static bool longFired = false;
  bool pressed = (digitalRead(PIN_BUTTON) == LOW);

  if (pressed) {
    if (downAt == 0) downAt = millis();
    if (!longFired && millis() - downAt > 1000) {
      longFired = true;
      prefs.putBool("cfg", true);
      saveEpochForRestart();
      Serial.println("Button: rebooting into config mode");
      delay(100);
      ESP.restart();
    }
  } else {
    if (downAt != 0 && !longFired) {
      uint32_t held = millis() - downAt;
      if (held >= 50 && held < 1000) {     // короткий, не случайный щелчок
        Serial.println("Button: opening Zigbee network for 3 min (no reboot)");
        Zigbee.openNetwork(180);
      }
    }
    downAt = 0;
    longFired = false;
  }
}

// ===================== SETUP / LOOP =====================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  prefs.begin("meteo", false);
  loadSettings();
  bool wantConfig  = prefs.getBool("cfg", false);
  bool wantZbReset = prefs.getBool("zbreset", false);
  bool wantSync    = prefs.getBool("sync", false);
  syncFailFlag     = prefs.getBool("syncfail", false);
  wifiErr          = prefs.getUChar("werr", WE_UNKNOWN);
  homeLat          = prefs.getFloat("lat", NAN);
  homeLon          = prefs.getFloat("lon", NAN);

  // время, сохранённое перед плановым рестартом (флаг сразу сбрасываем)
  if (prefs.getBool("epok", false)) {
    epochBase = prefs.getUInt("ep", 0);
    epochBaseMs = millis();
    prefs.putBool("epok", false);
  }

  // история графика и прогноз из флеша
  memset(&hist, 0, sizeof(hist));
  if (prefs.getBytes("hist", &hist, sizeof(hist)) != sizeof(hist) || hist.magic != HMAGIC) memset(&hist, 0, sizeof(hist));
  memset(&fc, 0, sizeof(fc));
  if (prefs.getBytes("fc", &fc, sizeof(fc)) != sizeof(fc) || fc.magic != FCMAGIC) memset(&fc, 0, sizeof(fc));

  SPI.begin(PIN_EPD_SCK, -1, PIN_EPD_MOSI, PIN_EPD_CS);

  if (wantConfig) {
    prefs.putBool("cfg", false);       // чтобы не зациклиться при сбое
    runConfigMode();                   // не возвращается
  }

  if (wantSync) {
    prefs.putBool("sync", false);      // чтобы не зациклиться при сбое
    runSyncMode();                     // не возвращается (экран не трогаем — картинка остаётся)
  }

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  if (!sht4.begin(&Wire)) Serial.println("SHT45 not found!");
  else sht4.setPrecision(SHT4X_HIGH_PRECISION);
  if (!bme.begin(0x76, &Wire) && !bme.begin(0x77, &Wire)) Serial.println("BME280 not found!");

  scd4x.begin(Wire, SCD41_I2C_ADDR_62);
  scd4x.stopPeriodicMeasurement();     // на случай, если уже была запущена раньше
  delay(500);
  if (scd4x.startPeriodicMeasurement() != 0) Serial.println("SCD40 not found!");

  Serial1.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  clockBaseMillis = millis();

  zbThermostat.onTempReceive(onOutTemp);
  zbThermostat.onHumidityReceive(onOutHum);
  zbThermostat.setManufacturerAndModel("Meteo", "MainModule");
  Zigbee.addEndpoint(&zbThermostat);
  Zigbee.setRebootOpenNetwork(180);    // 3 минуты после загрузки можно подключать наружный модуль

  if (!Zigbee.begin(ZIGBEE_COORDINATOR)) {
    Serial.println("Zigbee failed to start, rebooting");
    delay(1000);
    ESP.restart();
  }

  if (wantZbReset) {
    prefs.putBool("zbreset", false);
    Serial.println("Zigbee factory reset");
    Zigbee.factoryReset();             // перезагрузит плату
  }
}

void loop() {
  static uint32_t lastPollMs = 0, lastDrawMs = 0;
  static bool firstPoll = true, firstDraw = true;
  uint32_t now = millis();

  while (Serial1.available()) gps.encode(Serial1.read());
  checkButton();
  updateHome();
  sampleHistory();
  syncCheck(now);

  // Пока наружный модуль не привязан — раз в минуту снова открываем сеть для подключения
  static uint32_t lastOpenMs = 0;
  if (!zbThermostat.bound() && now - lastOpenMs >= 60000UL) {
    lastOpenMs = now;
    Zigbee.openNetwork(60);
  }

  // Опрос наружного модуля с заданным в настройках периодом
  if (zbThermostat.bound() && (firstPoll || now - lastPollMs >= cfg.zbIntervalSec * 1000UL)) {
    firstPoll = false;
    lastPollMs = now;
    zbThermostat.getTemperature();
    zbThermostat.getHumidity();
  }

  // SCD40: новое значение готово примерно раз в 5 с
  if (now - co2LastMs >= 5000UL) {
    co2LastMs = now;
    bool ready = false;
    if (scd4x.getDataReadyStatus(ready) == 0 && ready) {
      uint16_t co2; float t, h;
      if (scd4x.readMeasurement(co2, t, h) == 0 && co2 != 0) co2Val = co2;
    }
  }

  // История давления для иконки (см. PH_INTERVAL_MS вверху файла — сейчас тестовое значение)
  if (pCount == 0 || now - pLastMs >= PH_INTERVAL_MS) {
    pLastMs = now;
    pushPressure(bme.readPressure() / 100.0f);
  }

  // Обновление экрана (ночью — реже)
  uint32_t periodMs = (isNight() ? cfg.nightRefreshMin : cfg.refreshMin) * 60UL * 1000UL;
  if (firstDraw || now - lastDrawMs >= periodMs) {
    firstDraw = false;
    lastDrawMs = now;

    sensors_event_t hum, temp;
    float inT = NAN, inH = NAN;
    if (sht4.getEvent(&hum, &temp)) { inT = temp.temperature; inH = hum.relative_humidity; }
    float pr = bme.readPressure() / 100.0f;

    Serial.printf("IN %.1fC %.1f%% | %.1f hPa (%.0f mmHg) | OUT %.1fC %.1f%% | zb=%d\n",
                  inT, inH, pr, pr * 0.750062f, outT, outH, zbLinkOk());
    drawMain(inT, inH, pr);
  }

  delay(10);
}
