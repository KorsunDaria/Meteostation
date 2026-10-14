
// Библиотеки: GxEPD2, Adafruit GFX

#include <GxEPD2_BW.h>
#include <Fonts/FreeMonoBold24pt7b.h>

// Пины (поменяй, если подключил иначе)
#define PIN_CS   10
#define PIN_DC   2
#define PIN_RST  3
#define PIN_BUSY 4
#define PIN_SCK  6
#define PIN_MOSI 7

GxEPD2_BW<GxEPD2_750_GDEY075T7, GxEPD2_750_GDEY075T7::HEIGHT> display(
  GxEPD2_750_GDEY075T7(PIN_CS, PIN_DC, PIN_RST, PIN_BUSY)
);

void setup() {
  Serial.begin(115200);

  // Настраиваем аппаратный SPI на нужные пины
  SPI.begin(PIN_SCK, -1, PIN_MOSI, PIN_CS);

  display.init(115200, true, 50, false);
  display.setRotation(1);
  display.setFullWindow();

  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setTextColor(GxEPD_BLACK);
    display.setFont(&FreeMonoBold24pt7b);
    display.setCursor(50, 100);
    display.print("Hello, Daria!");
  } while (display.nextPage());

  display.hibernate();
  Serial.println("Done");
}

void loop() {
}
