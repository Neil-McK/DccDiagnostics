#ifndef oleddisplay_h
#define oleddisplay_h

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 16

#include <Adafruit_SSD1306.h>

class OledDisplayClass {
public:
  bool begin(int sdaPin, int sclPin) {
    Serial.print("OLED SDA/SCL=");
    Serial.print(sdaPin);
    Serial.print("/");
    Serial.println(sclPin);
    Wire.begin(sdaPin, sclPin);
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
      Serial.println("Can't connect to OLED display!");
      return false;
    }
    display.clearDisplay();
    display.display();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setTextWrap(false);
    //display.setRotation(2);
    Serial.println("OLED initialised");
    return true;
  };

  void update(const char *buffer) {
    const char *cptr = buffer;
    // locate nominated line
    if (firstLine > 0) {
      for (int line=0; *cptr != 0; cptr++) {
        if (*cptr == '\n') {
          if (++line == firstLine) {
            cptr++;
            break;
          }
        }
      }
    }
    display.clearDisplay();
    display.setCursor(0,0);
    display.print(cptr);
    display.display();
  };

private:
  Adafruit_SSD1306 display = Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
  int firstLine = 0;
};

OledDisplayClass OledDisplay;

#endif
