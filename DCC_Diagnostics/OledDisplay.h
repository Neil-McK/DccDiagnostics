/*
 * OledDisplay class
 * 
 * Handles the Oled Display output from the DCC diagnostic program.
 * 
 * The output to be displayed is passed to the program as parameter
 * to the function update().  The string is sent to the screen (at least
 * as much as fits on the screen).  The buffer reference is held so that,
 * if the user presses the button defined by BUTTONPIN, the screen is updated
 * starting at a later line in the buffer.  In this way, the user can 
 * scroll through the text.  
 * If the current scroll line number is greater than the number of lines
 * of text when a refresh is due, then the text is displayed from the 
 * beginning once more.
 */ 

#ifndef oleddisplay_h
#define oleddisplay_h

// Only compile if USE_OLED is defined above the #include call for this file.
// If not, then the entire contents are ignored by the compiler.
#ifdef USE_OLED

#include "Config.h"

#include <Adafruit_SSD1306.h>

class OledDisplayClass {
public:
  // Function called to initialise the object instance.  
  //  Connects to the OLED display and puts it into a 
  //  suitable mode.
  bool begin(int sdaPin, int sclPin) {
    Serial.print("OLED SDA/SCL=");
    Serial.print(sdaPin);
    Serial.print("/");
    Serial.println(sclPin);
    #if defined(ESP32) || defined(ESP8266)
    Wire.begin(sdaPin, sclPin);
    #endif
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

    // Initialise the button pin.
    #ifdef BUTTONPIN
    pinMode(BUTTONPIN, INPUT_PULLUP);
    #endif
    
    Serial.println("OLED initialised");
    return true;
  }

  // Function called to provide a new buffer of data
  // and force an update of the screen.
  void update(const char *buffer) {
    currentBuffer = buffer;
    refresh();
  }

  // Function called to force an update of the screen,
  //  starting with the line referenced by 'firstLine'.
  void refresh() {
    if (!currentBuffer) return;

    // locate nominated line
    display.clearDisplay();
    display.setCursor(0,0);
    display.print(selectedLine());
    display.display();
  }

  // Check the button pin input to see if the button has
  //  been pressed.  If so, increment 'firstLine' and refresh,
  //  so that the displayed text scrolls upwards.
  void checkButton() {
    #ifdef BUTTONPIN
    static int lastButtonState = 0;
    int buttonState = digitalRead(BUTTONPIN);
    if (buttonState == 0 && lastButtonState == 1) {
      Serial.println("NextLine");
      // Button pressed.  Move a few lines down.
      firstLine += 5;
      refresh();
    }
    lastButtonState = buttonState;
    #endif
  }

private:
  // OLED Display
  Adafruit_SSD1306 display = Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
  // Pointer to current buffer of text for display
  const char *currentBuffer = 0;
  // Line number (starting with 0) of first line to be displayed.
  int firstLine = 0;

  // Locate pointer to first character of line 'firstLine' in buffer 'currentBuffer'.
  // If line doesn't exist, then go back to the beginning of the buffer.
  const char *selectedLine() {
    if (firstLine > 0) {
      int thisLineNo = 0;
      for (const char *cptr=currentBuffer; *cptr != 0; cptr++) {
        if (*cptr == '\n') {
          if (++thisLineNo == firstLine) {
            cptr++;  // Move to start of next line following newline
            return cptr;
          }
        }
      }
      // If we get here, then there aren't enough lines!
      firstLine = 0;
    }
    return currentBuffer;
  }

} /* class OledDisplayClass */;

// Singleton instance of class
OledDisplayClass OledDisplay;

#endif
#endif
