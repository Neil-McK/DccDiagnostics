#ifndef config_h
#define config_h

// The following logic should work for all supported platforms, hopefully.
#if defined(ESP_PLATFORM)
  #if !defined(ESP32)
    #define ESP32
  #endif
#elif defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO)
  #define ARDUINO_UNO_NANO
#elif defined(ARDUINO_AVR_MEGA2560)
  #define ARDUINO_MEGA
#else
  #error "Platform not recognised"
#endif

// Uncomment the "#define USE_DIO2" to use the 'Fast digital i/o library' DIO2
// in place of the normal digitalRead() and digitalWrite().
// Reduces CPU load by about 17%.  Only applicable to Arduino Uno/Mega/Nano.
#define USE_DIO2

// Uncomment the "#define USETIMER" to perform the timing using the ATmega328's 
// and ESP32's Input Capture mode which captures the time through hardware instead
// of software.  This enables a higher accuracy and consistency, and because we can capture the 
// Timer counter register (TCNTn) at the time of the digital change, it is immune to 
// timing errors caused by other interrupts.
// Works on Arduino Uno (Timer1/pin D8)
//          Arduino Nano (Timer1/pin D8)
//          Arduino Mega (Timer4/pin D49)
// If we don't use this, then the selected input pin must support change interrupt 
// (defaults to pin D2 on Uno, Nano and Mega, GPIO5 on ESP8266/ESP32.
#define USETIMER 

// Uncomment following lines to enable OLED output on pins SDA_OLED and SCL_OLED.
#define USE_OLED
#ifndef ESP32        // Heltec Kit 32 has pins 4/15 predefined for I2C rather than 4/5.
  #define SDA_OLED 4   
  #define SCL_OLED 5
  #define OLED_RESET -1
#else
  // #define SDA_OLED 4
  // #define SCL_OLED 15
  #define OLED_RESET 16
#endif
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Button for selecting new page on OLED
#define BUTTONPIN 0


// LED pin definitions - uncomment as required.
//#define LEDPIN_ACTIVE 13    // Shows interrupts being received, ie DCC active
//#define LEDPIN_LOCOSPEED 3  // Driven from loco speed packet for loco 3
//#define LEDPIN_DECODING 7   // lights when a packet with valid checksum is received
//#define LEDPIN_FAULT 6      // Lights when a checksum error or glitch is encountered.

#define SERIAL_SPEED 115200

// Input pin definitions.  Defaults:
//    Nano (USETIMER):               8
//    Uno (USETIMER):                8
//    Mega (USETIMER):               49
//    Nano/Uno/Mega, Non-USETIMER:   2
//    ESP32:                         5
//    ESP8266:                       5
//    Other:                         2
#if defined(USETIMER)
  #if defined(ARDUINO_UNO_NANO)
    #define INPUTPIN 8
  #elif defined(ARDUINO_MEGA)
    #define INPUTPIN 49
  #elif defined(ESP32) || defined(ESP8266)
    #define INPUTPIN 5
  #else
    // Assume timer not supported, use default pin
    #undef USETIMER
    #define INPUTPIN 2
  #endif
#else
  #if defined(ESP32) || defined(ESP8266)
    #define INPUTPIN 5
  #else
    #define INPUTPIN 2
  #endif
#endif

// WiFi declarations
//const char *ssid = "myssid";
//const char *password = "mypassword";
#include "..\..\Wifi.h"
const char *dnsName = "DccInspector";

#endif
