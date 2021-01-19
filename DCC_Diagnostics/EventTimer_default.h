/* 
 *  Timer functions for measuring elapsed time between events.  On this platform,
 *  the functions use the micros() function available on all Arduino controllers
 *  as well as ESP8266, ESP32 etc.
 *  
 *  This module acts as the fallback if no specific support is available e.g. 
 *  through an Input Capture Register.
 */

#ifndef eventtimer_default_h
#define eventtimer_default_h

#include <Arduino.h>

#define TICKSPERMICROSEC 1

#if defined(ESP32) || defined(ESP8266)
  #define INTERRUPT_SAFE IRAM_ATTR
#else
  #define INTERRUPT_SAFE
#endif

// Predeclare event handler function for later...
void eventHandler();

/* 
 *  User event handler is sent the time since the last valid event.
 *  If the user event handler decides that this event isn't valid 
 *  (e.g. time since last one is too short) then it should return 
 *  false to reject the event.  If the event is valid, then it 
 *  should return true to accept it.
 */
typedef bool EventHandler(unsigned long eventSpacing);

class EventTimerClass {
public:

  bool begin(int pin, EventHandler userHandler) {
    this->pin = pin;
    int interruptNumber = digitalPinToInterrupt(pin);
    if (interruptNumber < 0) {
      Serial.print("ERROR: Pin "); Serial.print(pin); Serial.println(" has no interrupt support");
      return false;
    }
    this->callUserHandler = userHandler;
    attachInterrupt(interruptNumber, eventHandler, CHANGE);
    return true;
  };
  
  unsigned long INTERRUPT_SAFE elapsedTicksSinceLastEvent() {
    return micros() - thisEventTicks;
  };
  
  void INTERRUPT_SAFE processInterrupt(unsigned long thisEventTicks) {
    this->thisEventTicks = thisEventTicks;
    unsigned long eventSpacing = thisEventTicks - lastValidEventTicks;
    bool accepted = callUserHandler(eventSpacing);
    if (accepted) {
      lastValidEventTicks = thisEventTicks;
    }
  };

  bool inputCaptureMode() { return false; };

private:
  EventHandler *callUserHandler = 0;
  unsigned long lastValidEventTicks = 0;
  unsigned long thisEventTicks = 0;
  int pin = -1;
};

int interruptCount;

EventTimerClass EventTimer;

void INTERRUPT_SAFE eventHandler() {
  EventTimer.processInterrupt(micros());
}

#endif
