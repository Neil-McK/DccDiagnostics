/* 
 *  Timer functions for measuring elapsed time between events.  On the
 *  AtMega328 and AtMega2560 controllers, the functions use the Input Capture
 *  capability of the microcontroller to capture the timer value very precisely
 *  using hardware.
 *  Since the Timer counter is limited to 16 bits, it will overflow if the interval exceeds 
 *  about 32milliseconds, so we use micros() to determine if the counter has overflowed.
 *  
 */

#ifndef eventtimer_default_h
#define eventtimer_default_h

#include <Arduino.h>
#include <DIO2.h>

#define TICKSPERMICROSEC 2   // 2 (0.5us) or 16 (62.5ns)

#if defined(ARDUINO_UNO_NANO)
  #define ICP_INPUTPIN 8
  #define TCNTn TCNT1 // TimerN Counter Register
  #define ICRn ICR1   // TimerN Input Change Register
  #define TCCRnA TCCR1A // TimerN configuration register
  #define TCCRnB TCCR1B // TimerN configuration register
  #define TIMSKn TIMSK1 // TimerN interrupt register
  #define ICIEn ICIE1  // Interrupt mask
  #define ICESn ICES1  // Mask
  #define CAPTURE_INTERRUPT TIMER1_CAPT_vect  // ISR vector
#elif defined (ARDUINO_MEGA)
  #define ICP_INPUTPIN 49 
  #define TCNTn TCNT4 // TimerN Counter Register
  #define ICRn ICR4   // TimerN Input Change Register
  #define TCCRnA TCCR4A // TimerN configuration register
  #define TCCRnB TCCR4B // TimerN configuration register
  #define TIMSKn TIMSK4 // TimerN interrupt register
  #define ICIEn ICIE4  // Interrupt mask
  #define ICESn ICES4  // Mask
  #define CAPTURE_INTERRUPT TIMER4_CAPT_vect  // ISR vector
#else
  #error "Architecture not supported by EventTimer library."
#endif

/* 
 *  User event handler is sent the time since the last valid event.
 *  If the user event handler decides that this event isn't valid 
 *  (e.g. time since last one is too short) then it should return 
 *  false to reject the event.  If the event is valid, then it 
 *  should return true to accept it.
 */
typedef bool EventHandler(unsigned long eventInterval);

class EventTimerClass {
public:

  bool begin(int pin, EventHandler userHandler) {
    if (pin != ICP_INPUTPIN) {
      Serial.print("ERROR: pin="); Serial.print(pin); Serial.print(", ICP="); Serial.println(ICP_INPUTPIN);
      return false;
    }

    this->pin = pin;
    this->callUserHandler = userHandler;

    // Set up input capture.
    // Configure Timer to increment TCNTn on a 2MHz clock,
    // (every 0.5us), or 16MHz (every 62.5ns), no interrupt.
    // Use Input Capture Pin ICP to capture time of input change
    // and interrupt the CPU.
    TCCRnA = 0;
    #if TICKSPERMICROSEC==2
    TCCRnB = (1 << CS11); // Prescaler CLK/8
    #elif TICKSPERMICROSEC==16
    TCCRnB = (1 << CS10); // Prescaler CLK/1
    #else
    #error "TICKSPERMICROSEC" not 2 or 16.
    #endif  
    TIMSKn = (1 << ICIEn); // Input capture interrupt enable

    return true;
  };
  
  unsigned long elapsedTicksSinceLastEvent() {
    return (unsigned int)(TCNTn - thisEventTicks);
  };
  
  void processInterrupt() {
    byte diginState = digitalRead2(this->pin);
    thisEventTicks = ICRn;
    unsigned long thisEventMicros = micros();

    // Initially estimate time using micros values.
    unsigned long eventSpacing = (thisEventMicros - lastValidEventMicros) * TICKSPERMICROSEC;
    if (eventSpacing < 60000L) {
      // Estimated time is well within range of timer count, so calculate ticks
      eventSpacing = thisEventTicks - lastValidEventTicks;
    }
    bool accepted = callUserHandler(eventSpacing);
    if (accepted) {
      lastValidEventTicks = thisEventTicks;
      lastValidEventMicros = thisEventMicros;
    }
    // Set up input capture for next edge.
    if (diginState) 
      TCCRnB &= ~(1 << ICESn); // Capture next falling edge on input
    else
      TCCRnB |= (1 << ICESn); // Capture next rising edge on input
  };

  unsigned int ticksPerMicrosec() {
    return TICKSPERMICROSEC;
  };

  bool inputCaptureMode() { return true; };

private:
  EventHandler *callUserHandler = 0;
  unsigned int lastValidEventTicks = 0;
  unsigned long lastValidEventMicros = 0;
  unsigned int thisEventTicks = 0;
  int pin = -1;
};

EventTimerClass EventTimer;


ISR(CAPTURE_INTERRUPT) {
  EventTimer.processInterrupt();
}

#endif
