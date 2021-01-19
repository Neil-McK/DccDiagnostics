
#if defined(ARDUINO_UNO_NANO) || defined(ARDUINO_AVR_MEGA)
  #include "EventTimer_AtMega.h"
#elif defined(ESP32)
  #include "EventTimer_ESP32.h"
#else
  #include "EventTimer_default.h"
#endif
