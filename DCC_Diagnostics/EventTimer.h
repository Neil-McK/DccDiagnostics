
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO) || defined(ARDUINO_AVR_MEGA)
  #include "EventTimer_AtMega.h"
#elif defined(ESP_PLATFORM)
  #include "EventTimer_ESP32.h"
#else
  #include "EventTimer_default.h"
#endif
