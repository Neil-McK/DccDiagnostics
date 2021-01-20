///////////////////////////////////////////////////////
//
// Move configuration items to config.h: NMcK Jan 2021
// Add OLED, and web server support on ESP32: NMcK Jan 2021
// Add capture for ESP32; refactor timers: NMcK Jan 2021
// Improve accuracy of timing and time calculations: NMcK Jan 2021
// Add support for compilation for ESP8266 and ESP32: NMcK Jan 2021
// Use ICR for pulse measurements, and display pulse length histogram: NMcK Dec 2020
// Add CPU load figures for the controller: NMcK December 2020
// Count, and optionally filter, edge noise from input: NMcK December 2020
// Display half-bit lengths rather than bit lengths: NMcK December 2020
// Improved comments; moved some strings to flash: NMcK November 2020
// Moved frame construction into interrupt code: NMcK July 2020
// Removed use of AVR timer interrupts: NMcK June 2020
// DCC packet analyze: Ruud Boer, October 2015
//
// The DCC signal is detected on Arduino digital pin 8 (ICP1 on Uno/Nano),
// or pin 49 (ICP4 on Mega), or pin GPIO5 on ESP8266/ESP32.  This causes an interrupt,
// and in the interrupt response code the time between interrupts is measured.
//
// Use an opto-isolator between the track signal (~30V p-p) and the
// digital input (0 to 5V or 0 to 3.3V) to prevent burn-out.
//
// Written originally for Uno but also tested on Mega and Nano,
// should work on other architectures if simple options chosen (no USETIMER and
// no USE_DIO2).
//
// Also tried on ESP8266 (NodeMCU) and ESP32 (Heltek Kit 32) but not extensively tested.
// It does run, and will decode bits on the input pin which is GPIO5, labelled D1 on the NodeMCU.
// The faster clock speeds on the ESP8266/ESP32 mean that interrupt jitter is less, but there is 
// still around +/- 4us evident on the ESP8266.  Also, the ESP8266 and ESP32 micros() do
// actually give a resolution of 1 microsecond (as opposed to 4us on the Arduino).  These
// architectures require some inspired way of eliminating the effects of other interrupts; for
// example on the ESP32, there is the option of using the other CPU perhaps.
//
// The use of ICPn on the Arduino enables accurate timing to within 1us.  The millis() function in the 
// Arduino is no better than 10.5us or so accurate (6.5us interrupt jitter caused by the
// Arduino Timer0 interrupts, and 4us timer resolution) which is inadequate for diagnostic purposes.
//
// The selected digital input must have interrupt support, either via
// the level change interrupt (e.g. Arduino pin 2 or 3) or, preferably, input capture interrupt
// (e.g. pin 8 on Arduino Uno, pin 49 on Mega).  The ESP8266/ESP32 do not have input capture functionality
// but any pin may be used for level change interrupt.
//
// The bit decoding is done by measuring the time between successive
// interrupts using the 'micros()' function so should be pretty portable.
// Optionally, it will use a faster 16-bit timer for higher resolution.  This is
// the default mode for Uno, Nano and Mega (USETIMER option).
//
// The counter SpareLoopCount is used to see how heavily loaded the processor is.
// With DCC interrupts off, it measures the count.  When the interrupts are attached
// it sees how much the count drops.  The percent load due to the interrupts is
// the percentage difference between the two figures.
// For example (my measurements on an Uno):
//    DCC off, SpareLoopCount=1043042 (over 4 secs)
//    DCC on, SpareLoopCount=768830 (over 4 secs)
//    CPU Load = (1043042-766830)/1043042*100 = 26%
//
// Loco address 3's speed command is optionally mapped onto a PWM output pin, allowing an 
// LED to be used to confirm that a controller is able to send recognisable 
// DCC commands.
//
// When outputting decoded DCC packets, duplicate loco speed packets and idle packets encountered
// within a time period will not be logged (as they are sent continuously).  However, all 
// accessory and loco function packets are displayed, even if repeated.
//
// Set the Serial Monitor Baud Rate to 115200 !!
//
// Keyboard commands that can be sent via Serial Monitor:
// 1 = 1s refresh time
// 2 = 2s 
// 3 = 4s (default)
// 4 = 8s
// 5 = 16s
// 6 = 4 DCC packet buffer
// 7 = 8
// 8 = 16
// 9 = 32 (default)
// 0 = 64
// a = show accessory packets toggle
// l = show locomotive packets toggle
// d = show diagnostics toggle
// h = show heartbeat toggle
// b = show half-bit counts by length toggle
// c = show cpu/irc usage in sniffer toggle
// f = input filter toggle
// ? = help (show this information)
//
////////////////////////////////////////////////////////

// Configurable parameter items now in separate include file.
#include "Config.h"

////////////////////////////////////////////////////////

#if defined(USE_DIO2) && (defined(ARDUINO_UNO_NANO) || defined(ARDUINO_MEGA))
#define GPIO_PREFER_SPEED
#include <DIO2.h>
#define digitalWrite(pin,state) digitalWrite2(pin,state)
#define digitalRead(pin) digitalRead2(pin)
#endif

#ifdef USETIMER
  #include "EventTimer.h"
#else
  #include "EventTimer_default.h"
#endif

// On ESP32, add web server
#if defined(ESP32) || defined(ESP8266)
#include "HttpManager.h"
#endif

// Include OLED if required.
#if defined(USE_OLED)
#include "OledDisplay.h"
#endif

// Statistics structures
#include "DccStats.h"

#if defined(ESP32) || defined(ESP8266) 
#define INTERRUPT_SAFE IRAM_ATTR
#else
#define INTERRUPT_SAFE
#endif

const int nPackets=8; // Number of packet buffers
const int pktLength=8; // Max length+1 in bytes of DCC packet

// Variables shared by interrupt routine and main loop
volatile byte dccPacket[nPackets][pktLength]; // buffer to hold packets 
volatile byte packetsPending=0; // Count of unprocessed packets
volatile byte activePacket=0;  // indicate which buffer is currently being filled
volatile bool filterInput = true; // conditions input to remove transient changes
volatile struct stats activeStats;
volatile bool enableStats = true;


// Variables used by main loop
byte refreshTime = 4; // Time between DCC packets buffer refreshes in secs
byte packetHashListSize = 32; // DCC packets checksum buffer size
bool showLoc=true;
bool showAcc=true;
bool showHeartBeat=true;
bool showDiagnostics=true;
bool showBitLengths=false;
bool showCpuStats=false;

byte inputPacket=0; // Index of next packet to be analysed in dccPacket array
byte pktByteCount=0;
int bufferCounter=0;
struct stats lastStats; // Snapshot of activeStats.
unsigned long maxSpareLoopCountPerSec = 0; // baseline for CPU load calculation
unsigned int packetHashList[64];
bool calibrated = false;
unsigned long lastRefresh = 0;

// Buffer for HTTP and OLED output.
#if defined(ESP32)
char espBuffer[1024] = "";
char pktBuffer[512] = "";
#else
char espBuffer[1] = "";
char pktBuffer[1] = "";
#endif

//=======================================================================
// Perform the setup functions for the application.

void setup() {
  Serial.begin(SERIAL_SPEED);
  Serial.println(F("---"));
  Serial.println(F("DCC Packet Analyze initialising"));

  #ifdef LEDPIN_ACTIVE
  pinMode(LEDPIN_ACTIVE, OUTPUT);
  #endif
  #ifdef LEDPIN_DECODING
  pinMode(LEDPIN_DECODING, OUTPUT);
  #endif
  #ifdef LEDPIN_DECODING
  pinMode(LEDPIN_DECODING, OUTPUT);
  #endif
  #ifdef LEDPIN_FAULT
  pinMode(LEDPIN_FAULT, OUTPUT);
  #endif
 
  // Enable pullup in case there's no external pullup resistor.
  // External resistor is preferred as it can be lower, so 
  // improve the switching speed of the Optocoupler.
  pinMode(INPUTPIN, INPUT_PULLUP);

  if (!EventTimer.inputCaptureMode()) {
    // Output health warning...
    Serial.println(F("\r\n** WARNING Measurements will occasionally be out up to ~10us either way **"));
    Serial.println(F("**         because of difficulties with accurate timing in this mode.   **"));
  }

  #if defined(USE_OLED)
  OledDisplay.begin(SDA_OLED, SCL_OLED);
  OledDisplay.update("Initialising..");
  #endif
  
  #if defined(ESP32) || defined(ESP8266)
  HttpManager.begin(ssid, password, dnsName);
  HttpManager.setBuffer(&espBuffer[0]);
  #endif

  // Delay first output of statistics 
  lastRefresh = millis();

  Serial.print(F("Calibrating... "));
}

//=======================================================================
// Main program loop.  
  
void loop() {
  static unsigned int innerSpareLoopCount = 0;
  bool somethingDone = false;
  
  if (millis() >= lastRefresh + (unsigned long)refreshTime * 1000) {
    
    if (!calibrated) {
      // Calibration cycle done, record the details.
      Serial.println(F("done."));
      Serial.print(F("Updates every "));
      Serial.print(refreshTime);
      Serial.println(F(" seconds"));
      Serial.println(F("---"));
      maxSpareLoopCountPerSec = activeStats.spareLoopCount / refreshTime;
      calibrated = true;
      
      // Start recording data from DCC.
      ClearDccData();
      if (!EventTimer.begin(INPUTPIN, capture)) {
        Serial.println(F("Unable to start EventTimer, check configured pin"));
        while(1) ;
      }
    } else {
      // Normal cycle, copy. reset, and output diagnostics.
      if (showHeartBeat) Serial.println('-');
      ClearDccData();
      OutputStatistics(Serial);   

      #if defined(ESP_PLATFORM) || defined(USE_OLED)
      WriteStatistics(espBuffer, sizeof(espBuffer));
      #endif
      #if defined(USE_OLED)
      OledDisplay.update(&espBuffer[0]);
      #endif
    } 
   
    lastRefresh = millis();
    somethingDone = true;
  }

  // Check for DCC packets - if found, analyse and display them
  somethingDone |= processDCC(Serial);

  // Check for commands received over the USB serial connection.
  somethingDone |= processCommands();
    
  #if defined(USE_OLED)
  OledDisplay.checkButton();
  #endif

//Increment CPU loop counter.  This is done if nothing else was.
  // If the counter never gets incremented, it means that the 
  // CPU is fully loaded doing other things and has no spare time.
  if (!somethingDone) {
    if (++innerSpareLoopCount >= F_CPU/1000000L) {
      activeStats.spareLoopCount++; 
      innerSpareLoopCount = 0;
    }
  }

  #if defined(ESP32) || defined(ESP8266)
  HttpManager.process();
  #endif
  
  UpdateLED();
}

//=======================================================================
// Function invoked (from interrupt handler) on change of state of INPUTPIN.  
//  It measures the time between successive changes (half-cycle of DCC
//  signal).  Depending on the value, it decodes 0 or a 1 for alternate 
//  half-cycles.  A 0 half-bit is nominally 100us per half-cycle (NMRA says 90-10000us)
//  and a 1 half-bit is nominally 58us (52-64us).  We treat a half-bit duration < 80us 
//  as a '1' half-bit, and a duration >= 80us as a '0' half-bit. 
//  Prologue and framing bits are detected and stripped, and data bytes are
//  then stored in the packet queue for processing by the main loop.
//
bool INTERRUPT_SAFE capture(unsigned long halfBitLengthTicks) {
  
  static byte preambleOneCount = 0;
  static boolean preambleFound = false;
  static int newByte = 0;   // Accumulator for input bits until complete byte found.
  static int inputBitCount = 0;   // Number of bits read in current newByte.
  static int inputByteNumber = 0;  // Number of bytes read into active dccPacket buffer so far
  static byte interruptCount = 0;
  static byte previousBitValue = 0, previousDiginState = 0;
  static unsigned int previousHalfBitLengthTicks = 0;
  static byte altbit = 0;   // 0 for first half-bit and 1 for second.
  byte bitValue;

  // The most critical parts are done first - read state of digital input.
  byte diginState = digitalRead(INPUTPIN);

  // Set a high bound on the half bit length
  if (halfBitLengthTicks > 1200*TICKSPERMICROSEC) 
    halfBitLengthTicks = 1200*TICKSPERMICROSEC; // microseconds.
  
  // Calculate time between interrupts in microseconds.
  unsigned int interruptInterval = halfBitLengthTicks / TICKSPERMICROSEC;
  
  // Precondition input?
  if (filterInput) {
    // Check that the digital input has actually changed since last interrupt, and that
    // the gap between interrupts is realistic.
    if (interruptCount > 0 && (diginState == previousDiginState || interruptInterval <= 3)) {
      // No change in digital, or it was fleeting.  Ignore.
      activeStats.glitchCount++;
      return false;   // reject interrupt
    }
  }
  
  // If we get here, the interrupt looks valid, i.e. the digital input really did
  // change its state more than 3us after its last change.
  // Calculate difference between current bit half and preceding one, rounding up to next microsecond.
  // This will only be recorded on alternate half-bits, i.e. where the previous and current half-bit
  // make a complete bit.
  unsigned int delta = (abs((int)halfBitLengthTicks - (int)previousHalfBitLengthTicks) 
      + TICKSPERMICROSEC - 1) / TICKSPERMICROSEC;

  // Record input state and timer values ready for next interrupt
  previousDiginState = diginState;
  previousHalfBitLengthTicks = halfBitLengthTicks;  

  // If first or second interrupt, then exit as the previous state is incomplete.
  if (interruptCount < 2) {
    interruptCount++;
    return true;
  }

  #ifdef LEDPIN_ACTIVE
  digitalWrite(LEDPIN_ACTIVE, 1);
  #endif

  // Check length of half-bit
  if (halfBitLengthTicks < 80 * TICKSPERMICROSEC) 
    bitValue = 1;
  else
    bitValue = 0;

  // Check if we're on the first or second half of the bit.
  if (bitValue != previousBitValue) {
    // First half of new bit received
    altbit = false;
  } else {
    // Toggle for alternate half-bits
    altbit = !altbit;
  }
  previousBitValue = bitValue;
  
  // Update statistics
  activeStats.count++;
  if (bitValue == 0) {
    activeStats.count0++;
    if (interruptInterval > activeStats.max0) activeStats.max0 = interruptInterval;
    if (interruptInterval < activeStats.min0) activeStats.min0 = interruptInterval;
    activeStats.total0 += interruptInterval;
    if (altbit && (delta > activeStats.max0BitDelta))
      activeStats.max0BitDelta = delta;
  } else {
    activeStats.count1++; 
    if (interruptInterval > activeStats.max1) activeStats.max1 = interruptInterval;
    if (interruptInterval < activeStats.min1) activeStats.min1 = interruptInterval;
    activeStats.total1 += interruptInterval;
    if (altbit & (delta > activeStats.max1BitDelta))
      activeStats.max1BitDelta = delta;
  }
  if (interruptInterval < minBitLength) 
    interruptInterval = minBitLength;
  else if (interruptInterval > maxBitLength) 
    interruptInterval = maxBitLength;
  activeStats.countByLength[altbit][interruptInterval - minBitLength]++;

  // Store interrupt interval for use on next interrupt.
  previousHalfBitLengthTicks = halfBitLengthTicks;

  // If this is the second half-bit then we've got a whole bit!!
  if (altbit) {

    // Now we've got a bit, process it.  The message comprises the following:
    //   Preamble: 10 or more '1' bits followed by a '0' start bit.
    //   Groups of 9 bits each containing data byte of 8 bits, followed by a 
    //   '0' bit (if message not yet finished), or a '1' bit (if the byte is 
    //   the last byte of the message, i.e. the checksum).
    //
    if (!preambleFound) {
      if (bitValue==1) {
        // Reading preamble perhaps...
        preambleOneCount++;
      } else if (preambleOneCount < 10) { // and bitValue==0)
        // Preamble not yet found, but zero bit encountered.  Restart preable count.
        preambleOneCount = 0;
      } else { // preambleOneCount >= 10 and bitValue==0
        // Start bit found at end of preamble, so prepare to process data.
        preambleFound = true;
        newByte = 0;
        inputBitCount = 0;
        inputByteNumber = 0;
      }
    } else {   // Preamble previously found, so this is a message bit
      if (packetsPending==nPackets) {
        // Previous DCC packets haven't been processed by the main loop,
        // so there is no buffer for the incoming message. 
        // Discard incoming message and scan for another preamble.
        preambleFound = false;
        preambleOneCount = 0;

        // Record this event in a counter.
        activeStats.countLostPackets++;

      } else {
        // Preamble read, packet buffer available, so message bit can be stored!
        if (inputBitCount == 8) { // Byte previously completed, so this bit is the interbyte marker
          if (bitValue==0) {  // Interbyte marker is zero, so prepare for next byte of data
            inputBitCount = 0;
          } else { // one-bit found, marks end of packet
            // End of packet found
            dccPacket[activePacket][0] = inputByteNumber; // save number of bytes
            packetsPending++; // flag that packet is ready for processing
            if (++activePacket >= nPackets) activePacket = 0;  // move to next packet buffer
            activeStats.packetCount++;
            preambleFound = false; // scan for another preamble
            preambleOneCount = 1;  // allow the current bit to be counted in the preamble.
          }
        } else { // Reading packet data at this point.
          // Append received bit to the current new byte.
          newByte = (newByte << 1) | bitValue;
          if (++inputBitCount == 8) { // Completed byte, save byte (if room)
            if (inputByteNumber < pktLength-1)
              dccPacket[activePacket][++inputByteNumber] = newByte;
            else {  // packet has filled buffer so no more bits can be stored!
              packetsPending++; // flag that packet is ready for processing
              if (++activePacket >= nPackets) activePacket = 0;  // move to next packet buffer
              preambleFound = false;  // scan for another preamble
              preambleOneCount = 0; 
              // Record this event in a counter.
              activeStats.countLongPackets++;
            }             
            newByte = 0;
          }
        }
      }
    }      
  }
  
  #ifdef LEDPIN_ACTIVE
  // Turn out ACTIVE LED.
  digitalWrite(LEDPIN_ACTIVE, 0);
  #endif

  // Calculate time taken in interrupt code between the measured time of event to POINTB.
  
  unsigned int interruptDuration = EventTimer.elapsedTicksSinceLastEvent() / TICKSPERMICROSEC;   // POINTB
  
  // Assume that there are about 25 cycles of instructions in this function that are not measured, and that
  // the prologue in dispatching the function (saving registers etc) is around 51 cycles and the epilogue
  // (restoring registers etc) is around 35 cycles.  This adds a further (51+25+35)/16MHz=6.9us to the calculation.
  // See https://billgrundmann.wordpress.com/2009/03/02/the-overhead-of-arduino-interrupts/.  However, if 
  // the Input Capture mode is used, then this will be much smaller.  So ignore it.
  //interruptDuration += 7;
  
  // Record result
  if (interruptDuration > activeStats.maxInterruptTime) activeStats.maxInterruptTime = interruptDuration;
  if (interruptDuration < activeStats.minInterruptTime) activeStats.minInterruptTime = interruptDuration;
  activeStats.totalInterruptTime += interruptDuration;

  return true; // Accept interrupt.
}

//=======================================================================
// Connect the scan routine to the interrupt.  It will execute on
// all changes (0->1 and 1->0).

void beginBitDetection() {
  EventTimer.begin(INPUTPIN, capture);
}

//=======================================================================
// PrintByte prints one byte of data in hex with leading zero
// when necessary.

void printByte(byte x) {
  byte mask = 0x80;
  while(mask != 0) {
    if ((x & mask) != 0) 
      Serial.print('1');
    else
      Serial.print('0');  
    mask >>= 1;
  }
}

//=======================================================================
// PrintPacket prints the raw DCC packet contents to the 
// USB serial connection.

void printPacket(int index) {
  Serial.print(' ');
  for (byte n=1; n<dccPacket[index][0]; n++) {
    Serial.print(' ');
    printByte(dccPacket[index][n]);
  }
  Serial.println(' ');
}

//=======================================================================
// OutputStatistics writes the last set of statistics to the serial stream

void OutputStatistics(Print &output) {

  if (showDiagnostics) {
    
    // These counts are for half-bits, so divide by two.
    output.print(F("Bit Count/"));
    output.print(refreshTime);
    output.print(F(" sec="));
    output.print(lastStats.count/2);
    output.print(F(" (Zeros="));
    output.print(lastStats.count0/2);
    output.print(F(", Ones="));
    output.print(lastStats.count1/2);
    output.print(F("), Glitches="));
    output.println(lastStats.glitchCount);

    output.print(F("Packets received="));
    output.print(lastStats.packetCount);
    output.print(F(", Checksum Error="));
    output.print(lastStats.checksumError);
    output.print(F(", Lost pkts="));
    output.print(lastStats.countLostPackets);
    output.print(F(", Long pkts="));
    output.println(lastStats.countLongPackets);

    output.print(F("0 half-bit length (us): "));
    if (lastStats.min0 <= lastStats.max0) {
      output.print((float)lastStats.total0/lastStats.count0,1);
      output.print(F(" ("));
      output.print(lastStats.min0);
      output.print(F("-"));
      output.print(lastStats.max0);
      output.print(F(")"));
      output.print(F(" delta <= "));
      output.print(lastStats.max0BitDelta);
    } else
      output.print(F("<none>"));
    output.println();
    output.print(F("1 half-bit length (us): "));
    if (lastStats.min1 <= lastStats.max1) {
      output.print((float)lastStats.total1/lastStats.count1,1);
      output.print(F(" ("));
      output.print(lastStats.min1);
      output.print(F("-"));
      output.print(lastStats.max1);
      output.print(F(")"));
      output.print(F(" delta <= "));
      output.print(lastStats.max1BitDelta);
    } else
      output.print(F("<none>"));
    output.println();

    if (showCpuStats) {
      output.print(F("IRC Duration (us): "));
      if (lastStats.minInterruptTime <= lastStats.maxInterruptTime) {
        output.print((float)lastStats.totalInterruptTime/lastStats.count,1);
        output.print(F(" ("));
        output.print(lastStats.minInterruptTime);
        output.print(F("-"));
        output.print(lastStats.maxInterruptTime);
        output.print(F(")"));
      } else 
        output.print(F("<none>"));
        
      // Calculate and display cpu load
      unsigned long spareLoopCountPerSec = lastStats.spareLoopCount / refreshTime;
      output.print(F(",  CPU load: "));
      output.print(100.0f * (1.0f - (float)spareLoopCountPerSec / maxSpareLoopCountPerSec), 1);
      output.print(F("%"));
      output.println();
    }
  }
  
  if (showBitLengths) {
    output.println(F("------ Half-bit count by length (us) -------"));
    for (int i=minBitLength; i<=maxBitLength; i++) {
      unsigned long c0 = lastStats.countByLength[0][i-minBitLength];
      unsigned long c1 = lastStats.countByLength[1][i-minBitLength];        
      if (c0 > 0 || c1 > 0) {
        if (i == minBitLength) output.print(F("<="));
        else if (i == maxBitLength) output.print(F(">="));
        output.print(i);
        output.print('\t');
        output.print(c0);
        output.print('\t');
        output.println(c1);
      }
    }
  }
  output.println(F("--------------------------------------------"));
}

//=======================================================================
// ClearDCCData clears the contents of the packetHashList array.
// This array normally contains the checksums of received DCC packets, 
// and is used to suppress the decoding of repeated packets.

void ClearDccData() {
  for (byte n=0; n<packetHashListSize; n++) packetHashList[n]=0;
  bufferCounter=0;

  // Copy and reset active stats.  Disabling interrupts would improve
  // consistency a bit but causes bits to be lost (CRC errors recorded).
  // Using memcpy and memset minimises the time at risk of inconsistency.
  memcpy(&lastStats, (void *)&activeStats, sizeof(struct stats));
  memset((void *)&activeStats, 0, sizeof(struct stats));
  activeStats.minInterruptTime = activeStats.min0 = activeStats.min1 = 65535;
}

//=======================================================================
// UpdateLED is called in the main loop to set/reset the LED fault indication
// in the event of a fault being detected within the sample period.

void UpdateLED() {
  
  #ifdef LEDPIN_FAULT
  static bool ledLit = false;
  if (activeStats.glitchCount > 0 || activeStats.checksumError > 0 || 
      activeStats.countLongPackets > 0 || activeStats.countLostPackets > 0) {
    if (!ledLit) {
      digitalWrite(LEDPIN_FAULT, 1);
      ledLit = true;
    }
  } else {
    if (ledLit) {
      digitalWrite(LEDPIN_FAULT, 0);
      ledLit = false;
    }
  }
  #endif
}


//=======================================================================
// Validate received packet and pass to decoder.
// Return false if nothing done.

bool processDCC(Print &output) {
  byte isDifferentPacket=0;
  
  if (!packetsPending) {
    return false;
  }
  
  pktByteCount = dccPacket[inputPacket][0];
  // Check packet isn't empty
  if (pktByteCount > 0) {
    // Calculate and verify checksum
    byte checksum = 0;
    for (byte n = 1; n <= pktByteCount; n++) 
      checksum ^= dccPacket[inputPacket][n];
    if (checksum) {  // Result should be zero, if not it's an error!
      activeStats.checksumError++;
    } else { 
      // There is a new packet with a correct checksum
      #ifdef LEDPIN_DECODING
      digitalWrite(LEDPIN_DECODING, 1);
      #endif
      
      // Generate a cyclic hash based on the packet contents for checking if we've seen a similar packet before.
      isDifferentPacket=true;
      unsigned int hash = dccPacket[inputPacket][pktByteCount];  // calculate checksum
      for (byte n=1; n<pktByteCount; n++)
        hash = ((hash << 5) | (hash >> 11)) ^ dccPacket[inputPacket][n];
        
      // Check if packet's checksum is already in the list. 
      for (byte n=0; n<packetHashListSize ; n++) {
        if (hash==packetHashList[n]) 
          isDifferentPacket=false; 
      }
  
      if (isDifferentPacket) {
        packetHashList[bufferCounter++] = hash; // add new packet's hash to the list
        if (bufferCounter >= packetHashListSize) bufferCounter = 0;
        
        DecodePacket(output, inputPacket, isDifferentPacket);
      }

      // Optional test led whose brightness depends on loco speed setting.
      #ifdef LEDPIN_LOCOSPEED
      // Output to LED
      if (dccPacket[inputPacket][1]==B00000011 && dccPacket[inputPacket][2] == B00111111) { 
        analogWrite(LEDPIN_LOCOSPEED, map(dccPacket[inputPacket][3] & B01111111, 0, 127, 0, 255));
      }
      #endif

      #ifdef LEDPIN_DECODING
      digitalWrite(LEDPIN_DECODING, 0);
      #endif
    }
  }
  packetsPending--;  // Free packet buffer.
  if (++inputPacket >= nPackets) inputPacket = 0;

  return true;
}

void put(char buffer[], int len, int &pos, const char *str) {
  strncpy(&buffer[pos], str, len);
  pos += len;
}
    
//=======================================================================
// Read data from the dccPacket structure and decode into 
// textual representation.  Send results out over the USB serial
// connection.

void DecodePacket(Print &output, int inputPacket, bool isDifferentPacket) {
  byte instrByte1;
  byte decoderType; //0=Loc, 1=Acc
  unsigned int decoderAddress;
  byte speed;
  char buffer[35];
  char val1Buf[15], val2buf[15];
  val1Buf[0] = val2buf[0] = '\0';

  // First determine the decoder type and address.
  if (dccPacket[inputPacket][1]==B11111111) { //Idle packet
    if (isDifferentPacket)
      snprintf(buffer, sizeof(buffer), "Idle ");
    decoderType = 255;
  } else if (!bitRead(dccPacket[inputPacket][1],7)) { //bit7=0 -> Loc Decoder Short Address
    decoderAddress = dccPacket[inputPacket][1];
    instrByte1 = dccPacket[inputPacket][2];
    decoderType = 0;
  }  else {
    if (bitRead(dccPacket[inputPacket][1],6)) { //bit7=1 AND bit6=1 -> Loc Decoder Long Address
      decoderAddress = 256 * (dccPacket[inputPacket][1] & B00111111) + dccPacket[inputPacket][2];
      instrByte1 = dccPacket[inputPacket][3];
      decoderType = 0;
    }
    else { //bit7=1 AND bit6=0 -> Accessory Decoder
      decoderAddress = dccPacket[inputPacket][1]&B00111111;
      instrByte1 = dccPacket[inputPacket][2];
      decoderType = 1;
    }
  }

  // Handle decoder type 0 and 1 separately.
  if (decoderType == 1) { // Accessory Basic
    if (showAcc) {
      if (instrByte1 & B10000000) { // Basic Accessory
        decoderAddress = (((~instrByte1) & B01110000) << 2) + decoderAddress;
        byte port = (instrByte1&B00000110)>>1;
        snprintf(buffer, sizeof(buffer), "Acc %d %d:%d %d %s",
          (decoderAddress-1)*4 + port + 1,
          decoderAddress,
          port,
          bitRead(instrByte1, 3),
          bitRead(instrByte1, 0) ? " On" : "Off");
      } else { // Accessory Extended NMRA spec is not clear about address and instruction format !!!
        itoa(dccPacket[inputPacket][3], val1Buf, 2); 
        snprintf(buffer, sizeof(buffer), "Acc Ext %d Asp %s",
          (decoderAddress << 5) + ((instrByte1 & B01110000) >> 2) + ((instrByte1 & B00000110) >> 1),
          val1Buf);
      }
    }
  }
  else if (decoderType == 0)  { // Loco / Multi Function Decoder
    if (showLoc && isDifferentPacket) {
      const char *op = "";
      byte value;
      byte instructionType = instrByte1 >> 5;
      switch (instructionType) {
        case 0:
          snprintf(buffer, sizeof(buffer), "Loc%d Control", decoderAddress);
          break;

        case 1: // Advanced Operations
          if (instrByte1==B00111111) { //128 speed steps
            if (bitRead(dccPacket[inputPacket][pktByteCount-1], 7)) 
              op = "Fw128";
            else 
              op = "Rv128";
            byte speed = dccPacket[inputPacket][pktByteCount-1] & B01111111;
            if (!speed) 
              op = "Stop";
            else if (speed==1) 
              op = "Estop";
            else 
              itoa(speed-1, val1Buf, 10);
          } else if (instrByte1==B00111110) { //Speed Restriction
            if (bitRead(dccPacket[inputPacket][pktByteCount-1], 7)) 
              op = "On";
            else 
              op = "Off";
            itoa(dccPacket[inputPacket][pktByteCount-1] & B01111111, val1Buf, 10);;
          }
          snprintf(buffer, sizeof(buffer), "Loc%d %s %s",
            decoderAddress, op, val1Buf);

          break;

        case 2: // Reverse speed step
          speed = ((instrByte1 & B00001111) << 1) - 3 + bitRead(instrByte1,4);
          if (speed==253 || speed==254) 
            op = "Stop";
          else if (speed==255 || speed==0) 
            op = "EStop";
          else {
            op = "Rev";
            itoa(speed, val1Buf, 10);
          }
          snprintf(buffer, sizeof(buffer), "Loc%d %s %s",
            decoderAddress, op, val1Buf);
          break;

        case 3: // Forward speed step
          speed = ((instrByte1 & B00001111) << 1) - 3 + bitRead(instrByte1,4);
          if (speed==253 || speed==254) 
            op = "Stop";
          else if (speed==255 || speed==0) 
            op = "EStop";
          else {
            op = "Forw";
            itoa(speed, val1Buf, 10);
          }
          snprintf(buffer, sizeof(buffer), "Loc%d %s %s",
            decoderAddress, op, val1Buf);
          break;

        case 4: // Loc Function L-4-3-2-1
          op = "L F4-F1";
          itoa(instrByte1 & B00011111, val1Buf, 2); // 5 bit binary
          snprintf(buffer, sizeof(buffer), "Loc%d %s %s",
            decoderAddress, op, val1Buf);
          break;

        case 5: // Loc Function 8-7-6-5
          if (bitRead(instrByte1,4)) {
            op = "F8-F5";
            itoa(instrByte1 & B00001111, val1Buf, 2); // 4 bit binary
          }
          else { // Loc Function 12-11-10-9
            op = "F12-F9";
            itoa(instrByte1 & B00001111, val1Buf, 2); // 4 bit binary
          }
          snprintf(buffer, sizeof(buffer), "Loc%d %s %s",
            decoderAddress, op, val1Buf);
          break;

        case 6: // Future Expansions
          switch (instrByte1 & B00011111) {
            case 0: // Binary State Control Instruction long form
              op = "BinLong";
              itoa(256 * dccPacket[inputPacket][pktByteCount-1] + (dccPacket[inputPacket][pktByteCount-2] & B01111111),
                val1Buf, 10);
              if bitRead(dccPacket[inputPacket][pktByteCount-2], 7) strcpy(val2buf, "On");
              else strcpy(val2buf, "Off");
              break;
            case B00011101: // Binary State Control
              op = "BinShort";
              itoa(dccPacket[inputPacket][pktByteCount-1] & B01111111, val1Buf, 10);
              if bitRead(dccPacket[inputPacket][pktByteCount-1], 7) strcpy(val2buf, "On");
              else strcpy(val2buf, "Off");
              break;
            case B00011110: // F13-F20 Function Control
              op = "F20-F13";
              itoa(dccPacket[inputPacket][pktByteCount-1], val1Buf, 2); // 8 bit binary
              break;
            case B00011111: // F21-F28 Function Control
              op = "F28-F21";
              itoa(dccPacket[inputPacket][pktByteCount-1], val1Buf, 2); // 8 bit binary
              break;
            default:
              op = "unknown";
              break;
          }
          snprintf(buffer, sizeof(buffer), "Loc%d %s %s %s",
            decoderAddress, op, val1Buf, val2buf);
          break;

        case 7:
          value = dccPacket[inputPacket][pktByteCount-1];
          if (instrByte1&B00010000) { // CV Short Form
            byte cvType=instrByte1 & B00001111;
            switch (cvType) {
              case B00000010:
                strcpy(val1Buf, "23");
                break;
              case B00000011:
                strcpy(val1Buf, "24");
                break;
              case B00001001:
                strcpy(val1Buf, "Lock");
                break;
              default:
                strcpy(val1Buf, "Unknown");
                break;
            }
            snprintf(buffer, sizeof(buffer), "Loc%d CV %s %d",
              decoderAddress, val1Buf, value);
          }
          else { // CV Long Form
            int cvAddress = 256 * (instrByte1 & B00000011) + dccPacket[inputPacket][pktByteCount-2] + 1;
            switch (instrByte1 & B00001100) {
              case B00000100: // Verify Byte
                strcpy(val1Buf, "Vrfy");
                itoa(value, val2buf, 10);
                break;
              case B00001100: // Write Byte
                strcpy(val1Buf, "Write");
                itoa(value, val2buf, 10);
                break;
              case B00001000: // Bit Write
                if (value & B00010000) strcpy(val1Buf, "BitVfy");
                else strcpy(val1Buf, "BitWrt");
                //*****************
                output.print(value & B00000111);
                output.print(' ');
                output.print((value & B00001000)>>3);
                //*************
                break;
              default:
                strcpy(val1Buf, "unknown");
                break;
            }
            snprintf(buffer, sizeof(buffer), "Loc%d CV%d %s %d",
              decoderAddress, cvAddress, val1Buf, value);
          }
          break;

        default: 
          op = "unknown";
          break;
      }
    }
  }
  output.print(buffer);
  printPacket(inputPacket);
  strncat(pktBuffer, buffer, sizeof(pktBuffer));
  strncat(pktBuffer, "\n", sizeof(pktBuffer));
}

//=======================================================================
// Process commands sent over the USB serial connection.
//  Return false if nothing done.

bool processCommands() {
  if (Serial.available()) {
    switch (Serial.read()) {
      case 49: 
        Serial.println(F("Refresh Time = 1s"));
        refreshTime=1;
      break;
      case 50:
        Serial.println(F("Refresh Time = 2s"));
        refreshTime=2;
      break;
      case 51:
        Serial.println(F("Refresh Time = 4s"));
        refreshTime=4;
      break;
      case 52:
        Serial.println(F("Refresh Time = 8s"));
        refreshTime=8;
      break;
      case 53:
        Serial.println(F("Refresh Time = 16s"));
        refreshTime=16;
      break;
      case 54:
        Serial.println(F("Buffer Size = 4"));
        packetHashListSize=2;
      break;
      case 55:
        Serial.println(F("Buffer Size = 8"));
        packetHashListSize=8;
      break;
      case 56:
        Serial.println(F("Buffer Size = 16"));
        packetHashListSize=16;
      break;
      case 57:
        Serial.println(F("Buffer Size = 32"));
        packetHashListSize=32;
      break;
      case 48:
        Serial.println(F("Buffer Size = 64"));
        packetHashListSize=64;
      break;
      case 'a': case 'A':
        showAcc=!showAcc;
        Serial.print(F("show accessory packets = "));
        Serial.println(showAcc);
      break;
      case 'l': case 'L':
        showLoc=!showLoc;
        Serial.print(F("show loco packets = "));
        Serial.println(showLoc);
      break;
      case 'h': case 'H':
        showHeartBeat = !showHeartBeat;
        Serial.print(F("show heartbeat = "));
        Serial.println(showHeartBeat);
      break;
      case 'd': case 'D':
        showDiagnostics = !showDiagnostics;
        Serial.print(F("show diagnostics = "));
        Serial.println(showDiagnostics);
      break;
      case 'f': case 'F':
        filterInput = !filterInput;
        Serial.print(F("filter input = "));
        Serial.println(filterInput);
        break;
      case 'b': case 'B':
        showBitLengths = !showBitLengths;
        Serial.print(F("show bit lengths = "));
        Serial.println(showBitLengths);
        break;
      case 'c': case 'C':
        showCpuStats = !showCpuStats;
        Serial.print(F("show Cpu stats = "));
        Serial.println(showCpuStats);
        break;        
      case '?': 
        Serial.println();
        Serial.println(F("Keyboard commands that can be sent via Serial Monitor:"));
        Serial.println(F("1 = 1s refresh time"));
        Serial.println(F("2 = 2s"));
        Serial.println(F("3 = 4s (default)"));
        Serial.println(F("4 = 8s"));
        Serial.println(F("5 = 16s"));
        Serial.println(F("6 = 4 DCC packet buffer"));
        Serial.println(F("7 = 8"));
        Serial.println(F("8 = 16"));
        Serial.println(F("9 = 32 (default)"));
        Serial.println(F("0 = 64"));
        Serial.println(F("a = show accessory packets toggle"));
        Serial.println(F("l = show locomotive packets toggle"));
        Serial.println(F("d = show diagnostics toggle"));
        Serial.println(F("h = show heartbeat toggle"));
        Serial.println(F("b = show half-bit counts by length toggle"));
        Serial.println(F("c = show cpu/irc usage in sniffer"));
        Serial.println(F("f = input filter toggle"));
        Serial.println(F("? = help (show this information)"));
        Serial.print(F("ShowLoco "));
        Serial.print(showLoc);
        Serial.print(F(" / ShowAcc "));
        Serial.print(showAcc);
        Serial.print(F(" / RefreshTime "));
        Serial.print(refreshTime);
        Serial.print(F(" / BufferSize "));
        Serial.println(packetHashListSize);
        Serial.println();
      break;
    }
    return true;
  } else
    return false;
}


// Write Statistics summary to a buffer for use by the HTTP Server and 
// for writing to the OLED display.
void WriteStatistics(char *buffer, size_t bufferSize) {
  if (lastStats.count > 0) {
    snprintf(buffer, bufferSize, 
      "Bits/%d sec: %lu\n"
      " 0: %lu 1: %lu\n"
      "Lengths (us)\n"
      " 0:%.1f (%u-%u)\n"
      " 1:%.1f (%u-%u)\n"
      "Deltas: 0:<%d 1:<%d\n"
      "Frames: %u\n"
      "CksumErr: %u\n"
      "--\n"
      "%s", 
      refreshTime,
      lastStats.count/2, 
      lastStats.count0/2,
      lastStats.count1/2,
      (double)lastStats.total0/lastStats.count0,
      lastStats.min0,
      lastStats.max0,
      (double)lastStats.total1/lastStats.count1,
      lastStats.min1,
      lastStats.max1,
      lastStats.max0BitDelta,
      lastStats.max1BitDelta,
      lastStats.packetCount,
      lastStats.checksumError,
      pktBuffer);
  } else {
    snprintf(buffer, bufferSize,
      "Bits/%d sec: 0\n"
      " 0: 0 1: 0\n"
      "Lengths (us)\n"
      " 0: N/A\n"
      " 1: N/A\n"
      "Deltas: N/A\n"
      "Frames: 0\n"
      "CksumErr: 0\n"
      "--", 
      refreshTime);
  }
  // Reinitialise packet buffer.
  pktBuffer[0] = '\0';
}
