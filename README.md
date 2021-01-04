# DccDiagnostics

Diagnostic/sniffer program for DCC analysis, based on Arduino target.
The Arduino may be directly connected to another Arduino, e.g. in a
DCC++ or DCC++ EX system, but if used to monitor DCC signals, which are
normally bipolar 12-18V signals, an optocoupler circuit will be required.
Various ones are discussed on the internet, but if you use a 6N137 I 
would recommend a 1nF capacitor across the input pins 2 and 3 to stabilise
the input, and use a pull-up resistor on the output pin 6 of 470 ohms 
connected to +5V - a larger resistor (even the Arduino's internal pull-up)
will generally work but will slow down the optocoupler.  No 
pull-up resistor is required on pin 7.

The diagnostic program supports the Arduino Uno, Nano and Mega particularly.

Measurements are performed at an accuracy of 1us using Timer1.  For 
increased compatibility, it is possible to use micros() instead but
this introduces up to 4us error in the micros() result, plus up to
6.5us uncertainty in the scheduling of the interrupt which samples
the micros() value; consequently, the error in measurement may vary 
by +/- 10.5us when using micros().

The Timer1 module allows the time of a digital input change to be 
captured through hardware at a resolution of up to 1/16th microsecond.
I'm using a resolution of 1 microsecond only.

It has also been compiled successfully for the ESP8266 and ESP32 but not tested.
On these targets, the timing is performed using the millis() function, 
some inaccuracies due to interrupts will still be present, although smaller (because of 
the faster clock rate).

The DccDiagnostics produces output to the Serial stream, by default
once every 4 seconds.  The output statistics includes bit counts, 
bit lengths, checksum errors, packets too long, and estimated 
interrupt time within the DccDiagnostics sketch.  Fleeting input state changes
(<= 3us) are optionally filtered and counted.

In between statistics output, received DCC packets are decoded and 
printed; duplicate throttle packets and idle packets are not printed more than once per period.

Press '?' to see help of the commands available.  By default the 
breakdown of pulse lengths is not displayed, press 'B' to enabled it.

Example output from DCC++ Classic (5V direct connection, Main Track):

`-
`Bit Count=29836 (Zeros=6698, Ones=23138), Glitches=0
`Packets received=609, Checksum Error=0, Lost pkts=0, Long pkts=0
`0 half-bit length (us): 100.0 (100-100) delta <= 1
`1 half-bit length (us): 58.0 (58-58) delta <= 1
`------ Half-bit count by length (us) -------
`58      23139   23139
`100     6698    6698
`--------------------------------------------
`Idle 
`-
`Bit Count=27677 (Zeros=9544, Ones=18133), Glitches=0
`Packets received=477, Checksum Error=0, Lost pkts=0, Long pkts=0
`0 half-bit length (us): 100.0 (100-100) delta <= 1
`1 half-bit length (us): 58.0 (58-58) delta <= 1
`--------------------------------------------
`Loc 3 Forw128 45  00000011 00111111 10101110 
`-

Same for DCC++ EX (5V direct connection, Main Track):

`-
`Bit Count=27641 (Zeros=6910, Ones=20731), Glitches=0
`Packets received=628, Checksum Error=0, Lost pkts=0, Long pkts=0
`0 half-bit length (us): 116.0 (110-122) delta <= 13
`1 half-bit length (us): 58.0 (52-64) delta <= 13
`--------------------------------------------
`Idle 
`-

##Command Summary

` Keyboard commands that can be sent via Serial Monitor:
` 1 = 1s refresh time
` 2 = 2s 
` 3 = 4s (default)
` 4 = 8s
` 5 = 16s
` 6 = 4 DCC packet buffer
` 7 = 8
` 8 = 16
` 9 = 32 (default)
` 0 = 64
` a = show accessory packets toggle
` l = show locomotive packets toggle
` d = show diagnostics toggle
` h = show heartbeat toggle
` b = show half-bit counts by length
` c = show cpu/irc usage in sniffer
` f = input filter toggle
` ? = help (show this information)
