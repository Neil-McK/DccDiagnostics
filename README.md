# DccDiagnostics

Diagnostic/sniffer program for DCC analysis, based on Arduino target.

Supports Uno, Nano and Mega particularly.

Measurements are performed at an accuracy of 1us using Timer1.  For 
increased compatibility, it is possible to use micros() instead but
this introduces up to 4us error in the micros() result, plus up to
6.5us uncertainty in the scheduling of the interrupt which samples
the micros() value; consequently, the error in measurement may vary 
by +/- 10.5us when using micros().

The Timer1 module allows the time of a digital input change to be 
captured through hardware at a resolution of up to 1/16th microsecond.
I'm using a resolution of 1 microsecond only.

The DccDiagnostics produces output to the Serial stream, by default
once every 4 seconds.  The output statistics includes bit counts, 
bit lengths, checksum errors, packets too long, and estimated 
interrupt time within the DccDiagnostics sketch.  Fleeting interrupts
are optionally filtered and counted.

In between statistics output, received DCC frames are decoded and 
printed; duplicate frames are not printed more than once per period.

Press '?' to see help of the commands available.  By default the 
breakdown of pulse lengths is not displayed, press 'B' to enabled it.