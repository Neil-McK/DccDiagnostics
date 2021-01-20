#ifndef dccstats_h
#define dccstats_h

// Range of bit lengths (us) for recording count by bit length.
const int minBitLength = 45; // microseconds (58 - 20% - 1)
const int maxBitLength = 141; // microseconds (116 + 20% + 1)

// Statistics structure.  Sizes of counts are chosen to be large enough not to overflow.
struct stats {
  unsigned long count=0, count0=0, count1=0;
  unsigned int packetCount=0, checksumError=0, countLongPackets=0, countLostPackets=0;
  unsigned int max1=0, min1=65535, max0=0, min0=65535;
  unsigned long total1=0, total0=0, totalInterruptTime=0;
  unsigned int maxInterruptTime=0, minInterruptTime=65535;
  unsigned int max1BitDelta=0, max0BitDelta=0;
  unsigned long glitchCount=0, spareLoopCount=0;
  unsigned int countByLength[2][maxBitLength-minBitLength+1];
};

#endif