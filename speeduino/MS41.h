// This is to use the DS2 library in a way that it emulates MS42 ecu for K-line.

#ifndef MS42_H
#define MS42_H
#include "DS2.h"

#if defined(CORE_STM32)
  DS2 DS2(Serial3);
#else
  DS2 DS2(Serial2);
#endif

uint8_t data[55]; // For DS2 data
bool responseSent; // to keep track if we have responded to data request or not.

void setupDS2();
void checkDS2();
uint32_t convertValue(float, float, float);
void addValToData(uint16_t, uint8_t, uint8_t, uint8_t);
void sendReply(uint8_t);
void sendEcuId(uint8_t);

#endif