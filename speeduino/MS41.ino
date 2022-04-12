// This is to use the DS2 library in a way that it emulates MS41 ecu for K-line.

#include "MS41.h"

void setupDS2() {
  #if defined(CORE_STM32)
  Serial3.begin(9600, SERIAL_8E1);
  Serial3.setTimeout(ISO_TIMEOUT);
  #else
  Serial2.begin(9600, SERIAL_8E1);
  Serial2.setTimeout(ISO_TIMEOUT);
  #endif
  responseSent = false;
}

uint32_t convertValue(float val, float mul = 1, float add = 0) {
  // uint32_t meaning we can set up to 4 bytes this way
  uint32_t convertedVal = (uint32_t) ((val - add)/mul); // we need to reverse what we do on logger side
  return convertedVal;
}

void addValToData(uint16_t val, uint8_t data[], uint8_t offset, uint8_t length = 1) {
  if(length == 1) {
    data[offset] = lowByte(val);
  }
  else if (length == 2) {
    data[offset] = highByte(val);
    data[offset + 1] = lowByte(val);
  }
}

void sendReply(uint8_t data[]) {
  data[0] = 0x12; // Not needed as our data already have that
  data[1] = 38;   // ms42 response lenght is 38 (26 hex)
  data[2] = 0xA0; // Ack
  uint8_t offset = 3; // payload starts after 3 initial bytes

  // Here is where payload starts:
  uint16_t valueToSend;
  
  // RPM
  //addValToData(valueToSend, data, valueOffset + offset, valueLength);
  addValToData(currentStatus.RPM, data, 0 + offset, 2);
  
  // VSS
  addValToData(currentStatus.vss, data, 2 + offset, 1);
  
  // TPS
  valueToSend = convertValue(currentStatus.TPS, 0.390625);
  addValToData(valueToSend, data, 4 + offset, 1);
  
  // MAF
  valueToSend = convertValue(currentStatus.VE, 0.25);
  addValToData(valueToSend, data, 5 + offset, 2);
  
  // IAT
  valueToSend = convertValue(currentStatus.IAT, 0.75, -8);
  addValToData(valueToSend, data, 7 + offset, 1);
  
  // CLT
  valueToSend = convertValue(currentStatus.coolant, 0.75, -8);
  addValToData(valueToSend, data, 8 + offset, 1);
  
  // Oil temp
  valueToSend = convertValue(currentStatus.canin[1], 0.75, -8);
  addValToData(valueToSend, data, 9 + offset, 1);
  
  // Ignition Angle
  valueToSend = convertValue(currentStatus.advance, -0.375, 72);
  addValToData(valueToSend, data, 11 + offset, 1);
  
  // IPW
  valueToSend = convertValue((currentStatus.PW1 / 1000), 0.04);
  addValToData(valueToSend, data, 12 + offset, 2);

  // ICV?
  valueToSend = convertValue(currentStatus.idleLoad, 0.001526);
  addValToData(valueToSend, data, 14 + offset, 2);
  
  // ICV duty
  valueToSend = convertValue(currentStatus.idleLoad, 0.001526);
  addValToData(valueToSend, data, 16 + offset, 2);
  
  // Battery Voltage
  addValToData(currentStatus.battery10, data, 20 + offset, 1);

  // Lambda Int 1
  valueToSend = convertValue(currentStatus.egoCorrection, 0.0015258789, -50);
  addValToData(valueToSend, data, 21 + offset, 2);
  
  // Lambda Int 2
  valueToSend = convertValue(0, 0.0015258789, -50);
  addValToData(valueToSend, data, 23 + offset, 2);

  // Load
  valueToSend = convertValue(currentStatus.MAP, 0.021);
  addValToData(valueToSend, data, 29 + offset, 2);
  
  // Knock Voltage 1
  valueToSend = convertValue(0, 0.01952);
  addValToData(valueToSend, data, 31 + offset, 1);
  
  // Knock Voltage 2
  valueToSend = convertValue(0, 0.01952);
  addValToData(valueToSend, data, 32 + offset, 1);
  
  // Checksum
  uint8_t checksum = 0;
  for(uint8_t i = 0; i < data[1]-1; i++) {
    checksum ^= data[i]; // Simple XOR checksum
  }
  data[data[1]-1] = checksum;
  DS2.writeData(data);
  responseSent = true;
}

void sendEcuId(uint8_t data[]) {
  data[0] = 0x12; // Not really needed as our data already have this
  data[1] = 11;   // response length
  data[2] = 0xA0; // Ack
  // Ecu Id in ASCII, we use ms42 C6-SW version id here
  data[3] = 0x37;  // 7
  data[4] = 0x35;  // 5
  data[5] = 0x30;  // 0
  data[6] = 0x30;  // 0
  data[7] = 0x32;  // 2
  data[8] = 0x35;  // 5
  data[9] = 0x35;  // 5
  
    // Checksum
  uint8_t checksum = 0;
  for(uint8_t i = 0; i < data[1]-1; i++) {
    checksum ^= data[i]; // Simple XOR checksum
  }
  data[data[1]-1] = checksum;
  DS2.writeData(data);
  responseSent = true;
}

void checkDS2() {
  if( responseSent == false ){
    // commands are 4 bytes long, so we only start reading RX buffer, when whe have full command there.
    if( (DS2.available() >= 4)){ // we also want to have new updated data available from speeduino, or it's not worth sending anything to K-line.
      if(DS2.readCommand(data)){  //Read command will ensure it's own length and if checksum is ok.
        switch(data[2]) {
          case 0x0B:
            if(data[3] == 0x03) sendReply(data);
            break;
          case 0x00:
            if(data[3] == 0x16) sendEcuId(data);
            break;
          case 0xA0:
            Serial.println("Error! DS2 Received Ack.");
            break;
          default:
            Serial.println("Not supported DS2 command");
          break;
        }
      }
    }
  }
  // if we have sent the response, we'll wait for the echo of to be filled in serial buffer and then we will just read it out to get rid of it.
  else if( responseSent == true ){
    if( DS2.available() >= DS2.getEcho() ){
      DS2.readCommand(data);
      responseSent = false; // there is no more echo on the RX buffer, so we are ready to read new command
    }
  }
}