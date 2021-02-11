/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/

/*
This is for handling the data broadcasted to various CAN dashes and isntrument clusters.
*/
#include "globals.h"
#include "cancomms.h"
#include "maths.h"
#include "errors.h"
#include "utilities.h"


void sendGaugeMessage(uint16_t DashMessageID)
{
  switch (DashMessageID)
  {
    case CAN_BMW_DME1:
      uint16_t temp_RPM;
      temp_RPM =  currentStatus.RPM * 6.4; //RPM conversion
      outMsg.id = gaugeMessageID;
      outMsg.len = 8;
      outMsg.buf[0] = 0x01;
      outMsg.buf[1] = 0x07;
      outMsg.buf[2] = lowByte(temp_RPM);
      outMsg.buf[3] = highByte(temp_RPM);
      outMsg.buf[4] = 0x65; //Indicated Engine Torque in %
      outMsg.buf[5] = 0x12; //Engine Torque Loss (due to engine friction, AC compressor and electrical power consumption) in %
      outMsg.buf[6] = 0x00; //Theorethical Engine Torque in % after traction control
      outMsg.buf[7] = 0x62;
    break;

    case CAN_BMW_DME2:
      uint8_t temp_TPS;
      uint8_t temp_BARO;
      int16_t temp_CLT;
      temp_TPS = map(currentStatus.TPS, 0, 100, 0, 254);//TPS value conversion (from 0x00 to 0xFE)
      temp_CLT = (((currentStatus.coolant - CALIBRATION_TEMPERATURE_OFFSET) + 48)*4/3); //CLT conversion (actual value to add is 48.373, but close enough)
      if (temp_CLT > 255) { temp_CLT = 255; } //CLT conversion can yield to higher values than what fits to byte, so limit the maximum value to 255.
      temp_BARO = currentStatus.baro;

      outMsg.id = gaugeMessageID;
      outMsg.len = 7;
      outMsg.buf[0] = 0x07;
      outMsg.buf[1] = temp_CLT;
      outMsg.buf[2] = temp_BARO;
      outMsg.buf[3] = 0x19;
      outMsg.buf[4] = 0x00;
      outMsg.buf[5] = temp_TPS;
      outMsg.buf[6] = 0x00;
    break;

    case 0x545:       //fuel consumption and CEl light for BMW e46/e39/e38 instrument cluster
                      //fuel consumption calculation not implemented yet. But this still needs to be sent to get rid of the CEL and EML fault lights on the dash.
      outMsg.id = gaugeMessageID;
      outMsg.len = 5;
      outMsg.buf[0] = 0x00;  //Check engine light (binary 10), Cruise light (binary 1000), EML (binary 10000).
      outMsg.buf[1] = 0x00;  //LSB Fuel consumption
      outMsg.buf[2] = 0x00;  //MSB Fuel Consumption
      if (currentStatus.coolant > 159) { outMsg.buf[3] = 0x08; } //Turn on overheat light if coolant temp hits 120 degrees celcius.
      else { outMsg.buf[3] = 0x00; } //Overheat light off at normal engine temps.
      outMsg.buf[4] = 0x7E; //this is oil temp
    break;

    case 0x280:       //RPM for VW instrument cluster
      temp_RPM =  currentStatus.RPM * 4; //RPM conversion
      outMsg.id = gaugeMessageID;
      outMsg.len = 8;
      outMsg.buf[0] = 0x49;
      outMsg.buf[1] = 0x0E;
      outMsg.buf[2] = lowByte(temp_RPM);
      outMsg.buf[3] = highByte(temp_RPM);
      outMsg.buf[4] = 0x0E;
      outMsg.buf[5] = 0x00;
      outMsg.buf[6] = 0x1B;
      outMsg.buf[7] = 0x0E;
    break;

    case 0x5A0:       //VSS for VW instrument cluster
      uint16_t temp_VSS;
      temp_VSS =  currentStatus.vss / 0.0075; //VSS conversion
      outMsg.id = gaugeMessageID;
      outMsg.len = 8;
      outMsg.buf[0] = 0xFF;
      outMsg.buf[1] = lowByte(temp_VSS);
      outMsg.buf[2] = highByte(temp_VSS);
      outMsg.buf[3] = 0x00;
      outMsg.buf[4] = 0x00;
      outMsg.buf[5] = 0x00;
      outMsg.buf[6] = 0x00;
      outMsg.buf[7] = 0xAD;
    break;

    default:
    break;
  }
}
#endif
