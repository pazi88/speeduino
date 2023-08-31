/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/

/*
This is for handling the data broadcasted to various CAN dashes and instrument clusters.
*/
#if defined(NATIVE_CAN_AVAILABLE)
#include "globals.h"

// Forward declare
void DashMessage(uint16_t DashMessageID);

void sendBMWCluster()
{
  dashMessage(CAN_BMW_DME1);
  Can0.write(outMsg);
  dashMessage(CAN_BMW_DME2);
  Can0.write(outMsg);
  dashMessage(CAN_BMW_DME4);
  Can0.write(outMsg);
}

void sendVAGCluster()
{
  dashMessage(CAN_VAG_RPM);
  Can0.write(outMsg);
  dashMessage(CAN_VAG_VSS);
  Can0.write(outMsg);
}

void sendHaltech30Hz()
{
  dashMessage(0x360);
  Can0.write(outMsg);
  if( (configPage10.fuelPressureEnable > 0) || (configPage10.oilPressureEnable > 0) )  //Fuel/Oil pressures are only valid info here
  {
    dashMessage(0x361);
    Can0.write(outMsg);
  }
  dashMessage(0x362);
  Can0.write(outMsg);
}

void sendHaltech15Hz()
{
  haltechDashMessage(0x363);
  Can0.write(outMsg);
  haltechDashMessage(0x368);
  Can0.write(outMsg);
  haltechDashMessage(0x369);
  Can0.write(outMsg);
  /*
  if( configPage10.knock_mode != KNOCK_MODE_OFF )
  {
    haltechDashMessage(0x36A);
    Can0.write(outMsg);
  }
  haltechDashMessage(0x36B);
  Can0.write(outMsg);
  haltechDashMessage(0x36C);
  Can0.write(outMsg);
  */
  haltechDashMessage(0x36D);
  Can0.write(outMsg);
  haltechDashMessage(0x36E);
  Can0.write(outMsg);
  if( configPage6.boostEnabled == 1 )  //Boost duty is only valid info here
  {
    haltechDashMessage(0x36F);
    Can0.write(outMsg);
  }
  haltechDashMessage(0x370);
  Can0.write(outMsg);
}

void sendHaltech10Hz()
{
  //haltechDashMessage(0x371);
  //Can0.write(outMsg);
  haltechDashMessage(0x372);
  Can0.write(outMsg);
  //haltechDashMessage(0x373);
  //Can0.write(outMsg);
  //haltechDashMessage(0x374);
  //Can0.write(outMsg);
  //haltechDashMessage(0x375);
  //Can0.write(outMsg);
}
void sendHaltech4Hz()
{
  haltechDashMessage(0x3E0);
  Can0.write(outMsg);
  //haltechDashMessage(0x3E1);
  //Can0.write(outMsg);
  //haltechDashMessage(0x3E2);
  //Can0.write(outMsg);
  haltechDashMessage(0x3E3);
  Can0.write(outMsg);
  haltechDashMessage(0x3E4);
  Can0.write(outMsg);
}

// switch case for gathering all data to message based on CAN Id.
void dashMessage(uint16_t dashMessageID)
{
  switch (dashMessageID)
  {
    case CAN_BMW_DME1:
      uint32_t temp_RPM;
      temp_RPM = currentStatus.RPM * 64;  //RPM conversion is currentStatus.RPM * 6.4, but this does it without floats.
      temp_RPM = temp_RPM / 10;
      outMsg.id = DashMessageID;
      outMsg.len = 8;
      outMsg.buf[0] = 0x05;  //bitfield, Bit0 = 1 = terminal 15 on detected, Bit2 = 1 = the ASC message ASC1 was received within the last 500 ms and contains no plausibility errors
      outMsg.buf[1] = 0x0C;  //Indexed Engine Torque in % of C_TQ_STND TBD do torque calculation.
      outMsg.buf[2] = lowByte(uint16_t(temp_RPM));  //lsb RPM
      outMsg.buf[3] = highByte(uint16_t(temp_RPM)); //msb RPM
      outMsg.buf[4] = 0x0C;  //Indicated Engine Torque in % of C_TQ_STND TBD do torque calculation!! Use same as for byte 1
      outMsg.buf[5] = 0x15;  //Engine Torque Loss (due to engine friction, AC compressor and electrical power consumption)
      outMsg.buf[6] = 0x00;  //not used
      outMsg.buf[7] = 0x35;  //Theorethical Engine Torque in % of C_TQ_STND after charge intervention
    break;

    case CAN_BMW_DME2:
      uint8_t temp_TPS;
      uint8_t temp_BARO;
      uint16_t temp_CLT;
      temp_TPS = map(currentStatus.TPS, 0, 100, 0, 254);//TPS value conversion (from 0x00 to 0xFE)
      temp_CLT = (((currentStatus.coolant - CALIBRATION_TEMPERATURE_OFFSET) + 48)*4/3); //CLT conversion (actual value to add is 48.373, but close enough)
      if (temp_CLT > 255) { temp_CLT = 255; } //CLT conversion can yield to higher values than what fits to byte, so limit the maximum value to 255.
      temp_BARO = currentStatus.baro;

      outMsg.id = dashMessageID;
      outMsg.len = 7;
      outMsg.buf[0] = 0x11;  //Multiplexed Information
      outMsg.buf[1] = temp_CLT;
      outMsg.buf[2] = temp_BARO;
      outMsg.buf[3] = 0x08;  //bitfield, Bit0 = 0 = Clutch released, Bit 3 = 1 = engine running
      outMsg.buf[4] = 0x00;  //TPS_VIRT_CRU_CAN (Not used)
      outMsg.buf[5] = temp_TPS;
      outMsg.buf[6] = 0x00;  //bitfield, Bit0 = 0 = brake not actuated, Bit1 = 0 = brake switch system OK etc...
      outMsg.buf[7] = 0x00;  //not used, but set to zero just in case.
    break;

    case 0x545:       //fuel consumption and CEl light for BMW e46/e39/e38 instrument cluster
                      //fuel consumption calculation not implemented yet. But this still needs to be sent to get rid of the CEL and EML fault lights on the dash.
      outMsg.id = dashMessageID;
      outMsg.len = 5;
      outMsg.buf[0] = 0x00;  //Check engine light (binary 10), Cruise light (binary 1000), EML (binary 10000).
      outMsg.buf[1] = 0x00;  //LSB Fuel consumption
      outMsg.buf[2] = 0x00;  //MSB Fuel Consumption
      if (currentStatus.coolant > 159) { outMsg.buf[3] = 0x08; } //Turn on overheat light if coolant temp hits 120 degrees celsius.
      else { outMsg.buf[3] = 0x00; } //Overheat light off at normal engine temps.
      outMsg.buf[4] = 0x7E; //this is oil temp
    break;

    case 0x280:       //RPM for VW instrument cluster
      temp_RPM =  currentStatus.RPM * 4; //RPM conversion
      outMsg.id = dashMessageID;
      outMsg.len = 8;
      outMsg.buf[0] = 0x49;
      outMsg.buf[1] = 0x0E;
      outMsg.buf[2] = lowByte(uint16_t(temp_RPM));  //lsb RPM
      outMsg.buf[3] = highByte(uint16_t(temp_RPM)); //msb RPM
      outMsg.buf[4] = 0x0E;
      outMsg.buf[5] = 0x00;
      outMsg.buf[6] = 0x1B;
      outMsg.buf[7] = 0x0E;
    break;

    case 0x5A0:       //VSS for VW instrument cluster
      uint16_t tempA;
      tempA =  currentStatus.vss * 133; //VSS conversion
      outMsg.id = dashMessageID;
      outMsg.len = 8;
      outMsg.buf[0] = 0xFF;
      outMsg.buf[1] = lowByte(tempA);
      outMsg.buf[2] = highByte(tempA);
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

// switch case for gathering all data to Haltech broadcast protocol message based on CAN Id.
void haltechDashMessage(uint16_t dashMessageID)
{

  uint16_t tempA;    //used in calcs
  uint16_t tempB;    //used in calcs 
  uint16_t tempC;    //used in calcs 
  uint16_t tempD;    //used in calcs

  outMsg.id = dashMessageID;
  outMsg.len = 8;

  switch (dashMessageID)
  {
    case 0x360:
      tempA =  currentStatus.MAP * 10; //Haltech V2 protocol has MAP in 0.1kPa accurasy
      tempB =  currentStatus.TPS * 5; //Haltech V2 protocol has TPS in 0.1% accurasy

      outMsg.buf[0] = highByte(currentStatus.RPM);
      outMsg.buf[1] = lowByte(currentStatus.RPM);
      outMsg.buf[2] = highByte(tempA);
      outMsg.buf[3] = lowByte(tempA);
      outMsg.buf[4] = highByte(tempB);
      outMsg.buf[5] = lowByte(tempB);
      outMsg.buf[6] = 0x00;  //highByte for Coolant pressure
      outMsg.buf[7] = 0x00;  //lowByte for Coolant pressure
    break;

    case 0x361:
      tempA = currentStatus.fuelPressure * 69; //Haltech V2 protocol has fuel pressure in 0.1kPa accurasy, so we need to convert PSI to kPa. Actual number is 68.9475729.. but no floats allowed
      tempB = currentStatus.oilPressure * 69; //Haltech V2 protocol has oil pressure in 0.1kPa accurasy, so we need to convert PSI to kPa. Actual number is 68.9475729.. but no floats allowed
      tempC =  currentStatus.TPS * 5; //Haltech V2 protocol has TPS in 0.1% accurasy
      outMsg.buf[0] = highByte(tempA);
      outMsg.buf[1] = lowByte(tempA);
      outMsg.buf[2] = highByte(tempB);
      outMsg.buf[3] = lowByte(tempB);
      outMsg.buf[4] = highByte(tempC); //Acceleration Pedal Pos in protocol documentation
      outMsg.buf[5] = lowByte(tempC);
      outMsg.buf[6] = 0x00;  //highByte - Wastegate Pressure
      outMsg.buf[7] = 0x00;  //lowByte - Wastegate Pressure
    break;

    case 0x362:
      tempA = ( halfPercentage(currentStatus.PW1, revolutionTime) ) * 5; //Haltech V2 protocol has injector duty in 0.1% accurasy
      tempB = ( halfPercentage(currentStatus.PW3, revolutionTime) ) * 5; //PW3 is for secondary injectors in staged mode
      
      tempC = currentStatus.advance * 10; //Haltech V2 protocol has ignition angle in 0.1 degree accurasy
      if (configPage4.sparkMode == IGN_MODE_ROTARY)
      {
        uint8_t splitDegrees = 0;
        splitDegrees = table2D_getValue(&rotarySplitTable, currentStatus.ignLoad);
        tempD = tempC - splitDegrees;
      }
      else
      {
        tempD = tempC;
      }

      outMsg.buf[0] = highByte(tempA); //Injector Duty Cycle (Primary)
      outMsg.buf[1] = lowByte(tempA);
      outMsg.buf[2] = highByte(tempB); //Injector Duty Cycle (Secondary)
      outMsg.buf[3] = lowByte(tempB);
      outMsg.buf[4] = highByte(tempC);  //Ignition Angle (Leading)
      outMsg.buf[5] = lowByte(tempC);
      outMsg.buf[6] = highByte(tempD);  //Ignition Angle (Trailing)
      outMsg.buf[7] = lowByte(tempD);
    break;

    case 0x363:
      tempA =  currentStatus.EMAP * 10; //Haltech V2 protocol has MAP in 0.1kPa accurasy
      outMsg.buf[0] = 0x00;  //highByte - Wheel Slip
      outMsg.buf[1] = 0x00;  //lowByte - Wheel Slip
      outMsg.buf[2] = 0x00;  //highByte - Wheel Diff
      outMsg.buf[3] = 0x00;  //lowByte - Wheel Diff
      outMsg.buf[4] = highByte(currentStatus.rpmDOT); //Engine Acceleration (RPM/s)
      outMsg.buf[5] = lowByte(currentStatus.rpmDOT);
      outMsg.buf[6] = highByte(tempA); //Manifold Pressure 2
      outMsg.buf[7] = lowByte(tempA);
    break;

    case 0x368:
      tempA =  ( currentStatus.O2 / configPage2.stoich ) * 10; //Haltech V2 protocol has lambda in 0.001 accurasy
      tempB =  ( currentStatus.O2_2 / configPage2.stoich ) * 10;
      outMsg.buf[0] = highByte(tempA);  //Lambda1
      outMsg.buf[1] = lowByte(tempA);
      outMsg.buf[2] = highByte(tempB);  //Lambda2
      outMsg.buf[3] = lowByte(tempB);
      outMsg.buf[4] = 0x00;  //highByte - Lambda3
      outMsg.buf[5] = 0x00;  //lowByte - Lambda3
      outMsg.buf[6] = 0x00;  //highByte - Lambda4
      outMsg.buf[7] = 0x00;  //lowByte - Lambda4
    break;
 
    case 0x369:
      tempA = uint16_t(currentStatus.startRevolutions / 2); //This isn't strictly the same thing as Haltech sends out.
      outMsg.buf[0] = 0x00;  //highByte - Miss Count
      outMsg.buf[1] = currentStatus.syncLossCounter;  //lowByte - Miss Count
      outMsg.buf[2] = 0x00;  //highByte - Trigger Counter
      outMsg.buf[3] = 0x00;  //lowByte - Trigger Counter
      outMsg.buf[4] = highByte(tempA);  //Home Counter
      outMsg.buf[5] = lowByte(tempA);
      outMsg.buf[6] = 0x00;  //highByte - Triggers Since Last Home
      outMsg.buf[7] = 0x00;  //highByte - Triggers Since Last Home
    break;

    /*
    case 0x36A:
      tempA = currentStatus.knockRetard * 10; //Haltech V2 protocol has knock retard in 0.1 degree accurasy
      outMsg.buf[0] = 0x00;  //highByte for knock level
      outMsg.buf[1] = 0x00;  //lowByte for knock level
      outMsg.buf[2] = 0x00;  //highByte for knock level 2
      outMsg.buf[3] = 0x00;  //lowByte for knock level 2
      outMsg.buf[4] = highByte(tempA); //knock retard - bank 1
      outMsg.buf[5] = lowByte(tempA);
      outMsg.buf[6] = highByte(tempA); //knock retard - bank 2
      outMsg.buf[7] = lowByte(tempA);
    break;

    case 0x36B:
      outMsg.buf[0] = 0x00;  //highByte - Brake Pressure
      outMsg.buf[1] = 0x00;  //lowByte - Brake Pressure
      outMsg.buf[2] = 0x00;  //highByte - NOS Pressure
      outMsg.buf[3] = 0x00;  //lowByte - NOS Pressure
      outMsg.buf[4] = 0x00;  //highByte - Turbo Speed Sensor
      outMsg.buf[5] = 0x00;  //lowByte - Turbo Speed Sensor
      outMsg.buf[6] = 0x00;  //highByte - G-Sensor
      outMsg.buf[7] = 0x00;  //lowByte - G-Sensor
    break;

    case 0x36C:
      outMsg.buf[0] = 0x00;  //highByte - Wheelspeed Front Left
      outMsg.buf[1] = 0x00;  //lowByte - Wheelspeed Front Left
      outMsg.buf[2] = 0x00;  //highByte - Wheelspeed Front Right
      outMsg.buf[3] = 0x00;  //lowByte - Wheelspeed Front Right
      outMsg.buf[4] = 0x00;  //highByte - Wheelspeed Rear Left
      outMsg.buf[5] = 0x00;  //lowByte - Wheelspeed Rear Left
      outMsg.buf[6] = 0x00;  //highByte - Wheelspeed Rear Right
      outMsg.buf[7] = 0x00;  //lowByte - Wheelspeed Rear Right
    break;
    */

    case 0x36D:
      tempA = currentStatus.vss * 10; //Haltech V2 protocol has VSS in 0.1km/h accurasy
      tempB = currentStatus.vvt2Angle * 5; //Haltech V2 protocol has cam angle in 0.1 degree accurasy
      outMsg.buf[0] = highByte(tempA);  //Wheelspeed Front
      outMsg.buf[1] = lowByte(tempA);
      outMsg.buf[2] = highByte(tempA);  //Wheelspeed Rear
      outMsg.buf[3] = lowByte(tempA);
      outMsg.buf[4] = highByte(tempB);  //Exhaust Cam Angle 1
      outMsg.buf[5] = lowByte(tempB);
      outMsg.buf[6] = 0x00;  //highByte - Exhaust Cam Angle 2
      outMsg.buf[7] = 0x00;  //lowByte - Exhaust Cam Angle 2
    break;

    case 0x36E:
      if ( ( (configPage2.hardCutType == HARD_CUT_FULL) || (configPage6.engineProtectType == PROTECT_CUT_FUEL) ) && BIT_CHECK(currentStatus.spark, BIT_SPARK_HRDLIM) )
      {
        tempA = 0;
      }
      else { tempA = 1000; }

      if ( currentStatus.launchingSoft == true )
      {
        tempB = ( currentStatus.advance1 - configPage6.lnchRetard ) * 10; //This calculates amount of ign. retard when lauching.
      }
      else { tempB = 0; }

      tempC = currentStatus.launchCorrection * 10;
      outMsg.buf[0] = highByte(tempA);  //Fuel Cut Percentage
      outMsg.buf[1] = lowByte(tempA);
      outMsg.buf[2] = highByte(tempB);  //Launch Control Ign Retard
      outMsg.buf[3] = lowByte(tempB);
      outMsg.buf[4] = highByte(tempC);  //Launch Control Fuel Enrich
      outMsg.buf[5] = lowByte(tempC);
      outMsg.buf[6] = 0x00;  //Reserved
      outMsg.buf[7] = 0x00;  //Reserved
    break;

    case 0x36F:
      tempA = currentStatus.boostDuty / 10;
      outMsg.buf[0] = 0x00;  //Reserved
      outMsg.buf[1] = 0x00;  //Reserved
      outMsg.buf[2] = highByte(tempA);  //Boost control output
      outMsg.buf[3] = lowByte(tempA);
      outMsg.buf[4] = 0x00;  //highByte - Timed Duty Output Duty 1
      outMsg.buf[5] = 0x00;  //lowByte - Timed Duty Output Duty 1
      outMsg.buf[6] = 0x00;  //highByte - Timed Duty Output Duty 2
      outMsg.buf[7] = 0x00;  //lowByte - Timed Duty Output Duty 2
    break;

    case 0x370:
      tempA = currentStatus.vss * 10;  //Haltech V2 protocol has VSS in 0.1km/h accurasy
      tempB = currentStatus.vvt1Angle * 5; //Haltech V2 protocol has cam angle in 0.1 degree accurasy
      outMsg.buf[0] = highByte(tempA);  //Wheelspeed General 0.2
      outMsg.buf[1] = lowByte(tempA);
      outMsg.buf[2] = 0x00;  //highByte - Gear
      outMsg.buf[3] = currentStatus.gear;
      outMsg.buf[4] = highByte(tempB);  //Intake Cam Angle 1
      outMsg.buf[5] = lowByte(tempB);
      outMsg.buf[6] = 0x00;  //highByte - Intake Cam Angle 2
      outMsg.buf[7] = 0x00;  //lowByte - Intake Cam Angle 2
    break;

    /*
    case 0x371:
      outMsg.buf[0] = 0x00;  //highByte - Fuel Flow
      outMsg.buf[1] = 0x00;  //lowByte - Fuel Flow
      outMsg.buf[2] = 0x00;  //highByte - Fuel Flow Return
      outMsg.buf[3] = 0x00;  //lowByte - Fuel Flow Return
      outMsg.buf[4] = 0x00;  //highByte - Fuel Flow Differential
      outMsg.buf[5] = 0x00;  //lowByte - Fuel Flow Differential
      outMsg.buf[6] = 0x00;  //Reserved
      outMsg.buf[7] = 0x00;  //Reserved
    break;
    */

    case 0x372:
      tempA = currentStatus.boostTarget * 10;  //Haltech V2 protocol has boost target in 0.1 kPa accurasy
      tempB = currentStatus.baro * 10;  //Haltech V2 protocol has baro in 0.1 kPa accurasy
      outMsg.buf[0] = 0x00;  //highByte - Battery Voltage
      outMsg.buf[1] = lowByte(currentStatus.battery10);
      outMsg.buf[2] = 0x00;  //highByte - Air Temp Sensor 2
      outMsg.buf[3] = 0x00;  //lowByte - Air Temp Sensor 2
      outMsg.buf[4] = highByte(tempA);  //Target Boost Level
      outMsg.buf[5] = lowByte(tempA);
      outMsg.buf[6] = highByte(tempB);  //Barometric Pressure
      outMsg.buf[7] = lowByte(tempB);
    break;

    /*
    case 0x373:
      outMsg.buf[0] = 0x00;  //highByte - EGT1
      outMsg.buf[1] = 0x00;  //lowByte - EGT1
      outMsg.buf[2] = 0x00;  //highByte - EGT2
      outMsg.buf[3] = 0x00;  //lowByte - EGT2
      outMsg.buf[4] = 0x00;  //highByte - EGT3
      outMsg.buf[5] = 0x00;  //lowByte - EGT3
      outMsg.buf[6] = 0x00;  //highByte - EGT4
      outMsg.buf[7] = 0x00;  //lowByte - EGT4
    break;

    case 0x374:
      outMsg.buf[0] = 0x00;  //highByte - EGT5
      outMsg.buf[1] = 0x00;  //lowByte - EGT5
      outMsg.buf[2] = 0x00;  //highByte - EGT6
      outMsg.buf[3] = 0x00;  //lowByte - EGT6
      outMsg.buf[4] = 0x00;  //highByte - EGT7
      outMsg.buf[5] = 0x00;  //lowByte - EGT7
      outMsg.buf[6] = 0x00;  //highByte - EGT8
      outMsg.buf[7] = 0x00;  //lowByte - EGT8
    break;

    case 0x375:
      outMsg.buf[0] = 0x00;  //highByte - EGT9
      outMsg.buf[1] = 0x00;  //lowByte - EGT9
      outMsg.buf[2] = 0x00;  //highByte - EGT10
      outMsg.buf[3] = 0x00;  //lowByte - EGT10
      outMsg.buf[4] = 0x00;  //highByte - EGT11
      outMsg.buf[5] = 0x00;  //lowByte - EGT11
      outMsg.buf[6] = 0x00;  //highByte - EGT12
      outMsg.buf[7] = 0x00;  //lowByte - EGT12
    break;
    */

    case 0x3E0:
      tempA = currentStatus.coolant * 2331; //Haltech V2 protocol has temps in kelvin and in 0.1 degree accurasy
      tempB = currentStatus.IAT * 2331;
      tempC = currentStatus.fuelTemp * 2331;
      //tempD = currentStatus.oilTemp * 2331;
      outMsg.buf[0] = highByte(tempA);  //Coolant temp
      outMsg.buf[1] = lowByte(tempA);
      outMsg.buf[2] = highByte(tempB);  //Air Temp
      outMsg.buf[3] = lowByte(tempB);
      outMsg.buf[4] = highByte(tempC);  //Fuel Temp
      outMsg.buf[5] = lowByte(tempC);
      outMsg.buf[6] = 0x00;  //highByte - Oil Temp
      outMsg.buf[7] = 0x00;  //lowByte - Oil Temp
    break;

    /*
    case 0x3E1:
      outMsg.buf[0] = 0x00;  //highByte - Transmission Oil Temp
      outMsg.buf[1] = 0x00;  //lowByte - Transmission Oil Temp
      outMsg.buf[2] = 0x00;  //highByte - Diff Oil Temp
      outMsg.buf[3] = 0x00;  //lowByte - Diff Oil Temp
      outMsg.buf[4] = 0x00;  //highByte - Fuel Composition
      outMsg.buf[5] = 0x00;  //lowByte - Fuel Composition
      outMsg.buf[6] = 0x00;  //Reserved
      outMsg.buf[7] = 0x00;  //Reserved
    break;

    case 0x3E2:
      outMsg.buf[0] = 0x00;  //Reserved (Fuel Level)
      outMsg.buf[1] = 0x00;  //Reserved (Fuel Level)
      outMsg.buf[2] = 0x00;  //Fuel Consumption Rate
      outMsg.buf[3] = 0x00;  //Fuel Consumption Rate
      outMsg.buf[4] = 0x00;  //Average Fuel Economy
      outMsg.buf[5] = 0x00;  //Average Fuel Economy
      outMsg.buf[6] = 0x00;  //Reserved (Distance to Empty)
      outMsg.buf[7] = 0x00;  //Reserved (Distance to Empty)
    break;
    */

    case 0x3E3:
      tempA = currentStatus.egoCorrection * 10; //Haltech V2 protocol has ego correction in 0.1% accurasy
      outMsg.buf[0] = highByte(tempA);  //Fuel Trim Short Term Bank 1
      outMsg.buf[1] = lowByte(tempA);
      outMsg.buf[2] = 0x00;  //highByte - Fuel Trim Short Term Bank 2
      outMsg.buf[3] = 0x00;  //lowByte - Fuel Trim Short Term Bank 2
      outMsg.buf[4] = 0x00;  //highByte - Fuel Trim Long Term Bank 1
      outMsg.buf[5] = 0x00;  //lowByte - Fuel Trim Long Term Bank 1
      outMsg.buf[6] = 0x00;  //highByte - Fuel Trim Long Term Bank 2
      outMsg.buf[7] = 0x00;  //lowByte - Fuel Trim Long Term Bank 2
    break;

    case 0x3E4:
      outMsg.buf[0] = 0x00;  //Reserved
      outMsg.buf[1] = 0x00;  //bitfield
      outMsg.buf[2] = 0x00;  //bitfield
      outMsg.buf[3] = 0x00;  //Reserved
      outMsg.buf[4] = 0x00;  //Reserved
      outMsg.buf[5] = 0x00;  //Reserved
      outMsg.buf[6] = 0x00;  //Reserved
      outMsg.buf[7] = 0x00;  //bitfield, Bit0 = MIL (check engine light, Bit1 = Battery Light, Bit2 = Limp Mode, Bit3 = Left Indicator, Bit4 = Right Indicator, Bit5 = High Beam, Bit6 = Handbrake, Bit7 = Reserved

    default:
    break;
  }
}
#endif