/*
DBW add on for speeduino by Pazi88
this only works on STM32
*/
#include "globals.h"
#include "maths.h"
#include "sensors.h"
#include "src/PID_v1/PID_v1.h"

integerPID DBWPID(&DBWError, &DBWDutyModifier, &DBWErrorTarget, configPage15.DBWKP, configPage15.DBWKI, configPage15.DBWKD, DIRECT);

void initialiseDBW()
{
  DBWdir_pin_port = portOutputRegister(digitalPinToPort(pinDBWdir));
  DBWdir_pin_mask = digitalPinToBitMask(pinDBWdir);
  DBWdir2_pin_port = portOutputRegister(digitalPinToPort(pinDBWdir2));
  DBWdir2_pin_mask = digitalPinToBitMask(pinDBWdir2);
  DBWErrorTarget = 0;

  //DBW will use hardware PWM output on STM32 to prevent the high frequency PWM output taking too much CPU time
  Timer10.setOverflow(20000, HERTZ_FORMAT); //set the output to 20KHz to prevent the motor control being audible
  Timer10.setCaptureCompare(1, 0, RESOLUTION_12B_COMPARE_FORMAT); // Dutycycle: [0.. 4095]
  Timer10.resume();
  DBWPID.SetOutputLimits(0, 8190); //PID works with duty values from 0 to 8190. 0-4095 is backwards direction, 4096 - 8190 is forward direction
  DBWPID.SetTunings(configPage15.DBWKP, configPage15.DBWKI, configPage15.DBWKD);
  DBWPID.SetSampleTime(33); //30Hz is 33,33ms
  DBWPID.SetMode(AUTOMATIC); //Turn PID on
  int CalTimer = 0;
  BIT_CLEAR(currentStatus.DBWstatus, BIT_DBWSTATUS_CAL_ONGOING); //disable calibration

}
void DBWControl()
{
  if ( BIT_CHECK(currentStatus.DBWstatus, BIT_DBWSTATUS_CAL_ONGOING) == false ) //normal operation
  {
    readTPS();
    readTPS2();
    readPedal1();
    readPedal2();
    currentStatus.DBWTarget = get3DTableValue(&DBWtargetTable, (currentStatus.Pedal/4) , currentStatus.RPM);
    currentStatus.DBWDuty = get3DTableValue(&DBWdutyTable, (currentStatus.Pedal/4) , currentStatus.DBWTarget);
    if ( currentStatus.DBWduty >= 0 )  // Open the DBW
    {
      if (configPage15.DBWdir == 0)
      {
        //Normal direction
        *DBWdir_pin_port |= (DBWdir_pin_mask);  // Switch 1st direction pin to high
        *DBWdir2_pin_port &= ~(DBWdir2_pin_mask);  // Switch 2nd direction pin to low
      }
      else
      {
        //Reversed direction
        *DBWdir_pin_port &= ~(DBWdir_pin_mask);  // Switch 1st direction pin to high
        *DBWdir2_pin_port |= (DBWdir2_pin_mask);  // Switch 2nd direction pin to low
      }
    }
    else
    {
      if (configPage15.DBWdir == 0)
      {
        //Normal direction
        *DBWdir_pin_port &= ~(DBWdir_pin_mask);  // Switch 1st direction pin to high
        *DBWdir2_pin_port |= (DBWdir2_pin_mask);  // Switch 2nd direction pin to low
      }
      else
      {
        //Reversed direction
        *DBWdir_pin_port |= (DBWdir_pin_mask);  // Switch 1st direction pin to high
        *DBWdir2_pin_port &= ~(DBWdir2_pin_mask);  // Switch 2nd direction pin to low
      }
    }
    Timer10.setCaptureCompare(1, abs(currentStatus.DBWduty), RESOLUTION_12B_COMPARE_FORMAT); // Dutycycle for the DBW PWM output: [0.. 4095]

    bool PID_compute = DBWPID.Compute(false);
    if(PID_compute == true)
    {
    }
  }
  else //calibration flag is set
  {
    if ( CalTimer < 10 )
    {
      Timer10.setCaptureCompare(1, 0, RESOLUTION_12B_COMPARE_FORMAT); // Dutycycle: 0 (min value = fully closed)
    }
    else if ( CalTimer == 10 )
    {
        // Read ADC values when throttle flap is fully closed and store those as min values
        configPage2.tpsMin = analogRead(pinTPS);
        configPage15.tps2Min = analogRead(pinTPS2);
    }
    else if ( CalTimer > 10 )
    {
      Timer10.setCaptureCompare(1, 0, RESOLUTION_12B_COMPARE_FORMAT); // Dutycycle: 4096 (max value = fully open)
    }
    else if ( CalTimer == 20 )
    {
        // Read ADC values when throttle flap is fully open and store those as max values
        configPage15.tpsMax = analogRead(pinTPS);
        configPage15.tps2Max = analogRead(pinTPS2);
        BIT_CLEAR(currentStatus.DBWstatus, BIT_DBWSTATUS_CAL_ONGOING); //calibration done
        writeConfig(2); // Need to manually save the new config value as it will not trigger a burn in tunerStudio due to use of ControllerPriority
        writeConfig(15);
        CalTimer = 0;
    }
    CalTimer++;
  }
}