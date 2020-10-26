/*
DBW add on for speeduino by Pazi88
this only works on STM32
*/
#include "globals.h"
#include "maths.h"
#include "sensors.h"
#include "src/PID_v1/PID_v1.h"

integerPID DBWPID(&currentStatus.TPS, &currentStatus.DBWduty, &currentStatus.Pedal_1, configPage9.DBWKP, configPage9.DBWKI, configPage9.DBWKD, DIRECT);

void initialiseDBW()
{
  //DBW will use hardware PWM output on STM32 to prevent the high frequency PWM output taking too much CPU time
  Timer10.setOverflow(20000, HERTZ_FORMAT); //set the output to 20KHz to prevent the motor control being audible
  Timer10.setCaptureCompare(1, 0, RESOLUTION_12B_COMPARE_FORMAT); // Dutycycle: [0.. 4095]
  Timer10.resume();
  DBWPID.SetOutputLimits(0, 4095);
  DBWPID.SetTunings(configPage9.DBWKP, configPage9.DBWKI, configPage9.DBWKD);
  DBWPID.SetSampleTime(66); //15Hz is 66,66ms
  DBWPID.SetMode(AUTOMATIC); //Turn PID on
  int CalTimer = 0;
  BIT_CLEAR(currentStatus.DBWstatus, BIT_DBWSTATUS_CAL_ONGOING); //disable calibration

}
void DBWControl()
{
  if ( BIT_CHECK(currentStatus.DBWstatus, BIT_DBWSTATUS_CAL_ONGOING) == false ) //normal operation
  {
    readTPS(false);
    readTPS2();
    readPedal1();
    readPedal2();
    bool PID_compute = DBWPID.Compute(false);
    if(PID_compute == true)
    {
      Timer10.setCaptureCompare(1, currentStatus.DBWduty, RESOLUTION_12B_COMPARE_FORMAT); // Dutycycle: [0.. 4095]
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
        configPage9.tps2Min = analogRead(pinTPS2);
    }
    else if ( CalTimer > 10 )
    {
      Timer10.setCaptureCompare(1, 0, RESOLUTION_12B_COMPARE_FORMAT); // Dutycycle: 4096 (max value = fully open)
    }
    else if ( CalTimer == 20 )
    {
        // Read ADC values when throttle flap is fully open and store those as max values
        configPage2.tpsMax = analogRead(pinTPS);
        configPage9.tps2Max = analogRead(pinTPS2);
        BIT_CLEAR(currentStatus.DBWstatus, BIT_DBWSTATUS_CAL_ONGOING); //calibration done
        writeConfig(2); // Need to manually save the new config value as it will not trigger a burn in tunerStudio due to use of ControllerPriority
        writeConfig(9);
        CalTimer = 0;
    }
    CalTimer++;
  }
}