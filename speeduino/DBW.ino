/*
DBW add on for speeduino by Pazi88
this only works on STM32
*/
#include "globals.h"
#include "maths.h"
#include "src/PID_v1/PID_v1.h"

integerPID DBWPID(&currentStatus.TPS, &currentStatus.DBWduty, &currentStatus.Pedal_1, configPage2.DBWKP, configPage2.DBWKI, configPage2.DBWKD, DIRECT);

void initialiseDBW()
{
  //DBW will use hardware PWM output on STM32 to prevent the high frequency PWM output taking too much CPU time
  Timer10.setOverflow(20000, HERTZ_FORMAT); //set the output to 20KHz to prevent the motor control being audible
  Timer10.setCaptureCompare(1, 0, RESOLUTION_12B_COMPARE_FORMAT); // Dutycycle: [0.. 4095]
  Timer10.resume();
  DBWPID.SetOutputLimits(0, 4095);
  DBWPID.SetTunings(configPage2.DBWKP, configPage2.DBWKI, configPage2.DBWKD);
  DBWPID.SetSampleTime(66); //15Hz is 66,66ms
  DBWPID.SetMode(AUTOMATIC); //Turn PID on
  int CalTimer = 0;
  configPage2.DoDBWCal = false; //disable calibration

}
void DBWControl()
{
  if configPage2.DoDBWCal = false; //normal operation
  {
    currentStatus.TPS2 = analogRead(pinTPS2);
    currentStatus.Pedal_1 = analogRead(pinPedal);
    currentStatus.Pedal_2 = analogRead(pinPedal2);
    //just to test the PWM output. Duty = TPS value
    //currentStatus.DBWduty = currentStatus.tps * 4;
    bool PID_compute = DBWPID.Compute(false);
    if(PID_compute == true)
    {
      Timer10.setCaptureCompare(1, currentStatus.DBWduty, RESOLUTION_12B_COMPARE_FORMAT); // Dutycycle: [0.. 4095]
    }
  }
  else //calibration flag is set
  {
    if CalTimer < 10;
    {
      Timer10.setCaptureCompare(1, 0, RESOLUTION_12B_COMPARE_FORMAT); // Dutycycle: 0 (min value = fully closed)
    }
    else if CalTimer == 10;
    {
        // Read ADC values when throttle flap is fully closed and store those as min values
        configPage2.TPS1_min = analogRead(pinTPS);
        configPage2.TPS2_min = analogRead(pinTPS2);
    }
    else if CalTimer > 10;
    {
      Timer10.setCaptureCompare(1, 0, RESOLUTION_12B_COMPARE_FORMAT); // Dutycycle: 4096 (max value = fully open)
    }
    else if CalTimer == 20;
    {
        // Read ADC values when throttle flap is fully open and store those as max values
        configPage2.TPS1_max = analogRead(pinTPS);
        configPage2.TPS2_max = analogRead(pinTPS2);
        configPage2.DoDBWCal = false; //calibration done
        CalTimer = 0;
    }
    CalTimer++;
  }
}