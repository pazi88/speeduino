/*
DBW add on for speeduino by Pazi88
this only works on STM32
*/
#include "globals.h"
#include "maths.h"
#include "src/PID_v1/PID_v1.h"

void initialiseDBW()
{
  //DBW will use hardware PWM output on STM32 to prevent the high frequency PWM output taking too much CPU time
  Timer10.setOverflow(20000, HERTZ_FORMAT); //set the output to 20KHz to prevent the motor control being audible
  Timer10.setCaptureCompare(1, 0, RESOLUTION_12B_COMPARE_FORMAT); // Dutycycle: [0.. 4095]
  Timer10.resume();

}
void DBWControl()
{
  currentStatus.TPS2 = analogRead(pinTPS2);
  currentStatus.Pedal_1 = analogRead(pinPedal);
  currentStatus.Pedal_2 = analogRead(pinPedal2);
  //just to test the PWM output. Duty = TPS value
  currentStatus.DBWduty = currentStatus.tpsADC * 4;
  Timer10.setCaptureCompare(1, currentStatus.DBWduty, RESOLUTION_12B_COMPARE_FORMAT); // Dutycycle: [0.. 4095]
}