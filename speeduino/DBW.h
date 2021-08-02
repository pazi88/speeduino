/*
DBW add on for speeduino by Pazi88
this only works on STM32
*/

#include BOARD_H //Note that this is not a real file, it is defined in globals.h. 

#define DBWdir_PIN_LOW()    *DBWdir_pin_port &= ~(DBWdir_pin_mask)
#define DBWdir_PIN_HIGH()   *DBWdir_pin_port |= (DBWdir_pin_mask)
#define DBWdir2_PIN_LOW()    *DBWdir2_pin_port &= ~(DBWdir2_pin_mask)
#define DBWdir2_PIN_HIGH()   *DBWdir2_pin_port |= (DBWdir2_pin_mask)

volatile PORT_TYPE *DBWdir_pin_port;
volatile PINMASK_TYPE DBWdir_pin_mask;
volatile PORT_TYPE *DBWdir2_pin_port;
volatile PINMASK_TYPE DBWdir2_pin_mask;

long DBWError;
long DBWDutyModifier;
long DBWErrorTarget; //this is abviously 0
int CalTimer;
void initialiseDBW();
void DBWControl();