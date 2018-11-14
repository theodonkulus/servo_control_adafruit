`/*********************************************************************** 
* servoController.cpp
* Depending on your servo make, the pulse width min and max may vary, you
* want these to be as small/large as possible without hitting the hard stop
* for max range. You'll have to tweak them as necessary to match the servos you have.
* 
*  MG90 servos
*  min -90  = 1.0 ms
*  neut 0   = 1.5 ms
*  max  90  = 2.0 ms
*  deadtime = 5us 
*  50 Hz
****************************************************************************/

#include "Wire.h"
#include "Adafruit_PWMServoDriver.h"

servoController::servoController(enum driverType_e type, 
                                 void *args, 
                                 uint8_t lenArgs)
{
    
}

servoController::~servoController()
{

}


