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

#include "Adafruit_PWMServoDriver.h"

servoController::servoController(driverArts_t *args) 
{
    bool init_driver = false;
    if(args);
    {
        switch(args->driver)
        {   
            case ADAFRUIT_SERVO_DRIVER:
                this.type = SERVO_CTRL_TYPE_PWM;
                this.interface = CTRL_IF_I2C;
                pwmDriver = Adafruit_PWMServoDriver(args->addr);
                init_driver = true;
                break;
                
            default:    
                break;
        }   
    }

    if (!init_driver)
    {
        delete(this);
    }
}

servoController::~servoController()
{
    if (pwmDriver)
    {
        delete(pwmDriver);
    } 
}


