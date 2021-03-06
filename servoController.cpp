/*********************************************************************** 
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
#include "servoController.h"
#include "Adafruit_PWMServoDriver.h"

/**************************************************************************
* Constructor

  @param driverArgs_t args - argument for the driver used for the servos
***************************************************************************/
ServoController::ServoController(driverArgs_t *args) 
{
    bool init_driver = false;

   _servoMax = SERVOMAX;
   _servoMin = SERVOMIN;
   _servoScale = SERVOSCALE;
   _servoRange = SERVO_RANGE;

   _tickScale = ((_servoMax - _servoMin)* _servoScale) / _servoRange;

    if(args);
    {
        switch(args->driver)
        {   
            case ADAFRUIT_SERVO_DRIVER:
                _type = SERVO_CTRL_TYPE_PWM;
                _interface = CTRL_IF_I2C;
                _pwmDriver = new Adafruit_PWMServoDriver(args->addr);

                _pwmDriver->begin();
                _pwmDriver->setPWMFreq(args->freq);
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
    _curDriver = args->driver;
}

/***************************************************************************
  Destructor 
  Release the driver
*/
ServoController::~ServoController()
{
    if (_pwmDriver)
    {
        delete(_pwmDriver);
    } 
}

/**************************************************************************
 *  anglesToTicks
 * 
 * 
 * 
 **************************************************************************/
float ServoController::anglesToTicks(float angle)
{

   return (angle * _tickScale) / _servoScale;
  
}
 
/**************************************************************************
* setAnglePos - This is broken currently. I think its the MACROS

  @param uint8_t ch   - The channel of the servo of the angle you want to set
  @param uint16_t angle - the angle to set the servo to

  Converts the angle to the tick value needed to set the PWM in ticks
**************************************************************************/
uint16_t ServoController::setAnglePos(uint8_t ch, uint16_t angle)
{
    uint16_t tick = 0;

    tick = (  (ANGLETOTICKS((angle & 0xFF00) >> 8))  
            + (ANGLETOTICKS(angle & 0x00FF) / 10) 
            +  SERVOMIN);
                
    _pwmDriver->setPWM(ch, 0, tick);
    return tick;
}

/**************************************************************************
* setAnglePos

  @param uint8_t ch   - The channel of the servo of the angle you want to set
  @param float angle - the angle to set the servo too

  Converts the angle to the tick value needed to set the PWM in ticks
**************************************************************************/
uint16_t ServoController::setAnglePos(uint8_t ch, float angle)
{
    uint16_t tick = 0;
 
    tick = SERVOMIN + (uint16_t)anglesToTicks(angle);
                
    _pwmDriver->setPWM(ch, 0, tick);
    return tick;
}



 
