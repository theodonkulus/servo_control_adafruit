#include "motion.h"

MotionDriver::MotionDriver(ServoController * ctrl, 
                              uint8_t numServos)
{
    if (ctrl)
    {
        _ctrlCtx = ctrl;     
        if (numServos < NUM_JOINTS)
        {
            _numServos = numServos;

            for (uint8_t i = 0; i < numServos; i++)
            {
                _servo[i] = new ServoState(i, _ctrlCtx);
            }
        }
    }
   
}

MotionDriver::~MotionDriver()
{
    for(uint8_t i = 0; i < _numServos; i++)
    {
        if(_servo[i])
        {
            delete(_servo[i]);
        }
    }
}

void MotionDriver::setPose(void)
{
    
}

void MotionDriver::updatePose()
{
    for (uint8_t servoIdx = 0; servoIdx < _numServos; servoIdx++)
    {
        if (_servo[servoIdx]->getPosUpdate())
        {
            _servo[servoIdx]->moveServo();           
        }
    }
}
