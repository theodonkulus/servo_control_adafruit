/*********************************************************************** 
* servoState.cpp
*
*   Control and state aquisition from each servo in the system
*
************************************************************************/

#include "systemDefs.h"
#include "servoState.h"
#include "servoController.h"

//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

ServoState::ServoState(unsigned char chan, 
                       ServoController *hwCtx)
{
    if (!_ctrlCtx)
    {
        delete this;
    }

    _ctrlCtx    = hwCtx;
    _channel    = chan;
    _curAngle   = 90;  /* Start at zero degrees */
    _prevAngle  = ANGLE_NOT_DEFINED;
    _nextAngle  = ANGLE_NOT_DEFINED;
    _curTick    = SERVONEUT;
}

ServoState::~ServoState()
{
}

uint16_t ServoState::getAngle(void)
{
    return _curAngle;   
}

void ServoState::setAngle(uint16_t angleVal)
{
    _nextAngle = angleVal;
    _updateStatus |= 1 << UPDATE_SERVO_POS;
}

void ServoState::setAngle(uint16_t angleVal, uint16_t subAngle)
{
    _nextAngle = (0x00FF & angleVal << 8) | (0x00FF & subAngle);
    _updateStatus |= 1 << UPDATE_SERVO_POS;
}


uint8_t ServoState::getUpdateStatus(void)
{
    return _updateStatus;
}

uint8_t ServoState::getPosUpdate(void)
{
    return (_updateStatus & (1 << UPDATE_SERVO_POS));
}

int8_t ServoState::moveServo(void)
{
    int8_t err = ERROR;

    if(_nextAngle != ANGLE_NOT_DEFINED)
    {
        _prevAngle = _curAngle;
        _curAngle  = _nextAngle;

        _newTick = _ctrlCtx->setAnglePos(_channel, _nextAngle);
        _curTick = _newTick;
        _updateStatus &= ~(1 << UPDATE_SERVO_POS);
        err = OK;
    }
   
    return err;
}


