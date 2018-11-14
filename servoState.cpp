`/*********************************************************************** 
* servoState.cpp
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

#include "systemDefs.h"
#include "servoState.h"

/* servo channel stuff */
#define SERVOMIN  201 // this is the 'minimum' pulse length count (out of 4096)
#define SERVONEUT 305
#define SERVOMAX  410 // this is the 'maximum' pulse length count (out of 4096) 
#define SERVO_RANGE 180 //Range of the servo 0-180 (90 degrees neutral)
#define SERVOSCALE  313 
#define TICK_SCALE (((SERVOMAX - SERVOMIN) * SERVOSCALE) / SERVO_RANGE) 
#define ANGLETOTICKS(X)(TICK_SCALE * X) / SERVOSCALE

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

ServoState::ServoState(unsigned char pwmChan, 
                       servoController *pwmCtx)
{
    if (!pwmCtx)
    {
        delete this;
    }

    this.pwmCtx     = pwmCtx;
    this.pwmChannel = pwmChan;
    this.curAngle   = 90;  /* Start at zero degrees */
    this.prevAngle  = ANGLE_NOT_DEFINED;
    this.nextAngle  = ANGLE_NOT_DEFINED;
    this.curTick    = SERVO_NEUT;
}

ServoState::~ServoState()
{
}

uint16_t ServoState::getAngle(void)
{
    return curAngle;   
}

void ServoState::setAngle(uint16_t angleVal)
{
    nextAngle = angleVal;
    updateStatus |= 1 << UPDATE_SERVO_POS;
}

void ServoState::setAngle(uint16_t angleVa, uint16_t subAngle)
{
    nextAngle = (0x00FF & angleVal << 8) | (0x00FF & subAngle);
    updateStatus |= 1 << UPDATE_SERVO_POS;
}


uint8_t getUpdateStatus(void)
{
    return updateStatus;
}

int8_t ServoState::moveServo(void)
{
    int8_t err = ERROR;

    if(nextAngle != ANGLE_NOT_DEFINED)
    {
        prevAngle = curAngle;
        curAngle  = nextAngle;
        newTick =  ((ANGLETOTICKS((nextAngle & 0xFF00) >> 8) ) 
                +  ((ANGLETOTICKS(nextAngle & 0x00FF) / 10) + SERVOMIN;
                
        pwm->setPWM(pwmChannel, 0, newTick);
        curTick = newTick;
        updateStatus &= ~(1 << UPDATE_SERVO_POS);
        err = OK;
    }
   
    return err;
}


