#include "motion.h"
#include "Arduino.h"

/*******************************************************
*  This updates the motion of the servos in the system

   Stores a state for each joint 
*
********************************************************/


MotionDriver::MotionDriver(ServoController * ctrl, 
                              uint8_t numServos,
                            bool debug)
{
    if (ctrl)
    {
        _ctrlCtx = ctrl;     
        _numServos = numServos;

        for (uint8_t i = 0; i < _numServos; i++)
        {
            _servo[i] = new ServoState(i, _ctrlCtx);
        }
    }

    _rotateFlag = 0x11;
    _debug = debug;
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

/**/
void MotionDriver::setPose(void)
{
        

}

float MotionDriver::rotateAnkle(uint8_t ch, float angle)
{
    float newAngle = angle;

    if (_rotateFlag & (1 << ch)) {
        newAngle = 180.00f - angle;
    }

    return newAngle;
}

/***************************************************************
* setTrim
*
*
*****************************************************************
void MotionDriver::setTrim(float trim)
{
        

} */

/****************************************************************
    moveAnkles

    @param float angle   - the target angle position in degrees

    Move all the ankles at the same time in the current
    frame
    
    @return: None
*****************************************************************/
void MotionDriver::moveAnkles(float angle)
{
    for (uint8_t i = ANKLE_0; i< NUM_JOINTS; i += LEG_SPACING) 
    {
       float corAngle = rotateAnkle(i, angle);
        _servo[i]->setAngleF(corAngle);
    }
}

/****************************************************************
    moveHips

    @param float angle   - the target angle position in degrees

    Move all the hipss at the same time in the current
    frame
    
    @return: None
*****************************************************************/
void MotionDriver::setHips(float angle)
{
   for(uint8_t i = HIP_0; i < NUM_JOINTS; i += HIP_SPACING)
   {
       _servo[i]->setAngleF(angle);  
   }
}

/****************************************************************
 updatePose()
  

  Updates the pose of the robot aand checks the flags of all the servos
  in the chain before an update

  @return: None
******************************************************************/
void MotionDriver::updatePose()
{
    if (_debug) {
        Serial.print("updatePose Start Servos:");
        Serial.println(_numServos);
    }

    for (uint8_t servoIdx = 0; servoIdx < _numServos; servoIdx++)
    {
        if (_debug) {
                Serial.print("Check S");
                Serial.println(servoIdx);
        }

        if (_servo[servoIdx]->getPosUpdate())
        {
            if (_debug) {
                    Serial.print("Update S");
                    Serial.println(servoIdx);
            }
            _servo[servoIdx]->moveServoF();           
        }
    }
}
