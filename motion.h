/**************************************************
*motion.h
*
* Motion layer that controls the movement of the servos
* for us to modify the robot's pose
****************************************************/
#ifndef __MOTION_H_
#define __MOTION_H_

#include "servoController.h"
#include "servoState.h"

#define NUM_HIPS    4
#define HIP_SPACING 2
#define NUM_ANKLES  4
#define LEG_SPACING 2
typedef enum servo_type_e 
{
    ANKLE_0        = 0,
    HIP_0,      /* = 1 */  
    ANKLE_1,    /* = 2 */
    HIP_1,      /* = 3 */
    ANKLE_2,    /* = 4 */
    HIP_2,      /* = 5 */
    ANKLE_3,    /* = 6 */
    HIP_3,      /* = 7 */
    NUM_JOINTS  /* = 8 */
}servo_type_t;



class MotionDriver {
    public:
        MotionDriver(ServoController *ctrl,
                     uint8_t numServos);
        ~MotionDriver();

        void setPose();
        void setHips(float angle);
        void updatePose();

    private:
        ServoController *_ctrlCtx;
        ServoState  *_servo[NUM_JOINTS];
        uint8_t _numServos;

};



#endif /* __MOTION_H_ */
