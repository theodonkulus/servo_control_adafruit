/**************************************************
*motionModel.h
*
* Motion model that descrobes the movement of the servos
* for us to modify the robot's pose
****************************************************/
#ifndef __MOTIONMODEL_H_
#define __MOTIONMODEL_H_

#include "geometry.h"

class MotionModel {
    public:
        enum chainType_e {
            CHAIN_BODY = 0,
            CHAIN_ARM,
            CHAIN_LEG,
            NUM_CHAIN_TYPES
        };
        MotionModel(enum chainType_e, uint8_t numJoints);
        ~MotionModel();

        uint8_t getChainLength();

    private:
        uint8_t _chainLength;
        Transformation *_transform;
        Point *_endEffector;

}


#endif /* __MOTIONMODEL_H_ */
