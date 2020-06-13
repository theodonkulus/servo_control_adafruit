/*************************************************************************
* inverse_kinematics.h
*
* Maps output foot positions of the droid to the appropriate servo angles for 
* the robot platform. 
*
*
*************************************************************************/
#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include "motion.h"

class IK_engine {
    #define NUM_LEGS 4

    typedef struct coord_s {
        double x;
        double y;
    } coord_t;

    public:
        IK_engine();
        ~IK_engine();
       
        //Absolute commands
        int setLegX(unsigned char legNum, double xCoord);
        int setLegY(unsigned char legNum, double yCoord); 
        int setLegCoord(unsigned char legNum, double xCoord, double yCoord);

        //Relative commands
        int setRelativeLegX(unsigned char legNum, double xCoord);      
        int setRelativeLegY(unsigned char legNum, double yCoord);      
        int setRelativeLegCoord(unsigned char legNum, double xCoord, double yCoord);
        double getRelativeLegX(unsigned char legNum);
        double getRelativeLegY(unsigned char legNum);
        coord_t getRelativeLegCoord(unsigned char legNum);

    private:
        int generateLegServoAngles(unsigned legNum);
        
    public:

    private:
        double _LegTransformationMatrix[4][4];
        
        //variables related to position
        coord_t _tgtFootPos[NUM_LEGS];

        coord_t _footCoord[NUM_LEGS];
        double _hipAngle[NUM_LEGS];
        double _kneeAngle[NUM_LEGS];
};

#endif /* INVERSE_KINEMATICS_H */
