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

#include "math.h"
#include "motion.h"

#define NUM_LEGS 4
#define NUM_LINKS 3

typedef struct coord_s {
    double x;
    double y;
    double z;
} coord_t;

typedef struct angle_s {
    double roll; /* About X */
    double pitch;/* About Y */
    double yaw;  /* About Z */
} angle_t;

class IK_engine 
{
    enum joint_e {
        COXA = 0,
        FEMUR,
        TIBIA,
        NUM_JOINTS
    };

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

        int setRelativeBodyPos(coord_t bodyPos);
        int setRelativeBodyTilt(angle_t bodyAngle);

    private:
        int bodyIK(void);
        int bodyFK(void);
        int LegIK(void);
        int LegIK2(uint8_t ch);
        int generateLegServoAngles(unsigned legNum);
        
    public:

    private:
    
        /* Geometric values that dont change */
        double  _linkLength[NUM_LINKS]; // in mm
        coord_t _legOffset[NUM_LEGS]; // in mm
        double  _legAngleOffset[NUM_LEGS]; // in mm
        coord_t _initFootPos[NUM_LEGS];
        double  _bodyLength;

        /* Body states */
        angle_t _bodyCurTilt;
        coord_t _bodyCurPos;
   
        /* Transform Matricies per leg relative to world space */
        float tRb[4][4];
        float tRf[4][4];
        float tLb[4][4];
        float tLf[4][4];

        coord_t _bodyIK;

        angle_t _bodyTgtTilt;
        coord_t _bodyTgtPos;
 
        /* Body state to use for foot angle calculations */
        coord_t _bodyToFootIK[NUM_LEGS];

        /* variables related to foot states */
        coord_t _footCoord[NUM_LEGS];
        coord_t _tgtFootPos[NUM_LEGS];

        /* Output of IK given foot position and ctrl input */
        double _hipAngle[NUM_LEGS];
        double _kneeAngle[NUM_LEGS];              
};

#endif /* INVERSE_KINEMATICS_H */
