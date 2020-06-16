/*****************************************************
* Inverse kinematics class and stuff related to getting this robot to move. 
* Adatped from some sorces online for hexapods with a bit of massaging.
* eg) http://toglefritz.com/hexapod-inverse-kinematics-equations/
*   
* TODO: Make this not suck and cleanup his math/implementation. Some things seem to be
* able to be precalculated
*******************************************************/

#include "inverse_kinematics.h"
//#include "cmath.h"


/* Cosntructor 

    This should setup the droid and all the given parameters/relationships on startup.
*/
IK_engine::IK_engine()
{
   /* Setup hardcoded defaults for the robots geometry */
   _linkLength[COXA] = 28.00; 
   _linkLength[FEMUR] = 40.00; 
   _linkLength[TIBIA] = 55.00; 

   _bodyLength = 42.00; //Same because droid is a circle 

   /* Offset from center of the body */
   _legOffset[0].x =  42.00;
   _legOffset[0].y =  42.00;
   _legOffset[1].x =  42.00;
   _legOffset[1].x = -42.00;
   _legOffset[2].x = -42.00;
   _legOffset[2].y = -42.00;
   _legOffset[3].x = -42.00;
   _legOffset[3].y =  42.00;

    /* cos(angle /180 * pi) * link0 * link1 , 
      since legs are pi/4 offset we save calculating this  just resolves to scaling by 1/sqrt(2)*/
   _initFootPos[0].x =  (_linkLength[COXA] * _linkLength[FEMUR]) / sqrt(2);
   _initFootPos[1].x =  (_linkLength[COXA] * _linkLength[FEMUR]) / sqrt(2);
   _initFootPos[2].x = -(_linkLength[COXA] * _linkLength[FEMUR]) / sqrt(2);
   _initFootPos[3].x = -(_linkLength[COXA] * _linkLength[FEMUR]) / sqrt(2);

    /*sin(angle/180 * pi) * link0 * link1 */
   _initFootPos[0].y =  (_linkLength[COXA] * _linkLength[FEMUR]) / sqrt(2);
   _initFootPos[1].y = -(_linkLength[COXA] * _linkLength[FEMUR]) / sqrt(2);
   _initFootPos[2].y = -(_linkLength[COXA] * _linkLength[FEMUR]) / sqrt(2);
   _initFootPos[3].y =  (_linkLength[COXA] * _linkLength[FEMUR]) / sqrt(2);
   
    for(int i = 0; i < NUM_LEGS; i++)
    {
        _initFootPos[i].z = _linkLength[TIBIA];
    }

    _legAngleOffset[0] = PI * 1/4;
    _legAngleOffset[1] = PI * 3/4;
    _legAngleOffset[2] = PI * 5/4;
    _legAngleOffset[3] = PI * 7/4;
}

IK_engine::~IK_engine()
{
}

/* Setters for absolute or relative position of the feet */
int IK_engine::setLegX(unsigned char legNum, double xCoord) 
{
    _tgtFootPos[legNum].x = xCoord;
    return 0;
}

int IK_engine::setLegY(unsigned char legNum, double yCoord)
{
    _tgtFootPos[legNum].y = yCoord;
    return 0;
}

int IK_engine::setLegCoord(unsigned char legNum, double xCoord, double yCoord)
{
    _tgtFootPos[legNum].x = xCoord;
    _tgtFootPos[legNum].y = yCoord;
    return 0;
}

int IK_engine::setRelativeLegX(unsigned char legNum, double xCoord)
{
    
    _tgtFootPos[legNum].x += xCoord;
    return 0;
}

int IK_engine::setRelativeLegY(unsigned char legNum, double yCoord)
{
    _tgtFootPos[legNum].y += yCoord;
    return 0;
}

int IK_engine::setRelativeLegCoord(unsigned char legNum, double xCoord,double yCoord)
{
    _tgtFootPos[legNum].x += xCoord;
    _tgtFootPos[legNum].y += yCoord;
    return 0;
}

/* Getters */
double IK_engine::getRelativeLegX(unsigned char legNum)
{
    return _footCoord[legNum].x;
}

double IK_engine::getRelativeLegY(unsigned char legNum)
{
    return _footCoord[legNum].y;
}


coord_t IK_engine::getRelativeLegCoord(unsigned char legNum)
{
    return _footCoord[legNum];
}

int IK_engine::bodyIK(void)
{   
    double totalX[NUM_LEGS] = {0};
    double totalY[NUM_LEGS] = {0};
    double distBodyCenterFeet[NUM_LEGS] = {0};
    double angleBodyCenter[NUM_LEGS] = {0};
    double rollZ[NUM_LEGS] = {0};
    double pitchZ[NUM_LEGS] = {0};

    for (int i = 0; i < NUM_LEGS; i++) 
    {
        totalX[i] = _initFootPos[i].x + _bodyLength + _bodyTgtPos.x;                         
        totalY[i] = _initFootPos[i].y + _bodyLength + _bodyTgtPos.y;

        distBodyCenterFeet[i] = sqrt(pow(totalX[i], 2) + pow(totalY[i], 2));
        angleBodyCenter[i] = (PI * 0.5) - atan2(totalY[i], totalX[i]);                       
        rollZ[i]  = tan(_bodyTgtTilt.roll * PI/180.00) * totalX[i];
        pitchZ[i] = tan(_bodyTgtTilt.yaw * PI/180.00) * totalY[i];

       double val = angleBodyCenter[i] + (_bodyTgtTilt.pitch * PI/180);
        _bodyToFootIK[i].x = cos(val) * distBodyCenterFeet[i] - totalX[i];
        _bodyToFootIK[i].y = sin(val) * distBodyCenterFeet[i] - totalY[i];
        _bodyToFootIK[i].z = rollZ[i] + pitchZ[i];
    }
}

int IK_engine::LegIK(void)
{
    coord_t newFootPos;
    double coxaFootDist[NUM_LEGS] = {0};
    double IKSW[NUM_LEGS] = {0};
    double IKA1[NUM_LEGS] = {0};
    double IKA2[NUM_LEGS] = {0};
    double TAngle[NUM_LEGS]       = {0};
    double IKTibiaAngle[NUM_LEGS] = {0};
    double IKFemurAngle[NUM_LEGS] = {0};
    double IKCoxaAngle[NUM_LEGS]  = {0};

    for (int i = 0; i < NUM_LEGS; i++) 
    {
        newFootPos.x = _initFootPos[i].x + _bodyTgtPos.x + _bodyToFootIK[i].x;
        newFootPos.y = _initFootPos[i].y + _bodyTgtPos.y + _bodyToFootIK[i].y;
        newFootPos.z = _initFootPos[i].z + _bodyTgtPos.z + _bodyToFootIK[i].z;

        coxaFootDist[i] = sqrt(pow(newFootPos.x, 2) + pow(newFootPos.y, 2));

        IKSW[i] = sqrt(pow((coxaFootDist[i] - _linkLength[COXA]), 2) + newFootPos.z);
        IKA1[i] = atan( (coxaFootDist[i] - _linkLength[COXA]) / newFootPos.z);
        IKA2[i] = acos( (pow(_linkLength[TIBIA], 2) - pow(_linkLength[FEMUR], 2) - pow(IKSW[i], 2))/ (-2 * IKSW[i] * _linkLength[COXA]));

        TAngle[i] = acos((pow(IKSW[i], 2) - pow(_linkLength[TIBIA], 2) - pow(_linkLength[COXA],2) ) / (-2 * _linkLength[FEMUR] * _linkLength[TIBIA]));
        
        IKTibiaAngle[i] = 90 - TAngle[i] * 180 / PI;
        IKFemurAngle[i] = 90 - (IKA1[i] + IKA2[i]) * 180/PI;
        IKCoxaAngle[i]  = 90 - atan2(_bodyTgtPos.y, _bodyTgtPos.x) * 180/PI;
    }
}

int IK_engine::generateLegServoAngles(unsigned legNum)
{
    

}
