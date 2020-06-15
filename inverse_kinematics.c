/*****************************************************
* Inverse kinematics class and stuff related to getting this robot to move. 
* Adatped from some sorces online for hexapods with a bit of massaging.
* eg) http://toglefritz.com/hexapod-inverse-kinematics-equations/
*   
* TODO: Make this not suck and cleanup his math/implementation. Some things seem to be
* able to be precalculated
*******************************************************/

#include "inverse_kinematics.h"
#include "cmath.h"


/* Cosntructor 

    This should setup the droid and all the given parameters/relationships on startup.
*/
IK_engine::IK_engine()
{
   /* Setup hardcoded defaults for the robots geometry */
   _linkLength[0] = 28.00; 
   _linkLength[1] = 40.00; 
   _linkLength[2] = 55.00; 

   _bodyLength = 42.00; //Same because droid is a circle 

   /* Offset from center of the body */
   _legOffset[0].x =  42.00
   _legOffset[0].y =  42.00
   _legOffset[1].x =  42.00;
   _legOffset[1].x = -42.00;
   _legOffset[2].x = -42.00;
   _legOffset[2].y = -42.00;
   _legOffset[3].x = -42.00;
   _legOffset[3].y =  42.00;

    /* cos(angle /180 * pi) * link0 * link1 , 
      since legs are pi/4 offset we save calculating this  just resolves to scaling by 1/sqrt(2)*/
   _initFootPos[0].x =  (_linkLength[0] * _linkLength[1]) / sqrt(2);
   _initFootPos[1].x =  (_linklength[0] * _linklength[1]) / sqrt(2);
   _initFootPos[2].x = -(_linklength[0] * _linklength[1]) / sqrt(2);
   _initFootPos[3].x = -(_linklength[0] * _linklength[1]) / sqrt(2);

    /*sin(angle/180 * pi) * link0 * link1 */
   _initFootPos[0].y =  (_linklength[0] * _linklength[1]) / sqrt(2);
   _initFootPos[1].y = -(_linklength[0] * _linklength[1]) / sqrt(2);
   _initFootPos[2].y = -(_linklength[0] * _linklength[1]) / sqrt(2);
   _initFootPos[3].y =  (_linklength[0] * _linklength[1]) / sqrt(2);
   
    for(int i = 0; i < NUM_LEGS; i++)
    {
        _initFootPos[i].z = _linkLength[2];
    }

    _legAngleOffset[0] = pi * 1/4;
    _legAngleOffset[1] = pi * 3/4;
    _legAngleOffset[2] = pi * 5/4;
    _legAngleOffset[3] = pi * 7/4;
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

int IK_engine::setLegY(unsigned char legNum, double YCoord)
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
        totalX[i] = _initFootPos[i].x + _bodyLength + _bodyTgtPos.x                         
        totalY[i] = _initFootPos[i].y + _bodyLength + _bodyTgtPos.y

        distBodyCenterFeet[i] = sqrt(totalX[i] ^ 2 + totalY[i] ^2);
        angleBodyCenter[i] = (PI * 0.5) - atan2(totalY[i], totalX[i]);                       
        rollZ[i]  = tan(_bodyTgtTilt.roll * PI/180.00) * totalX[i];
        pitchZ[i] = tan(_bodyTgtTilt.yaw * PI/180.00) * totalY[i];

       double val = angleBodyCenter[i] + (_bodyTgTilt.pitch * PI/180);
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
        newFootPos[i].x = _initFeetPos[i].x + _bodyTgtPos.x + _bodytoFootIK[i].x;
        newFootPos[i].y = _initFeetPos[i].y + _bodyTgtPos.y + _bodytoFootIK[i].y;
        newFootPos[i].z = _initFeetPos[i].z + _bodyTgtPos.z + _bodytoFootIK[i].z;

        coxaFootDist[i] = sqrt(newFootPos[i].x ^2 + newFootPos[i].y ^2);

        IKSW[i] = sqrt((coxaFeetFist[i] - _linkLength[0])^2 + newFootPos[i].z);
        IKA1[i] = atan( (coxaFeetFist[i] - _linkLength[0]) / newFootPos[i].z);
        IKA2[i] = acos( (_linkLength[2]^2 - _linKlength[1] ^2 - IKSW[i] ^2)/ (-2 & IKSW[i] * _linkLength[0]));

        TAngle[i] = acos((IKSW[i]^2 - _linkLength[2]^2 - _linkLength[0]^2 ) / (-2 * _linkLength[1] * _linkLength[2]));
        
        IKTibiaAngle[i] = 90 - Tangle[i] * 180 / PI;
        IKFemurAngle[i] = 90 - (IKA1[i] + IKA2[i]) * 180/PI;
        IKCoxaAngle[i]  = 90 - atan2(_bodyTgtPos.y, _bodyTgtPos.x) * 180/PI;
    }
}

int IK_engine::generateLegServoAngles(unsigned legNum)
{
    

}


