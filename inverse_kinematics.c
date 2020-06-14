/*****************************************************
*
*
*
*
*******************************************************/

#include "inverse_kinematics.h"

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


}

int IK_engine::generateLegServoAngles(unsigned legNum)
{
    

}


