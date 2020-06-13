/*****************************************************
*
*
*
*
*******************************************************/

#include "inverse_kinematics.h"

IK_engine::IK_engine()
{
  
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


int IK_engine::generateLegServoAngles(unsigned legNum)
{
    

}


