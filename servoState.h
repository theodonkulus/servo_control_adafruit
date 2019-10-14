#ifndef SERVOSTATE_H
#define SERVOSTATE_H

#include "servoController.h"

/*
typedef struct servo_s
{
  uint16_t curAngle; //8.8 -90 to 90 , with two sig digs.
  uint16_t prevAngle; //8.8 -90 to 90 , with two sig digs.
  uint16_t nextAngle; //8.8 -90 to 90 , with two sig digs.
  unsigned long long curTick; 
  unsigned long long prevTick;
  unsigned long long newTick;
  unsigned char needsUpdate;
  unsigned char pwmChannel; //channel for the PWM controller to modify  
  unsigned char jointNum;//What joint in the chain it is
  servo_type_t type;
} servo_t; */

enum servo_uflags_e{
    UPDATE_SERVO_POS = 0,
    UPDATE_SERVO_VEL,
    UPDATE_SERVO_ACCEL,
    UPDATE_SERVO_TORQ,
    UPDATE_SERVO_TEMP,
    MAX_NUM_UPDATE_FLAGS
};

class ServoState{
    public:
        ServoState(unsigned char channel, 
                   ServoController *hwCtx);
        ~ServoState();

    public: 
        /* Getter/Setters */
        uint16_t getAngle(void);
        float    getAngleF(void);
        void     setAngle(uint16_t angleVal);
        void     setAngleF(float  angleVal);
        void     setAngle(uint16_t angleVal, uint16_t subAngle);
        uint8_t  getUpdateStatus(void);
        uint8_t  getPosUpdate(void);
        /* Stuff that doess work */
        int8_t   moveServo(void);
        int8_t   moveServoF(void);

        
    private: 
        ServoController *_ctrlCtx;

        float    _curAngleF;
        float    _prevAngleF;
        float    _nextAngleF;

        uint16_t _curAngle;//8.8  0 to 180 , with two sig digs.
        uint16_t _prevAngle;//8.8 0 to 180 , with two sig digs.
        uint16_t _nextAngle;//8.8 0 to 180 , with two sig digs.
        unsigned long long _curTick; 
        unsigned long long _prevTick;
        unsigned long long _newTick;
        uint8_t _updateStatus;
        unsigned char _channel; //channel for the PWM controller to modify  
        unsigned char _jointNum;//What joint in the chain it is
};


#endif /* SERVOSTATE_H */
