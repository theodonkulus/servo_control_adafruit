#ifndef SERVOSTATE_H
#define SERVOSTATE_H

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
} servo_t;

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
        ServoState(unsigned char pwmChannel, servoController *hwCtx);
        ~ServoState();

        /* Getter/Setters */
        uint16_t getAngle(void);
        void     setAngle(uint16_t angleVal);
        void     setAngle(uint16_t angleVal, uint16_t subAngle);
        uint8_t  getUpdateStatus(void);

        /* Stuff that doess work */
        int8_t   moveServo(void);

    private:
        
    private:
        servoController          *ctrlCtx;

        uint16_t curAngle; //8.8  0 to 180 , with two sig digs.
        uint16_t prevAngle; //8.8 0 to 180 , with two sig digs.
        uint16_t nextAngle; //8.8 0 to 180 , with two sig digs.
        unsigned long long curTick; 
        unsigned long long prevTick;
        unsigned long long newTick;
        uint8_t needsUpdate;
        unsigned char pwmChannel; //channel for the PWM controller to modify  
        unsigned char jointNum;//What joint in the chain it is
        servo_type_t type;
}


#endif /* SERVOSTATE_H */
