/****************************************************************************
* servoController

* Depending on your servo make, the pulse width min and max may vary, you
* want these to be as small/large as possible without hitting the hard stop
* for max range. You'll have to tweak them as necessary to match the servos you have.
* 
****************************************************************************/

#ifndef SERVOCONTROLLER_H
#define SERVOCONTROLLER_H

#include "Adafruit_PWMServoDriver.h"
#include "servoDefsMG90.h"

/* servo channel stuff */

#define TICK_SCALE (((SERVOMAX - SERVOMIN) * SERVOSCALE) / SERVO_RANGE) 
#define ANGLETOTICKS(X)(TICK_SCALE * X) / SERVOSCALE

enum supportedDriver_e {
    ADAFRUIT_SERVO_DRIVER = 0,
    MAX_NUM_SUPPORTED_DRIVERS
};

typedef struct driverArgs_s
{
    uint8_t addr;
    enum supportedDriver_e driver;
    float freq;
} driverArgs_t;


class ServoController {
        public: 
            enum driverType{
                SERVO_CTRL_TYPE_PWM = 0,
                SERVO_CTRL_TYPE_CMD,
                MAX_SERVO_CTRL_TYPES
            };

            enum driverInterface{
                CTRL_IF_UART = 0,
                CTRL_IF_I2C,
                CTRL_IF_SPI,
                MAX_DRIVER_IF_TYPES
            };
            ServoController(driverArgs_t *args);
            ~ServoController();
    
            enum driverType getCtrlType(){return _type;};
            enum driverInterface getCtrlInterface(){return _interface;};        
   
            uint16_t setAnglePos(uint8_t ch, uint16_t angle);
            uint16_t setAnglePos(uint8_t ch, float angle);
            
        private:
            float anglesToTicks(float angle);
            
            float _servoMax;
            float _servoMin;
            float _servoScale;
            float _servoRange;
            float _tickScale;

            enum supportedDriver_e _curDriver;
            enum driverType _type;
            enum driverInterface _interface;
            Adafruit_PWMServoDriver *_pwmDriver;

};

#endif /* SERVOCONTROLLER_H */
