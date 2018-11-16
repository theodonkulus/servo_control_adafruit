/****************************************************************************
* servoController

* Depending on your servo make, the pulse width min and max may vary, you
* want these to be as small/large as possible without hitting the hard stop
* for max range. You'll have to tweak them as necessary to match the servos you have.
* 
*  MG90 servos
*  min -90  = 1.0 ms
*  neut 0   = 1.5 ms
*  max  90  = 2.0 ms
*  deadtime = 5us 
*  50 Hz
****************************************************************************/

#ifndef SERVOCONTROLLER_H
#define SERVOCONTROLLER_H

#include "Adafruit_PWMServoDriver.h"

/* servo channel stuff */
#define SERVOMIN  201 // this is the 'minimum' pulse length count (out of 4096)
#define SERVONEUT 305
#define SERVOMAX  410 // this is the 'maximum' pulse length count (out of 4096) 
#define SERVO_RANGE 180 //Range of the servo 0-180 (90 degrees neutral)
#define SERVOSCALE  313 
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

        private:
            enum supportedDriver_e _curDriver;
            enum driverType _type;
            enum driverInterface _interface;
            Adafruit_PWMServoDriver *_pwmDriver;

};

#endif /* SERVOCONTROLLER_H */
