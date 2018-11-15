#ifndef SERVOCONTROLLER_H
#define SERVOCONTROLLER_H

#include "Adafruit_PWMServoDriver.h"

enum supportedDriver_e {
    ADAFRUIT_SERVO_DRIVER = 0,
    MAX_NUM_SUPPORTED_DRIVERS
};

typedef struct driverArgs_s;
{
    uint8_t addr;
    enum supportedDriver_e driver;
    float freq;
}driverArgs_t;

class servoController {
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
            servoController(driverArgs_t *args);
            ~servoController();
    
            getCtrlType(){return type;};
            getCtrlInterface(){return interface;};        
   
        private:
            enum driverType_e type;
            enum driverInterface interface;
            Adafruit_PWMServoDriver *pwmDriver;

}

#endif /* SERVOCONTROLLER_H */
