#ifndef SERVOCONTROLLER_H
#define SERVOCONTROLLER_H

#include "Adafruit_PWMServoDriver.h"

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
            servoController(enum driverType_e type, void * args, uint8_t lenArgs);
            ~servoController();
    
            getCtrlType(){return type;};
            getCtrlInterface(){return interface;};        
   
        private:
            enum driverType_e type;
            enum driverInterface interface;
            Adafruit_PWMServoDriver *pwmDriver;

}

#endif /* SERVOCONTROLLER_H */
