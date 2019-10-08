#ifndef __SERVODEFSMG90__
#define __SERVODEFSMG90__

/** 
*  Definitions for the MG90 servo found  experimentally
*
*  MG90 servos
*  min -90  = 1.0 ms
*  neut 0   = 1.5 ms
*  max  90  = 2.0 ms
*  deadtime = 5us 
*  50 Hz

**/

#define SERVO_FREQ_HZ      50
#define SERVO_PERIOD_MS    20
#define SERVO_PERIOD_US    20000
#define SERVO_DEADTIME_US  5

/*Definitions using the Adafruit PWM driver */ 
#define SERVOMIN           201 // this is the 'minimum' pulse length count (out of 4096)
#define SERVONEUT          305
#define SERVOMAX           410 // this is the 'maximum' pulse length count (out of 4096) 
#define SERVO_RANGE        180 //Range of the servo 0-180 (90 degrees neutral)
#define SERVOSCALE         313 

#endif /*__SERVODEFSMG90__*/
