#include <TimerOne.h>

#include <StaticThreadController.h>
#include <ThreadController.h>
#include <Thread.h>

#include "Wire.h"
#include "Adafruit_PWMServoDriver.h"



// called this way, it uses the default address 0x40
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

// Depending on your servo make, the pulse width min and max may vary, you
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!

//MG90 servos
//min -90  = 1.0 ms
//neut 0   = 1.5 ms
//max  90  = 2.0 ms
//deadtime = 5us 
//50 Hz

//servo channel stuff
#define SERVOMIN  201 // this is the 'minimum' pulse length count (out of 4096)
#define SERVONEUT 305
#define SERVOMAX  410 // this is the 'maximum' pulse length count (out of 4096)
#define NUM_CHANNELS 16

typedef enum servo_type_e 
{
    ANKLE_0 = 0,
    HIP_0,      /* 1 */  
    ANKLE_1,    /* 2 */
    HIP_1,      /* 3 */
    ANKLE_2,    /* 4 */
    HIP_2,      /* 5 */
    ANKLE_3,    /* 6 */
    HIP_3,      /* 7 */
    NUM_JOINTS  /* 8 */
}servo_type_t;

unsigned char jointPwmChLUT[NUM_JOINTS] = {0, 1, 2, 3, 4, 5, 6, 7};

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

static servo_t joints[NUM_JOINTS];
static unsigned long long sysTime = 0;

unsigned short ch[NUM_CHANNELS];
unsigned short p_ch[NUM_CHANNELS];

//radio channel defines
#define NUM_RADIO_CHAN 6
int rad_ch[NUM_RADIO_CHAN]; // Here's where we'll keep our channel values
int prev_rad_ch[NUM_RADIO_CHAN]; // Here's where we'll keep our channel values
unsigned char toggle_switch_on = 0;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

/* thread related */
Thread rcCommandUpdateThread = Thread();
Thread serialCmdThread = Thread();

StaticThreadController<2> threadCtrl (&rcCommandUpdateThread, &serialFrameCommandPop);


bool rcRecieverPoll(unsigned int timeWindow)
{
    bool result = false;
    unsigned int frame;

    //update values based on input pulses from transmitter
    for(int i = 0; i < NUM_RADIO_CHAN; i++)
    {
       rad_ch[i] = pulseIn(i+2, HIGH, timeWindow);
       if( (abs(rad_ch[i] - prev_rad_ch[i]) > 20)) 
       {
           /*Serial.print(frame, DEC);
           Serial.print(" Ch");
           Serial.print(i, DEC);
           Serial.print(" idx");
           Serial.print(i+2, DEC);
           Serial.print(" ");
           Serial.print(rad_ch[i]);
           Serial.print(" ");
           Serial.println(map(rad_ch[i], 1000,2000,-500,500)); // center at 0*/
                    
           result = true;
       }    
    }    
    return result;
}

/******************************************
*   readIncomingCommands()
*
*   Read command over serial and update the desired
*   system state 
*   
******************************************/
void readIncommingCommands()
{
    unsigned char incoming_char;
    unsigned char checksum;
    int bytes_available = Serial.available();

    if(bytes_available > 0) 
    {
        incoming_char = Serial.read(); 
        if(incoming_char == 0x27)
        {
            bytes_available--;
            while(bytes_available > 1)
            {   
                unsigned char c;
                unsigned int data;
                
                incoming_char = Serial.read();
                checksum += incoming_char;
                
                c = ((0xF0 & incoming_char) >> 4);
                data = ((0x0F & incoming_char) << 8);
                
                incoming_char = Serial.read();
                checksum += incoming_char;
                
                data |= incoming_char;
                ch[c] = data;

                bytes_available -= 2;
            }  
            incoming_char = Serial.read();
            bytes_available--;

            if(incoming_char == checksum)
            {
                Serial.write(checksum);
                ret = 1;
            }
        }
        Serial.println("16 channel Servo Aquire complete!");
    }
}

/******************************************
*   moveServos()
*
*   Move servos based on the needsUpdate flag and newTick value 
*   
*   Returns: Nothing 
******************************************/
void moveServos()
{
    unsigned char update;
    unsigned char channel;
     
    for(i = 0; i < NUM_JOINTS; i++)
    {
        update  = joints[i].needsUpdate;
        channel = joints[i].pwmChannel;
        newTick = joints[i].newTick;
    
        if(update)
        {
            pwm.setPWM(channel, 0, newTick);
            /* update previous state */
            joints[i].prevTick = joints[i].curTick;
            /* Set current state to new value */
            joints[i].curTick = newTick;
        }   
    }
}

void timerCallback() 
{
    sysTime++;
    threadCtrl.run();
}
                                 
void setup()
{
    Serial.begin(115200);
    Serial.println("16 channel Servo test!");

    pwm.begin();
    pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
    
    //set all joint channels to neutral position on startup
    for(unsigned int i = 0; i < NUM_JOINTS; i++)
    {
        ch[i] = 0;
        p_ch[i] = ch[i];
        joints[i].curTick  = SERVONEUT;
        joints[i].prevTick = SERVONEUT;
        joints[i].newTick  = SERVONEUT;
        joints[i].curAngle = 0;
        joints[i].prevAngle = 0;
        joints[i].nextAngle = 0;
        joints[i].PWMChannel = jointPwmChLUT[i];
        joints[i].type = i;
        pwm.setPWM(joints[i].PWMChannel, 0, SERVONEUT);
    }
    
    //Setup reciever defines for each incomming signal
    pinMode(4, INPUT); //Elev
    pinMode(5, INPUT); //Rudd
    pinMode(6, INPUT); //Gear 
    pinMode(7, INPUT); //AUX1 
    pinMode(3, INPUT); //Aile
    pinMode(2, INPUT); //Throt

    /* Use a 1us timer to gate threads */
    /* Set a 1us timer for the timing of threads. */
    Timer1.initialize(1);
    Timer1.attachInterrupt(timerCallback);
    
    /* Thread setup callbacks*/
    serialCmdThread->onRun(readIncommingCommands);
    serialCmdThread->setInterval(20000);
    
}

void loop()
{
    
    /* ToDo: Use IRQS to grab reciever values */
    diff = rcRecieverPoll(200000);
    delay(20);    
    if(diff)
    {
        processRcCommand();
    }

    if(toggle_switch_on == 1) 
    {
       update_pwm = readIncommingCommands()
    }

    moveServos();
}
