#include <TimerOne.h>
#include <StaticThreadController.h>
#include <ThreadController.h>
#include <Thread.h>


/* *********************************************************
*  called this way, it uses the default address 0x40
*  you can also call it with a different address you want 
************************************************************/




unsigned char jointPwmChLUT[NUM_JOINTS] = {ANKLE_0, 
                                             HIP_0, 
                                           ANKLE_1,
                                             HIP_1,
                                           ANKLE_2,
                                             HIP_2,       
                                           ANKLE_3,
                                             HIP_3
                                          };


static servo_t joints[NUM_JOINTS];

/* Inverse Kinematics related */
static float bodyIKTransform[4][4];
static float legIKTransform[4][4];

typedef struct position_s
{
  int32_t x;
  int32_t y;
  int32_t z;
} position_t;

#define NUM_LEGS 4
#define JOINTS_PER_LEG 2
typedef struct leg_s
{
    position_t current_p;  /* position of end effector */
    position_t next_p;     
    position_t previous_p;  
    servo_t*   servo[JOINTS_PER_LEG]; /*servos associated with the joint */
}leg_t;

typedef struct body_s
{
    position_t current_p;
    position_t next_p;
    position_t previous_p;
    leg_t legs[NUM_LEGS];
}body_t;

body_t robot_body;

/*radio channel defines */
typedef enum radio_channel_e 
{
    THROTTLE          = 0,
    AILE,          /* = 1 */  
    ELEV,          /* = 2 */
    RUDD,          /* = 3 */
    GEAR,          /* = 4 */
    AUX,           /* = 5 */
    NUM_RADIO_CHAN /* = 6 */
}radio_channel_t;

typedef struct radio_channel_s
{
  unsigned long long startTick;
  unsigned long long endTick;
} radio_t;


static volatile radio_t radio_channel[NUM_RADIO_CHAN];
volatile static unsigned long long sysTime = 0;

/* thread related */
Thread getRcRecieverThread = Thread();
Thread getImuDataThread = Thread();
Thread getSerialCmdThread = Thread();
Thread setMoveServos = Thread();

StaticThreadController<4> threadCtrl (&getRcRecieverThread, &getImuDataThread, &getSerialCmdThread, &setMoveServos);

/**********************************
 * getThrottleStartTime()
 * 
 * Get the sysTime when the Throttle PWM starts
 * Currently using the Throttle as the Start of frame.
 *********************************/
 void getThrottleStartTime()
 {
    radio_channel[THROTTLE].startTick = sysTime;
 }

 /**********************************
 * getThrottleStartTime()
 * 
 * Get the sysTime when the ThrottlePWM ends
 * Currently using the Throttle as the Start of frame.
 *********************************/
 void getThrottleEndTime()
 {    
    radio_channel[THROTTLE].endTick = sysTime;
 }

/**********************************
 * getAileStartTime()
 * 
 * Get the sysTime when the AilePWM starts
 * Currently using the Aile as sinc in frame.
 *********************************/
 void getAileStartTime()
 {
    radio_channel[AILE].startTick = sysTime;
 }

 /**********************************
 * getAileEndTime()
 * 
 * Get the sysTime when the AilePWM ends
 * Currently using the Aile as sync in the frame.
 *********************************/
 void getAileEndTime()
 {    
    radio_channel[AILE].endTick = sysTime;
 }

/******************************************
*   readIncomingCommands()
*
*   Read command over serial and update the desired
*   system state 
*   
******************************************/
/*void readIncommingCommands()
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
                //ch[c] = data;

                bytes_available -= 2;
            }  
            incoming_char = Serial.read();
            bytes_available--;

            if(incoming_char == checksum)
            {
                Serial.write(checksum);
            }
        }
        Serial.println("16 channel Servo Aquire complete!");
    }
}*/

/******************************************
*   moveServos()
*
*   Move servos based on the needsUpdate flag and newTick value 
*   
*   Returns: Nothing 
******************************************/
void moveServos()
{
    bool updateMove;
    unsigned char channel;
    unsigned char newTick;
    unsigned char i;
 
    for(i = 0; i < NUM_JOINTS; i++)
    {
        updateMove = joints[i].needsUpdate;
        channel = joints[i].pwmChannel;
        newTick = joints[i].newTick;
    
        if(updateMove)
        {
            pwm.setPWM(channel, 0, newTick);
            /* update previous state */
            joints[i].prevTick = joints[i].curTick;
            /* Set current state to new value */
            joints[i].curTick = newTick;
        }   
    }
}

/***********************************************************************
* Sets the position of the body in 3 space before sending its coordinate
* down to the legs
*************************************************************************/
void posForBodyIK(uint16_t target_x_pos, uint16_t target_y_pos)
{
    
}

void setIKForLeg(unsigned char legNumber, uint16_t)
{

}

void solveIK(unsigned int)
{


}

void setLegTargetPos(unsigned char legNumber, uint16_t tx, uint16_t ty, uint16_t yz)
{

}

/* need to flush out more of the backend but the creep should do this */
/*void creepGait()
{
    //STARTING POSITION
    Set_Leg_Position_0Cartes(‘A’, 50,10,down);
    Set_Leg_Position_0Cartes(‘B’, 50, -10,down);
    Set_Leg_Position_0Cartes(‘C’, -50,50,down);
    Set_Leg_Position_0Cartes(‘D’, -50, -50,down);


    //STEP 1
    Step_in_Y(‘B’, -10,110);


    //SHIFT1
    Move_in_Y(‘A’, 10,-50);
    Move_in_Y(‘B’, 110,50);
    Move_in_Y(‘C’, 50,-10);
    Move_in_Y(‘D’, -50,-110);


    //STEP 2
    Step_in_Y(‘D’, -110,10);


    //STEP 3
    Step_in_Y(‘C’, -10,110);


    //SHIFT1
    Move_in_Y(‘A’, -50,-110);
    Move_in_Y(‘B’, 50,-10);
    Move_in_Y(‘C’, 110,50);
    Move_in_Y(‘D’, 10,-50);


    //STEP 4
    Step_in_Y(‘A’, -110,10);
}*/

void timerCallback() 
{
    sysTime++;
    threadCtrl.run();
}
                                 
void setup()
{
    //Serial.begin(115200);
    //Serial.println("Droid debug Start!");
    /* Analog servos run at ~50 Hz updates*/
    pwm.begin();
    pwm.setPWMFreq(50);  
    
    //set all joint channels to neutral position on startup
    for(unsigned int i = 0; i < NUM_JOINTS; i++)
    {
        joints[i].curTick  = SERVONEUT;
        joints[i].prevTick = SERVONEUT;
        joints[i].newTick  = SERVONEUT;
        joints[i].curAngle = 0;
        joints[i].prevAngle = 0;
        joints[i].nextAngle = 0;
        joints[i].pwmChannel = jointPwmChLUT[i];
        joints[i].type = (servo_type_t)i;
        pwm.setPWM(joints[i].pwmChannel, 0, SERVONEUT);
    }
    
    /* ***********************************************************************
     *  Setup reciever defines for each incomming signal 20ms ` 21.8 Frames
     * Signals are staggered from an initial commands 
     ************************************************************************/
    pinMode(4, INPUT); /* Elev    top/down from right control stick 1ms-2ms */
    pinMode(5, INPUT); /* Rudd    left right on control stick 1.5ms center 0.2ms swing */
    pinMode(6, INPUT); /* Gear    ch5 right encoder 1.5ms center 1ms-2ms */
    pinMode(7, INPUT); /* AUX1      Mirrors what*/  
    pinMode(3, INPUT); /* Aile    Left right on right stick 1ms-2ms 1.5ms center */
    pinMode(2, INPUT); /* Throt   Forward back on left stick 1ms-2ms 1.5ms center */

     
    /* Thread setup callbacks*/
    /*getSerialCmdThread.onRun(readIncommingCommands);
    getSerialCmdThread.setInterval(20000); */

   /* setup interrupt pins for start of rc frame updates */
    attachInterrupt(digitalPinToInterrupt(2), getThrottleStartTime, HIGH);
    attachInterrupt(digitalPinToInterrupt(2), getThrottleEndTime, LOW);
    attachInterrupt(digitalPinToInterrupt(3), getAileStartTime, HIGH);
    attachInterrupt(digitalPinToInterrupt(3), getAileEndTime, LOW);

    /* Use a 1us timer to gate threads */
    /* Set a 1us timer for the timing of threads. */
    Timer1.initialize(1);
    Timer1.attachInterrupt(timerCallback);
    Timer1.start();
    
}

void loop()
{
    
    /* ToDo: Use IRQS to grab reciever values */

}
