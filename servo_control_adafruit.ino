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
//50 Hz

#define SERVOMIN  201 // this is the 'minimum' pulse length count (out of 4096)
#define SERVONEUT 305
#define SERVOMAX  410 // this is the 'maximum' pulse length count (out of 4096)
#define NUM_CHANNELS 16



unsigned char incoming_char;
unsigned char checksum;
bool update_pwm = false;

short ch[NUM_CHANNELS];
short p_ch[NUM_CHANNELS];

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
                                               
void setup()
{
    Serial.begin(115200);
    Serial.println("16 channel Servo test!");

    pwm.begin();
    pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
    //set all channels to neutral position on startup
    
    for(unsigned int i = 0; i < NUM_CHANNELS; i++)
    {
        ch[i] = 0;
        p_ch[i] = ch[i];
        pwm.setPWM(i, 0, SERVONEUT);
    }
}


void loop()
{
    //Read incomming serial commands then set PWM freq/params
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
                update_pwm = 1;
            }
        }
    }

    if(update_pwm)
    {
        //update all the PWM channels
        for(int i = 0; i < NUM_CHANNELS; i++)
        {
            if(ch[i] != p_ch[i])
            {
                p_ch[i] = ch[i];
                pwm.setPWM(i, 0, ch[i]);
            }
        }
    }
    
    update_pwm = 0;
    checksum = 0;
}
