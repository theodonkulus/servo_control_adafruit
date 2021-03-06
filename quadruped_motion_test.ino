#include <Octosnake.h>

#include "servoController.h"
#include "servoDefsMG90.h"
#include "motion.h"

static Oscillator oscLeg0;
static Oscillator oscLeg1;
static Oscillator oscLeg2;
static Oscillator oscLeg3;

static ServoController *ctrlCtx = 0;
static MotionDriver    *robotMotion = 0;


enum {
    DEBUG_HOME = 0,
    DEBUG_OSC,
    DEBUG_OFF,
    DROID_RUN,
    NUM_DEBUG_MODES
};

unsigned int mode = DEBUG_HOME;

void setup() {
  mode = DEBUG_HOME;
  //Serial.begin(115200);
  
  // put your setup code here, to run once:
  driverArgs_t ctrlArgs;
  ctrlArgs.addr = 0x41;
  ctrlArgs.driver = ADAFRUIT_SERVO_DRIVER;
  ctrlArgs.freq = 50;

  ctrlCtx = new ServoController(&ctrlArgs);

  
  oscLeg0.setPeriod(SERVO_PERIOD_MS);
  oscLeg0.setOffset(90);
  oscLeg0.setAmplitude(40);
  oscLeg0.setPhase(0);
  oscLeg0.start();

  /*oscLeg2.setPeriod(SERVO_PERIOD_MS);
  oscLeg2.setOffset(90);
  oscLeg2.setAmplitude(40);
  oscLeg2.setPhase(90);
  oscLeg2.start(); */

  robotMotion = new MotionDriver(ctrlCtx, NUM_JOINTS, false);
  robotMotion->setHips(90);
  
}

void loop()
{
    float oscOutput = oscLeg0.refresh();
/*    static int i = 0;
     i++;
        if ((i % 1000) == 0) 
        {
           Serial.println(i);
        }*/
        robotMotion->moveAnkles(oscOutput);
        robotMotion->updatePose(); 
        //delay(1);
}
