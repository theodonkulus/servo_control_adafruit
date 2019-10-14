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
  // put your setup code here, to run once:
  driverArgs_t ctrlArgs;
  ctrlArgs.addr = 0x41;
  ctrlArgs.driver = ADAFRUIT_SERVO_DRIVER;
  ctrlArgs.freq = 50;

  ctrlCtx = new ServoController(&ctrlArgs);

  
  oscLeg0.setPeriod(SERVO_PERIOD_MS);
  oscLeg0.setOffset(90);
  oscLeg0.setAmplitude(40);
  oscLeg0.setPhase(90);
  oscLeg0.start();

  oscLeg2.setPeriod(SERVO_PERIOD_MS);
  oscLeg2.setOffset(90);
  oscLeg2.setAmplitude(40);
  oscLeg2.setPhase(90);
  oscLeg2.start();

  robotMotion = new MotionDriver(ctrlCtx, NUM_JOINTS);
  robotMotion->setHips(90);

  mode = DEBUG_HOME;
  Serial.begin(115200);
  
}
 
static char outAngleStr[15];

void loop()
{
        Serial.println("Test");
        robotMotion->updatePose(); 
}
