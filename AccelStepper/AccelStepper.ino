#include <AccelStepper.h>

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, 7, 6);

int pos;
int start;
int i;

void setup()
{  
  stepper.setMaxSpeed(5000);
  stepper.setAcceleration(5000);

  
  start=1;
  pos=0;
  i=0;
}

void loop()
{
  while(i <50){

    pos = pos+10;
    stepper.moveTo(pos);

    while (stepper.distanceToGo() !=0) {
      stepper.runSpeedToPosition();
    }

    i++;
  }
}
