#include <AccelStepper.h>

AccelStepper stepper1(AccelStepper::DRIVER, 7, 6);    // Base
AccelStepper stepper2(AccelStepper::DRIVER, 9, 8);    // Shoulder
AccelStepper stepper3(AccelStepper::DRIVER, 5, 4);    // Elbow
AccelStepper stepper4(AccelStepper::DRIVER, 3, 2);    // Wrist


const int potSpeedPin0 = A0;   // Shoulder 
const int potSpeedPin1 = A1;   // Base 
const int potSpeedPin2 = A2;   // Elbow 
const int potSpeedPin3 = A3;   // Wrist 

const int deadZoneMin = 430;
const int deadZoneMax = 560;

const int minEffectiveSpeed = 100;  // Minimum speed to avoid choppy motion
const int fixedAcceleration = 300;  // Constant acceleration

void setup() {
  Serial.begin(9600);
  stepper1.setMaxSpeed(1000);
  stepper2.setMaxSpeed(1000);
  stepper3.setMaxSpeed(1000);
  stepper4.setMaxSpeed(1000);
  
  stepper1.setAcceleration(fixedAcceleration);
  stepper2.setAcceleration(fixedAcceleration);
  stepper3.setAcceleration(fixedAcceleration);
  stepper4.setAcceleration(fixedAcceleration);
    
}

void loop() {
  int rawSpeedValue0 = analogRead(potSpeedPin0);
  int rawSpeedValue1 = analogRead(potSpeedPin1);
  int rawSpeedValue2 = analogRead(potSpeedPin2);
  int rawSpeedValue3 = analogRead(potSpeedPin3);  
    
  if (rawSpeedValue0 >= deadZoneMin && rawSpeedValue0 <= deadZoneMax) {
    stepper1.stop();  // Smooth stop
  } else {
    long speed = map(rawSpeedValue0, 0, 1023, -1000, 1000);
    
    if (speed > 0 && speed < minEffectiveSpeed)
      speed = minEffectiveSpeed;
    else if (speed < 0 && speed > -minEffectiveSpeed)
      speed = -minEffectiveSpeed;

    stepper1.setSpeed(speed);
  }

  if (rawSpeedValue1 >= deadZoneMin && rawSpeedValue1 <= deadZoneMax) {
    stepper2.stop();  // Smooth stop
  } else {
    long speed = map(rawSpeedValue1, 0, 1023, -1000, 1000);
    
    if (speed > 0 && speed < minEffectiveSpeed)
      speed = minEffectiveSpeed;
    else if (speed < 0 && speed > -minEffectiveSpeed)
      speed = -minEffectiveSpeed;

    stepper2.setSpeed(speed);
  }

  if (rawSpeedValue2 >= deadZoneMin && rawSpeedValue2 <= deadZoneMax) {
    stepper3.stop();  // Smooth stop
  } else {
    long speed = map(rawSpeedValue2, 0, 1023, -1000, 1000);
    
    if (speed > 0 && speed < minEffectiveSpeed)
      speed = minEffectiveSpeed;
    else if (speed < 0 && speed > -minEffectiveSpeed)
      speed = -minEffectiveSpeed;

    stepper3.setSpeed(speed);
  }


  if (rawSpeedValue3 >= deadZoneMin && rawSpeedValue3 <= deadZoneMax) {
    stepper4.stop();  // Smooth stop
  } else {
    long speed = map(rawSpeedValue3, 0, 1023, -1000, 1000);
    
    if (speed > 0 && speed < minEffectiveSpeed)
      speed = minEffectiveSpeed;
    else if (speed < 0 && speed > -minEffectiveSpeed)
      speed = -minEffectiveSpeed;

    stepper4.setSpeed(speed);
  }


  stepper1.run();
  stepper2.run();
  stepper3.run();
  stepper4.run();
  
}
