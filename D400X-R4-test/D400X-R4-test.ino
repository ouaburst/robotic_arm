#include <AccelStepper.h>

AccelStepper stepper(AccelStepper::DRIVER, 7, 6);

const int potSpeedPin = A0;

const int deadZoneMin = 450;
const int deadZoneMax = 550;

const int minEffectiveSpeed = 100;  // Minimum speed to avoid choppy motion
const int fixedAcceleration = 300;  // Constant acceleration

void setup() {
  Serial.begin(9600);
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(fixedAcceleration);
}

void loop() {
  int rawSpeedValue = analogRead(potSpeedPin);

  if (rawSpeedValue >= deadZoneMin && rawSpeedValue <= deadZoneMax) {
    stepper.stop();  // Smooth stop
  } else {
    long speed = map(rawSpeedValue, 0, 1023, -1000, 1000);
    
    if (speed > 0 && speed < minEffectiveSpeed)
      speed = minEffectiveSpeed;
    else if (speed < 0 && speed > -minEffectiveSpeed)
      speed = -minEffectiveSpeed;

    stepper.setSpeed(speed);
  }

  stepper.run();
}
