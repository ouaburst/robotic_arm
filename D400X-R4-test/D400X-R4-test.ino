#include <AccelStepper.h>

AccelStepper stepper(AccelStepper::DRIVER, 7, 6);

const int potSpeedPin = A0;
const int potAccelPin = A1;

const int deadZoneMin = 450;
const int deadZoneMax = 550;

const int minEffectiveSpeed = 100;  // Minimum speed to avoid choppy motion

void setup() {
  Serial.begin(9600);
  stepper.setMaxSpeed(1000);      // Reasonable default max
  stepper.setAcceleration(300);   // Reasonable default acceleration
}

void loop() {
  int rawSpeedValue = analogRead(potSpeedPin);
  int rawAccelValue = analogRead(potAccelPin);

  if (rawSpeedValue >= deadZoneMin && rawSpeedValue <= deadZoneMax) {
    stepper.stop();        // Smooth stop using acceleration
  } else {
    // Map speed and apply a minimum threshold
    long speed = map(rawSpeedValue, 0, 1023, -1000, 1000);
    if (speed > 0 && speed < minEffectiveSpeed)
      speed = minEffectiveSpeed;
    else if (speed < 0 && speed > -minEffectiveSpeed)
      speed = -minEffectiveSpeed;

    float acceleration = map(rawAccelValue, 0, 1023, 100, 1000);  // Avoid 0 accel

    stepper.setAcceleration(acceleration);
    stepper.setSpeed(speed);
  }

  // Always call run() to allow stepping
  stepper.run();

  // Optional debug
  /*
  Serial.print("Speed Pot: "); Serial.print(rawSpeedValue);
  Serial.print(" | Speed: "); Serial.print(stepper.speed());
  Serial.print(" | Accel Pot: "); Serial.print(rawAccelValue);
  Serial.print(" | Accel: "); Serial.println(stepper.acceleration());
  */

  // No delay — we want tight stepping loop
}
