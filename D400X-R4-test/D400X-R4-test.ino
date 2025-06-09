#include <AccelStepper.h>

#define pinSwich 43
#define buzzer 31

AccelStepper stepper4(AccelStepper::DRIVER, 3, 2);  // Wrist

const int potPin = A3;
const int deadZoneMin = 420;
const int deadZoneMax = 570;
const int minEffectiveSpeed = 100;

int pinSwichOn = 0;
int wristStepMove1 = 1;
int wristStepMove2 = 1;

void setup() {
  Serial.begin(9600);
  pinMode(pinSwich, INPUT_PULLUP);
  pinMode(buzzer, OUTPUT);
  noTone(buzzer);

  stepper4.setMaxSpeed(1000);
  stepper4.setAcceleration(400);  // Enable acceleration
}

void loop() {
  int potValue = analogRead(potPin);
  pinSwichOn = !digitalRead(pinSwich);  // LOW = pressed

  long speed = 0;

  // Buzzer feedback
  if (pinSwichOn) {
    tone(buzzer, 1950);
  } else {
    noTone(buzzer);
  }

  if (potValue > deadZoneMax) {
    // Moving positive
    if (pinSwichOn) {
      wristStepMove1 = 0;
    } else {
      wristStepMove1 = 1;
      wristStepMove2 = 1;
    }

    if (!pinSwichOn || !wristStepMove2) {
      speed = map(potValue, deadZoneMax, 1023, minEffectiveSpeed, 1000);
    }

  } else if (potValue < deadZoneMin) {
    // Moving negative
    if (pinSwichOn) {
      wristStepMove2 = 0;
    } else {
      wristStepMove2 = 1;
      wristStepMove1 = 1;
    }

    if (!pinSwichOn || !wristStepMove1) {
      speed = map(potValue, deadZoneMin, 0, -minEffectiveSpeed, -1000);
    }

  } else {
    // Dead zone
    speed = 0;
  }

  stepper4.setSpeed(speed);
  stepper4.run();  
}
