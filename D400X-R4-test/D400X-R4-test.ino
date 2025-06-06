#include <AccelStepper.h>
#include <Servo.h>

AccelStepper stepper1(AccelStepper::DRIVER, 7, 6);    // Base
AccelStepper stepper2(AccelStepper::DRIVER, 9, 8);    // Shoulder
AccelStepper stepper3(AccelStepper::DRIVER, 5, 4);    // Elbow
AccelStepper stepper4(AccelStepper::DRIVER, 3, 2);    // Wrist

const int potSpeedPin0 = A0;   // Shoulder 
const int potSpeedPin1 = A1;   // Base 
const int potSpeedPin2 = A2;   // Elbow 
const int potSpeedPin3 = A3;   // Wrist 
const int potSpeedPin4 = A4;   // Gripper

const int deadZoneMin = 420;
const int deadZoneMax = 570;

const int minEffectiveSpeed = 100;
const int fixedAcceleration = 300;

#define SERVO_PIN 44
#define MIN_GRIPPER_POS 0
#define MAX_GRIPPER_POS 145

#define pinSwich 43  // Shared limit switch pin
int buzzer = 31;

int servoPos = 0;
bool gripperActive = false;
bool limitTriggered = false;

Servo myservo;

void setup() {
  Serial.begin(9600);

  stepper1.setMaxSpeed(1000); stepper1.setAcceleration(fixedAcceleration);
  stepper2.setMaxSpeed(1000); stepper2.setAcceleration(fixedAcceleration);
  stepper3.setMaxSpeed(1000); stepper3.setAcceleration(fixedAcceleration);
  stepper4.setMaxSpeed(1000); stepper4.setAcceleration(fixedAcceleration);

  pinMode(SERVO_PIN, OUTPUT);
  myservo.attach(SERVO_PIN);
  myservo.write(servoPos);

  pinMode(pinSwich, INPUT_PULLUP); 
  pinMode(buzzer, OUTPUT);
  noTone(buzzer);
}

void loop() {
  int rawSpeedValue0 = analogRead(potSpeedPin0);
  int rawSpeedValue1 = analogRead(potSpeedPin1);
  int rawSpeedValue2 = analogRead(potSpeedPin2);
  int rawSpeedValue3 = analogRead(potSpeedPin3);  
  int rawSpeedValue4 = analogRead(potSpeedPin4);  

  // Read shared switch state: LOW means pressed
  limitTriggered = !digitalRead(pinSwich);

  if (limitTriggered) {
    tone(buzzer, 1950);
  } else {
    noTone(buzzer);
  }

  /*
   * Servo1 (Gripper) control overrides everything else
   */
  if (rawSpeedValue4 > deadZoneMax || rawSpeedValue4 < deadZoneMin) {
    gripperActive = true;

    if (rawSpeedValue4 > deadZoneMax && servoPos < MAX_GRIPPER_POS) {
      servoPos += 1;
      myservo.write(servoPos);
      delay(5);
    }

    if (rawSpeedValue4 < deadZoneMin && servoPos > MIN_GRIPPER_POS) {
      servoPos -= 1;
      myservo.write(servoPos);
      delay(5);
    }

    return;  // Skip stepper control if gripper is active
  } else {
    gripperActive = false;
  }

  /*
   * Stepper control — only if no gripper active and no limit triggered
   */
  if (!limitTriggered) {
    if (rawSpeedValue0 >= deadZoneMin && rawSpeedValue0 <= deadZoneMax) {
      stepper1.stop();
    } else {
      long speed = map(rawSpeedValue0, 0, 1023, -1000, 1000);
      if (abs(speed) < minEffectiveSpeed)
        speed = (speed > 0 ? minEffectiveSpeed : -minEffectiveSpeed);
      stepper1.setSpeed(speed);
    }

    if (rawSpeedValue1 >= deadZoneMin && rawSpeedValue1 <= deadZoneMax) {
      stepper2.stop();
    } else {
      long speed = map(rawSpeedValue1, 0, 1023, -1000, 1000);
      if (abs(speed) < minEffectiveSpeed)
        speed = (speed > 0 ? minEffectiveSpeed : -minEffectiveSpeed);
      stepper2.setSpeed(speed);
    }

    if (rawSpeedValue2 >= deadZoneMin && rawSpeedValue2 <= deadZoneMax) {
      stepper3.stop();
    } else {
      long speed = map(rawSpeedValue2, 0, 1023, -1000, 1000);
      if (abs(speed) < minEffectiveSpeed)
        speed = (speed > 0 ? minEffectiveSpeed : -minEffectiveSpeed);
      stepper3.setSpeed(speed);
    }

    if (rawSpeedValue3 >= deadZoneMin && rawSpeedValue3 <= deadZoneMax) {
      stepper4.stop();
    } else {
      long speed = map(rawSpeedValue3, 0, 1023, -1000, 1000);
      if (abs(speed) < minEffectiveSpeed)
        speed = (speed > 0 ? minEffectiveSpeed : -minEffectiveSpeed);
      stepper4.setSpeed(speed);
    }
  }

  /*
   * Only run steppers if gripper is inactive and no switch is triggered
   */
  if (!gripperActive && !limitTriggered) {
    stepper1.run();
    stepper2.run();
    stepper3.run();
    stepper4.run();
  }
}
