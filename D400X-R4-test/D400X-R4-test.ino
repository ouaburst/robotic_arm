#include <AccelStepper.h>

#define pinSwich 43
#define buzzer 31

AccelStepper stepper4(AccelStepper::DRIVER, 3, 2);  // Wrist

int wristPos = 0;
int wristStepMove1 = 1;
int wristStepMove2 = 1;
int pinSwichOn = 0;

void setup() {
  Serial.begin(9600);
  pinMode(pinSwich, INPUT_PULLUP);
  pinMode(buzzer, OUTPUT);

  stepper4.setMaxSpeed(1000);
  stepper4.setAcceleration(500);

  noTone(buzzer);  // Ensure buzzer is off at start
}

void loop() {
  // Simulate joystick input (replace with ps2x.Analog or analogRead(A3) in your setup)
  int joystickLY = analogRead(A3);  // Replace with actual control
  pinSwichOn = !digitalRead(pinSwich);  // LOW = switch is pressed

  // Buzzer feedback
  if (pinSwichOn) {
    tone(buzzer, 1950);
  } else {
    noTone(buzzer);
  }

  // ---- Wrist movement logic (same as original) ----
  if (joystickLY > 1000) {  // Forward
    if (pinSwichOn) {
      wristStepMove1 = 0;
    } else {
      wristStepMove1 = 1;
      wristStepMove2 = 1;
    }

    if (!pinSwichOn || !wristStepMove2) {
      wristPos += 1;
    }

  } else if (joystickLY < 30) {  // Backward
    if (pinSwichOn) {
      wristStepMove2 = 0;
    } else {
      wristStepMove2 = 1;
      wristStepMove1 = 1;
    }

    if (!pinSwichOn || !wristStepMove1) {
      wristPos -= 1;
    }
  }

  // ---- Move wrist ----
  stepper4.moveTo(wristPos);
  stepper4.setSpeed(400);  // Or STEP_MOTOR_SPEED_WRIST
  while (stepper4.distanceToGo() != 0) {
    stepper4.runSpeedToPosition();
  }
}
