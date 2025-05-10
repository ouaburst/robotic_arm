#include <PsxControllerHwSpi.h>
#include <AccelStepper.h>

// Pins for Hardware SPI (Uno: MOSI=11, MISO=12, SCK=13)
#define PIN_ATT 53  // Attention pin

PsxControllerHwSpi<PIN_ATT> psx;

// Stepper motor driver pins: Step pin = 7, Direction pin = 6
AccelStepper stepper(AccelStepper::DRIVER, 7, 6); 

void setup() {
    Serial.begin(115200);

    // Initialize PS2 controller
    if (!psx.begin()) {
        Serial.println("Controller not found");
        while (true);  // Halt if not found
    } else {
        Serial.println("Controller found !!");
    }

    psx.enterConfigMode();
    psx.enableAnalogSticks();
    psx.exitConfigMode();

    // Initialize stepper settings
    stepper.setMaxSpeed(1000);     // Max speed in steps per second
    stepper.setAcceleration(500);  // Acceleration in steps per second squared
}

void loop() {
    if (psx.read()) {
        if (psx.buttonPressed(PSB_PAD_UP)) {
            Serial.println("PSB_PAD_UP pressed");
            stepper.setSpeed(300);  // Positive direction
            stepper.runSpeed();     // Non-blocking
        }
        else if (psx.buttonPressed(PSB_PAD_DOWN)) {
            Serial.println("PSB_PAD_DOWN pressed");
            stepper.setSpeed(-300);  // Negative direction
            stepper.runSpeed();      // Non-blocking
        }
        else {
            stepper.setSpeed(0);  // Stop if no button is pressed
        }
    }

    stepper.runSpeed();  // Keep running the motor
    delay(1);
}
