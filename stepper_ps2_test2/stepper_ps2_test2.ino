#include <PsxControllerHwSpi.h>
#include <AccelStepper.h>

// Pins for Hardware SPI (Mega: MOSI=51, MISO=50, SCK=52)
#define PIN_ATT 53  // Attention pin

PsxControllerHwSpi<PIN_ATT> psx;

// Stepper motor driver pins: Step = 7, Dir = 6
AccelStepper stepper(AccelStepper::DRIVER, 7, 6);

void setup() {
    Serial.begin(115200);

    if (!psx.begin()) {
        Serial.println("Controller not found");
        while (true);
    } else {
        Serial.println("Controller found !!");
    }

    psx.enterConfigMode();
    psx.enableAnalogSticks();
    psx.exitConfigMode();

    stepper.setMaxSpeed(1000);     
    stepper.setAcceleration(300);  
}

void loop() {
    psx.read();  // Always read controller input

    if (psx.buttonPressed(PSB_PAD_UP)) {
        stepper.setSpeed(500);  // Faster forward
    }
    else if (psx.buttonPressed(PSB_PAD_DOWN)) {
        stepper.setSpeed(-500);  // Faster backward
    }
    else {
        //stepper.setSpeed(0);  // Stop motor
        stepper.stop();
    }

    //stepper.runSpeed();  // Must be called frequently
    stepper.run();
    
    delay(2);

}
