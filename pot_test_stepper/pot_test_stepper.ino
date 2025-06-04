// Define the analog pin where the potentiometer is connected
const int potSpeedPin3 = A3;

void setup() {
  Serial.begin(9600); // Initialize serial communication
}

void loop() {
  // Read analog value (0 to 1023)
  int rawSpeedValue3 = analogRead(potSpeedPin3);

  // Print the value to the Serial Monitor
  Serial.print("Potentiometer value: ");
  Serial.println(rawSpeedValue3);

  delay(200); // Wait for 200 ms before the next reading
}
