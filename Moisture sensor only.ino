// --- Capacitive Moisture Sensor Test Sketch ---
// This code reads the raw analog value from a capacitive soil moisture sensor
// and prints it to the Serial Monitor for calibration and testing.

// Define the Analog pin where the sensor's analog output (AOUT) is connected.
// A0 is a common default for testing.
const int MOISTURE_PIN = A0; 

// Define the time delay between readings (in milliseconds)
const int READING_INTERVAL = 500; // 0.5 seconds

void setup() {
  // Initialize serial communication at 9600 baud rate.
  // This is required to see the output in the Arduino Serial Monitor.
  Serial.begin(9600);
  Serial.println("Capacitive Moisture Sensor Reader Initialized.");
  Serial.println("Moisture Sensor Values (0-1023):");
  // No pinMode is required for analog inputs.
}

void loop() {
  // Read the raw analog value from the sensor
  int moisture_reading = analogRead(MOISTURE_PIN);

  // Print the value to the Serial Monitor
  // Note: Higher values usually mean DRY soil, and lower values mean WET soil.
  Serial.print("Raw Reading: ");
  Serial.println(moisture_reading);

  // Wait for the defined interval before taking the next reading
  delay(READING_INTERVAL);
}
