// Required Libraries
// 1. LiquidCrystal_I2C: For the 16x2 I2C LCD
// 2. DHT sensor library by Adafruit: For the DHT11 sensor

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

// --- Hardware Pin Definitions ---

// DHT Sensor
#define DHTPIN 2      // Digital pin connected to the DHT sensor data pin
#define DHTTYPE DHT11 // DHT 11 sensor type

// Solenoid Valves (Relay Switches - Check if Active-LOW or Active-HIGH)
const int VALVE_1_PIN = 3; // Digital pin for Garden 1 Solenoid Valve
const int VALVE_2_PIN = 4; // Digital pin for Garden 2 Solenoid Valve

// Status LEDs
const int LED_1_PIN = 5; // Digital pin for Garden 1 Status LED (Irrigating)
const int LED_2_PIN = 6; // Digital pin for Garden 2 Status LED (Irrigating)

// Capacitive Moisture Sensors
const int MOISTURE_1_PIN = A2; // Analog pin for Garden 1 Sensor
const int MOISTURE_2_PIN = A3; // Analog pin for Garden 2 Sensor

// LCD I2C Connections (SCL to A5, SDA to A4 - standard Uno I2C pins)
// LCD I2C Address (Commonly 0x27 or 0x3F - Check your module if 0x27 fails)
const int LCD_I2C_ADDRESS = 0x27; 
LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, 16, 2);

// Initialize DHT sensor
DHT dht(DHTPIN, DHTTYPE);

// --- Configuration Variables ---

// Calibration Note: Capacitive sensors read HIGHER values when DRY, and LOWER values when WET.
// This threshold (600) means any reading >= 600 is considered DRY.
// YOU MUST CALIBRATE THIS VALUE FOR YOUR SOIL TYPE.
const int DRY_THRESHOLD = 600; 

// Time delay between reading cycles (in milliseconds)
const int CYCLE_DELAY = 5000; // 5 seconds wait

// --- Global Variables ---
int moisture_1_value = 0;
int moisture_2_value = 0;
float temperature = 0.0;
float humidity = 0.0;


// NOTE: The 'control_garden' function has been removed. 
// All control logic is now implemented directly in 'loop()' using two independent 'if/else' blocks.


// Function to update the 16x2 LCD display
void update_lcd() {
  // Clear the screen for fresh data
  lcd.clear();
  
  // Line 1: Environmental Data (Temp and Humidity)
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(temperature, 1); // Display temperature with 1 decimal place
  lcd.print((char)223); // Degree symbol
  lcd.print("C H:");
  lcd.print(humidity, 1);  // Display humidity with 1 decimal place
  lcd.print("%");
  
  // Line 2: Moisture Status for both Gardens - Now using only DRY or WET state
  lcd.setCursor(0, 1);
  lcd.print("M1:");
  
  // Check the actual state of the valve
  if (digitalRead(VALVE_1_PIN) == LOW) {
    lcd.print("DRY"); // Garden 1 is currently irrigating (DRY state)
  } else {
    lcd.print("WET"); // Garden 1 is sufficiently moist (WET state)
  }
  
  lcd.setCursor(9, 1);
  lcd.print("M2:");
  if (digitalRead(VALVE_2_PIN) == LOW) {
    lcd.print("DRY"); // Garden 2 is currently irrigating (DRY state)
  } else {
    lcd.print("WET"); // Garden 2 is sufficiently moist (WET state)
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("--- Automatic Irrigation System Initialized ---");
  Serial.println("Logic: IRRIGATE only if [Soil Moisture is DRY]. Logic for Garden 1 and 2 is fully independent.");

  // Initialize Outputs (Valves and LEDs)
  pinMode(VALVE_1_PIN, OUTPUT);
  pinMode(VALVE_2_PIN, OUTPUT);
  pinMode(LED_1_PIN, OUTPUT);
  pinMode(LED_2_PIN, OUTPUT);

  // Ensure all valves are OFF and LEDs are OFF at startup
  digitalWrite(VALVE_1_PIN, HIGH); // OFF (Active-LOW)
  digitalWrite(VALVE_2_PIN, HIGH); // OFF (Active-LOW)
  digitalWrite(LED_1_PIN, LOW);    // OFF
  digitalWrite(LED_2_PIN, LOW);    // OFF

  // Initialize the DHT sensor
  dht.begin();
  
  // Initialize the LCD
  lcd.init();
  lcd.backlight(); // Turn on the backlight
  
  // Initial LCD message
  lcd.print("Initializing...");
  lcd.setCursor(0, 1);
  lcd.print("System Ready.");
  delay(2000);
}

void loop() {
  // --- 1. Read Environmental Data (Temperature and Humidity) ---
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();

  // Check if any reads failed and handle the error
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
    temperature = 0.0; 
    humidity = 0.0;
  }
  
  // --- 2. Read Soil Moisture Sensors ---
  moisture_1_value = analogRead(MOISTURE_1_PIN);
  moisture_2_value = analogRead(MOISTURE_2_PIN);

  // Print raw data to Serial Monitor for debugging and calibration
  Serial.print("\n[T/H]: ");
  Serial.print(temperature);
  Serial.print("C | ");
  Serial.print(humidity);
  Serial.print("% | [M1/M2]: ");
  Serial.print(moisture_1_value);
  Serial.print(" / ");
  Serial.println(moisture_2_value);

  // --- 3. Apply Irrigation Logic and Control Outputs ---
  
  // =========================================================
  // Garden 1 Control Logic (Independent If/Else Block)
  // =========================================================
  if (moisture_1_value >= DRY_THRESHOLD) {
    // Soil is Dry - Start Irrigation
    digitalWrite(VALVE_1_PIN, LOW);  // Turn Solenoid Valve ON (Active-LOW Relay)
    digitalWrite(LED_1_PIN, HIGH);   // Turn Status LED ON
    
    Serial.println("Garden 1 is DRY. Valve OPEN.");
  } else {
    // Soil is Wet/Moist enough - Stop Irrigation
    digitalWrite(VALVE_1_PIN, HIGH); // Turn Solenoid Valve OFF (Active-LOW Relay)
    digitalWrite(LED_1_PIN, LOW);    // Turn Status LED OFF

    Serial.println("Garden 1 is MOIST. Valve CLOSED.");
  }

  // =========================================================
  // Garden 2 Control Logic (Fully Independent If/Else Block)
  // =========================================================
  if (moisture_2_value >= DRY_THRESHOLD) {
    // Soil is Dry - Start Irrigation
    digitalWrite(VALVE_2_PIN, LOW);  // Turn Solenoid Valve ON (Active-LOW Relay)
    digitalWrite(LED_2_PIN, HIGH);   // Turn Status LED ON
    
    Serial.println("Garden 2 is DRY. Valve OPEN.");
  } else {
    // Soil is Wet/Moist enough - Stop Irrigation
    digitalWrite(VALVE_2_PIN, HIGH); // Turn Solenoid Valve OFF (Active-LOW Relay)
    digitalWrite(LED_2_PIN, LOW);    // Turn Status LED OFF

    Serial.println("Garden 2 is MOIST. Valve CLOSED.");
  }


  // --- 4. Update LCD Display ---
  update_lcd();

  // Wait for the next cycle
  delay(CYCLE_DELAY);
}
