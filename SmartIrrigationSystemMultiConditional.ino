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
// NOTE: These are used to get the reading, but the pin argument is removed from control_garden for clarity.
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

// ** NEW WEATHER-BASED THRESHOLDS (Worst Case / High Demand Scenario) **
// Irrigation ONLY occurs if ALL three conditions are met: Soil is Dry AND Temp is HIGH AND RH is LOW.
const float TEMP_HIGH_THRESHOLD = 23.0; // Temperature (C) for HIGH water loss
const int RH_LOW_THRESHOLD = 70;        // Relative Humidity (%) for HIGH water loss

// --- Global Variables ---
int moisture_1_value = 0;
int moisture_2_value = 0;
float temperature = 0.0;
float humidity = 0.0;


// Function to read moisture and control irrigation for a single garden
// NOTE: I've removed the unused 'moisturePin' argument for cleaner code.
void control_garden(int valvePin, int ledPin, int gardenId, int moistureReading, float temp, float hum) {
  
  // 1. Determine if the soil is dry enough to warrant watering
  bool isSoilDry = (moistureReading >= DRY_THRESHOLD);
  
  // 2. Determine if the atmospheric demand is HIGH (Worst Case Scenario)
  // HIGH DEMAND is defined as: Temp >= 23.0 AND Hum < 70
  bool isHighDemandWeather = (temp >= TEMP_HIGH_THRESHOLD && hum < RH_LOW_THRESHOLD);

  // Irrigation Decision:
  // We only irrigate if the soil is dry AND the weather conditions maximize water loss.
  if (isSoilDry && isHighDemandWeather) {
    // WORST CASE: Dry Soil + High Water Loss Weather -> IRRIGATE NOW
    
    digitalWrite(valvePin, LOW);  // Turn Solenoid Valve ON (Assuming Active-LOW Relay)
    digitalWrite(ledPin, HIGH);   // Turn Status LED ON
    
    Serial.print("Garden ");
    Serial.print(gardenId);
    Serial.print(" is DRY and HIGH DEMAND. Valve OPEN @ ");
    Serial.print(temp);
    Serial.println("C.");
    
  } else {
    // BEST CASE/OTHER CASES: Keep the valve closed to conserve water.
    // This includes: Wet Soil (Moist) OR Low/Moderate Demand Weather
    
    digitalWrite(valvePin, HIGH); // Turn Solenoid Valve OFF (Assuming Active-LOW Relay)
    digitalWrite(ledPin, LOW);    // Turn Status LED OFF

    Serial.print("Garden ");
    Serial.print(gardenId);
    Serial.print(" is not irrigating (");

    if (!isSoilDry) {
      Serial.print("Soil MOIST");
    } else {
      Serial.print("Soil DRY, but Low Demand");
    }
    Serial.println("). Valve CLOSED.");
  }
}

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
  
  // Line 2: Moisture Status for both Gardens - Now using combined logic status
  lcd.setCursor(0, 1);
  lcd.print("G1:");
  
  // Check the actual state of the valve, which reflects the combined logic
  if (digitalRead(VALVE_1_PIN) == LOW) {
    // Valve is OPEN (LOW) means it's currently Irrigating due to HIGH DEMAND
    lcd.print("IRRIGATING"); 
  } else {
    // Valve is CLOSED (HIGH) means the conditions are not met
    lcd.print("HOLD"); 
  }
  
  // Adjust position for Garden 2 display
  lcd.setCursor(11, 1);
  lcd.print("G2:");
  if (digitalRead(VALVE_2_PIN) == LOW) {
    lcd.print("IRR"); // Shortened for space
  } else {
    lcd.print("HOLD"); // Shortened for space
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("--- Automatic Irrigation System Initialized ---");
  Serial.println("Logic: IRRIGATE only if [Soil Dry] AND [Temp >= 23.0C AND Hum < 70%]");

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
  lcd.print("Smart Irrigation");
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
    Serial.println("Failed to read from DHT sensor! Using last valid data.");
    // Keep last valid values if possible, or use 0.0 (though it might trigger the HIGH demand logic if used as 0.0)
    // For simplicity, we skip if data is NaN, but we need T/H to run the logic.
    // If we can't read, it's safer to skip the cycle, or default to a "HOLD" state.
    // We will proceed, but the T/H check will rely on the last successful read or default 0.0.
    
    // To ensure safety, we will assume "Best Case" (LOW Demand) if we cannot read T/H
    temperature = TEMP_HIGH_THRESHOLD - 1.0; // Forces weather check to fail (LOW Demand)
    humidity = RH_LOW_THRESHOLD + 1;         // Forces weather check to fail (LOW Demand)
    
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
  
  // Garden 1 Control
  // Pass current T/H readings to the control function
  control_garden(VALVE_1_PIN, LED_1_PIN, 1, moisture_1_value, temperature, humidity);

  // Garden 2 Control
  // Pass current T/H readings to the control function
  control_garden(VALVE_2_PIN, LED_2_PIN, 2, moisture_2_value, temperature, humidity);

  // --- 4. Update LCD Display ---
  update_lcd();

  // Wait for the next cycle
  delay(CYCLE_DELAY);
}
