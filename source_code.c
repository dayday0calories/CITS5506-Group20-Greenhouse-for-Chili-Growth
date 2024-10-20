#define BLYNK_TEMPLATE_ID "TMPL6Wz86sixo"          // Blynk Template ID (from your Blynk project)
#define BLYNK_TEMPLATE_NAME "Quickstart Template"  // Blynk Template Name
#define BLYNK_AUTH_TOKEN "YourAuthTokenHere"       // Blynk Auth Token (replace with your actual token)

#include <Wire.h>               // For I2C communication
#include <Adafruit_Sensor.h>    // Base class for sensor libraries
#include <Adafruit_BME280.h>    // Library for BME280 sensor (temperature, humidity, pressure)
#include <WiFi.h>               // For Wi-Fi connectivity
#include <BlynkSimpleEsp32.h>   // Blynk library for ESP32
#include <Adafruit_VEML7700.h>  // Library for VEML7700 light sensor

// Define GPIO pins for controlling fan, heater, and reading soil moisture
#define RELAY_PIN_FAN 23      // GPIO pin connected to relay controlling the fan
#define RELAY_PIN_HEATER 19   // GPIO pin connected to relay controlling the heater
#define SOIL_MOISTURE_PIN 36  // GPIO36 (VP) - Analog input pin connected to soil moisture sensor

// Define GPIO pins for controlling water pump and light bulb
#define PUMP_RELAY_PIN 26  // GPIO26 connected to relay controlling the water pump
#define LIGHT_BULB_PIN 25  // GPIO25 connected to relay controlling the light bulb

// Define temperature thresholds (in degrees Celsius)
#define TEMP_HIGH_THRESHOLD 30.0  // If temperature exceeds this value, turn on the fan
#define TEMP_LOW_THRESHOLD 25.0   // If temperature falls below this value, turn on the heater

// Soil moisture percentage thresholds (values between 0% and 100%)
#define SOIL_MOISTURE_MIN_PERCENT 0       // Minimum soil moisture percentage (completely dry)
#define SOIL_MOISTURE_OPT_LOW_PERCENT 40  // Lower bound of optimal soil moisture percentage
#define SOIL_MOISTURE_MAX_PERCENT 100     // Maximum soil moisture percentage (completely wet)

// Light intensity threshold (in lux) for the plant
#define LIGHT_INTENSITY_THRESHOLD 400  // If light intensity falls below this value, turn on the light bulb

// I2C address for the BME280 sensor
#define BME280_ADDRESS 0x76  // Default I2C address for BME280 sensor

// Create sensor objects
Adafruit_BME280 bme;            // BME280 sensor object for temperature and humidity
Adafruit_VEML7700 lightSensor;  // VEML7700 light sensor object for ambient light

// Global variables to store system states
bool autoMode = true;          // Flag to indicate whether system is in automatic mode
int pumpState = 0;             // Current state of water pump (0: OFF, 1: ON)
int lightState = 0;            // Current state of light bulb (0: OFF, 1: ON)
bool bme280Available = false;  // Flag to indicate whether BME280 sensor is available

// Wi-Fi credentials
char ssid[] = "YourNetworkSSID";      // Your Wi-Fi SSID (replace with your network SSID)
char pass[] = "YourNetworkPassword";  // Your Wi-Fi Password (replace with your network password)

// Soil moisture sensor calibration values (raw ADC values)
// Adjust these based on your specific sensor's calibration
int soilMoistureMinRaw = 3000;  // Raw ADC value when soil is completely dry
int soilMoistureMaxRaw = 1500;  // Raw ADC value when soil is completely wet

void setup() {
  Serial.begin(115200);  // Initialize serial communication at 115200 baud rate
  Serial.println("Setup started");

  // Initialize relay pins as outputs to control devices
  pinMode(RELAY_PIN_FAN, OUTPUT);
  pinMode(RELAY_PIN_HEATER, OUTPUT);
  pinMode(PUMP_RELAY_PIN, OUTPUT);
  pinMode(LIGHT_BULB_PIN, OUTPUT);
  Serial.println("Relay pins initialized");

  // Initialize the soil moisture sensor pin as input
  pinMode(SOIL_MOISTURE_PIN, INPUT);
  Serial.println("Soil moisture sensor pin initialized");

  // Initialize the BME280 sensor for temperature and humidity
  Serial.println("Attempting to initialize BME280...");
  if (bme.begin(BME280_ADDRESS)) {  // Try to initialize BME280 sensor
    Serial.println("BME280 sensor initialized successfully");
    bme280Available = true;  // Set flag to true if sensor is available
  } else {
    Serial.println("Could not find a valid BME280 sensor, check wiring or try a different address!");
    Serial.println("SensorID was: 0x" + String(bme.sensorID(), 16));
    // Provide possible reasons and sensor IDs
    Serial.println("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085");
    Serial.println("   ID of 0x56-0x58 represents a BMP 280,");
    Serial.println("        ID of 0x60 represents a BME 280.");
    Serial.println("        ID of 0x61 represents a BME 680.");
  }

  // Initialize the VEML7700 light sensor for ambient light measurement
  Serial.println("Attempting to initialize VEML7700 light sensor...");
  if (!lightSensor.begin()) {  // If initialization fails
    Serial.println("Failed to initialize VEML7700 light sensor!");
    while (1) {
      delay(1000);
      Serial.println("Light sensor initialization failed. System halted.");
    }
  }
  Serial.println("VEML7700 light sensor initialized successfully");

  // Configure light sensor settings
  lightSensor.setGain(VEML7700_GAIN_1);               // Set gain (1x)
  lightSensor.setIntegrationTime(VEML7700_IT_100MS);  // Set integration time (100ms)
  Serial.println("Light sensor settings configured");

  // Ensure all devices are turned off initially
  digitalWrite(RELAY_PIN_FAN, LOW);     // Turn off fan
  digitalWrite(RELAY_PIN_HEATER, LOW);  // Turn off heater
  digitalWrite(PUMP_RELAY_PIN, LOW);    // Turn off water pump
  digitalWrite(LIGHT_BULB_PIN, LOW);    // Turn off light bulb
  Serial.println("All devices set to initial OFF state");

  // Initialize connection to Blynk platform
  Serial.println("Attempting to connect to Blynk...");
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);  // Connect to Blynk using auth token and Wi-Fi credentials
  Serial.println("Blynk initialized");

  Serial.println("Setup completed");
}

// Blynk function to handle changes in Virtual Pin V2 (Auto/Manual mode toggle)
// When the user toggles the switch in the Blynk app connected to Virtual Pin V2,
// this function will be called, and the value will determine whether the system
// is in automatic mode or manual mode.
BLYNK_WRITE(V2) {
  autoMode = param.asInt();  // Read the value sent from Blynk app (1 for auto, 0 for manual)
  Serial.println(autoMode ? "Switched to Automatic Mode" : "Switched to Manual Mode");
}

// Blynk function to manually control the fan (Virtual Pin V0)
// This function allows the user to control the fan manually via the Blynk app.
// The function will only act if the system is in manual mode.
BLYNK_WRITE(V0) {
  if (!autoMode) {                          // Only allow manual control if in manual mode
    int fanState = param.asInt();           // Read fan state from Blynk app (1: ON, 0: OFF)
    digitalWrite(RELAY_PIN_FAN, fanState);  // Set the fan relay accordingly
    Serial.println(fanState == 1 ? "Fan ON (manual)" : "Fan OFF (manual)");
  }
}

// Blynk function to manually control the heater (Virtual Pin V1)
// This function allows the user to control the heater manually via the Blynk app.
BLYNK_WRITE(V1) {
  if (!autoMode) {                                // Only allow manual control if in manual mode
    int heaterState = param.asInt();              // Read heater state from Blynk app
    digitalWrite(RELAY_PIN_HEATER, heaterState);  // Set the heater relay accordingly
    Serial.println(heaterState == 1 ? "Heater ON (manual)" : "Heater OFF (manual)");
  }
}

// Blynk function to manually control the water pump (Virtual Pin V6)
// This function allows the user to control the water pump manually via the Blynk app.
BLYNK_WRITE(V6) {
  if (!autoMode) {                              // Only allow manual control if in manual mode
    int pumpControl = param.asInt();            // Read pump state from Blynk app
    digitalWrite(PUMP_RELAY_PIN, pumpControl);  // Set the pump relay accordingly
    pumpState = pumpControl;                    // Update pumpState variable
    Serial.println(pumpControl == 1 ? "Water Pump ON (manual)" : "Water Pump OFF (manual)");
  }
}

// Blynk function to manually control the light bulb (Virtual Pin V7)
// This function allows the user to control the light bulb manually via the Blynk app.
BLYNK_WRITE(V7) {
  if (!autoMode) {                               // Only allow manual control if in manual mode
    int lightControl = param.asInt();            // Read light control state from Blynk app
    digitalWrite(LIGHT_BULB_PIN, lightControl);  // Set the light bulb relay accordingly
    lightState = lightControl;                   // Update lightState variable
    Serial.println(lightControl == 1 ? "Light Bulb ON (manual)" : "Light Bulb OFF (manual)");
  }
}

void loop() {
  Serial.println("Entering loop");
  Blynk.run();  // Run Blynk process to handle communication with app

  // Variables to hold sensor readings
  float temperature = 0;  // Variable to hold temperature reading
  float humidity = 0;     // Variable to hold humidity reading

  // Check if BME280 sensor is available and read data
  if (bme280Available) {                  // If BME280 sensor is available
    temperature = bme.readTemperature();  // Read temperature from sensor
    humidity = bme.readHumidity();        // Read humidity from sensor

    // Print temperature and humidity readings to serial monitor
    Serial.print("Temperature = ");
    Serial.print(temperature);
    Serial.println(" *C");

    Serial.print("Humidity = ");
    Serial.print(humidity);
    Serial.println(" %");

    // Send temperature and humidity readings to Blynk app (Virtual Pins V3 and V4)
    Blynk.virtualWrite(V3, temperature);  // Update temperature in app
    Blynk.virtualWrite(V4, humidity);     // Update humidity in app
  } else {
    Serial.println("BME280 not available. Skipping temperature and humidity readings.");
  }

  // Read soil moisture sensor value (analog input)
  int soilMoistureRawValue = analogRead(SOIL_MOISTURE_PIN);  // Read raw ADC value
  // Map the raw sensor value to a percentage (0% to 100%)
  // Note: The map function is used to translate the raw value to a percentage based on calibration
  int soilMoisturePercent = map(soilMoistureRawValue, soilMoistureMinRaw, soilMoistureMaxRaw, 0, 100);
  // Ensure the percentage is within 0% to 100% range
  soilMoisturePercent = constrain(soilMoisturePercent, 0, 100);

  // Print soil moisture readings to serial monitor
  Serial.print("Soil Moisture Raw Value = ");
  Serial.println(soilMoistureRawValue);
  Serial.print("Soil Moisture Percentage = ");
  Serial.print(soilMoisturePercent);
  Serial.println(" %");

  // Read light intensity from the VEML7700 light sensor
  float lightIntensity = lightSensor.readLux();  // Read light intensity in lux
  Serial.print("Light Intensity = ");
  Serial.print(lightIntensity);
  Serial.println(" lux");

  // Send soil moisture and light intensity readings to Blynk app (Virtual Pins V5 and V8)
  Blynk.virtualWrite(V5, soilMoisturePercent);  // Update soil moisture in app
  Blynk.virtualWrite(V8, lightIntensity);       // Update light intensity in app

  if (autoMode) {  // If system is in automatic mode
    Serial.println("Running in Automatic Mode");
    if (bme280Available) {  // If BME280 sensor is available
      // Control fan based on temperature
      if (temperature > TEMP_HIGH_THRESHOLD) {  // If temperature exceeds high threshold
        digitalWrite(RELAY_PIN_FAN, HIGH);      // Turn on fan
        Serial.println("Fan ON (auto)");
      } else {
        digitalWrite(RELAY_PIN_FAN, LOW);  // Turn off fan
        Serial.println("Fan OFF (auto)");
      }

      // Control heater based on temperature
      if (temperature < TEMP_LOW_THRESHOLD) {  // If temperature is below low threshold
        digitalWrite(RELAY_PIN_HEATER, HIGH);  // Turn on heater
        Serial.println("Heater ON (auto)");
      } else {
        digitalWrite(RELAY_PIN_HEATER, LOW);  // Turn off heater
        Serial.println("Heater OFF (auto)");
      }
    } else {
      // If BME280 sensor is not available, ensure fan and heater are off
      digitalWrite(RELAY_PIN_FAN, LOW);     // Turn off fan
      digitalWrite(RELAY_PIN_HEATER, LOW);  // Turn off heater
      Serial.println("Fan and Heater OFF (BME280 not available)");
    }

    // Control water pump based on soil moisture
    if (soilMoisturePercent < SOIL_MOISTURE_OPT_LOW_PERCENT) {  // If soil moisture is below optimal lower bound
      digitalWrite(PUMP_RELAY_PIN, HIGH);                       // Turn on water pump
      pumpState = 1;                                            // Update pumpState variable to ON
      Serial.println("Water Pump ON (auto - soil too dry)");
    } else {
      digitalWrite(PUMP_RELAY_PIN, LOW);  // Turn off water pump
      pumpState = 0;                      // Update pumpState variable to OFF
      Serial.println("Water Pump OFF (auto - soil moisture sufficient)");
    }

    // Control light bulb based on light intensity
    if (lightIntensity < LIGHT_INTENSITY_THRESHOLD) {  // If light intensity is below threshold
      digitalWrite(LIGHT_BULB_PIN, HIGH);              // Turn on light bulb
      lightState = 1;                                  // Update lightState variable to ON
      Serial.println("Light Bulb ON (auto - low light intensity)");
    } else {
      digitalWrite(LIGHT_BULB_PIN, LOW);  // Turn off light bulb
      lightState = 0;                     // Update lightState variable to OFF
      Serial.println("Light Bulb OFF (auto - sufficient light)");
    }
  } else {
    // If system is in manual mode, devices are controlled via Blynk app
    Serial.println("Running in Manual Mode: Waiting for user input");
  }

  // Print current water pump state to serial monitor
  Serial.print("Water Pump State: ");
  Serial.println(pumpState == 1 ? "ON" : "OFF");

  // Update Blynk app with current states of devices
  Blynk.virtualWrite(V6, pumpState);   // Update water pump state (Virtual Pin V6)
  Blynk.virtualWrite(V7, lightState);  // Update light bulb state (Virtual Pin V7)

  // Update Blynk app with sensor availability status
  Blynk.virtualWrite(V9, bme280Available ? 1 : 0);  // Send 1 if BME280 is available, else 0 (Virtual Pin V9)

  Serial.println("Loop completed. Waiting for next iteration...");
  delay(6000);  // Wait for 6 seconds before next iteration to prevent rapid looping
}