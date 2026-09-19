#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>
#include "esp_timer.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

// Variables for screen and button control
int currentScreen = 0;             // 0 = MPU6050, 1 = BMP180
const unsigned long debounceDelay = 200; // Debounce protection delay (ms)

esp_timer_handle_t sensor_timer;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_MPU6050 mpu;
Adafruit_BMP085 bmp;


// Flag indicating that the timer has fired
volatile bool timerFlag = false;
volatile bool buttonPressedFlag = false;
volatile unsigned long lastInterruptTime = 0;

// SHORT interrupt service routine (ISR)
void IRAM_ATTR onButtonPress() {
  unsigned long currentTime = millis();
  // Time-based debounce filter directly in the ISR
  if (currentTime - lastInterruptTime > debounceDelay) { // 200 ms debounce protection
    buttonPressedFlag = true;
    lastInterruptTime = currentTime;
  }
}

// Function called by the hardware timer on schedule
void IRAM_ATTR onTimer(void* arg) {
  timerFlag = true; // Simply set the flag when the timer fires
}

#define SDA_PIN 8
#define SCL_PIN 9

// Sensor pins
#define SENSORS_SDA 4
#define SENSORS_SCL 5

// Button pin
#define BUTTON_PIN 10

void setup() {
  // Put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Hello, ESP32-S3!");

  // Configure the button pin with the internal PULLUP resistor
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  // Attach an interrupt to the button pin on the falling edge
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), onButtonPress, FALLING);

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }

  Wire1.begin(SENSORS_SDA, SENSORS_SCL);

  display.clearDisplay();
  display.setTextSize(1);           
  display.setTextColor(SSD1306_WHITE);      
  display.setCursor(0,0); 
  display.println("Starting...");
  display.display();

  // Start the MPU-6050
  if (!mpu.begin(0x68, &Wire1)) { // Specify address 0x68 and our Wire bus
    Serial.println("Error: MPU6050 not found!");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("MPU6050 ERROR!");
    display.display();
    for (;;);
  }

  if (!bmp.begin(0x77, &Wire1)) {
    Serial.println("Error: BMP180 not found on Wire1!");
    for (;;);
  }

  // Configure MPU6050 sensitivity
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Configure the hardware timer
  const esp_timer_create_args_t timer_args = {
    .callback = &onTimer,
    .name = "sensor_timer"
  };

  esp_timer_create(&timer_args, &sensor_timer);

  // Start the periodic timer at 500,000 microseconds (500 ms = 0.5 s)
  esp_timer_start_periodic(sensor_timer, 500000);


  display.setCursor(2, 16);
  display.print("Setup done");
  // display.println(ssid);
  display.display();
  Serial.println("Setup done");

  delay(1000);
}

void sensorsRead()
{
  // Display data according to the selected mode
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  if (currentScreen == 0) {
    // ===== SCREEN 1: MPU6050 =====
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Output to the Serial Monitor
    Serial.printf("t=%lu ms ax=%.2f ay=%.2f az=%.2f mode=MPU6050\n", 
              millis(), a.acceleration.x, a.acceleration.y, a.acceleration.z);

    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("--- MPU6050 Data ---");

    // Acceleration (m/s²)
    display.setCursor(0, 16);
    display.print("AccX: "); display.println(a.acceleration.x, 1);
    display.setCursor(0, 28);
    display.print("AccY: "); display.println(a.acceleration.y, 1);
    display.setCursor(0, 40);
    display.print("AccZ: "); display.println(a.acceleration.z, 1);

    // Temperature
    display.setCursor(0, 53);
    display.printf("Temp: %.2f C", temp.temperature);

  } else if (currentScreen == 1) {
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("--- BMP180 Data ---");
    // ===== SCREEN 2: BMP180 =====
    float temp_bmp = bmp.readTemperature();
    float pressure_hpa = bmp.readPressure() / 100.0F; // Convert Pa to hPa
    float altitude = bmp.readAltitude();

    Serial.printf("t=%lu ms temp=%.2f press=%.2f alt=%.2f mode=bmp180\n", 
                    millis(), temp_bmp, pressure_hpa, altitude);
    // Row 4: Temperature
    display.setCursor(0, 16);
    display.printf("Temp: %.2f C", temp_bmp);

    // Row 5: Pressure (hPa)
    display.setCursor(0, 28);
    display.printf("Press: %.2f hPa", pressure_hpa);

    // Row 6: Altitude (meters)
    display.setCursor(0, 40);
    display.printf("Alt: %.2f m", altitude);
  }


  display.display();
}

void handleUartCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim(); // Remove extra spaces and newline characters

    if (command == "mode") {
      currentScreen = (currentScreen + 1) % 2;
      Serial.printf("OK: mode switched to: %d\n", currentScreen);
      sensorsRead();
    } else if (command.startsWith("p ")) {
      // Example timer period changes: "p 200" or "p 1000"
      int newPeriodMs = command.substring(2).toInt();
      if (newPeriodMs >= 100 && newPeriodMs <= 5000) {
        esp_timer_stop(sensor_timer); // sensor_timer must be a global variable
        esp_timer_start_periodic(sensor_timer, newPeriodMs * 1000ULL);
        Serial.printf("OK: period changed to %d ms\n", newPeriodMs);
      } else {
        Serial.println("ERR: period out of range (100-5000)");
      }
    } else if (command.length() > 0) {
      Serial.println("ERR: unknown command");
    }
  }
}

void loop() {
  handleUartCommands();

  // Handle button presses
  if (buttonPressedFlag) {
    buttonPressedFlag = false; // Reset the flag

    // Change the screen/mode
    currentScreen = (currentScreen + 1) % 2;
    Serial.printf("OK: mode switched to: %d\n", currentScreen);

    // Update the screen immediately after the button press
    sensorsRead();
  }

  // Handle the periodic timer
  if (timerFlag) {
    timerFlag = false;
    sensorsRead();
  }

  // Yield to the operating system to reduce processor power consumption
  vTaskDelay(10 / portTICK_PERIOD_MS);

  // delay(200); // Update every 0.2 seconds
}
