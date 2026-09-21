//USED AI FOR THIS TRIAL CODE
#include <SPI.h>
#include <SD.h>

// -------------------------
// Pin definitions
// -------------------------

const int echoPin = 2;
const int trigPin = 3;

const int chipSelect = 4;

// -------------------------
// Setup
// -------------------------

void setup() {
  Serial.begin(9600);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial.println("Starting SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization FAILED!");
    
    while (1) {
      // Stop here if SD card fails
    }
  }

  Serial.println("SD card initialized successfully!");

  // Create/open the file
  File dataFile = SD.open("distance.txt", FILE_WRITE);

  if (dataFile) {
    dataFile.println("Reading,Distance_cm");
    dataFile.close();

    Serial.println("distance.txt ready.");
  }
  else {
    Serial.println("Error opening distance.txt");
  }
}

// -------------------------
// Main loop
// -------------------------

void loop() {

  // Send ultrasonic trigger pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  // Measure echo duration
  long duration = pulseIn(echoPin, HIGH);

  // Calculate distance
  float distance = duration * 0.0343 / 2.0;

  // Display on Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Save to SD card
  File dataFile = SD.open("distance.txt", FILE_WRITE);

  if (dataFile) {

    static unsigned long readingNumber = 0;
    readingNumber++;

    dataFile.print(readingNumber);
    dataFile.print(",");
    dataFile.println(distance);

    dataFile.close();

    Serial.println("Data saved.");
  }
  else {
    Serial.println("ERROR: Could not open distance.txt");
  }

  // Wait 100 ms before next measurement
  delay(100);
}
