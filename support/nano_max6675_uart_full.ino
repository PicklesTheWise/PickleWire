#include <SPI.h>
#include <max6675.h>

// Pin definitions
int thermoCLK = 13;
int thermoCS  = 10;
int thermoDO  = 12;

// Initialize MAX6675 library
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

void setup() {
  Serial.begin(115200); // Match ESP32-S3 baud rate
  delay(500);
}

void loop() {
  // Check if a request character is available
  if (Serial.available()) {
    char req = Serial.read();
    if (req == 'R') {
      double c = thermocouple.readCelsius();
      if (isnan(c)) {
        Serial.println("TEMP: nan");
      } else {
        Serial.print("TEMP: ");
        Serial.println(c, 2);
      }
    }
  }
  delay(10); // Small delay to avoid busy loop
}
