// Simple MAX6675 SPI Test for ESP32-S3
// This isolates just the SPI communication to diagnose the issue

#include <Arduino.h>
#include "driver/spi_master.h"

// MAX6675 pin assignments (avoiding conflicts)
#define MAX6675_PIN_CS   18    // Changed from 15 to avoid RC conflict
#define MAX6675_PIN_CLK  43    // SCK
#define MAX6675_PIN_SO   44    // MISO/SO

#define MAX6675_SPI_HOST SPI3_HOST

spi_device_handle_t max6675_spi;

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("=== MAX6675 SPI Test ===");
  Serial.printf("Pins: CS=%d, CLK=%d, SO=%d\n", MAX6675_PIN_CS, MAX6675_PIN_CLK, MAX6675_PIN_SO);
  
  // Set up pins manually first
  pinMode(MAX6675_PIN_CS, OUTPUT);
  pinMode(MAX6675_PIN_CLK, OUTPUT);
  pinMode(MAX6675_PIN_SO, INPUT);
  digitalWrite(MAX6675_PIN_CS, HIGH);
  
  Serial.println("Pins configured");
  
  // Test SO pin response
  Serial.println("Testing SO pin response...");
  digitalWrite(MAX6675_PIN_CS, LOW);
  delayMicroseconds(100);
  int so_low = digitalRead(MAX6675_PIN_SO);
  digitalWrite(MAX6675_PIN_CS, HIGH);
  delayMicroseconds(100);
  int so_high = digitalRead(MAX6675_PIN_SO);
  Serial.printf("SO pin: CS_LOW=%d, CS_HIGH=%d\n", so_low, so_high);
  
  // Initialize SPI
  Serial.println("Initializing SPI...");
  
  spi_bus_config_t buscfg = {};
  buscfg.mosi_io_num = -1;
  buscfg.miso_io_num = MAX6675_PIN_SO;
  buscfg.sclk_io_num = MAX6675_PIN_CLK;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;
  buscfg.max_transfer_sz = 16;
  
  esp_err_t ret = spi_bus_initialize(MAX6675_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
  Serial.printf("spi_bus_initialize: %d (%s)\n", ret, ret == ESP_OK ? "OK" : "ERROR");
  
  if (ret != ESP_OK) {
    Serial.println("SPI bus init failed!");
    return;
  }
  
  spi_device_interface_config_t devcfg = {};
  devcfg.clock_speed_hz = 250000; // Even slower - 250kHz
  devcfg.mode = 0;
  devcfg.spics_io_num = MAX6675_PIN_CS;
  devcfg.queue_size = 1;
  devcfg.flags = 0; // Remove SPI_DEVICE_NO_DUMMY
  
  ret = spi_bus_add_device(MAX6675_SPI_HOST, &devcfg, &max6675_spi);
  Serial.printf("spi_bus_add_device: %d (%s)\n", ret, ret == ESP_OK ? "OK" : "ERROR");
  
  if (ret != ESP_OK) {
    Serial.println("SPI device add failed!");
    return;
  }
  
  Serial.println("SPI initialized successfully");
  Serial.println("Starting temperature readings...");
}

float readMAX6675() {
  uint8_t rx_data[2] = {0};
  
  spi_transaction_t trans = {};
  trans.length = 16; // 16 bits
  trans.rx_buffer = rx_data;
  trans.flags = 0; // No special flags
  
  esp_err_t ret = spi_device_transmit(max6675_spi, &trans);
  
  Serial.printf("SPI Result: ret=%d, bytes=[0x%02X, 0x%02X]\n", 
                ret, rx_data[0], rx_data[1]);
  
  if (ret != ESP_OK) {
    return NAN;
  }
  
  uint16_t raw = (rx_data[0] << 8) | rx_data[1];
  Serial.printf("Raw value: 0x%04X (%d)\n", raw, raw);
  
  // Check for thermocouple fault
  if (raw & 0x04) {
    Serial.println("Thermocouple fault detected!");
    return NAN;
  }
  
  // Extract temperature (bits 15-3, multiply by 0.25)
  float temp = ((raw >> 3) & 0xFFF) * 0.25;
  
  return temp;
}

void loop() {
  Serial.println("\n--- Reading MAX6675 ---");
  
  float temp = readMAX6675();
  
  if (isnan(temp)) {
    Serial.println("Temperature: ERROR/NAN");
  } else {
    Serial.printf("Temperature: %.2f°C\n", temp);
  }
  
  delay(2000);
}
