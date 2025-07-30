# Temperature Sensor Circuit - AD8495 K-Type Thermocouple Amplifier

## Circuit Configuration

```
                    ESP32S3                           AD8495 K-Type Amplifier
                 ┌─────────────┐                    ┌──────────────────────────┐
                 │             │                    │                          │
        3.3V ────┤ 3.3V        │                    │ VCC ←─── 3.3V            │
                 │             │                    │                          │
                 │         GPIO18 ├────────────────→ VOUT (5mV/°C)            │
                 │             │                    │                          │
                 │         GND ├────────────────────┤ GND                      │
                 │             │                    │                          │
                 └─────────────┘                    │ +IN ←─── K+ (Hot Junction)│
                                                    │                          │
                                                    │ -IN ←─── K- (Cold Ref)   │
                                                    │                          │
                                                    └──────────────────────────┘
                                                              │        │
                                                              │        │
                                                         ┌────┴────┐   │
                                                         │K-Type   │   │
                                                         │Thermo-  │   │
                                                         │couple   │   │
                                                         │         │   │
                                                         └────┬────┘   │
                                                              │        │
                                                          Hot Wire ────┘
                                                          (Heated End)
```

## AD8495 Analysis

### Output Formula:
```
V_out = (Temperature_°C × 5mV) + 1.25V
V_out = (T × 0.005) + 1.25V
```

### Temperature Examples:
- **25°C**: V_out = (25 × 0.005) + 1.25 = **1.375V** (ADC ~1700)
- **200°C**: V_out = (200 × 0.005) + 1.25 = **2.25V** (ADC ~2782)  
- **400°C**: V_out = (400 × 0.005) + 1.25 = **3.25V** (ADC ~4020)
- **500°C**: V_out = (500 × 0.005) + 1.25 = **3.75V** (ADC ~4636 - exceeds 3.3V!)

### ADC Range Considerations:
- **ESP32 max input**: 3.3V
- **AD8495 at 410°C**: 3.3V (safe maximum)
- **Usable range**: 0°C to **410°C** with 3.3V supply

## Physical Connections

### ESP32S3 Pins:
- **GPIO18**: ADC input (connects to AD8495 VOUT)
- **3.3V**: Power supply for AD8495
- **GND**: Ground reference

### AD8495 Connections:
- **VCC**: 3.3V power supply
- **GND**: Ground
- **VOUT**: Temperature output (5mV/°C + 1.25V offset) → GPIO18
- **+IN**: K-type thermocouple positive (hot junction)
- **-IN**: K-type thermocouple negative (cold reference)

### Wiring:
1. **AD8495 VCC** connects to **ESP32 3.3V**
2. **AD8495 GND** connects to **ESP32 GND**
3. **AD8495 VOUT** connects to **ESP32 GPIO18**
4. **K-type thermocouple** connects to **AD8495 +IN and -IN**
5. **Thermocouple hot junction** placed at **hot wire**

## Code Configuration (AD8495 K-Type Thermocouple):
```cpp
#define TEMP_SENSOR_PIN       18        // GPIO18 for AD8495 output
#define AD8495_OFFSET_V       1.25      // 1.25V offset at 0°C
#define AD8495_SCALE_MV       5.0       // 5mV per °C
#define AD8495_VREF           3.3       // ESP32 ADC reference voltage
#define ADC_RESOLUTION        4095.0    // 12-bit ADC (0-4095)

// Temperature calculation function
float readThermocoupleTemp() {
  int adcReading = analogRead(TEMP_SENSOR_PIN);
  float voltage = (adcReading / ADC_RESOLUTION) * AD8495_VREF;
  float temperature = (voltage - AD8495_OFFSET_V) / (AD8495_SCALE_MV / 1000.0);
  return temperature;
}
```

## ✅ CIRCUIT ISSUE RESOLVED:
- **Problem**: GPIO18 had internal ADC2/RTC analog circuits causing resistance interference
- **Root Cause**: GPIO18 = ADC2_CH7 + RTC_GPIO18 with built-in analog input circuits
- **Effect**: Internal analog circuits created parallel resistance path (4.7kΩ external → 2.7kΩ effective)
- **Solution**: Used measured effective resistance (2.7kΩ) in software compensation
- **Result**: Accurate temperature readings using actual measured pullup resistance on GPIO18

### Code Changes:
```cpp
// Back to GPIO18 with measured compensation:
#define TEMP_SENSOR_PIN       18        // GPIO18 with 2.7k effective resistance
#define SERIES_RESISTOR       2700      // Use measured effective resistance

// In setup():
pinMode(TEMP_SENSOR_PIN, INPUT);
gpio_pullup_dis((gpio_num_t)TEMP_SENSOR_PIN);   // Disable internal pullup
gpio_pulldown_dis((gpio_num_t)TEMP_SENSOR_PIN); // Disable internal pulldown
analogReadResolution(12);
```

## How It Works:

1. **Voltage Divider**: The 4.7kΩ pullup and thermistor form a voltage divider
2. **Temperature Change**: As temperature increases, thermistor resistance decreases
3. **Voltage Change**: Lower thermistor resistance = lower voltage at GPIO18
4. **ADC Conversion**: ESP32S3 reads voltage and converts to 12-bit value (0-4095)
5. **Temperature Calculation**: Steinhart-Hart equation converts resistance to temperature

## Benefits of 4.7kΩ Pullup:
- **Optimal sensitivity** around 200°C operating temperature
- **Good resolution** for hot wire temperature control
- **Standard value** used in 3D printer applications
- **Matches thermistor specifications**
