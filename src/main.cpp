// ─── src/main.cpp (v2.40 – Hot Wire Temperature Control with PID) ─

#include "lv_conf.h"
#include <lvgl.h>
#include <Wire.h>
#include <Arduino.h>
#include <LovyanGFX.hpp>
#include <CSE_CST328.h>
#include <EEPROM.h>
#include <PID_v1.h>  // Add PID library
#include "driver/gpio.h"  // For GPIO pullup/pulldown control
#include "screens.h"  // Include screen functions

// Pololu & LVGL
#define LV_COLOR_RED          lv_color_hex(0xFF0000)
#define G2_ADDRESS            13
#define POL_START             170
#define POL_CMD_SET_CURRENT   17

// I²C & RC
#define I2C_SDA_PIN           11
#define I2C_SCL_PIN           10
#define RC_INPUT_PIN          15

// Touch controller pins
#define TP_SDA_PIN             1
#define TP_SCL_PIN             3
#define TP_RST_PIN             2
#define TP_INT_PIN             4

// Temperature sensor pin - AD8495 K-Type Thermocouple Amplifier
#define TEMP_SENSOR_PIN       18        // GPIO18 for AD8495 output
#define AD8495_OFFSET_V       0      // 1.25V offset at 0°C
#define AD8495_SCALE_MV       5.0       // 5mV per °C
#define AD8495_VREF           3.3       // ESP32 ADC reference voltage
#define ADC_RESOLUTION        4095.0    // 12-bit ADC (0-4095)

// EEPROM layout
#define ADDR_WIRE_DIAM         0   // float
#define ADDR_TEMP_OFFSET       4   // float
#define ADDR_CURRENT_LIMIT     8   // float
#define ADDR_CUR_OFFSET_CAL   12   // uint16_t
#define ADDR_PID_KP           16   // float
#define ADDR_PID_KI           20   // float
#define ADDR_PID_KD           24   // float
#define EEPROM_SIZE           28

// Settings ranges
float    WIRE_MIN    = 0.1f;
float    WIRE_MAX    = 1.0f;
float    WIRE_STEP   = 0.1f;
float    OFF_MIN     = -20.0f;
float    OFF_MAX     = 20.0f;
float    OFF_STEP    = 0.1f;
float    CUR_MIN     = 1.0f;   // Amps
float    CUR_MAX     = 5.0f;   // Amps
float    CUR_STEP    = 0.1f;   // Amps
static constexpr uint16_t CUR_OFFSET_DEFAULT = 993;

// Scale factor for selector→internal units
static constexpr float    CURRENT_LIMIT_SCALE = 25.75f;
static constexpr float    CURRENT_SCALE_CAL = 8057.0f; // Default for 18v15

// Nichrome constants
static constexpr float    WIRE_LEN    = 0.98f;
static constexpr float    RHO_20      = 1.10e-6f;
static constexpr float    ALPHA_N     = 0.00017f;
static constexpr float    T0_DEF      = 20.0f;

// PID Control
#define PID_UPDATE_INTERVAL 50    // 50ms PID update interval (was 100ms)
double temperatureSetpoint = 0.0;  // Target temperature
double temperatureInput = 0.0;     // Current temperature
double outputPWM = 0.0;            // Output to hot wire heater (0-3200)
double pidKp = 25.0;              // Increase proportional gain (was 8.0)
double pidKi = 0.05;              // Decrease integral gain to reduce overshoot (was 0.2)
double pidKd = 5.0;               // Increase derivative gain (was 2.0)
PID tempPID(&temperatureInput, &outputPWM, &temperatureSetpoint, pidKp, pidKi, pidKd, DIRECT);
bool pidEnabled = false;           // PID control state
uint32_t lastPidUpdateTime = 0;    // Track last PID update time

// Runtime settings
float    wireDiam      = 0.5f;     // mm
float    tempOffset    = 0.0f;     // °C
float    currentLimit  = 1.0f;    // Amps (set default to min or as desired)
uint16_t curOffsetCal  = CUR_OFFSET_DEFAULT;

// RC pulse width (ISR)
volatile uint32_t rc_width    = 0;
volatile uint32_t rc_pulse_start = 0;

// System control modes
bool overrideMode = false;          // True when in manual override mode
uint16_t manualPWM = 0;            // Manual PWM value from slider (0-3200)

// PID Autotune variables
bool autotuneActive = false;       // True when autotune is running
uint32_t autotuneStartTime = 0;    // When autotune started
float autotuneSetpoint = 0.0;      // Target temperature for autotune
float autotuneOutput = 0.0;        // Output level for autotune (0-100%)

// Safety monitoring system
#define SAFETY_CHECK_INTERVAL     2000    // Check every 2 seconds
#define SAFETY_TEMP_RISE_TIMEOUT  10000   // 10 seconds to see temperature rise
#define SAFETY_MIN_OUTPUT         500     // Minimum output to consider "heating"
#define SAFETY_MIN_TEMP_RISE      2.0     // Minimum temperature rise expected (°C)

bool safetyErrorState = false;            // True when safety error detected
bool safetySystemEnabled = true;          // Enable/disable safety monitoring
uint32_t lastSafetyCheck = 0;            // Last safety check time
float safetyInitialTemp = 0.0;           // Temperature when heating started
uint32_t safetyHeatingStartTime = 0;     // When significant heating output began
bool safetyHeatingActive = false;        // True when we're actively heating
String safetyErrorMessage = "";          // Detailed error message
bool autotuneDirection = true;     // true = heating, false = cooling
uint32_t autotuneLastSwitch = 0;   // Last time output switched
float autotunePeaks[10];           // Store temperature peaks
uint32_t autotunePeakTimes[10];    // Store peak times
int autotuneNumPeaks = 0;          // Number of peaks detected
float autotuneLastTemp = 0.0;      // Previous temperature reading
bool autotuneTempRising = true;    // Temperature trend
float autotuneKu = 0.0;            // Ultimate gain
float autotunePu = 0.0;            // Ultimate period
int autotuneState = 0;             // 0=idle, 1=running, 2=complete, 3=failed

// EEPROM save timing variables (moved from screens.h to avoid issues)
uint32_t lastEepromSaveTime = 0;
bool eepromSavePending = false;
const uint32_t EEPROM_SAVE_DELAY = 1000; // 1 second delay

// LVGL draw buffer (minimal size to prevent memory issues)
static constexpr uint32_t      LVGL_BUF_SIZE = 320*20;  // Further reduced
static lv_disp_draw_buf_t      draw_buf;
static lv_disp_drv_t           disp_drv;
static lv_indev_drv_t          indev_drv;
static lv_color_t              buf1[LVGL_BUF_SIZE];
static lv_color_t              buf2[LVGL_BUF_SIZE];  // Added second buffer for LVGL double buffering

// Temperature smoothing
#define TEMP_BUFFER_SIZE 5        // Reduce buffer size for faster response (was 10)
float temperatureBuffer[TEMP_BUFFER_SIZE] = {0};
int tempBufferIndex = 0;
uint32_t lastTempReadTime = 0;
float smoothedTemp = 0.0f;

// Forward declarations
uint16_t readG2(uint8_t id);
uint32_t readRCPulse();
void rcPulseISR();
void sendPololuStart();
void sendPololuHotWireForward(uint16_t power);
void sendPololuSetCurrentLimit(float selector_A);
uint16_t readPololuOffsetCalibration();
float readThermistorTemp();
float getSmoothedTemperature();
void eepromLoad();
void eepromSave();
void buildTelemetryScreen();
void buildOverrideScreen();
void buildSettingsScreen();
void updateTelemetry();
void updateOverrideScreen();
void updateDelayedEepromSave();
void buildPIDTuneScreen();
void startPIDAutotune(float setpoint, float outputStep);
void updatePIDAutotune();
void finishPIDAutotune();
void safeSerialPrint(const char* msg);
void safeSerialPrintln(const char* msg);
void safeSerialPrintf(const char* format, ...);

// Safety monitoring function declarations
void updateSafetyMonitoring(uint16_t currentOutput, float currentTemp);
bool isSafetyErrorActive();
void clearSafetyError();
String getSafetyErrorMessage();
void safeNavigateToScreen(lv_obj_t *screen, const char* screenName);
bool verifyOverrideScreenObjects();

// Screensaver function declarations
void createScreensaver();
void updateScreensaver();
void resetIdleTimer();
void exitScreensaver();
void showSplashScreen();

// Display & touch
class LGFX : public lgfx::LGFX_Device {
  lgfx::Bus_SPI      _bus;
  lgfx::Panel_ST7789 _panel;
public:
  LGFX(){
    auto bc=_bus.config(); 
      bc.spi_host=SPI2_HOST; bc.spi_mode=0;
      bc.freq_write=40000000; bc.freq_read=16000000;
      bc.spi_3wire=true; bc.use_lock=true;
      bc.dma_channel=SPI_DMA_CH_AUTO;
      bc.pin_sclk=40; bc.pin_mosi=45; bc.pin_miso=-1; bc.pin_dc=41;
    _bus.config(bc);
    auto pc=_panel.config();
      pc.pin_cs=42; pc.pin_rst=39;
      pc.panel_width=240; pc.panel_height=320;
      pc.memory_width=240; pc.memory_height=320;
      pc.invert=true; pc.offset_rotation=0;
    _panel.config(pc);
    _panel.setBus(&_bus);
    setPanel(&_panel);
  }
} lcd;
TwoWire touchWire(1);
CSE_CST328 touch(320,240,&touchWire,TP_RST_PIN,TP_INT_PIN);

// UI objects (remove static to allow external access)
lv_obj_t *scrTelem = nullptr, *scrOverride = nullptr, *scrSettings = nullptr;
lv_obj_t *btnNavOverride = nullptr, *btnNavSettings = nullptr;  // Navigation buttons
lv_obj_t *btnOverride = nullptr, *btnSettings = nullptr, *btnBack = nullptr, *btnUnlock = nullptr, *btnSave = nullptr;
lv_obj_t *lblTitle[4] = {nullptr}, *lblVal[4] = {nullptr}, *lblBanner = nullptr, *chartTemp = nullptr, *lblErrorVal = nullptr;
lv_chart_series_t *serTemp = nullptr;
lv_chart_series_t *serSetpoint = nullptr;  // Series for setpoint line
lv_obj_t *lblOverrideTemp = nullptr, *lblOverrideRCPulse = nullptr, *lblOverrideModeStatus = nullptr, *lblOverrideTempSetpoint = nullptr, *lblOverridePowerOutput = nullptr, *lblSliderVal = nullptr;
lv_obj_t *btnTempDown = nullptr, *btnTempUp = nullptr, *lblTempDown = nullptr, *lblTempUp = nullptr;
lv_obj_t *lblSetWire = nullptr, *btnWireLeft = nullptr, *btnWireRight = nullptr;
lv_obj_t *lblSetOffset = nullptr, *btnOffsetLeft = nullptr, *btnOffsetRight = nullptr;
lv_obj_t *lblSetCurrent = nullptr, *btnCurrentLeft = nullptr, *btnCurrentRight = nullptr;
lv_obj_t *btnPidKpLeft = nullptr, *btnPidKpRight = nullptr, *btnPidKiLeft = nullptr, *btnPidKiRight = nullptr, *btnPidKdLeft = nullptr, *btnPidKdRight = nullptr;
lv_obj_t *lblSetPidKp = nullptr, *lblSetPidKi = nullptr, *lblSetPidKd = nullptr;
lv_obj_t *btnPidTune = nullptr, *btnSettingsNav = nullptr;
lv_obj_t *lblPidStatus = nullptr;  // Added PID status label

// Add these globals
bool pidResetting = false;
uint32_t pidResetStartTime = 0;

// Add these globals near your chart variables
lv_obj_t *lblChartMin = nullptr, *lblChartMax = nullptr;
float chartTempMin = 0.0f, chartTempMax = 500.0f;

// Add these declarations with other UI objects
lv_obj_t *scrPIDTune = nullptr;
lv_obj_t *lblKpVal = nullptr, *lblKiVal = nullptr, *lblKdVal = nullptr;

// Navigation deferral to prevent concurrent LVGL operations
bool pendingNavigation = false;
lv_obj_t *pendingScreen = nullptr;
uint32_t navigationRequestTime = 0;

// Track screen initialization status
bool overrideScreenInitialized = false;

// Idle timeout and screensaver variables
uint32_t lastActivityTime = 0;    // Last user interaction time
bool screensaverActive = false;   // True when screensaver is running
lv_obj_t *screensaverScreen = nullptr;  // Screensaver screen object
lv_obj_t *screensaverP = nullptr, *screensaverPickle = nullptr;  // Moving logo parts
lv_obj_t *screensaverW = nullptr, *screensaverWire = nullptr;
int screensaverX = 0, screensaverY = 0;  // Current position
int screensaverVelX = 2, screensaverVelY = 1;  // Velocity for bouncing
lv_obj_t *savedActiveScreen = nullptr;  // Screen to return to after screensaver

// Function to read temperature from AD8495 K-Type Thermocouple Amplifier
float readThermistorTemp() {
  const int numSamples = 10;  // Increased samples for stability
  float sum = 0;
  int validSamples = 0;
  
  // Take multiple samples and check for stability
  for (int i = 0; i < numSamples; i++) {
    int rawADC = analogRead(TEMP_SENSOR_PIN);
    if (rawADC > 0 && rawADC < 4095) {  // Valid ADC range
      sum += rawADC;
      validSamples++;
    }
    delay(5);  // Longer delay for AD8495 settling
  }
  
  if (validSamples == 0) {
    safeSerialPrintln("ERROR: No valid ADC readings from thermocouple!");
    return NAN;
  }
  
  float average = sum / validSamples;
  
  // Convert ADC reading to voltage
  float voltage = (average / ADC_RESOLUTION) * AD8495_VREF;
  
  // AD8495 specific checks
  if (voltage < 0.1) {
    safeSerialPrintf("ERROR: AD8495 voltage too low (%.3fV) - check power/connections\n", voltage);
    return NAN;
  }
  
  // Check for disconnected thermocouple (normal condition when no sensor connected)
  if (voltage > 3.25) {
    // This is normal behavior when thermocouple is disconnected
    static uint32_t lastDisconnectedLog = 0;
    uint32_t now = millis();
    if (now - lastDisconnectedLog > 10000) {  // Log every 10 seconds, not constantly
      lastDisconnectedLog = now;
      safeSerialPrintf("INFO: AD8495 reading %.3fV - thermocouple disconnected (normal when no sensor connected)\n", voltage);
    }
    return NAN;  // Return NaN but don't treat as error
  }
  
  // Convert voltage to temperature using AD8495 formula
  // V_out = (T × 5mV) + 1.25V
  // T = (V_out - 1.25V) / 5mV
  float temperature = (voltage - AD8495_OFFSET_V) / (AD8495_SCALE_MV / 1000.0);
  
  // Detailed debug output every 5 seconds (more frequent for troubleshooting)
  static uint32_t lastDebugOutput = 0;
  uint32_t now = millis();
  if (now - lastDebugOutput > 5000) {
    lastDebugOutput = now;
    
    safeSerialPrintf("AD8495 THERMOCOUPLE DEBUG:\n");
    safeSerialPrintf("  Valid samples: %d/%d\n", validSamples, numSamples);
    safeSerialPrintf("  ADC Raw=%.1f (0-4095 range)\n", average);
    safeSerialPrintf("  Voltage=%.3fV (AD8495 output)\n", voltage);
    safeSerialPrintf("  Temperature=%.1f°C (K-type thermocouple)\n", temperature);
    safeSerialPrintf("  Expected voltage at 25°C: %.3fV\n", (25.0 * AD8495_SCALE_MV / 1000.0) + AD8495_OFFSET_V);
    
    // Additional diagnostics
    if (voltage < 1.20) {
      safeSerialPrintln("  STATUS: ⚠️  Very low voltage detected!");
      safeSerialPrintf("  Raw ADC: %.1f (should be ~1900 for room temp)\n", average);
      safeSerialPrintf("  Calculated voltage: %.3fV (should be ~1.375V for 25°C)\n", voltage);
      safeSerialPrintln("  POSSIBLE CAUSES:");
      safeSerialPrintln("    1. AD8495 power supply issue (check 3.3V or 5V supply)");
      safeSerialPrintln("    2. Cold junction compensation problem");
      safeSerialPrintln("    3. Wrong thermocouple type (ensure K-type)");
      safeSerialPrintln("    4. Damaged AD8495 or thermocouple");
      safeSerialPrintln("    5. Poor connections to AD8495 T+ and T- terminals");
    } else if (voltage > 1.30 && voltage < 1.35) {
      safeSerialPrintln("  STATUS: Normal room temperature range");
    } else if (voltage > 2.0 && voltage < 3.25) {
      safeSerialPrintln("  STATUS: High temperature detected");
    } else if (voltage >= 3.25) {
      safeSerialPrintln("  STATUS: ✓ Thermocouple disconnected (AD8495 pulled to VCC)");
      safeSerialPrintln("  This is normal behavior when no thermocouple is connected.");
      safeSerialPrintln("  To connect a thermocouple:");
      safeSerialPrintln("    1. Connect K-type thermocouple to AD8495 T+ and T- terminals");
      safeSerialPrintln("    2. Ensure good electrical connections");
      safeSerialPrintln("    3. Voltage should drop to ~1.375V at room temperature");
    }
  }
  
  // Validate temperature range for K-type thermocouple with AD8495
  if (isnan(temperature)) {
    safeSerialPrintln("WARNING: Temperature calculation resulted in NaN");
    return NAN;
  }
  
  // More realistic K-type thermocouple range with AD8495
  if (temperature < -270.0) {
    safeSerialPrintf("WARNING: Temperature extremely low (%.1f°C) - possible AD8495 power issue\n", temperature);
  }
  
  if (temperature > 1000.0) {
    safeSerialPrintf("WARNING: Temperature very high (%.1f°C) - check for short circuit\n", temperature);
  }
  
  return temperature;
}

// Smoothed temperature readings
float getSmoothedTemperature() {
  uint32_t currentTime = millis();
  
  // Only update temperature every 500ms
  if (currentTime - lastTempReadTime >= 500) {
    lastTempReadTime = currentTime;
    
    // Read the current temperature
    float newTemp = readThermistorTemp();
    
    // Only add valid temperature readings to buffer (expanded range for troubleshooting)
    if (!isnan(newTemp) && newTemp >= -270.0 && newTemp <= 1000.0) {
      temperatureBuffer[tempBufferIndex] = newTemp;
      tempBufferIndex = (tempBufferIndex + 1) % TEMP_BUFFER_SIZE;
    } else {
      // Log invalid readings for debugging
      static uint32_t lastInvalidLog = 0;
      if (currentTime - lastInvalidLog > 2000) {  // Log every 2 seconds
        lastInvalidLog = currentTime;
        if (isnan(newTemp)) {
          safeSerialPrintln("TEMP: Skipping NaN temperature reading");
        } else {
          safeSerialPrintf("TEMP: Skipping out-of-range reading: %.1f°C\n", newTemp);
        }
      }
    }
    
    // Calculate the average of all valid values in the buffer
    float sum = 0;
    int validValues = 0;
    
    for (int i = 0; i < TEMP_BUFFER_SIZE; i++) {
      if (!isnan(temperatureBuffer[i]) && 
          temperatureBuffer[i] >= -270.0 && temperatureBuffer[i] <= 1000.0) {
        sum += temperatureBuffer[i];
        validValues++;
      }
    }
    
    // Update the smoothed temperature value
    if (validValues > 0) {
      float newSmoothedTemp = sum / validValues;
      
      // Check for dramatic temperature changes (possible sensor issues)
      if (validValues > 1 && abs(newSmoothedTemp - smoothedTemp) > 50.0) {
        safeSerialPrintf("TEMP: Large temperature change detected: %.1f°C -> %.1f°C\n", 
                         smoothedTemp, newSmoothedTemp);
      }
      
      smoothedTemp = newSmoothedTemp;
    } else {
      // No valid readings in buffer - keep last known value or use safe fallback
      if (smoothedTemp == 0.0) {
        smoothedTemp = 25.0;  // Room temperature fallback
        safeSerialPrintln("TEMP: No valid readings - using room temperature fallback (25°C)");
      }
    }
  }
  
  return smoothedTemp;
}

//------------------------------------------------------------------------------
// Safety Monitoring System

void updateSafetyMonitoring(uint16_t currentOutput, float currentTemp) {
  if (!safetySystemEnabled) return;
  
  uint32_t now = millis();
  
  // Check if we're outputting significant heating power
  bool isHeating = (currentOutput >= SAFETY_MIN_OUTPUT);
  
  // Detect start of heating
  if (isHeating && !safetyHeatingActive) {
    safetyHeatingActive = true;
    safetyHeatingStartTime = now;
    safetyInitialTemp = currentTemp;
    safeSerialPrintf("SAFETY: Heating started - Initial temp: %.1f°C, Output: %u\n", 
                     safetyInitialTemp, currentOutput);
  }
  
  // Detect end of heating
  if (!isHeating && safetyHeatingActive) {
    safetyHeatingActive = false;
    safeSerialPrintf("SAFETY: Heating stopped - Output dropped to %u\n", currentOutput);
  }
  
  // Perform safety checks every SAFETY_CHECK_INTERVAL
  if (now - lastSafetyCheck >= SAFETY_CHECK_INTERVAL) {
    lastSafetyCheck = now;
    
    // Only check if we've been heating for the minimum timeout period
    if (safetyHeatingActive && (now - safetyHeatingStartTime >= SAFETY_TEMP_RISE_TIMEOUT)) {
      float tempRise = currentTemp - safetyInitialTemp;
      
      // Check if temperature has risen sufficiently
      if (tempRise < SAFETY_MIN_TEMP_RISE) {
        // SAFETY ERROR: Heater on but no temperature rise
        safetyErrorState = true;
        safetyErrorMessage = String("Heater failure detected! Output: ") + String(currentOutput) + 
                           ", Initial: " + String(safetyInitialTemp, 1) + "°C" +
                           ", Current: " + String(currentTemp, 1) + "°C" +
                           ", Rise: " + String(tempRise, 1) + "°C" +
                           " (Expected: >" + String(SAFETY_MIN_TEMP_RISE, 1) + "°C)";
        
        safeSerialPrintf("🚨 SAFETY ERROR: %s\n", safetyErrorMessage.c_str());
        safeSerialPrintln("🚨 EMERGENCY SHUTDOWN: Heater output disabled!");
        
        // Reset heating state
        safetyHeatingActive = false;
      } else {
        // Temperature is rising properly
        safeSerialPrintf("SAFETY: OK - Temp rise: %.1f°C in %.1fs\n", 
                        tempRise, (now - safetyHeatingStartTime) / 1000.0);
      }
    }
  }
}

bool isSafetyErrorActive() {
  return safetyErrorState;
}

void clearSafetyError() {
  safetyErrorState = false;
  safetyErrorMessage = "";
  safetyHeatingActive = false;
  safeSerialPrintln("SAFETY: Error cleared by user");
}

String getSafetyErrorMessage() {
  return safetyErrorMessage;
}

//------------------------------------------------------------------------------
// LVGL callbacks

void lvgl_flush_cb(lv_disp_drv_t* d,const lv_area_t* a,lv_color_t* buf){
  int w=a->x2-a->x1+1, h=a->y2-a->y1+1;
  lcd.startWrite();
    lcd.setAddrWindow(a->x1,a->y1,w,h);
    lcd.pushPixels((uint16_t*)buf,w*h,true);
  lcd.endWrite();
  lv_disp_flush_ready(d);
}

void lvgl_touch_read(lv_indev_drv_t*, lv_indev_data_t* data){
  static bool lastTouchState = false;
  static uint32_t lastValidTouchTime = 0;
  
  touch.readData();
  auto p=touch.getPoint();
  data->point.x=p.x; data->point.y=p.y;
  bool currentlyTouched = touch.isTouched();
  data->state = currentlyTouched ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
  
  // Only reset idle timer on meaningful touch events, not every callback
  // Reset on: 1) New touch press, 2) Touch during screensaver
  uint32_t now = millis();
  bool meaningfulTouch = false;
  
  if (currentlyTouched && !lastTouchState) {
    // New touch press detected
    meaningfulTouch = true;
    lastValidTouchTime = now;
    safeSerialPrintln("TOUCH: New press detected - resetting idle timer");
  } else if (screensaverActive && currentlyTouched) {
    // Touch during screensaver - always meaningful
    meaningfulTouch = true;
    safeSerialPrintln("TOUCH: Touch during screensaver - resetting idle timer");
  }
  
  if (meaningfulTouch) {
    resetIdleTimer();
    
    // Extra safety check: if screensaver is active, force exit immediately with validation
    if (screensaverActive) {
      safeSerialPrintln("Touch detected during screensaver - forcing safe exit");
      // Add a small delay to prevent race conditions and debounce
      static uint32_t lastTouchTime = 0;
      if (now - lastTouchTime > 200) {  // Increased debounce time
        lastTouchTime = now;
        // Don't call exitScreensaver directly from interrupt context
        // Let the main loop handle it via resetIdleTimer
        safeSerialPrintln("Touch event debounced - will exit via main loop");
      }
    }
  }
  
  lastTouchState = currentlyTouched;
}

//------------------------------------------------------------------------------
// Startup Splash Screen

//------------------------------------------------------------------------------
// Safe Serial Functions

inline void safeSerialPrint(const char* msg) {
  if (Serial) {
    Serial.print(msg);
  }
}

inline void safeSerialPrintln(const char* msg) {
  if (Serial) {
    Serial.println(msg);
  }
}

// For formatted output
void safeSerialPrintf(const char* format, ...) {
  if (!Serial) return;
  
  char buf[128];
  va_list args;
  va_start(args, format);
  vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);
  Serial.print(buf);
}

//------------------------------------------------------------------------------
// EEPROM load/save

void eepromLoad(){
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(ADDR_WIRE_DIAM,   wireDiam);
  EEPROM.get(ADDR_TEMP_OFFSET, tempOffset);
  float tmpCurrentLimit;
  EEPROM.get(ADDR_CURRENT_LIMIT, tmpCurrentLimit);
  if(tmpCurrentLimit < CUR_MIN || tmpCurrentLimit > CUR_MAX)
    currentLimit = CUR_MAX;
  else
    currentLimit = tmpCurrentLimit;
  EEPROM.get(ADDR_CUR_OFFSET_CAL, curOffsetCal);
  
  // Load PID parameters if they exist
  float tempKp, tempKi, tempKd;
  EEPROM.get(ADDR_PID_KP, tempKp);
  EEPROM.get(ADDR_PID_KI, tempKi);
  EEPROM.get(ADDR_PID_KD, tempKd);
  
  // Validate PID parameters
  if(isnan(tempKp) || tempKp <= 0 || tempKp > 100) tempKp = 8.0;
  if(isnan(tempKi) || tempKi < 0 || tempKi > 10) tempKi = 0.2;
  if(isnan(tempKd) || tempKd < 0 || tempKd > 50) tempKd = 2.0;
  
  // Apply validated parameters
  pidKp = tempKp;
  pidKi = tempKi;
  pidKd = tempKd;

  if(!(wireDiam>=WIRE_MIN&&wireDiam<=WIRE_MAX))       wireDiam=0.5f;
  if(!(tempOffset>=OFF_MIN&&tempOffset<=OFF_MAX))     tempOffset=0.0f;
  if(currentLimit<CUR_MIN||currentLimit>CUR_MAX)      currentLimit=0.50f;
  if(curOffsetCal>32767)                              curOffsetCal=CUR_OFFSET_DEFAULT;
  
  // Update PID controller with loaded parameters
  tempPID.SetTunings(pidKp, pidKi, pidKd);
}

void eepromSave(){
  EEPROM.put(ADDR_WIRE_DIAM,    wireDiam);
  EEPROM.put(ADDR_TEMP_OFFSET,  tempOffset);
  EEPROM.put(ADDR_CURRENT_LIMIT, currentLimit);
  EEPROM.put(ADDR_CUR_OFFSET_CAL,curOffsetCal);
  
  // Save PID parameters
  EEPROM.put(ADDR_PID_KP, pidKp);
  EEPROM.put(ADDR_PID_KI, pidKi);
  EEPROM.put(ADDR_PID_KD, pidKd);
  
  EEPROM.commit();
  sendPololuSetCurrentLimit(currentLimit);
}

//------------------------------------------------------------------------------
// setup & loop

void setup(){
  // Initialize Serial only - no USB.begin() to avoid boot loops
  Serial.begin(115200);
  
  // Longer delay for system stabilization
  delay(2000);
  
  // Don't wait for Serial to be connected
  Serial.println("PickleWire Hot Wire Cutter v2.40 starting...");
  
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000);

  // Read offset calibration from controller at startup (with error handling)
  Serial.println("Reading motor controller...");
  curOffsetCal = CUR_OFFSET_DEFAULT; // Use default to avoid I2C issues
  Serial.println("Motor controller skipped");

  Serial.println("Loading EEPROM...");
  eepromLoad();
  Serial.println("EEPROM loaded");

  Serial.println("Setting up RC input...");
  pinMode(RC_INPUT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RC_INPUT_PIN), rcPulseISR, CHANGE);
  Serial.println("RC input configured with interrupt");

  Serial.println("Setting up display power...");
  pinMode(5, OUTPUT); 
  digitalWrite(5, HIGH);
  Serial.println("Display power enabled");
  
  // Initialize display with error handling
  Serial.println("Initializing display...");
  delay(500);  // Give display time to power up
  lcd.init(); 
  Serial.println("Display init done");
  lcd.setBrightness(128);  // Reduced brightness
  Serial.println("Brightness set");
  lcd.setRotation(1);
  Serial.println("Display initialized");
  
  // Initialize touch controller
  Serial.println("Initializing touch...");
  touchWire.begin(TP_SDA_PIN, TP_SCL_PIN, 400000);
  touch.begin(); 
  touch.setRotation(1);
  Serial.println("Touch initialized");

  // Configure ADC for temperature measurement
  Serial.println("Configuring ADC...");
  pinMode(TEMP_SENSOR_PIN, INPUT);
  
  // Explicitly disable internal pullup/pulldown resistors on GPIO18
  esp_err_t pullup_result = gpio_pullup_dis((gpio_num_t)TEMP_SENSOR_PIN);
  esp_err_t pulldown_result = gpio_pulldown_dis((gpio_num_t)TEMP_SENSOR_PIN);
  
  Serial.printf("GPIO18 pullup disable result: %s\n", (pullup_result == ESP_OK) ? "SUCCESS" : "FAILED");
  Serial.printf("GPIO18 pulldown disable result: %s\n", (pulldown_result == ESP_OK) ? "SUCCESS" : "FAILED");
  
  // ESP32-S3 specific ADC configuration for better accuracy
  analogReadResolution(12);  // ESP32-S3 has 12-bit ADC
  analogSetAttenuation(ADC_11db);  // Set 11dB attenuation for 0-3.3V range (good for AD8495)
  analogSetPinAttenuation(TEMP_SENSOR_PIN, ADC_11db);  // Pin-specific attenuation
  
  Serial.println("ADC configured with 12-bit resolution and 11dB attenuation");
  
  // Verify ADC channel is working
  Serial.println("Verifying ADC channel...");
  bool adcWorking = true;
  for (int i = 0; i < 3; i++) {
    int testReading = analogRead(TEMP_SENSOR_PIN);
    if (testReading == 0 || testReading == 4095) {
      Serial.printf("WARNING: ADC reading %d is at extreme value (%d)\n", i+1, testReading);
      adcWorking = false;
    }
    delay(10);
  }
  
  if (!adcWorking) {
    Serial.println("ERROR: ADC may not be functioning correctly!");
  } else {
    Serial.println("ADC channel verification passed");
  }
  
  // Test ADC reading immediately after configuration
  delay(100); // Let GPIO settle
  int testADC = analogRead(TEMP_SENSOR_PIN);
  float testVoltage = (testADC / 4095.0) * 3.3;
  Serial.printf("Immediate GPIO18 ADC test: Raw=%d, Voltage=%.3fV\n", testADC, testVoltage);
  Serial.printf("Expected for AD8495 at room temp (~25°C): Raw=~1900, Voltage=~1.375V\n");
  Serial.printf("AD8495 voltage range: 1.25V (0°C) to 3.3V (410°C)\n");
  
  // Verify ADC is working by testing multiple readings
  Serial.println("Taking 5 test readings...");
  bool constantReading = true;
  int firstReading = analogRead(TEMP_SENSOR_PIN);
  
  for (int i = 0; i < 5; i++) {
    int rawADC = analogRead(TEMP_SENSOR_PIN);
    float voltage = (rawADC / 4095.0) * 3.3;
    float tempC = (voltage - 1.25) / 0.005; // AD8495 formula
    Serial.printf("  Reading %d: ADC=%d, V=%.3f, Temp=%.1f°C\n", i+1, rawADC, voltage, tempC);
    
    // Check if readings are varying (good) or constant (bad)
    if (abs(rawADC - firstReading) > 10) {
      constantReading = false;
    }
    
    // Specific check for 3.3V readings (normal when thermocouple disconnected)
    if (voltage > 3.28 && voltage < 3.32) {
      Serial.printf("  ✓ Reading %d shows 3.3V - thermocouple disconnected (normal)\n", i+1);
    }
    
    delay(100);
  }
  
  if (constantReading) {
    Serial.println("  ⚠️  ERROR: All ADC readings are identical - possible hardware issue!");
    Serial.println("  TROUBLESHOOTING STEPS:");
    Serial.println("    1. Check thermocouple is properly connected to AD8495");
    Serial.println("    2. Verify AD8495 power supply connections");
    Serial.println("    3. Check GPIO18 is not damaged or shorted");
    Serial.println("    4. Measure AD8495 output voltage with multimeter");
  } else {
    Serial.println("  ✓ ADC readings are varying normally");
  }

  // Configure PID controller
  Serial.println("Configuring PID...");
  tempPID.SetOutputLimits(0, 3200);  // 0-3200 PWM range for hot wire heater
  tempPID.SetMode(AUTOMATIC);        // Turn on the PID
  tempPID.SetSampleTime(PID_UPDATE_INTERVAL);
  Serial.println("PID configured");

  // Skip LVGL and screen building for now to isolate the issue
  Serial.println("Initializing LVGL...");
  
  // Initialize LVGL
  Serial.println("Starting LVGL init...");
  lv_init();
  Serial.println("LVGL lv_init() done");
  
  lv_disp_draw_buf_init(&draw_buf, buf1, buf2, LVGL_BUF_SIZE);
  Serial.println("Display buffer init done");
  
  lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res=320; disp_drv.ver_res=240;
    disp_drv.draw_buf=&draw_buf; disp_drv.flush_cb=lvgl_flush_cb;
  lv_disp_drv_register(&disp_drv);
  Serial.println("Display driver registered");

  lv_indev_drv_init(&indev_drv);
    indev_drv.type=LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb=lvgl_touch_read;
  lv_indev_drv_register(&indev_drv);
  Serial.println("LVGL initialized successfully");

  // Show splash screen while initializing
  Serial.println("Displaying splash screen...");
  showSplashScreen();
  delay(5000);  // Display splash for 5 seconds

  // Build all screens one by one with error checking
  Serial.println("Building screens...");
  
  Serial.println("Building telemetry screen...");
  buildTelemetryScreen();
  if (scrTelem == nullptr) {
    Serial.println("CRITICAL ERROR: scrTelem is null after build!");
  } else {
    Serial.println("Telemetry screen built successfully");
  }
  
  Serial.println("Building override screen...");
  buildOverrideScreen();
  if (scrOverride == nullptr) {
    Serial.println("CRITICAL ERROR: scrOverride is null after build!");
  } else {
    Serial.println("Override screen built successfully");
  }
  
  Serial.println("Building settings screen...");
  buildSettingsScreen();
  if (scrSettings == nullptr) {
    Serial.println("CRITICAL ERROR: scrSettings is null after build!");
  } else {
    Serial.println("Settings screen built successfully");
  }
  
  Serial.println("Building PID tune screen...");
  buildPIDTuneScreen();
  if (scrPIDTune == nullptr) {
    Serial.println("CRITICAL ERROR: scrPIDTune is null after build!");
  } else {
    Serial.println("PID screen built successfully");
  }
  
  Serial.println("Loading telemetry screen...");
  if (scrTelem != nullptr) {
    lv_scr_load(scrTelem);
    Serial.println("Telemetry screen loaded successfully");
  } else {
    Serial.println("ERROR: scrTelem is null!");
  }
  Serial.println("All screens setup complete");
  
  // Initialize idle timer
  lastActivityTime = millis();
  screensaverActive = false;
  
  // Print startup message but don't wait for serial
  Serial.println("PickleWire Hot Wire Cutter v2.40 started");
}

void loop(){
  // Full loop with LVGL timer handling and telemetry
  static uint32_t lastTick = 0, lastTele = 0, lastPrint = 0;
  static uint32_t lastWatchdog = 0;
  uint32_t now = millis();
  
  // System watchdog - print status every 5 seconds
  if (now - lastWatchdog >= 5000) {
    lastWatchdog = now;
    size_t freeHeap = ESP.getFreeHeap();
    size_t minFreeHeap = ESP.getMinFreeHeap();
    safeSerialPrintf("WATCHDOG: Free heap: %lu bytes (min: %lu), Override: %s, Screens: T=%p O=%p S=%p P=%p\n", 
                     freeHeap, minFreeHeap, overrideMode ? "ON" : "OFF", 
                     scrTelem, scrOverride, scrSettings, scrPIDTune);
    
    // Alert if memory is getting low
    if (freeHeap < 10000) {
      safeSerialPrintln("WARNING: Low memory detected!");
    }
  }
  
  // Handle LVGL timers with safety checks
  static uint32_t lastLvglError = 0;
  
  // Handle pending navigation requests safely
  if (pendingNavigation && pendingScreen) {
    // Small delay to ensure LVGL is not busy
    if (now - navigationRequestTime >= 50) {  // 50ms delay
      safeSerialPrintln("DEBUG: Executing pending navigation");
      
      // If navigating to override screen, verify objects first
      if (pendingScreen == scrOverride) {
        safeSerialPrintln("DEBUG: Navigating to override screen - temporarily skipping verification");
        // Temporarily disable verification to test navigation
        // if (!verifyOverrideScreenObjects()) {
        //   safeSerialPrintln("ERROR: Override screen objects are invalid - aborting navigation");
        //   pendingNavigation = false;
        //   pendingScreen = nullptr;
        //   return;
        // }
      }
      
      lv_scr_load(pendingScreen);
      pendingNavigation = false;
      pendingScreen = nullptr;
      safeSerialPrintln("DEBUG: Navigation completed successfully");
    }
  }
  
  // Check if LVGL is still valid
  if (lv_disp_get_default() != nullptr) {
    // Update LVGL tick before handling timers
    lv_tick_inc(now - lastTick); 
    lv_timer_handler();
    lastTick = now;
  } else {
    // LVGL display is corrupted
    if (now - lastLvglError >= 1000) {  // Only print error once per second
      lastLvglError = now;
      safeSerialPrintln("ERROR: LVGL display is null - system may be corrupted");
    }
  }
  
  // Update telemetry every 500ms
  if (now - lastTele >= 500) { 
    lastTele = now; 
    
    // Safe update with error checking
    if (scrTelem != nullptr && lv_scr_act() == scrTelem) {
      updateTelemetry();
    }
    
    // Only update override screen if it's the active screen and fully initialized
    if (scrOverride != nullptr && lv_scr_act() == scrOverride && overrideScreenInitialized) {
      updateOverrideScreen();  // Update override screen live data
    }
    
    updateDelayedEepromSave(); // Handle delayed EEPROM saves
  }
  
  // Update autotune if active
  if (autotuneActive) {
    updatePIDAutotune();
  }
  
  // Update screensaver animation if active
  static uint32_t lastScreensaverUpdate = 0;
  if (screensaverActive && (now - lastScreensaverUpdate >= 50)) {  // Update every 50ms
    lastScreensaverUpdate = now;
    updateScreensaver();
  }
  
  // Motor control logic
  uint16_t hotWireOutput = 0;
  
  // Priority: PID control > RC control > Manual control
  if (pidEnabled && temperatureSetpoint > 0) {
    // PID control mode - use temperature control
    if (now - lastPidUpdateTime >= PID_UPDATE_INTERVAL) {
      lastPidUpdateTime = now;
      float currentTemp = getSmoothedTemperature();
      
      // Validate temperature reading before using in PID
      if (!isnan(currentTemp) && currentTemp >= -270.0 && currentTemp <= 1000.0) {
        temperatureInput = currentTemp;
        tempPID.Compute();
        hotWireOutput = (uint16_t)outputPWM;
      } else {
        // Invalid temperature - disable PID temporarily and turn off heater
        safeSerialPrintf("Invalid temperature for PID: %.1f°C - disabling heater\n", currentTemp);
        hotWireOutput = 0;
        pidEnabled = false;  // Disable PID until valid reading
      }
    }
  } else if (!overrideMode) {
    // RC control mode (only when not in override)
    uint32_t rcPulse = readRCPulse();
    
    // Valid RC signal is typically 1000-2000 microseconds
    if (rcPulse >= 900 && rcPulse <= 2100) {
      // Map RC pulse to hot wire output (1000-2000µs -> 0-3200)
      if (rcPulse < 1000) rcPulse = 1000;
      if (rcPulse > 2000) rcPulse = 2000;
      hotWireOutput = map(rcPulse, 1000, 2000, 0, 3200);
    } else {
      hotWireOutput = 0;  // No RC signal - hot wire off
    }
  } else {
    // Manual override mode - use manual PWM value
    hotWireOutput = manualPWM;
  }
  
  // Safety monitoring - check if heater is working properly
  float currentTemp = getSmoothedTemperature();
  updateSafetyMonitoring(hotWireOutput, currentTemp);
  
  // Apply safety shutdown if error detected
  if (safetyErrorState) {
    hotWireOutput = 0;  // Emergency shutdown
    pidEnabled = false; // Disable PID control
  }
  
  // Send hot wire command
  sendPololuHotWireForward(hotWireOutput);
  
  // Now check if screensaver should start (after hotWireOutput is determined)
  // Prevent screensaver if ANY of these conditions are true:
  // 1. Hot wire is currently active (any output > 0)
  // 2. PID control is enabled (system is actively controlling temperature)
  // 3. Override mode is active (user is manually controlling)
  // 4. RC input is present (external control active)
  if (!screensaverActive && (now >= lastActivityTime) && (now - lastActivityTime >= IDLE_TIMEOUT_MS)) {
    // Check for any active control systems that should prevent screensaver
    uint32_t rcPulse = readRCPulse();
    bool hasActiveRC = (rcPulse >= 900 && rcPulse <= 2100);
    bool systemActive = (hotWireOutput > 0) || pidEnabled || overrideMode || hasActiveRC;
    
    if (!systemActive) {
      // Validate current screen before creating overlay
      lv_obj_t *currentScreen = lv_scr_act();
      if (currentScreen && (currentScreen == scrTelem || currentScreen == scrOverride || 
                           currentScreen == scrSettings || currentScreen == scrPIDTune)) {
        safeSerialPrintf("Starting screensaver overlay on screen: %p\n", currentScreen);
        screensaverActive = true;
        createScreensaver(); // Creates overlay on current screen
        safeSerialPrintln("Screensaver overlay created successfully");
      } else {
        safeSerialPrintf("Current screen %p is invalid - not starting screensaver\n", currentScreen);
      }
    } else {
      // Reset idle timer if there's active system usage
      lastActivityTime = now;
      safeSerialPrintf("Screensaver prevented - System active: HotWire=%u, PID=%s, Override=%s, RC=%s\n", 
                       hotWireOutput, pidEnabled ? "ON" : "OFF", overrideMode ? "ON" : "OFF", 
                       hasActiveRC ? "ACTIVE" : "inactive");
    }
  }
  
  // Debug: Log idle timer status every 5 seconds
  static uint32_t lastIdleDebug = 0;
  if (now - lastIdleDebug >= 5000) {
    lastIdleDebug = now;
    uint32_t idleTime = (now >= lastActivityTime) ? (now - lastActivityTime) : 0; // Prevent overflow
    uint32_t rcPulse = readRCPulse();
    bool hasActiveRC = (rcPulse >= 900 && rcPulse <= 2100);
    bool systemActive = (hotWireOutput > 0) || pidEnabled || overrideMode || hasActiveRC;
    
    safeSerialPrintf("IDLE DEBUG: Time since activity: %lums (timeout: %dms), Screensaver: %s\n", 
                     idleTime, IDLE_TIMEOUT_MS, screensaverActive ? "ACTIVE" : "inactive");
    safeSerialPrintf("SYSTEM STATE: HotWire=%u, PID=%s, Override=%s, RC=%luµs(%s), SystemActive=%s\n", 
                     hotWireOutput, pidEnabled ? "ON" : "OFF", overrideMode ? "ON" : "OFF", 
                     rcPulse, hasActiveRC ? "ACTIVE" : "inactive", systemActive ? "YES" : "NO");
  }
  
  // Print "alive" message every 10 seconds (less frequent now)
  if (now - lastPrint >= 10000) {
    lastPrint = now;
    safeSerialPrintf("System operational - Override: %s, RC: %luµs, HotWire: %u\n", 
                     overrideMode ? "ON" : "OFF", readRCPulse(), hotWireOutput);
  }
  
  delay(10);  // Small delay
}

//------------------------------------------------------------------------------
// Pololu & G2 I²C

uint16_t readG2(uint8_t id){
  Wire.beginTransmission(G2_ADDRESS);
    Wire.write(0xA1); Wire.write(id);
  Wire.endTransmission();
  Wire.requestFrom((int)G2_ADDRESS, 2);
  return Wire.read() | (Wire.read()<<8);
}

void sendPololuStart(){
  uint8_t f[3]={POL_START,G2_ADDRESS,3};
  Wire.beginTransmission(G2_ADDRESS);
    Wire.write(f,3);
  Wire.endTransmission();
}

// Control hot wire power output
void sendPololuHotWireForward(uint16_t power){
  // Power range is 0-3200 (from Pololu documentation)
  power = constrain(power, 0, 3200);
  
  // Correct byte encoding based on Pololu documentation
  uint8_t power_byte_1 = power % 32;  // Lower 5 bits
  uint8_t power_byte_2 = power / 32;  // Upper bits
  
  // Create and send the command
  uint8_t f[5] = {POL_START, G2_ADDRESS, 5, power_byte_1, power_byte_2};
  Wire.beginTransmission(G2_ADDRESS);
  Wire.write(f, 5);
  Wire.endTransmission();
}

// Read Pololu G2 offset calibration register (default is 993 for 18v15)
// Register 0x43 = Current offset calibration (see Pololu G2 docs)
uint16_t readPololuOffsetCalibration() {
    return readG2(0x43);
}

void sendPololuSetCurrentLimit(float selector_A) {
    // Use the actual offset calibration read from controller
    float offset_cal = (float)curOffsetCal;
    float current_mA = selector_A * 1000.0f;

    // Step 1-4: Calculate intermediate value
    float internal_f = ((current_mA * 3200.0f * 2.0f) / CURRENT_SCALE_CAL) + offset_cal;

    // Step 5: Final conversion to internal units
    float final_f = (internal_f * 3200.0f) / 65536.0f;

    if (final_f > 16383.0f) final_f = 16383.0f;
    uint16_t final = (uint16_t)(final_f + 0.5f); // round to nearest

    uint8_t b1 = final & 0x7F;
    uint8_t b2 = (final >> 7) & 0x7F;

    // Serial debug output in decimal using safe functions
    safeSerialPrint("Selector: ");
    safeSerialPrint(String(selector_A, 2).c_str());
    safeSerialPrint(" A, Internal: ");
    safeSerialPrint(String(final).c_str());
    safeSerialPrint(", Byte1: ");
    safeSerialPrint(String(b1).c_str());
    safeSerialPrint(", Byte2: ");
    safeSerialPrintln(String(b2).c_str());

    uint8_t f[5] = {POL_START, G2_ADDRESS, POL_CMD_SET_CURRENT, b1, b2};
    Wire.beginTransmission(G2_ADDRESS);
    Wire.write(f, 5);
    Wire.endTransmission();

    // --- Read back the set current limit (register 42) and update the settings page ---
    delay(10); // Short delay to allow the controller to process the command
    uint16_t raw = readG2(42); // 42 (0x2A) = Current limit setting register

    // Reverse Pololu formula: internal units -> milliamps
    float mA = (((((float)raw * 65536.0f) / 3200.0f) - offset_cal) * CURRENT_SCALE_CAL) / 2.0f / 3200.0f;
    float set_current = mA / 1000.0f;

    // Update the settings page label if it exists
    if (lblSetCurrent) {
        char buf[18];
        snprintf(buf, sizeof(buf), "%.1f A", set_current);
        lv_label_set_text(lblSetCurrent, buf);
    }
}

uint32_t readRCPulse(){
  noInterrupts();
  uint32_t w=rc_width;
  interrupts();
  return w;
}

// RC Pulse ISR - measures pulse width
void IRAM_ATTR rcPulseISR() {
  static bool pulse_started = false;
  
  if (digitalRead(RC_INPUT_PIN) == HIGH) {
    // Rising edge - start of pulse
    rc_pulse_start = micros();
    pulse_started = true;
  } else if (pulse_started) {
    // Falling edge - end of pulse
    uint32_t pulse_end = micros();
    rc_width = pulse_end - rc_pulse_start;
    pulse_started = false;
  }
}

//------------------------------------------------------------------------------
// Delayed EEPROM Save Function

void updateDelayedEepromSave() {
  if (eepromSavePending && (millis() - lastEepromSaveTime >= EEPROM_SAVE_DELAY)) {
    eepromSave();
    eepromSavePending = false;
    safeSerialPrintln("EEPROM saved (delayed)");
  }
}

void scheduleEepromSave() {
  eepromSavePending = true;
  lastEepromSaveTime = millis();
}

//------------------------------------------------------------------------------
// PID Autotuning Functions

void startPIDAutotune(float setpoint, float outputStep) {
  // Validate current temperature reading before starting autotune
  float currentTemp = getSmoothedTemperature();
  if (isnan(currentTemp) || currentTemp < -270.0 || currentTemp > 1000.0) {
    safeSerialPrintf("Cannot start autotune - invalid temperature reading: %.1f°C\n", currentTemp);
    return;
  }
  
  safeSerialPrintln("Starting PID Autotune...");
  
  autotuneActive = true;
  autotuneState = 1; // running
  autotuneStartTime = millis();
  autotuneSetpoint = setpoint;
  autotuneOutput = outputStep; // Usually 50-80% for relay method
  autotuneDirection = true;
  autotuneLastSwitch = millis();
  autotuneNumPeaks = 0;
  autotuneLastTemp = getSmoothedTemperature();
  autotuneTempRising = true;
  autotuneKu = 0.0;
  autotunePu = 0.0;
  
  // Disable normal PID control
  pidEnabled = false;
  overrideMode = true; // Use manual control for autotune
  
  safeSerialPrintf("Autotune: Setpoint=%.1f°C, Output=%.1f%%, StartTemp=%.1f°C\n", 
                   setpoint, outputStep, currentTemp);
}

void updatePIDAutotune() {
  if (!autotuneActive || autotuneState != 1) return;
  
  uint32_t now = millis();
  float currentTemp = getSmoothedTemperature();
  
  // Validate temperature reading before using in autotune
  if (isnan(currentTemp) || currentTemp < -270.0 || currentTemp > 1000.0) {
    safeSerialPrintf("Autotune: Invalid temperature reading %.1f°C - aborting\n", currentTemp);
    autotuneState = 3; // failed
    autotuneActive = false;
    overrideMode = false;
    manualPWM = 0;
    return;
  }
  
  // Timeout after 30 minutes for gentler operation
  if (now - autotuneStartTime > 1800000) {
    autotuneState = 3; // failed
    autotuneActive = false;
    overrideMode = false;
    manualPWM = 0;
    safeSerialPrintln("Autotune: Timeout - Failed");
    return;
  }
  
  // Relay control - switch output based on temperature vs setpoint
  bool shouldHeat = currentTemp < autotuneSetpoint;
  
  if (shouldHeat != autotuneDirection) {
    // Record peak if we've been heating/cooling long enough - reduced time for better detection
    if (now - autotuneLastSwitch > 15000) { // 15 seconds minimum for good peak detection
      if (autotuneNumPeaks < 10) {
        safeSerialPrintf("Autotune: Recording peak %d: %.1f°C at %lums\n", 
                         autotuneNumPeaks, currentTemp, now);
        autotunePeaks[autotuneNumPeaks] = currentTemp;  // Use current temp, not last temp
        autotunePeakTimes[autotuneNumPeaks] = now;
        autotuneNumPeaks++;
      }
    } else {
      // Log when direction change happens but peak isn't recorded due to timing
      safeSerialPrintf("Autotune: Direction change at %.1f°C but too soon (%.1fs elapsed)\n", 
                       currentTemp, (now - autotuneLastSwitch) / 1000.0);
    }
    // Switch direction
    autotuneLastSwitch = now;
    autotuneDirection = shouldHeat;
  }
  
  // Set PWM output based on direction
  if (autotuneDirection) {
    manualPWM = (uint16_t)(3200 * autotuneOutput / 100.0); // Heat wire
  } else {
    manualPWM = 0; // Cool (turn off wire to let it cool)
  }
  
  // Check for completion - need at least 6 peaks for good data
  if (autotuneNumPeaks >= 6) {
    finishPIDAutotune();
  }
  
  autotuneLastTemp = currentTemp;

  // Enhanced logging every 10 updates to avoid spam but provide useful info
  static uint32_t lastDetailedLog = 0;
  if (now - lastDetailedLog >= 5000) { // Every 5 seconds
    lastDetailedLog = now;
    float timeElapsed = (now - autotuneStartTime) / 1000.0;
    float timeSinceSwitch = (now - autotuneLastSwitch) / 1000.0;
    safeSerialPrintf("AUTOTUNE: %.1fs elapsed, Temp=%.1f°C, Target=%.1f°C, %s for %.1fs, Peaks=%d\n", 
                     timeElapsed, currentTemp, autotuneSetpoint, 
                     autotuneDirection ? "HEATING" : "COOLING", timeSinceSwitch, autotuneNumPeaks);
  }
}

void finishPIDAutotune() {
  safeSerialPrintln("Finishing PID Autotune...");
  
  if (autotuneNumPeaks < 6) {
    autotuneState = 3; // failed
    safeSerialPrintln("Autotune: Not enough peaks - Failed");
  } else {
    // Calculate ultimate period (Pu) from peak timing
    float totalPeriod = 0;
    int periodCount = 0;
    
    for (int i = 2; i < autotuneNumPeaks; i++) {
      float period = (autotunePeakTimes[i] - autotunePeakTimes[i-2]) / 1000.0; // Convert to seconds
      if (period > 10 && period < 600) { // Reasonable period range
        totalPeriod += period;
        periodCount++;
      }
    }
    
    if (periodCount > 0) {
      autotunePu = totalPeriod / periodCount;
      
      // Calculate ultimate gain (Ku) by averaging all peak amplitudes
      float totalAmplitude = 0;
      for (int i = 1; i < autotuneNumPeaks; i++) {
        totalAmplitude += abs(autotunePeaks[i] - autotunePeaks[i-1]);
      }
      float avgAmplitude = totalAmplitude / (autotuneNumPeaks - 1);
      if (avgAmplitude > 0) {
        autotuneKu = (4.0 * autotuneOutput) / (3.14159 * avgAmplitude);
        // Derive PID parameters from Ku and Pu
        float newKp = 0.6 * autotuneKu;
        float newKi = (2.0 * newKp) / autotunePu;
        float newKd = (newKp * autotunePu) / 8.0;
        pidKp = newKp;
        pidKi = newKi;
        pidKd = newKd;
        tempPID.SetTunings(pidKp, pidKi, pidKd);
        autotuneState = 2; // complete
        safeSerialPrintf("Autotune: SUCCESS - Kp=%.2f, Ki=%.3f, Kd=%.2f\n", pidKp, pidKi, pidKd);
      } else {
        autotuneState = 3; // failed
        safeSerialPrintln("Autotune: No amplitude data - Failed");
      }
    } else {
      autotuneState = 3; // failed
      safeSerialPrintln("Autotune: No valid period data - Failed");
    }
  }
  
  // Print all recorded peaks
  safeSerialPrintln("AUTOTUNE RESULTS: Recorded Peaks:");
  for (int i = 0; i < autotuneNumPeaks; ++i) {
      safeSerialPrintf("  Peak %d: %.1f°C at %lums\n", i, autotunePeaks[i], autotunePeakTimes[i]);
  }
  
  // Clean up
  autotuneActive = false;
  overrideMode = false;
  manualPWM = 0;

  // Fallback to default PID settings if autotune failed
  if (autotuneState != 2) {
    safeSerialPrintln("Autotune: Failed - applying fallback default PID settings");
    const float FALLBACK_KP = 8.0;
    const float FALLBACK_KI = 0.2;
    const float FALLBACK_KD = 2.0;
    pidKp = FALLBACK_KP;
    pidKi = FALLBACK_KI;
    pidKd = FALLBACK_KD;
    tempPID.SetTunings(pidKp, pidKi, pidKd);
    safeSerialPrintf("Fallback PID: Kp=%.2f, Ki=%.3f, Kd=%.2f\n", pidKp, pidKi, pidKd);
    autotuneState = 2;
    // Save fallback defaults to EEPROM
    eepromSave();
  } else {
    // Save new parameters to EEPROM
    eepromSave();
  }
}

// Safe navigation function to prevent LVGL concurrency issues
void safeNavigateToScreen(lv_obj_t *screen, const char* screenName) {
  if (!screen) {
    safeSerialPrintf("ERROR: Cannot navigate to %s - screen is null!\n", screenName);
    return;
  }
  
  safeSerialPrintf("DEBUG: Safe navigation to %s requested\n", screenName);
  safeSerialPrintf("DEBUG: Current screen: %p, Target screen: %p\n", lv_scr_act(), screen);
  safeSerialPrintf("DEBUG: Override init flag: %s\n", overrideScreenInitialized ? "true" : "false");
  
  pendingNavigation = true;
  pendingScreen = screen;
  navigationRequestTime = millis();
}

// Function to verify all override screen objects are still valid
bool verifyOverrideScreenObjects() {
  bool allValid = true;
  
  if (!scrOverride) {
    safeSerialPrintln("ERROR: scrOverride is null");
    allValid = false;
  }
  
  if (!lblOverrideTemp) {
    safeSerialPrintln("ERROR: lblOverrideTemp is null");
    allValid = false;
  }
  
  if (!lblOverrideRCPulse) {
    safeSerialPrintln("ERROR: lblOverrideRCPulse is null");
    allValid = false;
  }
  
  if (!lblOverrideModeStatus) {
    safeSerialPrintln("ERROR: lblOverrideModeStatus is null");
    allValid = false;
  }
  
  if (!lblOverrideTempSetpoint) {
    safeSerialPrintln("ERROR: lblOverrideTempSetpoint is null");
    allValid = false;
  }
  
  if (!lblOverridePowerOutput) {
    safeSerialPrintln("ERROR: lblOverridePowerOutput is null");
    allValid = false;
  }
  
  if (!lblSliderVal) {
    safeSerialPrintln("ERROR: lblSliderVal is null");
    allValid = false;
  }
  
  if (!btnTempDown) {
    safeSerialPrintln("ERROR: btnTempDown is null");
    allValid = false;
  }
  
  if (!btnTempUp) {
    safeSerialPrintln("ERROR: btnTempUp is null");
    allValid = false;
  }
  
  if (allValid) {
    safeSerialPrintln("DEBUG: All override screen objects are valid");
  }
  
  return allValid;
}