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
#include <HardwareSerial.h>
#include "screens.h"  // Include screen functions

// Pololu & LVGL
#define LV_COLOR_RED          lv_color_hex(0xFF0000)
#define G2_ADDRESS            13
#define POL_START             170
#define POL_CMD_SET_CURRENT   17

// I²C & RC
#define I2C_SDA_PIN           11
#define I2C_SCL_PIN           10
// RC input pin
#define RC_INPUT_PIN          15    // RC input now on GPIO15

// Touch controller pins

#define TP_SDA_PIN             1
#define TP_SCL_PIN             3
#define TP_RST_PIN             2
#define TP_INT_PIN             4


// UART pins for Nano communication
#define NANO_UART_TX 43  // ESP32-S3 TX to Nano RX
#define NANO_UART_RX 44  // ESP32-S3 RX from Nano TX


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
#define SAFETY_CHECK_INTERVAL     5000    // Check every 5 seconds (was 2 seconds)
#define SAFETY_TEMP_RISE_TIMEOUT  15000   // 15 seconds to see temperature rise (was 10 seconds)
#define SAFETY_MIN_OUTPUT         1000    // Minimum output to consider "heating" (was 500)
#define SAFETY_MIN_TEMP_RISE      3.0     // Minimum temperature rise expected (°C) (was 2.0)

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


// Forward declarations
uint16_t readG2(uint8_t id);
uint32_t readRCPulse();
void rcPulseISR();
void sendPololuStart();
void sendPololuHotWireForward(uint16_t power);
void sendPololuSetCurrentLimit(float selector_A);
uint16_t readPololuOffsetCalibration();
void debugPrintG2Voltage();

void eepromLoad();
void eepromSave();
void buildTelemetryScreen();
void buildOverrideScreen();
void buildSettingsScreen();
void updateTelemetry();
void updateOverrideScreen();
void updateDelayedEepromSave();
void scheduleEepromSave();
void initCurrentLimitFromG2();
void buildPIDTuneScreen();
void startPIDAutotune(float setpoint, float outputStep);
void updatePIDAutotune();
void finishPIDAutotune();
void safeSerialPrint(const char* msg);
void safeSerialPrintln(const char* msg);
void safeSerialPrintf(const char* format, ...);
float getCachedTemp();

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
lv_obj_t *lblManualPower = nullptr;  // Manual power display label
lv_obj_t *btnPidEnable = nullptr;    // PID enable/disable button
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

// SPI configuration for MAX6675



// UART interface to Nano
HardwareSerial NanoSerial(2);

// Global temperature cache with smoothing
float globalTemperatureC = NAN;
uint32_t lastTempUpdateTime = 0;
float tempReadings[5] = {NAN, NAN, NAN, NAN, NAN};  // Rolling buffer for smoothing
int tempReadingIndex = 0;

// Read temperature from Nano over UART
float readNanoTemp() {
    static String tempLine = "";
    static float lastTempC = NAN;
    static uint32_t lastRequestTime = 0;
    // Non-blocking: send request every call, parse response if available
    uint32_t now = millis();
    // Send request every 250ms (or as needed)
    if (now - lastRequestTime > 250) {
        NanoSerial.write('R');
        lastRequestTime = now;
        
        // Debug: Track request sending
        static uint32_t requestCount = 0;
        requestCount++;
        safeSerialPrintf("UART REQ: Sent request #%lu at %lums\n", requestCount, now);
    }
    // Parse any available response
    while (NanoSerial.available()) {
        char c = NanoSerial.read();
        if (c == '\n' || c == '\r') {
            if (tempLine.length() > 0) {
                int idx = tempLine.indexOf(":");
                String valStr = (idx >= 0) ? tempLine.substring(idx + 1) : tempLine;
                valStr.trim();
                float newTempC = valStr.toFloat();
                
                // Validate temperature reading - reject obviously invalid values
                bool isValidReading = true;
                if (isnan(newTempC) || newTempC < -50.0 || newTempC > 400.0) {
                    isValidReading = false;
                    safeSerialPrintf("TEMP REJECT: Invalid reading %.2f°C (out of range -50 to 400°C)\n", newTempC);
                }
                
                // Check for sudden jumps that indicate bad readings
                if (isValidReading && !isnan(lastTempC)) {
                    float tempDiff = abs(newTempC - lastTempC);
                    
                    // Allow larger jumps if cached temp is invalid (0°C or very old)
                    uint32_t cacheAge = (millis() >= lastTempUpdateTime) ? (millis() - lastTempUpdateTime) : 0;
                    bool cacheIsStale = (lastTempC <= 1.0) || (cacheAge > 10000);  // 0°C or >10 seconds old
                    
                    float maxAllowedJump = cacheIsStale ? 300.0 : 100.0;  // Allow 300°C jump if cache is bad
                    
                    if (tempDiff > maxAllowedJump) {
                        isValidReading = false;
                        safeSerialPrintf("TEMP REJECT: Jump too large %.2f°C (diff=%.1f°C from %.2f°C, max=%.0f°C, cache_stale=%s)\n", 
                                         newTempC, tempDiff, lastTempC, maxAllowedJump, cacheIsStale ? "YES" : "NO");
                    } else if (cacheIsStale) {
                        safeSerialPrintf("TEMP RECOVERY: Accepting large jump %.2f°C (cache was stale: %.2f°C, age=%lums)\n", 
                                         newTempC, lastTempC, cacheAge);
                    }
                }
                
                if (isValidReading) {
                    lastTempC = newTempC;
                    safeSerialPrintf("Temperature accepted: %.2f°C\n", lastTempC);
                    
                    // Add to rolling average buffer for smoothing
                    tempReadings[tempReadingIndex] = lastTempC;
                    tempReadingIndex = (tempReadingIndex + 1) % 5;
                    
                    // Calculate smoothed temperature (average of last 5 valid readings)
                    float smoothedTemp = 0.0;
                    int validCount = 0;
                    for (int i = 0; i < 5; i++) {
                        if (!isnan(tempReadings[i])) {
                            smoothedTemp += tempReadings[i];
                            validCount++;
                        }
                    }
                    
                    if (validCount > 0) {
                        smoothedTemp /= validCount;
                        // Update global temperature cache with smoothed value
                        globalTemperatureC = smoothedTemp;
                        lastTempUpdateTime = now;
                        
                        // Debug: Show both raw and smoothed values
                        safeSerialPrintf("TEMP CACHE: Raw=%.2f°C, Smoothed=%.2f°C (from %d readings)\n", 
                                         lastTempC, smoothedTemp, validCount);
                    }
                } else {
                    safeSerialPrintf("TEMP CACHE: Keeping previous value %.2f°C (rejected %.2f°C)\n", 
                                     globalTemperatureC, newTempC);
                }
                
                tempLine = "";
            }
        } else {
            tempLine += c;
        }
    }
    // Return last valid temperature (may be NAN if not received yet)
    return lastTempC;
}

// Get cached temperature (faster, non-blocking)
float getCachedTemp() {
    uint32_t now = millis();
    uint32_t timeSinceUpdate = (now >= lastTempUpdateTime) ? (now - lastTempUpdateTime) : 0;
    
    // Debug: Log cache access
    static uint32_t lastCacheDebug = 0;
    if (now - lastCacheDebug >= 2000) {  // Debug every 2 seconds
        lastCacheDebug = now;
        safeSerialPrintf("TEMP CACHE: Value=%.2f°C, Age=%lums, Valid=%s\n", 
                         globalTemperatureC, timeSinceUpdate, 
                         isnan(globalTemperatureC) ? "NO" : "YES");
    }
    
    return globalTemperatureC;
}


//------------------------------------------------------------------------------
// Safety Monitoring System

void updateSafetyMonitoring(uint16_t currentOutput, float currentTemp) {
  if (!safetySystemEnabled) return;
  
  uint32_t now = millis();
  
  // Check if we're outputting significant heating power with hysteresis
  bool isHeating = false;
  if (!safetyHeatingActive) {
    // Not currently heating - use higher threshold to start monitoring
    isHeating = (currentOutput >= SAFETY_MIN_OUTPUT);
  } else {
    // Currently heating - use much lower threshold to stop monitoring (wide hysteresis)
    // This prevents rapid cycling from small PID output variations
    isHeating = (currentOutput >= (SAFETY_MIN_OUTPUT / 4));  // Changed from /2 to /4 for wider hysteresis
  }
  
  // Detect start of heating
  if (isHeating && !safetyHeatingActive) {
    safetyHeatingActive = true;
    safetyHeatingStartTime = now;
    safetyInitialTemp = currentTemp;
    safeSerialPrintf("SAFETY: Heating started - Initial temp: %.1f°C, Output: %u (threshold: %u)\n", 
                     safetyInitialTemp, currentOutput, SAFETY_MIN_OUTPUT);
  }
  
  // Detect end of heating
  if (!isHeating && safetyHeatingActive) {
    safetyHeatingActive = false;
    safeSerialPrintf("SAFETY: Heating stopped - Output: %u (below threshold: %u)\n", 
                     currentOutput, SAFETY_MIN_OUTPUT / 2);
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
  Serial.print(msg);
}

inline void safeSerialPrintln(const char* msg) {
  Serial.println(msg);
}

// For formatted output
void safeSerialPrintf(const char* format, ...) {
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
    currentLimit = CUR_MIN;  // Use CUR_MIN as default instead of CUR_MAX
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
  if(currentLimit<CUR_MIN||currentLimit>CUR_MAX)      currentLimit=CUR_MIN;  // Use CUR_MIN instead of 0.50f
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

  // Initialize Nano UART
  Serial.println("Initializing Nano UART...");
  NanoSerial.begin(115200, SERIAL_8N1, NANO_UART_RX, NANO_UART_TX);
  Serial.println("Nano UART initialized on GPIO43/44");

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


  // No MAX6675 SPI init needed


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
  
  // Initialize current limit display from G2 controller
  Serial.println("Initializing current limit from G2...");
  initCurrentLimitFromG2();
  
  // Initialize idle timer
  lastActivityTime = millis();
  screensaverActive = false;
  
  // Print startup message but don't wait for serial
  Serial.println("PickleWire Hot Wire Cutter v2.40 started");
}

void loop(){
  // Removed GPIO5 state check for cleaner operation
  // Removed blocking delay for smooth UI and telemetry
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

    // Add Pololu G2 input voltage debug
    debugPrintG2Voltage();

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
  
  // Ensure continuous temperature reading every 100ms regardless of other systems
  static uint32_t lastTempRead = 0;
  if (now - lastTempRead >= 100) {
    lastTempRead = now;
    
    // Debug: Track temperature reading calls
    static uint32_t tempReadCount = 0;
    tempReadCount++;
    if (tempReadCount % 10 == 0) {  // Every 10th call (every second)
      safeSerialPrintf("TEMP READ: Call #%lu at %lums\n", tempReadCount, now);
    }
    
    readNanoTemp();  // This will handle its own 250ms request timing and update global cache
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
  
  // Debug: Monitor setpoint changes
  static double lastLoggedSetpoint = -999.0;
  if (temperatureSetpoint != lastLoggedSetpoint) {
    safeSerialPrintf("SETPOINT CHANGE: %.1f°C -> %.1f°C, PID enabled: %s\n", 
                     lastLoggedSetpoint, temperatureSetpoint, pidEnabled ? "YES" : "NO");
    lastLoggedSetpoint = temperatureSetpoint;
  }
  
  // Priority: PID control > RC control > Manual control
  if (pidEnabled && temperatureSetpoint > 0) {
    // PID control mode - use temperature control
    if (now - lastPidUpdateTime >= PID_UPDATE_INTERVAL) {
      lastPidUpdateTime = now;
      float currentTemp = getCachedTemp();  // Use cached temperature for faster access
      
      // Debug: Log PID temperature reading
      static uint32_t lastPidTempLog = 0;
      if (now - lastPidTempLog >= 1000) {  // Log every second
        lastPidTempLog = now;
        safeSerialPrintf("PID: Reading temp=%.2f°C, setpoint=%.1f°C, enabled=%s\n", 
                         currentTemp, temperatureSetpoint, pidEnabled ? "YES" : "NO");
      }
      
      // Enhanced validation - check for reasonable temperature values
      bool validForPID = true;
      if (isnan(currentTemp)) {
        validForPID = false;
        safeSerialPrintln("PID: Temperature is NaN - skipping PID update");
      } else if (currentTemp < 0.0 || currentTemp > 500.0) {
        validForPID = false;
        safeSerialPrintf("PID: Temperature %.2f°C out of reasonable range (0-500°C) - skipping PID update\n", currentTemp);
      } else {
        // Check cache age - don't use stale data
        uint32_t cacheAge = (now >= lastTempUpdateTime) ? (now - lastTempUpdateTime) : 0;
        if (cacheAge > 2000) {  // 2 second timeout
          validForPID = false;
          safeSerialPrintf("PID: Temperature cache too old (%lums) - skipping PID update\n", cacheAge);
        }
      }
      
      if (validForPID) {
        temperatureInput = currentTemp;
        tempPID.Compute();
        hotWireOutput = (uint16_t)outputPWM;
        
        // Debug: Log PID computation details
        static uint32_t lastPidComputeLog = 0;
        if (now - lastPidComputeLog >= 2000) {  // Log every 2 seconds
          lastPidComputeLog = now;
          safeSerialPrintf("PID COMPUTE: Input=%.2f°C, Setpoint=%.1f°C, Output=%.1f (raw), HotWire=%u\n", 
                           temperatureInput, temperatureSetpoint, outputPWM, hotWireOutput);
          safeSerialPrintf("PID PARAMS: Kp=%.2f, Ki=%.3f, Kd=%.2f, Mode=%s\n", 
                           pidKp, pidKi, pidKd, tempPID.GetMode() == AUTOMATIC ? "AUTO" : "MANUAL");
        }
        
        // Reset invalid reading counter on successful reading
        static int invalidReadingCount = 0;
        invalidReadingCount = 0;
      } else {
        // Count consecutive invalid readings before disabling PID
        static int invalidReadingCount = 0;
        invalidReadingCount++;
        
        if (invalidReadingCount >= 10) {  // Disable after 10 consecutive invalid readings (500ms)
          safeSerialPrintf("Too many invalid temperature readings - disabling PID (count: %d)\n", invalidReadingCount);
          hotWireOutput = 0;
          pidEnabled = false;  // Disable PID until valid reading
          invalidReadingCount = 0;  // Reset counter
        } else {
          // Keep last valid hotWireOutput for brief invalid readings
          hotWireOutput = (uint16_t)outputPWM;
          safeSerialPrintf("Invalid temperature reading %d/10 - continuing with last PID output %u\n", 
                          invalidReadingCount, hotWireOutput);
        }
      }
    } else {
      // PID enabled but not time to update yet - keep last PID output
      hotWireOutput = (uint16_t)outputPWM;
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
  // Use the cached temperature reading for consistent data
  float currentTemp = getCachedTemp();
  
  // Debug: Log safety monitoring input values
  static uint32_t lastSafetyDebug = 0;
  if (now - lastSafetyDebug >= 1000) {  // Log every second
    lastSafetyDebug = now;
    safeSerialPrintf("SAFETY INPUT: hotWireOutput=%u, temp=%.2f°C, active=%s\n", 
                     hotWireOutput, currentTemp, safetyHeatingActive ? "YES" : "NO");
  }
  
  updateSafetyMonitoring(hotWireOutput, currentTemp);
  
  // Apply safety shutdown if error detected
  if (safetyErrorState) {
    static uint32_t lastSafetyLog = 0;
    if (now - lastSafetyLog >= 500) {  // Log every 500ms when safety active (more frequent)
      lastSafetyLog = now;
      safeSerialPrintf("🚨 SAFETY SHUTDOWN ACTIVE: hotWireOutput %u->0, PID disabled\n", hotWireOutput);
      safeSerialPrintf("🚨 SAFETY ERROR: %s\n", safetyErrorMessage.c_str());
      safeSerialPrintln("🚨 USE RESET BUTTON TO CLEAR SAFETY ERROR!");
    }
    hotWireOutput = 0;  // Emergency shutdown
    pidEnabled = false; // Disable PID control
  }
  
  // Send hot wire command
  sendPololuHotWireForward(hotWireOutput);
  
  // Debug: Log actual power output periodically
  static uint32_t lastPowerLog = 0;
  if (now - lastPowerLog >= 3000) {  // Log every 3 seconds
    lastPowerLog = now;
    safeSerialPrintf("POWER OUTPUT: Sending %u to G2 controller (%.1f%%), PID=%s, Override=%s, Setpoint=%.1f°C\n", 
                     hotWireOutput, (hotWireOutput/3200.0f)*100.0f, 
                     pidEnabled ? "ON" : "OFF", overrideMode ? "ON" : "OFF", temperatureSetpoint);
    safeSerialPrintf("CURRENT LIMIT: %.1f A (%.1f%% of max %.1f A), Safety=%s\n", 
                     currentLimit, (currentLimit/CUR_MAX)*100.0f, CUR_MAX,
                     safetyErrorState ? "ERROR" : "OK");
  }
  
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
  uint8_t low = Wire.read();
  uint8_t high = Wire.read();
  uint16_t value = low | (high << 8);
  safeSerialPrintf("G2 I2C: Read reg 0x%02X, value: %u (0x%04X)\n", id, value, value);
  return value;
}

// Debug: Read and print Pololu G2 input voltage (register 0x24)
void debugPrintG2Voltage() {
    uint16_t raw = readG2(0x24); // Register 0x24: Input voltage in millivolts
    float voltage = raw / 1000.0f;
    safeSerialPrintf("G2 I2C: Input voltage: %.3f V (raw: %u mV)\n", voltage, raw);
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

    // Read back the set current limit (register 42) to verify and update global variable
    delay(10); // Short delay to allow the controller to process the command
    uint16_t raw = readG2(42); // 42 (0x2A) = Current limit setting register

    // Reverse Pololu formula: internal units -> milliamps
    float mA = (((((float)raw * 65536.0f) / 3200.0f) - offset_cal) * CURRENT_SCALE_CAL) / 2.0f / 3200.0f;
    float actualCurrentLimit = mA / 1000.0f;
    
    // Update the global currentLimit variable with the actual value from G2
    currentLimit = actualCurrentLimit;
    
    // Update the settings page label if it exists (but only when not actively being pressed)
    // This prevents overwriting immediate UI feedback from button presses
    static uint32_t lastButtonUpdate = 0;
    static uint32_t buttonUpdateDelay = 500; // 500ms delay after button press
    uint32_t now = millis();
    
    if (now - lastButtonUpdate > buttonUpdateDelay && lblSetCurrent) {
        char buf[32];
        snprintf(buf, sizeof(buf), "Current Limit: %.1f A", actualCurrentLimit);
        lv_label_set_text(lblSetCurrent, buf);
        safeSerialPrintf("UI updated with G2 readback: %.1f A\n", actualCurrentLimit);
    } else {
        safeSerialPrintf("G2 readback: %.1f A (UI update skipped)\n", actualCurrentLimit);
    }
    
    // This function is called from button handlers, so mark the time
    lastButtonUpdate = now;
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

// Function to read current limit from G2 and update UI on startup
void initCurrentLimitFromG2() {
    if (!lblSetCurrent) return; // UI not ready yet
    
    safeSerialPrintln("Reading current limit from G2 controller...");
    
    // Read current limit setting register
    uint16_t raw = readG2(42); // 42 (0x2A) = Current limit setting register
    
    // Use the actual offset calibration
    float offset_cal = (float)curOffsetCal;
    
    // Reverse Pololu formula: internal units -> milliamps
    float mA = (((((float)raw * 65536.0f) / 3200.0f) - offset_cal) * CURRENT_SCALE_CAL) / 2.0f / 3200.0f;
    float actualCurrentLimit = mA / 1000.0f;
    
    // Update global variable and UI
    currentLimit = actualCurrentLimit;
    
    if (lblSetCurrent) {
        char buf[32];
        snprintf(buf, sizeof(buf), "Current Limit: %.1f A", actualCurrentLimit);
        lv_label_set_text(lblSetCurrent, buf);
        safeSerialPrintf("Current limit initialized from G2: %.1f A\n", actualCurrentLimit);
    }
}

//------------------------------------------------------------------------------
// PID Autotuning Functions

void startPIDAutotune(float setpoint, float outputStep) {
  // Validate current temperature reading before starting autotune
  float currentTemp = readNanoTemp();
  if (isnan(currentTemp) || currentTemp < -270.0 || currentTemp > 1000.0) {
    safeSerialPrintf("Cannot start autotune - invalid temperature reading: %.1f°C\n", currentTemp);
    return;
  }
  
  safeSerialPrintln("Starting PID Autotune...");
  
  // Set the global temperature setpoint so the system knows the target
  temperatureSetpoint = setpoint;
  
  autotuneActive = true;
  autotuneState = 1; // running
  autotuneStartTime = millis();
  autotuneSetpoint = setpoint;
  autotuneOutput = outputStep; // Usually 50-80% for relay method
  autotuneDirection = true;
  autotuneLastSwitch = millis();
  autotuneNumPeaks = 0;
  autotuneLastTemp = readNanoTemp();
  autotuneTempRising = true;
  autotuneKu = 0.0;
  autotunePu = 0.0;
  
  // Disable safety monitoring during autotune to prevent false alarms
  // Autotune uses relay control (on/off) which can trigger safety errors
  safetySystemEnabled = false;
  clearSafetyError(); // Clear any existing safety errors
  safeSerialPrintln("Autotune: Safety monitoring disabled during autotune");
  
  // Disable normal PID control during autotune
  pidEnabled = false;
  overrideMode = true; // Use manual control for autotune
  
  safeSerialPrintf("Autotune: Setpoint=%.1f°C, Output=%.1f%%, StartTemp=%.1f°C\n", 
                   setpoint, outputStep, currentTemp);
  safeSerialPrintf("Autotune: Global temperature setpoint updated to %.1f°C\n", temperatureSetpoint);
}

void updatePIDAutotune() {
  if (!autotuneActive || autotuneState != 1) return;
  
  uint32_t now = millis();
  float currentTemp = readNanoTemp();
  
  // Validate temperature reading before using in autotune
  if (isnan(currentTemp) || currentTemp < -270.0 || currentTemp > 1000.0) {
    safeSerialPrintf("Autotune: Invalid temperature reading %.1f°C - aborting\n", currentTemp);
    autotuneState = 3; // failed
    autotuneActive = false;
    overrideMode = false;
    manualPWM = 0;
    // Re-enable safety monitoring if autotune aborts
    safetySystemEnabled = true;
    safeSerialPrintln("Autotune: Safety monitoring re-enabled after abort");
    return;
  }
  
  // Timeout after 30 minutes for gentler operation
  if (now - autotuneStartTime > 1800000) {
    autotuneState = 3; // failed
    autotuneActive = false;
    overrideMode = false;
    manualPWM = 0;
    // Re-enable safety monitoring if autotune times out
    safetySystemEnabled = true;
    safeSerialPrintln("Autotune: Timeout - Failed, safety monitoring re-enabled");
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

  // Re-enable safety monitoring after autotune completes
  safetySystemEnabled = true;
  safeSerialPrintln("Autotune: Safety monitoring re-enabled after autotune completion");

  // Preserve the autotune setpoint for continued PID control
  float preservedSetpoint = autotuneSetpoint;  // Keep the target temperature from autotune
  safeSerialPrintf("Autotune: Preserving setpoint %.1f°C for continued PID control\n", preservedSetpoint);

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
  
  // Enable PID control after autotune completes (successful or failed)
  // User requested: Turn off power output when autotune completes
  temperatureSetpoint = 0.0;  // Set setpoint to 0 to disable PID control
  pidEnabled = false;          // Disable PID control
  safeSerialPrintf("Autotune: Completed - PID disabled, setpoint set to 0°C, power output OFF\n");
  safeSerialPrintf("Autotune: New PID parameters saved. Enable PID manually when ready.\n");
  
  // Update PID status display if on PID tuning screen
  if (lblPidStatus) {
    if (autotuneState == 2) {
      lv_label_set_text(lblPidStatus, "Status: Complete - PID OFF");
      lv_obj_set_style_text_color(lblPidStatus, lv_color_hex(0x00FF00), 0);
    } else {
      lv_label_set_text(lblPidStatus, "Status: Failed - PID OFF");
      lv_obj_set_style_text_color(lblPidStatus, lv_color_hex(0xFF4444), 0);
    }
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