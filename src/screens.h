#include <PID_v1.h>
#include <lvgl.h>
#include <Arduino.h>  // For String type

// Extern UI objects (so both main.cpp and screens.cpp can access them)
extern lv_obj_t *scrTelem, *scrOverride, *scrSettings, *scrPIDTune;
extern lv_obj_t *btnNavOverride, *btnNavSettings;  // Navigation buttons
extern lv_obj_t *lblSetWire, *lblSetOffset, *lblSetCurrent;
extern lv_obj_t *lblOverrideTemp, *lblOverrideRCPulse, *lblOverrideModeStatus, *lblSliderVal;  // Removed slider, kept label for temp display
extern lv_obj_t *lblOverridePowerOutput;   // Current power output label for override screen
extern lv_obj_t *btnTempDown, *btnTempUp, *lblTempDown, *lblTempUp;  // Temperature adjustment buttons
extern lv_obj_t *lblPidStatus;
extern lv_obj_t *lblChartMin, *lblChartMax;
extern lv_obj_t *lblTitle[4], *lblVal[4], *lblBanner, *lblErrorVal;
extern lv_obj_t *btnWireLeft, *btnWireRight, *btnOffsetLeft, *btnOffsetRight;
extern lv_obj_t *btnCurrentLeft, *btnCurrentRight;
extern lv_obj_t *btnPidKpLeft, *btnPidKpRight, *btnPidKiLeft, *btnPidKiRight, *btnPidKdLeft, *btnPidKdRight;
extern lv_obj_t *lblSetPidKp, *lblSetPidKi, *lblSetPidKd;
extern lv_obj_t *btnPidTune, *btnSettingsNav;
extern lv_chart_series_t *serTemp;
extern lv_chart_series_t *serSetpoint;  // Series for setpoint line
extern lv_obj_t *chartTemp;

// Shared variables
extern float wireDiam, tempOffset, currentLimit;
extern double pidKp, pidKi, pidKd;
extern bool pidEnabled;
extern PID tempPID;
extern float chartTempMin, chartTempMax;
extern double temperatureSetpoint, temperatureInput, outputPWM;
extern uint32_t lastPidUpdateTime;
extern bool pidResetting;
extern uint32_t pidResetStartTime;
extern uint16_t curOffsetCal;
extern float OFF_MIN, OFF_MAX;

// EEPROM save functions (implemented in main.cpp)
extern void scheduleEepromSave();
extern bool eepromSavePending;

// Control mode variables
extern bool overrideMode;
extern uint16_t manualPWM;

// Screen initialization tracking
extern bool overrideScreenInitialized;

// PID Autotune variables
extern bool autotuneActive;
extern int autotuneState;
extern float autotuneKu, autotunePu;
extern int autotuneNumPeaks;

// Screensaver and splash screen variables
#define IDLE_TIMEOUT_MS 60000     // 60 seconds (1 minute) idle timeout

extern bool screensaverActive;
extern uint32_t lastActivityTime;
extern lv_obj_t *screensaverScreen;
extern lv_obj_t *screensaverP, *screensaverPickle;
extern lv_obj_t *screensaverW, *screensaverWire;
extern int screensaverX, screensaverY;
extern int screensaverVelX, screensaverVelY;
extern lv_obj_t *savedActiveScreen;

// Constants (need to be extern to access from screens.cpp)
extern float WIRE_MIN, WIRE_MAX, WIRE_STEP;
extern float CUR_MIN, CUR_MAX, CUR_STEP, OFF_STEP;

// Function declarations
void buildTelemetryScreen();
void buildOverrideScreen();
void buildSettingsScreen();
void buildPIDTuneScreen();
void updateTelemetry();
void updateOverrideScreen();  // Add this function to update override screen live data
void updateDelayedEepromSave();  // Add delayed EEPROM save function
void triggerDelayedEepromSave(); // Trigger delayed EEPROM save
void updatePIDTuneStatus();

// Splash screen and screensaver functions
void showSplashScreen();
void createScreensaver();
void updateScreensaver();
void exitScreensaver();
void resetIdleTimer();

// External function declarations
extern void sendPololuStart();
extern void sendPololuHotWireForward(uint16_t power);
extern void sendPololuSetCurrentLimit(float selector_A);
extern uint16_t readG2(uint8_t id);
extern uint32_t readRCPulse();
extern float getSmoothedTemperature();
extern void eepromSave();
extern void startPIDAutotune(float setpoint, float outputStep);

// Safety monitoring functions
extern bool isSafetyErrorActive();
extern void clearSafetyError();
extern String getSafetyErrorMessage();

// Safe serial functions
extern void safeSerialPrint(const char* msg);
extern void safeSerialPrintln(const char* msg);
extern void safeSerialPrintf(const char* format, ...);
