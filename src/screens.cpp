#include "screens.h"
#include <lvgl.h>
#include <cstdio>
#include <cmath>
#include <algorithm>
#include "lv_conf.h"
#include <Arduino.h>
#include "lv_font_montserrat_decls.h"
#include <PID_v1.h>

// Include any global variables you need from main.cpp as extern
extern float wireDiam, tempOffset, currentLimit;
extern double pidKp, pidKi, pidKd;
extern bool pidEnabled;
extern bool overrideMode;
extern uint16_t manualPWM;
extern bool overrideScreenInitialized;
extern bool autotuneActive;
extern PID tempPID;
extern float chartTempMin, chartTempMax;
extern double temperatureSetpoint, temperatureInput, outputPWM;
extern uint32_t lastPidUpdateTime;
extern bool pidResetting;
extern uint32_t pidResetStartTime;
extern uint16_t curOffsetCal;
extern void sendPololuStart();
extern void sendPololuMotorForward(uint16_t speed);
extern void sendPololuSetCurrentLimit(float selector_A);
extern uint16_t readG2(uint8_t id);
extern uint32_t readRCPulse();

extern void scheduleEepromSave();
extern void safeSerialPrint(const char* msg);
extern void safeSerialPrintln(const char* msg);
extern void safeSerialPrintf(const char* format, ...);
extern void safeNavigateToScreen(lv_obj_t *screen, const char* screenName);
extern bool overrideScreenInitialized;

// All UI objects are defined in main.cpp and declared in screens.h
// No need to define them again here

void buildTelemetryScreen() {
    safeSerialPrintln("DEBUG: Starting buildTelemetryScreen");
    
    scrTelem = lv_obj_create(NULL);
    if (scrTelem == nullptr) {
        safeSerialPrintln("CRITICAL ERROR: Failed to create telemetry screen!");
        return;
    }
    
    safeSerialPrintln("DEBUG: Telemetry screen object created");
    lv_obj_set_style_bg_color(scrTelem, lv_color_hex(0x222222), 0);
    lv_obj_set_style_bg_opa(scrTelem, LV_OPA_COVER, 0);

    // Create chart FIRST so it's in the background
    chartTemp = lv_chart_create(scrTelem);
    if (chartTemp == nullptr) {
        safeSerialPrintln("CRITICAL ERROR: Failed to create temperature chart!");
        return;
    }
    
    safeSerialPrintln("DEBUG: Temperature chart created");
    // Chart positioned below telemetry labels (titles at y=30, values at ~y=50)
    lv_obj_set_size(chartTemp, 280, 80);
    lv_obj_align(chartTemp, LV_ALIGN_CENTER, 0, 35);  // Moved up by 10px
    lv_chart_set_type(chartTemp, LV_CHART_TYPE_LINE);
    lv_chart_set_range(chartTemp, LV_CHART_AXIS_PRIMARY_Y, chartTempMin, chartTempMax);
    lv_chart_set_point_count(chartTemp, 150);  // Increased to 150 for more historical data
    serTemp = lv_chart_add_series(chartTemp, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);
    serSetpoint = lv_chart_add_series(chartTemp, lv_palette_main(LV_PALETTE_GREEN), LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_set_update_mode(chartTemp, LV_CHART_UPDATE_MODE_SHIFT);
    lv_obj_set_style_bg_color(chartTemp, lv_color_hex(0x111111), 0);
    lv_obj_set_style_border_width(chartTemp, 0, 0);
    lv_obj_set_style_pad_all(chartTemp, 0, 0);
    
    // Configure series for smooth lines instead of dots
    lv_obj_set_style_line_width(chartTemp, 2, LV_PART_ITEMS);  // Line thickness
    lv_obj_set_style_size(chartTemp, 0, LV_PART_INDICATOR);    // Remove dots/points
    
    // Explicitly move chart to background
    lv_obj_move_background(chartTemp);

    // Title aligned top right
    lblBanner = lv_label_create(scrTelem);
    lv_label_set_text(lblBanner, "PickleWire Telemetry");
    lv_obj_align(lblBanner, LV_ALIGN_TOP_RIGHT, -10, 5);
    lv_obj_set_style_text_font(lblBanner, LV_FONT_DEFAULT, 0);
    lv_obj_set_style_text_color(lblBanner, lv_color_hex(0xFFFFFF), 0);

    // 4 telemetry items across the screen above the graph
    static const char *labels[] = {"Temp", "Setpt", "Power", "Current"};
    for (int i = 0; i < 4; ++i) {
        // Create title labels
        lblTitle[i] = lv_label_create(scrTelem);
        lv_label_set_text(lblTitle[i], labels[i]);
        // Position title labels across the screen, centered in each section
        lv_obj_align(lblTitle[i], LV_ALIGN_TOP_LEFT, 10 + i * 75, 40);
        lv_obj_set_style_text_font(lblTitle[i], LV_FONT_DEFAULT, 0);
        lv_obj_set_style_text_color(lblTitle[i], lv_color_hex(0xFFFFFF), 0);
        
        // Create value labels centered under title labels
        lblVal[i] = lv_label_create(scrTelem);
        lv_label_set_text(lblVal[i], "---");
        lv_obj_align_to(lblVal[i], lblTitle[i], LV_ALIGN_OUT_BOTTOM_MID, 0, 5);
        lv_obj_set_style_text_font(lblVal[i], LV_FONT_DEFAULT, 0);
        lv_obj_set_style_text_color(lblVal[i], lv_color_hex(0xFFFFFF), 0);
    }

    // Chart min/max labels
    lblChartMin = lv_label_create(scrTelem);
    char minBuffer[16];
    sprintf(minBuffer, "%.1f", chartTempMin);
    lv_label_set_text(lblChartMin, minBuffer);
    lv_obj_align_to(lblChartMin, chartTemp, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 0);
    lv_obj_set_style_text_color(lblChartMin, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(lblChartMin, LV_FONT_DEFAULT, 0);
    
    lblChartMax = lv_label_create(scrTelem);
    char maxBuffer[16];
    sprintf(maxBuffer, "%.1f", chartTempMax);
    lv_label_set_text(lblChartMax, maxBuffer);
    lv_obj_align_to(lblChartMax, chartTemp, LV_ALIGN_OUT_TOP_LEFT, 0, 0);
    lv_obj_set_style_text_color(lblChartMax, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(lblChartMax, LV_FONT_DEFAULT, 0);

    // Error label positioned to avoid overlap with settings button
    lblErrorVal = lv_label_create(scrTelem);
    lv_label_set_text(lblErrorVal, "");
    lv_obj_align(lblErrorVal, LV_ALIGN_BOTTOM_MID, 0, -10);  // Moved to bottom center
    lv_obj_set_style_text_color(lblErrorVal, lv_color_hex(0xFF3333), 0);
    lv_obj_set_style_text_font(lblErrorVal, LV_FONT_DEFAULT, 0);

    // Override navigation button
    btnNavOverride = lv_btn_create(scrTelem);
    if (btnNavOverride) {
        lv_obj_set_size(btnNavOverride, 80, 30);
        // Move to top-left corner
        lv_obj_align(btnNavOverride, LV_ALIGN_TOP_LEFT, 0, 0);
        // Button label
        lv_obj_t *lblNavOverride = lv_label_create(btnNavOverride);
        lv_label_set_text(lblNavOverride, "Override");
        lv_obj_center(lblNavOverride);
        // Event callback navigates to override screen
        lv_obj_add_event_cb(btnNavOverride, [](lv_event_t* e) {
            safeSerialPrintln("DEBUG: Override button clicked");
            if (!scrOverride) {
                safeSerialPrintln("ERROR: scrOverride is null - cannot navigate");
                return;
            }
            safeSerialPrintln("DEBUG: scrOverride exists, calling safeNavigateToScreen");
            safeNavigateToScreen(scrOverride, "Override");
        }, LV_EVENT_CLICKED, nullptr);
    } else {
        safeSerialPrintln("ERROR: Failed to create override button!");
    }

    // Settings button positioned to avoid overlap - use global variable
    btnNavSettings = lv_btn_create(scrTelem);
    if (btnNavSettings == nullptr) {
        safeSerialPrintln("CRITICAL ERROR: Failed to create settings button!");
        return;
    }
    safeSerialPrintln("DEBUG: Settings navigation button created");
    lv_obj_set_size(btnNavSettings, 80, 30);
    lv_obj_align(btnNavSettings, LV_ALIGN_BOTTOM_RIGHT, -2, -10);  // Moved closer to edge
    lv_obj_t *lblNavSettings = lv_label_create(btnNavSettings);
    lv_label_set_text(lblNavSettings, "Settings");
    lv_obj_center(lblNavSettings);
    lv_obj_add_event_cb(btnNavSettings, [](lv_event_t* e) {
        safeSerialPrintln("DEBUG: Settings button clicked");
        if (scrSettings) {
            safeSerialPrintln("DEBUG: Requesting safe navigation to settings screen");
            safeNavigateToScreen(scrSettings, "Settings");
        } else {
            safeSerialPrintln("ERROR: scrSettings is null!");
        }
    }, LV_EVENT_CLICKED, nullptr);
    
    safeSerialPrintln("DEBUG: buildTelemetryScreen completed successfully");
}

void buildOverrideScreen() {
    safeSerialPrintln("DEBUG: Starting buildOverrideScreen");
    
    scrOverride = lv_obj_create(NULL);
    if (scrOverride == nullptr) {
        safeSerialPrintln("CRITICAL ERROR: Failed to create override screen!");
        return;
    }
    
    safeSerialPrintln("DEBUG: Override screen object created");
    lv_obj_set_style_bg_color(scrOverride, lv_color_hex(0x1a1a1a), 0);
    lv_obj_set_style_bg_opa(scrOverride, LV_OPA_COVER, 0);

    // Title - top right
    lv_obj_t *lblTitle = lv_label_create(scrOverride);
    lv_label_set_text(lblTitle, "Hot Wire Control");
    lv_obj_align(lblTitle, LV_ALIGN_TOP_RIGHT, -10, 5);
    lv_obj_set_style_text_font(lblTitle, LV_FONT_DEFAULT, 0);
    lv_obj_set_style_text_color(lblTitle, lv_color_hex(0x00FF00), 0);

    // Temperature section - left side
    lv_obj_t *lblTempLabel = lv_label_create(scrOverride);
    lv_label_set_text(lblTempLabel, "Temperature");
    lv_obj_align(lblTempLabel, LV_ALIGN_TOP_LEFT, 10, 35);
    lv_obj_set_style_text_color(lblTempLabel, lv_color_hex(0xAAAAAAA), 0);
    
    lblOverrideTemp = lv_label_create(scrOverride);
    if (lblOverrideTemp == nullptr) {
        safeSerialPrintln("CRITICAL ERROR: Failed to create lblOverrideTemp!");
        return;
    }
    lv_label_set_text(lblOverrideTemp, "0.0 C");
    lv_obj_align(lblOverrideTemp, LV_ALIGN_TOP_LEFT, 10, 50);
    lv_obj_set_style_text_color(lblOverrideTemp, lv_color_hex(0xFFFFFF), 0);

    // Power section - center
    lv_obj_t *lblPowerLabel = lv_label_create(scrOverride);
    lv_label_set_text(lblPowerLabel, "Power");
    lv_obj_align(lblPowerLabel, LV_ALIGN_TOP_MID, 0, 35);
    lv_obj_set_style_text_color(lblPowerLabel, lv_color_hex(0xAAAAAAA), 0);
    
    lblOverridePowerOutput = lv_label_create(scrOverride);
    if (lblOverridePowerOutput == nullptr) {
        safeSerialPrintln("CRITICAL ERROR: Failed to create lblOverridePowerOutput!");
        return;
    }
    lv_label_set_text(lblOverridePowerOutput, "0 (0.0%)");
    lv_obj_align(lblOverridePowerOutput, LV_ALIGN_TOP_MID, 0, 50);
    lv_obj_set_style_text_color(lblOverridePowerOutput, lv_color_hex(0xFFFFFF), 0);

    // RC Pulse display - right side
    lblOverrideRCPulse = lv_label_create(scrOverride);
    if (lblOverrideRCPulse == nullptr) {
        safeSerialPrintln("CRITICAL ERROR: Failed to create lblOverrideRCPulse!");
        return;
    }
    lv_label_set_text(lblOverrideRCPulse, "RC: No Signal");
    lv_obj_align(lblOverrideRCPulse, LV_ALIGN_TOP_RIGHT, -10, 35);
    lv_obj_set_style_text_color(lblOverrideRCPulse, lv_color_hex(0xFFFFFF), 0);

    // Mode status - hidden but kept for update function compatibility
    lblOverrideModeStatus = lv_label_create(scrOverride);
    if (lblOverrideModeStatus == nullptr) {
        safeSerialPrintln("CRITICAL ERROR: Failed to create lblOverrideModeStatus!");
        return;
    }
    lv_label_set_text(lblOverrideModeStatus, "");
    lv_obj_align(lblOverrideModeStatus, LV_ALIGN_TOP_LEFT, -1000, -1000); // Hide off-screen
    lv_obj_set_style_text_color(lblOverrideModeStatus, lv_color_hex(0x44FF44), 0);

    // Temperature control section - center, moved up 20px
    lv_obj_t *lblTempTitle = lv_label_create(scrOverride);
    lv_label_set_text(lblTempTitle, "Target Temperature:");
    lv_obj_align(lblTempTitle, LV_ALIGN_CENTER, 0, -10);
    lv_obj_set_style_text_color(lblTempTitle, lv_color_hex(0xFFFFFF), 0);

    // Temperature decrease button
    btnTempDown = lv_btn_create(scrOverride);
    lv_obj_set_size(btnTempDown, 50, 35);
    lv_obj_align(btnTempDown, LV_ALIGN_CENTER, -80, 20);
    lblTempDown = lv_label_create(btnTempDown);
    lv_label_set_text(lblTempDown, "-");
    lv_obj_center(lblTempDown);
    lv_obj_add_event_cb(btnTempDown, [](lv_event_t* e) {
        temperatureSetpoint = max(0.0, temperatureSetpoint - 5.0);
        pidEnabled = true;
        scheduleEepromSave();
        
        // Update the display immediately
        if (lblSliderVal) {
            char buf[16];
            sprintf(buf, "%.1f C", (float)temperatureSetpoint);
            lv_label_set_text(lblSliderVal, buf);
        }
    }, LV_EVENT_CLICKED, NULL);

    // Temperature display
    lblSliderVal = lv_label_create(scrOverride);
    char tempBuf[16];
    sprintf(tempBuf, "%.1f C", (float)temperatureSetpoint);
    lv_label_set_text(lblSliderVal, tempBuf);
    lv_obj_align(lblSliderVal, LV_ALIGN_CENTER, 0, 20);
    lv_obj_set_style_text_color(lblSliderVal, lv_color_hex(0x00FF00), 0);

    // Temperature increase button
    btnTempUp = lv_btn_create(scrOverride);
    lv_obj_set_size(btnTempUp, 50, 35);
    lv_obj_align(btnTempUp, LV_ALIGN_CENTER, 80, 20);
    lblTempUp = lv_label_create(btnTempUp);
    lv_label_set_text(lblTempUp, "+");
    lv_obj_center(lblTempUp);
    lv_obj_add_event_cb(btnTempUp, [](lv_event_t* e) {
        temperatureSetpoint = min(500.0, temperatureSetpoint + 5.0);
        pidEnabled = true;
        scheduleEepromSave();
        
        // Update the display immediately
        if (lblSliderVal) {
            char buf[16];
            sprintf(buf, "%.1f C", (float)temperatureSetpoint);
            lv_label_set_text(lblSliderVal, buf);
        }
    }, LV_EVENT_CLICKED, NULL);

    // Emergency stop button - bottom left
    lv_obj_t *btnEmergencyStop = lv_btn_create(scrOverride);
    lv_obj_set_size(btnEmergencyStop, 60, 30);
    lv_obj_align(btnEmergencyStop, LV_ALIGN_BOTTOM_LEFT, 10, -40);
    lv_obj_set_style_bg_color(btnEmergencyStop, lv_color_hex(0xFF0000), 0);
    lv_obj_t *lblEmergencyStop = lv_label_create(btnEmergencyStop);
    lv_label_set_text(lblEmergencyStop, "STOP");
    lv_obj_center(lblEmergencyStop);
    lv_obj_set_style_text_color(lblEmergencyStop, lv_color_hex(0xFFFFFF), 0);
    
    lv_obj_add_event_cb(btnEmergencyStop, [](lv_event_t* e) {
        manualPWM = 0;
        temperatureSetpoint = 0;
        pidEnabled = false;
        overrideMode = false;  // Release override mode
        if (lblSliderVal) {
            lv_label_set_text(lblSliderVal, "0.0 C");
        }
    }, LV_EVENT_CLICKED, nullptr);

    // Back button - bottom left
    lv_obj_t *btnBack = lv_btn_create(scrOverride);
    lv_obj_set_size(btnBack, 60, 25);
    lv_obj_align(btnBack, LV_ALIGN_BOTTOM_LEFT, 10, -5);
    lv_obj_t *lblBack = lv_label_create(btnBack);
    lv_label_set_text(lblBack, "Back");
    lv_obj_center(lblBack);
    lv_obj_add_event_cb(btnBack, [](lv_event_t* e) {
        // Simply navigate back - leave PID state as user set it
        safeNavigateToScreen(scrTelem, "Telemetry");
    }, LV_EVENT_CLICKED, nullptr);

    // PID enable button - bottom center left
    lv_obj_t *btnPidEnable = lv_btn_create(scrOverride);
    lv_obj_set_size(btnPidEnable, 80, 25);
    lv_obj_align(btnPidEnable, LV_ALIGN_BOTTOM_MID, -45, -5);
    lv_obj_t *lblPidEnable = lv_label_create(btnPidEnable);
    lv_label_set_text(lblPidEnable, pidEnabled ? "PID: ON" : "PID: OFF");
    lv_obj_center(lblPidEnable);
    lv_obj_add_event_cb(btnPidEnable, [](lv_event_t* e) {
        pidEnabled = !pidEnabled;
        lv_obj_t *btn = lv_event_get_target(e);
        lv_obj_t *lbl = lv_obj_get_child(btn, 0);
        lv_label_set_text(lbl, pidEnabled ? "PID: ON" : "PID: OFF");
        
        if (pidEnabled) {
            // Enable PID control - PID will take priority over RC automatically
            lv_obj_set_style_bg_color(btn, lv_color_hex(0x44FF44), 0);
        } else {
            // Disable PID control - returns to RC or manual control
            lv_obj_set_style_bg_color(btn, lv_color_hex(0x888888), 0);
        }
    }, LV_EVENT_CLICKED, nullptr);
    
    // Set initial PID button state
    if (pidEnabled) {
        lv_obj_set_style_bg_color(btnPidEnable, lv_color_hex(0x44FF44), 0);
    } else {
        lv_obj_set_style_bg_color(btnPidEnable, lv_color_hex(0x888888), 0);
    }
    
    // Safety reset button - bottom center right
    lv_obj_t *btnSafetyReset = lv_btn_create(scrOverride);
    lv_obj_set_size(btnSafetyReset, 80, 25);
    lv_obj_align(btnSafetyReset, LV_ALIGN_BOTTOM_MID, 45, -5);
    lv_obj_t *lblSafetyReset = lv_label_create(btnSafetyReset);
    lv_label_set_text(lblSafetyReset, "RESET");
    lv_obj_center(lblSafetyReset);
    lv_obj_set_style_bg_color(btnSafetyReset, lv_color_hex(0xFF4444), 0);  // Red button
    lv_obj_add_event_cb(btnSafetyReset, [](lv_event_t* e) {
        clearSafetyError();
        safeSerialPrintln("User cleared safety error from override screen");
    }, LV_EVENT_CLICKED, nullptr);
    
    // Mark override screen as fully initialized
    overrideScreenInitialized = true;
    
    safeSerialPrintln("DEBUG: buildOverrideScreen completed successfully");
}

void buildSettingsScreen() {
    // Safe serial debug - won't crash if serial disconnected
    
    scrSettings = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scrSettings, lv_color_hex(0x1a1a1a), 0);
    lv_obj_set_style_bg_opa(scrSettings, LV_OPA_COVER, 0);

    // Title with improved styling
    lv_obj_t *lblTitle = lv_label_create(scrSettings);
    lv_label_set_text(lblTitle, "Settings");
    lv_obj_align(lblTitle, LV_ALIGN_TOP_MID, 0, 8);
    lv_obj_set_style_text_font(lblTitle, LV_FONT_DEFAULT, 0);
    lv_obj_set_style_text_color(lblTitle, lv_color_hex(0x00FF00), 0);

    // Settings container for better organization
    lv_obj_t *container = lv_obj_create(scrSettings);
    lv_obj_set_size(container, 300, 160);
    lv_obj_align(container, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(container, lv_color_hex(0x2a2a2a), 0);
    lv_obj_set_style_border_color(container, lv_color_hex(0x404040), 0);
    lv_obj_set_style_border_width(container, 1, 0);
    lv_obj_set_style_radius(container, 5, 0);
    lv_obj_set_style_pad_all(container, 10, 0);

    // Wire Diameter Section
    lblSetWire = lv_label_create(container);
    char wireBuffer[32];
    sprintf(wireBuffer, "Wire Dia: %.1f mm", wireDiam);
    lv_label_set_text(lblSetWire, wireBuffer);
    lv_obj_align(lblSetWire, LV_ALIGN_TOP_LEFT, 0, 5);
    lv_obj_set_style_text_font(lblSetWire, LV_FONT_DEFAULT, 0);
    lv_obj_set_style_text_color(lblSetWire, lv_color_hex(0xFFFFFF), 0);

    btnWireLeft = lv_btn_create(container);
    lv_obj_set_size(btnWireLeft, 35, 25);
    lv_obj_align(btnWireLeft, LV_ALIGN_TOP_RIGHT, -80, 5);
    lv_obj_set_style_bg_color(btnWireLeft, lv_color_hex(0xFF4444), 0);
    lv_obj_t *lblWireLeft = lv_label_create(btnWireLeft);
    lv_label_set_text(lblWireLeft, "-");
    lv_obj_center(lblWireLeft);
    lv_obj_set_style_text_color(lblWireLeft, lv_color_hex(0xFFFFFF), 0);

    btnWireRight = lv_btn_create(container);
    lv_obj_set_size(btnWireRight, 35, 25);
    lv_obj_align(btnWireRight, LV_ALIGN_TOP_RIGHT, -40, 5);
    lv_obj_set_style_bg_color(btnWireRight, lv_color_hex(0x44FF44), 0);
    lv_obj_t *lblWireRight = lv_label_create(btnWireRight);
    lv_label_set_text(lblWireRight, "+");
    lv_obj_center(lblWireRight);
    lv_obj_set_style_text_color(lblWireRight, lv_color_hex(0xFFFFFF), 0);

    // Temperature Offset Section
    lblSetOffset = lv_label_create(container);
    char offsetBuffer[32];
    sprintf(offsetBuffer, "Temp Offset: %.1f C", tempOffset);
    lv_label_set_text(lblSetOffset, offsetBuffer);
    lv_obj_align(lblSetOffset, LV_ALIGN_TOP_LEFT, 0, 40);
    lv_obj_set_style_text_font(lblSetOffset, LV_FONT_DEFAULT, 0);
    lv_obj_set_style_text_color(lblSetOffset, lv_color_hex(0xFFFFFF), 0);

    btnOffsetLeft = lv_btn_create(container);
    lv_obj_set_size(btnOffsetLeft, 35, 25);
    lv_obj_align(btnOffsetLeft, LV_ALIGN_TOP_RIGHT, -80, 40);
    lv_obj_set_style_bg_color(btnOffsetLeft, lv_color_hex(0xFF4444), 0);
    lv_obj_t *lblOffsetLeft = lv_label_create(btnOffsetLeft);
    lv_label_set_text(lblOffsetLeft, "-");
    lv_obj_center(lblOffsetLeft);
    lv_obj_set_style_text_color(lblOffsetLeft, lv_color_hex(0xFFFFFF), 0);

    btnOffsetRight = lv_btn_create(container);
    lv_obj_set_size(btnOffsetRight, 35, 25);
    lv_obj_align(btnOffsetRight, LV_ALIGN_TOP_RIGHT, -40, 40);
    lv_obj_set_style_bg_color(btnOffsetRight, lv_color_hex(0x44FF44), 0);
    lv_obj_t *lblOffsetRight = lv_label_create(btnOffsetRight);
    lv_label_set_text(lblOffsetRight, "+");
    lv_obj_center(lblOffsetRight);
    lv_obj_set_style_text_color(lblOffsetRight, lv_color_hex(0xFFFFFF), 0);

    // Current Limit Section
    lblSetCurrent = lv_label_create(container);
    char currentBuffer[32];
    sprintf(currentBuffer, "Current Limit: %.1f A", currentLimit);
    lv_label_set_text(lblSetCurrent, currentBuffer);
    lv_obj_align(lblSetCurrent, LV_ALIGN_TOP_LEFT, 0, 75);
    lv_obj_set_style_text_font(lblSetCurrent, LV_FONT_DEFAULT, 0);
    lv_obj_set_style_text_color(lblSetCurrent, lv_color_hex(0xFFFFFF), 0);

    btnCurrentLeft = lv_btn_create(container);
    lv_obj_set_size(btnCurrentLeft, 35, 25);
    lv_obj_align(btnCurrentLeft, LV_ALIGN_TOP_RIGHT, -80, 75);
    lv_obj_set_style_bg_color(btnCurrentLeft, lv_color_hex(0xFF4444), 0);
    lv_obj_t *lblCurrentLeft = lv_label_create(btnCurrentLeft);
    lv_label_set_text(lblCurrentLeft, "-");
    lv_obj_center(lblCurrentLeft);
    lv_obj_set_style_text_color(lblCurrentLeft, lv_color_hex(0xFFFFFF), 0);

    btnCurrentRight = lv_btn_create(container);
    lv_obj_set_size(btnCurrentRight, 35, 25);
    lv_obj_align(btnCurrentRight, LV_ALIGN_TOP_RIGHT, -40, 75);
    lv_obj_set_style_bg_color(btnCurrentRight, lv_color_hex(0x44FF44), 0);
    lv_obj_t *lblCurrentRight = lv_label_create(btnCurrentRight);
    lv_label_set_text(lblCurrentRight, "+");
    lv_obj_center(lblCurrentRight);
    lv_obj_set_style_text_color(lblCurrentRight, lv_color_hex(0xFFFFFF), 0);

    // PID Kp Section
    lblSetPidKp = lv_label_create(container);
    char kpBuffer[32];
    sprintf(kpBuffer, "PID Kp: %.1f", pidKp);
    lv_label_set_text(lblSetPidKp, kpBuffer);
    lv_obj_align(lblSetPidKp, LV_ALIGN_TOP_LEFT, 0, 110);
    lv_obj_set_style_text_font(lblSetPidKp, LV_FONT_DEFAULT, 0);
    lv_obj_set_style_text_color(lblSetPidKp, lv_color_hex(0xFFDD44), 0);

    btnPidKpLeft = lv_btn_create(container);
    lv_obj_set_size(btnPidKpLeft, 35, 25);
    lv_obj_align(btnPidKpLeft, LV_ALIGN_TOP_RIGHT, -80, 110);
    lv_obj_set_style_bg_color(btnPidKpLeft, lv_color_hex(0xFF4444), 0);
    lv_obj_t *lblKpLeft = lv_label_create(btnPidKpLeft);
    lv_label_set_text(lblKpLeft, "-");
    lv_obj_center(lblKpLeft);
    lv_obj_set_style_text_color(lblKpLeft, lv_color_hex(0xFFFFFF), 0);

    btnPidKpRight = lv_btn_create(container);
    lv_obj_set_size(btnPidKpRight, 35, 25);
    lv_obj_align(btnPidKpRight, LV_ALIGN_TOP_RIGHT, -40, 110);
    lv_obj_set_style_bg_color(btnPidKpRight, lv_color_hex(0x44FF44), 0);
    lv_obj_t *lblKpRight = lv_label_create(btnPidKpRight);
    lv_label_set_text(lblKpRight, "+");
    lv_obj_center(lblKpRight);
    lv_obj_set_style_text_color(lblKpRight, lv_color_hex(0xFFFFFF), 0);

    // Navigation and control buttons
    lv_obj_t *btnBack = lv_btn_create(scrSettings);
    lv_obj_set_size(btnBack, 60, 30);
    lv_obj_align(btnBack, LV_ALIGN_BOTTOM_LEFT, 10, -10);
    lv_obj_set_style_bg_color(btnBack, lv_color_hex(0x666666), 0);
    lv_obj_t *lblBack = lv_label_create(btnBack);
    lv_label_set_text(lblBack, "Back");
    lv_obj_center(lblBack);
    lv_obj_set_style_text_color(lblBack, lv_color_hex(0xFFFFFF), 0);

    btnPidTune = lv_btn_create(scrSettings);
    lv_obj_set_size(btnPidTune, 80, 30);
    lv_obj_align(btnPidTune, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
    lv_obj_set_style_bg_color(btnPidTune, lv_color_hex(0x4444FF), 0);
    lv_obj_t *lblPidTune = lv_label_create(btnPidTune);
    lv_label_set_text(lblPidTune, "PID Tune");
    lv_obj_center(lblPidTune);
    lv_obj_set_style_text_color(lblPidTune, lv_color_hex(0xFFFFFF), 0);

    // Event handlers with long press support for faster adjustment
    lv_obj_add_event_cb(btnWireLeft, [](lv_event_t* e) {
        lv_event_code_t code = lv_event_get_code(e);
        float step = (code == LV_EVENT_LONG_PRESSED_REPEAT) ? WIRE_STEP * 5 : WIRE_STEP;
        wireDiam -= step;
        if (wireDiam < WIRE_MIN) wireDiam = WIRE_MIN;
        if (lblSetWire) {
            char buffer[32];
            sprintf(buffer, "Wire Dia: %.1f mm", wireDiam);
            lv_label_set_text(lblSetWire, buffer);
        }
        scheduleEepromSave();
    }, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_event_cb(btnWireLeft, [](lv_event_t* e) {
        lv_event_code_t code = lv_event_get_code(e);
        float step = (code == LV_EVENT_LONG_PRESSED_REPEAT) ? WIRE_STEP * 5 : WIRE_STEP;
        wireDiam -= step;
        if (wireDiam < WIRE_MIN) wireDiam = WIRE_MIN;
        if (lblSetWire) {
            char buffer[32];
            sprintf(buffer, "Wire Dia: %.1f mm", wireDiam);
            lv_label_set_text(lblSetWire, buffer);
        }
        scheduleEepromSave();
    }, LV_EVENT_LONG_PRESSED_REPEAT, nullptr);

    lv_obj_add_event_cb(btnWireRight, [](lv_event_t* e) {
        lv_event_code_t code = lv_event_get_code(e);
        float step = (code == LV_EVENT_LONG_PRESSED_REPEAT) ? WIRE_STEP * 5 : WIRE_STEP;
        wireDiam += step;
        if (wireDiam > WIRE_MAX) wireDiam = WIRE_MAX;
        if (lblSetWire) {
            char buffer[32];
            sprintf(buffer, "Wire Dia: %.1f mm", wireDiam);
            lv_label_set_text(lblSetWire, buffer);
        }
        scheduleEepromSave();
    }, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_event_cb(btnWireRight, [](lv_event_t* e) {
        lv_event_code_t code = lv_event_get_code(e);
        float step = (code == LV_EVENT_LONG_PRESSED_REPEAT) ? WIRE_STEP * 5 : WIRE_STEP;
        wireDiam += step;
        if (wireDiam > WIRE_MAX) wireDiam = WIRE_MAX;
        if (lblSetWire) {
            char buffer[32];
            sprintf(buffer, "Wire Dia: %.1f mm", wireDiam);
            lv_label_set_text(lblSetWire, buffer);
        }
        scheduleEepromSave();
    }, LV_EVENT_LONG_PRESSED_REPEAT, nullptr);

    lv_obj_add_event_cb(btnOffsetLeft, [](lv_event_t* e) {
        lv_event_code_t code = lv_event_get_code(e);
        float step = (code == LV_EVENT_LONG_PRESSED_REPEAT) ? OFF_STEP * 10 : OFF_STEP;
        tempOffset -= step;
        if (tempOffset < OFF_MIN) tempOffset = OFF_MIN;
        if (lblSetOffset) {
            char buffer[32];
            sprintf(buffer, "Temp Offset: %.1f C", tempOffset);
            lv_label_set_text(lblSetOffset, buffer);
        }
        scheduleEepromSave();
    }, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_event_cb(btnOffsetLeft, [](lv_event_t* e) {
        lv_event_code_t code = lv_event_get_code(e);
        float step = (code == LV_EVENT_LONG_PRESSED_REPEAT) ? OFF_STEP * 10 : OFF_STEP;
        tempOffset -= step;
        if (tempOffset < OFF_MIN) tempOffset = OFF_MIN;
        if (lblSetOffset) {
            char buffer[32];
            sprintf(buffer, "Temp Offset: %.1f C", tempOffset);
            lv_label_set_text(lblSetOffset, buffer);
        }
        scheduleEepromSave();
    }, LV_EVENT_LONG_PRESSED_REPEAT, nullptr);

    lv_obj_add_event_cb(btnOffsetRight, [](lv_event_t* e) {
        lv_event_code_t code = lv_event_get_code(e);
        float step = (code == LV_EVENT_LONG_PRESSED_REPEAT) ? OFF_STEP * 10 : OFF_STEP;
        tempOffset += step;
        if (tempOffset > OFF_MAX) tempOffset = OFF_MAX;
        if (lblSetOffset) {
            char buffer[32];
            sprintf(buffer, "Temp Offset: %.1f C", tempOffset);
            lv_label_set_text(lblSetOffset, buffer);
        }
        scheduleEepromSave();
    }, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_event_cb(btnOffsetRight, [](lv_event_t* e) {
        lv_event_code_t code = lv_event_get_code(e);
        float step = (code == LV_EVENT_LONG_PRESSED_REPEAT) ? OFF_STEP * 10 : OFF_STEP;
        tempOffset += step;
        if (tempOffset > OFF_MAX) tempOffset = OFF_MAX;
        if (lblSetOffset) {
            char buffer[32];
            sprintf(buffer, "Temp Offset: %.1f C", tempOffset);
            lv_label_set_text(lblSetOffset, buffer);
        }
        scheduleEepromSave();
    }, LV_EVENT_LONG_PRESSED_REPEAT, nullptr);

    lv_obj_add_event_cb(btnCurrentLeft, [](lv_event_t* e) {
        lv_event_code_t code = lv_event_get_code(e);
        float step = (code == LV_EVENT_LONG_PRESSED_REPEAT) ? CUR_STEP * 5 : CUR_STEP;
        currentLimit -= step;
        if (currentLimit < CUR_MIN) currentLimit = CUR_MIN;
        if (lblSetCurrent) {
            char buffer[32];
            sprintf(buffer, "Current Limit: %.1f A", currentLimit);
            lv_label_set_text(lblSetCurrent, buffer);
        }
        scheduleEepromSave();
    }, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_event_cb(btnCurrentLeft, [](lv_event_t* e) {
        lv_event_code_t code = lv_event_get_code(e);
        float step = (code == LV_EVENT_LONG_PRESSED_REPEAT) ? CUR_STEP * 5 : CUR_STEP;
        currentLimit -= step;
        if (currentLimit < CUR_MIN) currentLimit = CUR_MIN;
        if (lblSetCurrent) {
            char buffer[32];
            sprintf(buffer, "Current Limit: %.1f A", currentLimit);
            lv_label_set_text(lblSetCurrent, buffer);
        }
        scheduleEepromSave();
    }, LV_EVENT_LONG_PRESSED_REPEAT, nullptr);

    lv_obj_add_event_cb(btnCurrentRight, [](lv_event_t* e) {
        lv_event_code_t code = lv_event_get_code(e);
        float step = (code == LV_EVENT_LONG_PRESSED_REPEAT) ? CUR_STEP * 5 : CUR_STEP;
        currentLimit += step;
        if (currentLimit > CUR_MAX) currentLimit = CUR_MAX;
        if (lblSetCurrent) {
            char buffer[32];
            sprintf(buffer, "Current Limit: %.1f A", currentLimit);
            lv_label_set_text(lblSetCurrent, buffer);
        }
        scheduleEepromSave();
    }, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_event_cb(btnCurrentRight, [](lv_event_t* e) {
        lv_event_code_t code = lv_event_get_code(e);
        float step = (code == LV_EVENT_LONG_PRESSED_REPEAT) ? CUR_STEP * 5 : CUR_STEP;
        currentLimit += step;
        if (currentLimit > CUR_MAX) currentLimit = CUR_MAX;
        if (lblSetCurrent) {
            char buffer[32];
            sprintf(buffer, "Current Limit: %.1f A", currentLimit);
            lv_label_set_text(lblSetCurrent, buffer);
        }
        scheduleEepromSave();
    }, LV_EVENT_LONG_PRESSED_REPEAT, nullptr);

    // PID Kp adjustment
    lv_obj_add_event_cb(btnPidKpLeft, [](lv_event_t* e) {
        lv_event_code_t code = lv_event_get_code(e);
        float step = (code == LV_EVENT_LONG_PRESSED_REPEAT) ? 5.0f : 1.0f;
        pidKp -= step;
        if (pidKp < 0.1f) pidKp = 0.1f;
        tempPID.SetTunings(pidKp, pidKi, pidKd);
        char buffer[32];
        sprintf(buffer, "PID Kp: %.1f", pidKp);
        lv_label_set_text(lblSetPidKp, buffer);
        scheduleEepromSave();
    }, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_event_cb(btnPidKpLeft, [](lv_event_t* e) {
        lv_event_code_t code = lv_event_get_code(e);
        float step = (code == LV_EVENT_LONG_PRESSED_REPEAT) ? 5.0f : 1.0f;
        pidKp -= step;
        if (pidKp < 0.1f) pidKp = 0.1f;
        tempPID.SetTunings(pidKp, pidKi, pidKd);
        char buffer[32];
        sprintf(buffer, "PID Kp: %.1f", pidKp);
        lv_label_set_text(lblSetPidKp, buffer);
        scheduleEepromSave();
    }, LV_EVENT_LONG_PRESSED_REPEAT, nullptr);

    lv_obj_add_event_cb(btnPidKpRight, [](lv_event_t* e) {
        lv_event_code_t code = lv_event_get_code(e);
        float step = (code == LV_EVENT_LONG_PRESSED_REPEAT) ? 5.0f : 1.0f;
        pidKp += step;
        if (pidKp > 100.0f) pidKp = 100.0f;
        tempPID.SetTunings(pidKp, pidKi, pidKd);
        char buffer[32];
        sprintf(buffer, "PID Kp: %.1f", pidKp);
        lv_label_set_text(lblSetPidKp, buffer);
        scheduleEepromSave();
    }, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_event_cb(btnPidKpRight, [](lv_event_t* e) {
        lv_event_code_t code = lv_event_get_code(e);
        float step = (code == LV_EVENT_LONG_PRESSED_REPEAT) ? 5.0f : 1.0f;
        pidKp += step;
        if (pidKp > 100.0f) pidKp = 100.0f;
        tempPID.SetTunings(pidKp, pidKi, pidKd);
        char buffer[32];
        sprintf(buffer, "PID Kp: %.1f", pidKp);
        lv_label_set_text(lblSetPidKp, buffer);
        scheduleEepromSave();
    }, LV_EVENT_LONG_PRESSED_REPEAT, nullptr);

    // Navigation buttons
    lv_obj_add_event_cb(btnBack, [](lv_event_t* e) {
        safeSerialPrintln("DEBUG: Back button (settings) clicked");
        if (scrTelem) {
            safeSerialPrintln("DEBUG: Loading telemetry screen from settings");
            lv_scr_load(scrTelem);
            safeSerialPrintln("DEBUG: Telemetry screen loaded successfully from settings");
        } else {
            safeSerialPrintln("ERROR: scrTelem is null!");
        }
    }, LV_EVENT_CLICKED, nullptr);

    lv_obj_add_event_cb(btnPidTune, [](lv_event_t* e) {
        safeSerialPrintln("DEBUG: PID Tune button clicked");
        if (scrPIDTune) {
            safeSerialPrintln("DEBUG: Loading PID tune screen");
            lv_scr_load(scrPIDTune);
            safeSerialPrintln("DEBUG: PID tune screen loaded successfully");
        } else {
            safeSerialPrintln("ERROR: scrPIDTune is null!");
        }
    }, LV_EVENT_CLICKED, nullptr);
    
    // Settings screen build complete
}



void buildPIDTuneScreen() {
    scrPIDTune = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scrPIDTune, lv_color_hex(0x1a1a1a), 0);
    lv_obj_set_style_bg_opa(scrPIDTune, LV_OPA_COVER, 0);

    // Title
    lv_obj_t *lblTitle = lv_label_create(scrPIDTune);
    lv_label_set_text(lblTitle, "PID Tuning");
    lv_obj_align(lblTitle, LV_ALIGN_TOP_MID, 0, 8);
    lv_obj_set_style_text_font(lblTitle, LV_FONT_DEFAULT, 0);
    lv_obj_set_style_text_color(lblTitle, lv_color_hex(0x00FF00), 0);

    // Container for better organization
    lv_obj_t *container = lv_obj_create(scrPIDTune);
    lv_obj_set_size(container, 300, 180);
    lv_obj_align(container, LV_ALIGN_CENTER, 0, -10);
    lv_obj_set_style_bg_color(container, lv_color_hex(0x2a2a2a), 0);
    lv_obj_set_style_border_color(container, lv_color_hex(0x404040), 0);
    lv_obj_set_style_border_width(container, 1, 0);
    lv_obj_set_style_radius(container, 5, 0);
    lv_obj_set_style_pad_all(container, 15, 0);

    // Current PID values display
    lv_obj_t *lblCurrentPID = lv_label_create(container);
    lv_label_set_text(lblCurrentPID, "Current PID Values:");
    lv_obj_align(lblCurrentPID, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_style_text_color(lblCurrentPID, lv_color_hex(0xFFFFFF), 0);

    lblSetPidKp = lv_label_create(container);
    char kpBuffer[32];
    sprintf(kpBuffer, "Kp: %.2f", pidKp);
    lv_label_set_text(lblSetPidKp, kpBuffer);
    lv_obj_align(lblSetPidKp, LV_ALIGN_TOP_LEFT, 10, 25);
    lv_obj_set_style_text_color(lblSetPidKp, lv_color_hex(0xCCCCCC), 0);

    lblSetPidKi = lv_label_create(container);
    char kiBuffer[32];
    sprintf(kiBuffer, "Ki: %.3f", pidKi);
    lv_label_set_text(lblSetPidKi, kiBuffer);
    lv_obj_align(lblSetPidKi, LV_ALIGN_TOP_LEFT, 10, 45);
    lv_obj_set_style_text_color(lblSetPidKi, lv_color_hex(0xCCCCCC), 0);

    lblSetPidKd = lv_label_create(container);
    char kdBuffer[32];
    sprintf(kdBuffer, "Kd: %.2f", pidKd);
    lv_label_set_text(lblSetPidKd, kdBuffer);
    lv_obj_align(lblSetPidKd, LV_ALIGN_TOP_LEFT, 10, 65);
    lv_obj_set_style_text_color(lblSetPidKd, lv_color_hex(0xCCCCCC), 0);

    // Autotune status
    lblPidStatus = lv_label_create(container);
    lv_label_set_text(lblPidStatus, "Status: Ready");
    lv_obj_align(lblPidStatus, LV_ALIGN_TOP_LEFT, 0, 95);
    lv_obj_set_style_text_color(lblPidStatus, lv_color_hex(0x00FF00), 0);

    // Autotune button
    lv_obj_t *btnAutotune = lv_btn_create(container);
    lv_obj_set_size(btnAutotune, 120, 35);
    lv_obj_align(btnAutotune, LV_ALIGN_TOP_RIGHT, -10, 85);
    lv_obj_set_style_bg_color(btnAutotune, lv_color_hex(0x0080FF), 0);
    lv_obj_t *lblAutotune = lv_label_create(btnAutotune);
    lv_label_set_text(lblAutotune, "Start Autotune");
    lv_obj_center(lblAutotune);
    
    lv_obj_add_event_cb(btnAutotune, [](lv_event_t* e) {
        if (!autotuneActive) {
            // Start autotune at 200°C with gentle 40% output for hot wire
            float targetTemp = 200.0f;  // Fixed target for consistent tuning
            startPIDAutotune(targetTemp, 40.0f);  // Gentle power for gradual ramp
            
            if (lblPidStatus) {
                lv_label_set_text(lblPidStatus, "Status: Running...");
                lv_obj_set_style_text_color(lblPidStatus, lv_color_hex(0xFFAA00), 0);
            }
        }
    }, LV_EVENT_CLICKED, nullptr);

    // Manual adjustment buttons (Kp)
    lv_obj_t *lblKpAdjust = lv_label_create(container);
    lv_label_set_text(lblKpAdjust, "Manual Adjust:");
    lv_obj_align(lblKpAdjust, LV_ALIGN_TOP_LEFT, 0, 125);
    lv_obj_set_style_text_color(lblKpAdjust, lv_color_hex(0xFFFFFF), 0);

    // Kp adjustment
    btnPidKpLeft = lv_btn_create(container);
    lv_obj_set_size(btnPidKpLeft, 25, 25);
    lv_obj_align(btnPidKpLeft, LV_ALIGN_TOP_LEFT, 120, 120);
    lv_obj_t *lblKpLeft = lv_label_create(btnPidKpLeft);
    lv_label_set_text(lblKpLeft, "-");
    lv_obj_center(lblKpLeft);
    
    btnPidKpRight = lv_btn_create(container);
    lv_obj_set_size(btnPidKpRight, 25, 25);
    lv_obj_align(btnPidKpRight, LV_ALIGN_TOP_LEFT, 190, 120);
    lv_obj_t *lblKpRight = lv_label_create(btnPidKpRight);
    lv_label_set_text(lblKpRight, "+");
    lv_obj_center(lblKpRight);

    // Add event handlers for manual Kp adjustment
    lv_obj_add_event_cb(btnPidKpLeft, [](lv_event_t* e) {
        pidKp -= 1.0f;
        if (pidKp < 0.1f) pidKp = 0.1f;
        tempPID.SetTunings(pidKp, pidKi, pidKd);
        
        char buffer[32];
        sprintf(buffer, "Kp: %.2f", pidKp);
        lv_label_set_text(lblSetPidKp, buffer);
        scheduleEepromSave();
    }, LV_EVENT_CLICKED, nullptr);

    lv_obj_add_event_cb(btnPidKpRight, [](lv_event_t* e) {
        pidKp += 1.0f;
        if (pidKp > 100.0f) pidKp = 100.0f;
        tempPID.SetTunings(pidKp, pidKi, pidKd);
        
        char buffer[32];
        sprintf(buffer, "Kp: %.2f", pidKp);
        lv_label_set_text(lblSetPidKp, buffer);
        scheduleEepromSave();
    }, LV_EVENT_CLICKED, nullptr);

    // Back button
    lv_obj_t *btnBack = lv_btn_create(scrPIDTune);
    lv_obj_set_size(btnBack, 60, 30);
    lv_obj_align(btnBack, LV_ALIGN_BOTTOM_LEFT, 10, -10);
    lv_obj_t *lblBack = lv_label_create(btnBack);
    lv_label_set_text(lblBack, "Back");
    lv_obj_center(lblBack);
    lv_obj_add_event_cb(btnBack, [](lv_event_t* e) {
        safeSerialPrintln("DEBUG: Back button (PID tune) clicked");
        if (scrSettings) {
            safeSerialPrintln("DEBUG: Loading settings screen from PID tune");
            lv_scr_load(scrSettings);
            safeSerialPrintln("DEBUG: Settings screen loaded successfully from PID tune");
        } else {
            safeSerialPrintln("ERROR: scrSettings is null!");
        }
    }, LV_EVENT_CLICKED, nullptr);
}

// Update PID tune screen status
void updatePIDTuneStatus() {
    if (!lblPidStatus) return;
    
    char buffer[64];
    
    if (autotuneActive) {
        if (autotuneState == 1) {
            sprintf(buffer, "Status: Running (Peaks: %d)", autotuneNumPeaks);
            lv_obj_set_style_text_color(lblPidStatus, lv_color_hex(0xFFAA00), 0);
        }
    } else {
        if (autotuneState == 2) {
            sprintf(buffer, "Status: Complete! Ku=%.1f", autotuneKu);
            lv_obj_set_style_text_color(lblPidStatus, lv_color_hex(0x00FF00), 0);
        } else if (autotuneState == 3) {
            sprintf(buffer, "Status: Failed");
            lv_obj_set_style_text_color(lblPidStatus, lv_color_hex(0xFF0000), 0);
        } else {
            sprintf(buffer, "Status: Ready");
            lv_obj_set_style_text_color(lblPidStatus, lv_color_hex(0x00FF00), 0);
        }
    }
    
    lv_label_set_text(lblPidStatus, buffer);
    
    // Update PID value displays
    if (lblSetPidKp) {
        sprintf(buffer, "Kp: %.2f", pidKp);
        lv_label_set_text(lblSetPidKp, buffer);
    }
    if (lblSetPidKi) {
        sprintf(buffer, "Ki: %.3f", pidKi);
        lv_label_set_text(lblSetPidKi, buffer);
    }
    if (lblSetPidKd) {
        sprintf(buffer, "Kd: %.2f", pidKd);
        lv_label_set_text(lblSetPidKd, buffer);
    }
}

void updateTelemetry() {
    if (!lblVal[0] || !lblVal[1] || !lblVal[2] || !lblVal[3]) return;
    
    float temp = readNanoTemp();
    float displayTemp = temp;  // Use raw temperature without offset
    
    // Read actual current from G2 motor controller (variable ID 44)
    uint16_t currentRaw = readG2(44);  // Current in milliamps
    float actualCurrent = currentRaw / 1000.0f;  // Convert to amps
    
    // Use sprintf to format strings, then set text
    char buffer[32];
    
    sprintf(buffer, "%.1f C", displayTemp);  // Use offset-adjusted temperature for display
    lv_label_set_text(lblVal[0], buffer);
    
    sprintf(buffer, "%.1f C", (float)temperatureSetpoint);
    lv_label_set_text(lblVal[1], buffer);
    
    // Show current PWM output (from PID, RC, or manual)
    uint16_t currentPWM = overrideMode ? manualPWM : (uint16_t)outputPWM;
    sprintf(buffer, "%.0f", (float)currentPWM);
    lv_label_set_text(lblVal[2], buffer);
    
    sprintf(buffer, "%.2f A", actualCurrent);  // Show actual current from G2
    lv_label_set_text(lblVal[3], buffer);
    
    // Update safety status display
    if (lblErrorVal) {
        if (isSafetyErrorActive()) {
            lv_label_set_text(lblErrorVal, "🚨 SAFETY ERROR: HEATER FAILURE!");
            lv_obj_set_style_text_color(lblErrorVal, lv_color_hex(0xFF0000), 0);  // Red
        } else {
            lv_label_set_text(lblErrorVal, "System OK");
            lv_obj_set_style_text_color(lblErrorVal, lv_color_hex(0x00FF00), 0);  // Green
        }
    }
    
    // Update chart
    if (chartTemp && serTemp && serSetpoint) {
        // Plot current temperature
        lv_chart_set_next_value(chartTemp, serTemp, displayTemp);
        // Plot setpoint as horizontal reference line
        lv_chart_set_next_value(chartTemp, serSetpoint, (float)temperatureSetpoint);
        // Auto-scale chart range based on observed temperature
        if (displayTemp > chartTempMax) {
            chartTempMax = displayTemp;
            lv_chart_set_range(chartTemp, LV_CHART_AXIS_PRIMARY_Y, chartTempMin, chartTempMax);
        }
        if (displayTemp < chartTempMin) {
            chartTempMin = displayTemp;
            lv_chart_set_range(chartTemp, LV_CHART_AXIS_PRIMARY_Y, chartTempMin, chartTempMax);
        }
    }
    
    if (lblChartMin) {
        sprintf(buffer, "%.1f", chartTempMin);
        lv_label_set_text(lblChartMin, buffer);
    }
    if (lblChartMax) {
        sprintf(buffer, "%.1f", chartTempMax);
        lv_label_set_text(lblChartMax, buffer);
    }
    
    // Update mode indicator banner
    if (lblBanner) {
        uint32_t rcPulse = readRCPulse();
        if (autotuneActive) {
            sprintf(buffer, "AUTOTUNE RUNNING - Peaks: %d", autotuneNumPeaks);
        } else if (overrideMode) {
            sprintf(buffer, "MANUAL MODE - Power: %d", manualPWM);
        } else if (rcPulse >= 900 && rcPulse <= 2100) {
            sprintf(buffer, "RC MODE - Pulse: %lu us", rcPulse);
        } else if (pidEnabled && temperatureSetpoint > 0) {
            sprintf(buffer, "PID MODE - Target: %.1f C", (float)temperatureSetpoint);
        } else {
            sprintf(buffer, "AUTO MODE - Wire Off");
        }
        lv_label_set_text(lblBanner, buffer);
    }
    
    // Update PID tune screen if active
    updatePIDTuneStatus();
}

// Update override screen with live data
void updateOverrideScreen() {
    // Live update override screen now enabled

    // Only update if we're currently on the override screen and it's fully initialized
    if (lv_scr_act() != scrOverride || !scrOverride || !overrideScreenInitialized) {
        return;
    }
    
    // Additional safety check - verify all critical UI objects exist and are valid LVGL objects
    if (!lblOverrideTemp || !lblOverrideRCPulse || !lblOverrideModeStatus || 
        !lblSliderVal || !lblOverridePowerOutput) {
        safeSerialPrintln("ERROR: updateOverrideScreen - one or more UI objects are null!");
        safeSerialPrintf("DEBUG: lblOverrideTemp=%p, lblOverrideRCPulse=%p, lblOverrideModeStatus=%p\n", 
                        lblOverrideTemp, lblOverrideRCPulse, lblOverrideModeStatus);
        safeSerialPrintf("DEBUG: lblSliderVal=%p, lblOverridePowerOutput=%p\n", 
                        lblSliderVal, lblOverridePowerOutput);
        return;
    }
    
    // Verify LVGL objects are still valid by checking if they have a parent
    if (!lblOverrideTemp->parent || !lblOverrideRCPulse->parent || !lblOverrideModeStatus->parent ||
        !lblSliderVal->parent || !lblOverridePowerOutput->parent) {
        safeSerialPrintln("ERROR: updateOverrideScreen - LVGL objects have been freed!");
        overrideScreenInitialized = false;  // Mark as corrupted
        return;
    }
    
    // Update temperature display with exception handling
    try {
        if (lblOverrideTemp) {
            float temp = readNanoTemp();
            char buf[32];
            sprintf(buf, "%.1f C", temp);
            lv_label_set_text(lblOverrideTemp, buf);
            
            // Color code temperature based on value
            if (temp > 300) {
                lv_obj_set_style_text_color(lblOverrideTemp, lv_color_hex(0xFF4444), 0); // Red for high temp
            } else if (temp > 150) {
                lv_obj_set_style_text_color(lblOverrideTemp, lv_color_hex(0xFFAA00), 0); // Orange for medium temp
            } else {
                lv_obj_set_style_text_color(lblOverrideTemp, lv_color_hex(0xFFFFFF), 0); // White for low temp
            }
        }
    } catch (...) {
        safeSerialPrintln("ERROR: Exception in updateOverrideScreen temperature update");
        overrideScreenInitialized = false;
        return;
    }
    
    // Update RC pulse display
    if (lblOverrideRCPulse) {
        uint32_t rcPulse = readRCPulse();
        char buf[32];
        
        if (rcPulse >= 900 && rcPulse <= 2100) {
            sprintf(buf, "RC: %luµs", rcPulse);
            lv_obj_set_style_text_color(lblOverrideRCPulse, lv_color_hex(0xAAFFAA), 0); // Green for valid RC
        } else {
            sprintf(buf, "RC: No Signal");
            lv_obj_set_style_text_color(lblOverrideRCPulse, lv_color_hex(0xFF6666), 0); // Red for no signal
        }
        lv_label_set_text(lblOverrideRCPulse, buf);
    }
    
    // Update mode status based on current state
    if (lblOverrideModeStatus) {
        if (isSafetyErrorActive()) {
            lv_label_set_text(lblOverrideModeStatus, "🚨 SAFETY ERROR");
            lv_obj_set_style_text_color(lblOverrideModeStatus, lv_color_hex(0xFF0000), 0); // Red for safety error
        } else if (autotuneActive) {
            lv_label_set_text(lblOverrideModeStatus, "AUTOTUNE");
            lv_obj_set_style_text_color(lblOverrideModeStatus, lv_color_hex(0xFFFF00), 0); // Yellow for autotune
        } else if (overrideMode) {
            lv_label_set_text(lblOverrideModeStatus, "MANUAL");
            lv_obj_set_style_text_color(lblOverrideModeStatus, lv_color_hex(0xFF6666), 0); // Red for manual
        } else {
            uint32_t rcPulse = readRCPulse();
            if (rcPulse >= 900 && rcPulse <= 2100) {
                lv_label_set_text(lblOverrideModeStatus, "RC CTRL");
                lv_obj_set_style_text_color(lblOverrideModeStatus, lv_color_hex(0x66AAFF), 0); // Blue for RC control
            } else if (pidEnabled && temperatureSetpoint > 0) {
                lv_label_set_text(lblOverrideModeStatus, "PID CTRL");
                lv_obj_set_style_text_color(lblOverrideModeStatus, lv_color_hex(0x44FF44), 0); // Green for PID
            } else {
                lv_label_set_text(lblOverrideModeStatus, "AUTO");
                lv_obj_set_style_text_color(lblOverrideModeStatus, lv_color_hex(0x888888), 0); // Gray for auto/off
            }
        }
    }
    
    // Update power output indicator
    if (lblOverridePowerOutput) {
        uint16_t currentPower;
        char buf[32];
        
        if (overrideMode) {
            currentPower = manualPWM;
            sprintf(buf, "%u (%.1f%%)", currentPower, (currentPower/3200.0f)*100.0f);
        } else {
            uint32_t rcPulse = readRCPulse();
            if (rcPulse >= 900 && rcPulse <= 2100) {
                currentPower = map(rcPulse, 1000, 2000, 0, 3200);
                sprintf(buf, "%u (%.1f%%) RC", currentPower, (currentPower/3200.0f)*100.0f);
            } else if (pidEnabled && temperatureSetpoint > 0) {
                currentPower = (uint16_t)outputPWM;
                sprintf(buf, "%u (%.1f%%) PID", currentPower, (currentPower/3200.0f)*100.0f);
            } else {
                currentPower = 0;
                sprintf(buf, "OFF");
            }
        }
        
        lv_label_set_text(lblOverridePowerOutput, buf);
        
        // Color code based on power level
        if (currentPower > 2400) { // > 75%
            lv_obj_set_style_text_color(lblOverridePowerOutput, lv_color_hex(0xFF4444), 0); // Red for high power
        } else if (currentPower > 1600) { // > 50%
            lv_obj_set_style_text_color(lblOverridePowerOutput, lv_color_hex(0xFFAA00), 0); // Orange for medium power
        } else if (currentPower > 0) {
            lv_obj_set_style_text_color(lblOverridePowerOutput, lv_color_hex(0xFFFF00), 0); // Yellow for low power
        } else {
            lv_obj_set_style_text_color(lblOverridePowerOutput, lv_color_hex(0x888888), 0); // Gray for off
        }
    }
    
    // Update temperature setpoint display (replacing slider functionality)
    if (lblSliderVal) {
        char buf[16];
        sprintf(buf, "%.1f C", (float)temperatureSetpoint);
        lv_label_set_text(lblSliderVal, buf);
        
        // Color code based on whether PID is active
        if (pidEnabled && temperatureSetpoint > 0) {
            lv_obj_set_style_text_color(lblSliderVal, lv_color_hex(0x00FF00), 0); // Green when PID active
        } else {
            lv_obj_set_style_text_color(lblSliderVal, lv_color_hex(0x00FFFF), 0); // Cyan when inactive
        }
    }
}

//------------------------------------------------------------------------------
// Splash Screen and Screensaver Functions

void showSplashScreen() {
  // Create splash screen
  lv_obj_t *splashScreen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(splashScreen, lv_color_hex(0x000000), 0); // Black background
  lv_obj_set_style_bg_opa(splashScreen, LV_OPA_COVER, 0);
  
  // Large "P" in purple - matching screensaver positioning exactly
  lv_obj_t *lblP = lv_label_create(splashScreen);
  lv_label_set_text(lblP, "P");
  lv_obj_align(lblP, LV_ALIGN_CENTER, -84, -30);  // Moved left 2px more (-82-2=-84)
  lv_obj_set_style_text_font(lblP, &lv_font_montserrat_48, 0);
  lv_obj_set_style_text_color(lblP, lv_color_hex(0x8A2BE2), 0); // Purple
  lv_obj_set_style_pad_all(lblP, 0, 0); // Remove all padding to match screensaver

  // "ickle" part - matching screensaver positioning exactly
  lv_obj_t *lblPickle = lv_label_create(splashScreen);
  lv_label_set_text(lblPickle, "ickle");
  lv_obj_align(lblPickle, LV_ALIGN_CENTER, -30, -26);  // P(-65) + 35 = -30, Y(-40) + 14 = -26
  lv_obj_set_style_text_font(lblPickle, &lv_font_montserrat_32, 0);
  lv_obj_set_style_text_color(lblPickle, lv_color_hex(0x00FF00), 0); // Green
  lv_obj_set_style_pad_all(lblPickle, 0, 0); // Remove all padding to match screensaver

  // Large "W" in purple - matching screensaver positioning exactly
  lv_obj_t *lblW = lv_label_create(splashScreen);
  lv_label_set_text(lblW, "W");
  lv_obj_align(lblW, LV_ALIGN_CENTER, 41, -30);  // Moved left 2px (43-2=41)
  lv_obj_set_style_text_font(lblW, &lv_font_montserrat_48, 0);
  lv_obj_set_style_text_color(lblW, lv_color_hex(0x8A2BE2), 0); // Purple
  lv_obj_set_style_pad_all(lblW, 0, 0); // Remove all padding to match screensaver

  // "ire" part - matching screensaver positioning exactly
  lv_obj_t *lblWire = lv_label_create(splashScreen);
  lv_label_set_text(lblWire, "ire");
  lv_obj_align(lblWire, LV_ALIGN_CENTER, 85, -26);  // P(-65) + 160 = 95, moved left 10px (95-10=85), Y(-40) + 14 = -26
  lv_obj_set_style_text_font(lblWire, &lv_font_montserrat_32, 0);
  lv_obj_set_style_text_color(lblWire, lv_color_hex(0x00FF00), 0); // Green
  lv_obj_set_style_pad_all(lblWire, 0, 0); // Remove all padding to match screensaver
  
  // Subtitle - using bigger font
  lv_obj_t *lblSubtitle = lv_label_create(splashScreen);
  lv_label_set_text(lblSubtitle, "Hotwire Controller");
  lv_obj_align(lblSubtitle, LV_ALIGN_CENTER, 0, 0);  // Adjusted position
  lv_obj_set_style_text_font(lblSubtitle, &lv_font_montserrat_22, 0);
  lv_obj_set_style_text_color(lblSubtitle, lv_color_hex(0xFFFFFF), 0); // White
  
  // Version info
  lv_obj_t *lblVersion = lv_label_create(splashScreen);
  lv_label_set_text(lblVersion, "v2.40 - ESP32S3 + AD8495");
  lv_obj_align(lblVersion, LV_ALIGN_CENTER, 0, 25);
  lv_obj_set_style_text_font(lblVersion, LV_FONT_DEFAULT, 0);
  lv_obj_set_style_text_color(lblVersion, lv_color_hex(0x888888), 0); // Gray
  
  // Feature highlights
  lv_obj_t *lblFeatures = lv_label_create(splashScreen);
  lv_label_set_text(lblFeatures, "PID Control • Safety Monitor • K-Type Sensor");
  lv_obj_align(lblFeatures, LV_ALIGN_CENTER, 0, 45);
  lv_obj_set_style_text_font(lblFeatures, LV_FONT_DEFAULT, 0);
  lv_obj_set_style_text_color(lblFeatures, lv_color_hex(0x00AAFF), 0); // Light blue
  
  // Loading message with animation effect
  lv_obj_t *lblLoading = lv_label_create(splashScreen);
  lv_label_set_text(lblLoading, ">>> Initializing System...");
  lv_obj_align(lblLoading, LV_ALIGN_BOTTOM_MID, 0, -15);
  lv_obj_set_style_text_font(lblLoading, LV_FONT_DEFAULT, 0);
  lv_obj_set_style_text_color(lblLoading, lv_color_hex(0x00FFFF), 0); // Cyan
  
  // Load splash screen
  lv_scr_load(splashScreen);
  
  // Process LVGL to display the splash
  lv_tick_inc(50);
  lv_timer_handler();
  
  // Animate W from purple to orange over 5 seconds
  uint32_t animationStart = millis();
  uint32_t animationDuration = 5000; // 5 seconds
  
  while (millis() - animationStart < animationDuration) {
    uint32_t elapsed = millis() - animationStart;
    float progress = (float)elapsed / (float)animationDuration; // 0.0 to 1.0
    
    // Interpolate from purple (0x8A2BE2) to orange (0xFF8000)
    uint8_t purple_r = 0x8A, purple_g = 0x2B, purple_b = 0xE2;
    uint8_t orange_r = 0xFF, orange_g = 0x80, orange_b = 0x00;
    
    uint8_t current_r = purple_r + (uint8_t)((orange_r - purple_r) * progress);
    uint8_t current_g = purple_g + (uint8_t)((orange_g - purple_g) * progress);
    uint8_t current_b = purple_b + (uint8_t)((orange_b - purple_b) * progress);
    
    // Add "hot" effects in the last 2 seconds
    if (elapsed >= 3000) { // Last 2 seconds (3000ms to 5000ms)
      float hotProgress = (float)(elapsed - 3000) / 2000.0; // 0.0 to 1.0 over last 2 seconds
      
      // Create pulsing/flickering effect
      float pulseFreq = 8.0; // 8 pulses per second for hot flickering
      float pulse = sin(elapsed * pulseFreq * 2.0 * PI / 1000.0); // -1 to 1
      float pulseIntensity = 0.3 * hotProgress; // Increase intensity over time
      
      // Add red/yellow "heat" flicker
      uint8_t heat_boost = (uint8_t)(pulse * pulseIntensity * 40); // ±40 color units
      current_r = constrain(current_r + heat_boost, 0, 255);
      current_g = constrain(current_g + heat_boost/2, 0, 255); // Less green boost
      
      // Add slight random flicker for realistic heat effect
      if (elapsed % 100 < 50) { // Every 100ms, flicker for 50ms
        current_r = constrain(current_r + (int8_t)(random(-20, 30)), 0, 255);
        current_g = constrain(current_g + (int8_t)(random(-10, 15)), 0, 255);
      }
    }
    
    uint32_t current_color = (current_r << 16) | (current_g << 8) | current_b;
    
    // Update W color
    lv_obj_set_style_text_color(lblW, lv_color_hex(current_color), 0);
    
    // Process LVGL to update display
    lv_tick_inc(50);
    lv_timer_handler();
    
    delay(50); // Update every 50ms for smooth animation
  }
  
  safeSerialPrintln("Enhanced splash screen displayed with stylized text and W glow animation");
}

void createScreensaver() {
  // Simple overlay approach - create screensaver as an overlay on current screen
  if (screensaverScreen) {
    lv_obj_del(screensaverScreen);  // Clean up if it exists
  }
  
  // Create screensaver as a full-screen overlay on the current screen
  screensaverScreen = lv_obj_create(lv_scr_act()); // Create on current screen, not as new screen
  lv_obj_set_size(screensaverScreen, LV_HOR_RES , LV_VER_RES );
  lv_obj_set_pos(screensaverScreen, 0, 0);
  lv_obj_set_style_bg_color(screensaverScreen, lv_color_hex(0x000000), 0);
  lv_obj_set_style_bg_opa(screensaverScreen, LV_OPA_COVER, 0);
  
  // Remove scroll bars completely
  lv_obj_clear_flag(screensaverScreen, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_scrollbar_mode(screensaverScreen, LV_SCROLLBAR_MODE_OFF);
  
  // Create the "P" in purple to match splash screen exactly
  screensaverP = lv_label_create(screensaverScreen);
  lv_label_set_text(screensaverP, "P");
  lv_obj_set_style_text_font(screensaverP, &lv_font_montserrat_48, 0);  // Use splash screen font
  lv_obj_set_style_text_color(screensaverP, lv_color_hex(0x8A2BE2), 0); // Purple
  lv_obj_set_style_pad_all(screensaverP, 0, 0); // Remove all padding
  
  // Create "ickle" in green to match splash screen exactly
  screensaverPickle = lv_label_create(screensaverScreen);
  lv_label_set_text(screensaverPickle, "ickle");
  lv_obj_set_style_text_font(screensaverPickle, &lv_font_montserrat_32, 0);  // Use splash screen font
  lv_obj_set_style_text_color(screensaverPickle, lv_color_hex(0x00FF00), 0); // Green
  lv_obj_set_style_pad_all(screensaverPickle, 0, 0); // Remove all padding
  
  // Create the "W" in purple to match splash screen exactly
  screensaverW = lv_label_create(screensaverScreen);
  lv_label_set_text(screensaverW, "W");
  lv_obj_set_style_text_font(screensaverW, &lv_font_montserrat_48, 0);  // Use splash screen font
  lv_obj_set_style_text_color(screensaverW, lv_color_hex(0x8A2BE2), 0); // Purple
  lv_obj_set_style_pad_all(screensaverW, 0, 0); // Remove all padding
  
  // Create "ire" in green to match splash screen exactly
  screensaverWire = lv_label_create(screensaverScreen);
  lv_label_set_text(screensaverWire, "ire");
  lv_obj_set_style_text_font(screensaverWire, &lv_font_montserrat_32, 0);  // Use splash screen font
  lv_obj_set_style_text_color(screensaverWire, lv_color_hex(0x00FF00), 0); // Green
  lv_obj_set_style_pad_all(screensaverWire, 0, 0); // Remove all padding
  
  // Initialize starting position - center the logo with proper bounds
  screensaverX = 60;   // Start position (more centered, away from edges)
  screensaverY = 80;   // Start position (more centered, away from edges)
  screensaverVelX = 1; // Slower horizontal movement to reduce flashing
  screensaverVelY = 1; // Slower vertical movement to reduce flashing
  
  safeSerialPrintln("Screensaver created as overlay with styled logo");
}

void updateScreensaver() {
  if (!screensaverActive || !screensaverScreen || !screensaverP) return;
  
  // Update position
  screensaverX += screensaverVelX;
  screensaverY += screensaverVelY;
  
  // Calculate proper bounds for the complete logo - using exact splash screen fonts
  // Updated measurements based on corrected positioning to prevent overlaps
  const int totalWidth = 165;  // Total width from P start to ire end (125 + ~40 for "ire")
  const int totalHeight = 50; // Height of largest font (montserrat_48)
  
  // Screen dimensions (320x240) - ensure logo stays completely on screen
  const int screenWidth = 320;
  const int screenHeight = 240;
  
  // Bounce off edges at exact screen boundaries - with padding to go slightly off-screen
  const int leftPadding = -15;  // Allow logo to go 15px off left edge before bouncing
  const int topPadding = -17;   // Allow logo to go 10px off top edge before bouncing (reduced by 5px)
  const int rightReduction = 58; // Reduce right boundary by 50px
  const int bottomReduction = 13; // Reduce bottom boundary by 10px
  
  if (screensaverX <= leftPadding || screensaverX >= (screenWidth - totalWidth - rightReduction)) {
    screensaverVelX = -screensaverVelX;
    screensaverX = constrain(screensaverX, leftPadding, screenWidth - totalWidth - rightReduction);
    safeSerialPrintf("SCREENSAVER: Horizontal bounce at X=%d (bounds: %d to %d), logo width=%d\n", screensaverX, leftPadding, screenWidth - totalWidth - rightReduction, totalWidth);
  }
  if (screensaverY <= topPadding || screensaverY >= (screenHeight - totalHeight - bottomReduction)) {
    screensaverVelY = -screensaverVelY;
    screensaverY = constrain(screensaverY, topPadding, screenHeight - totalHeight - bottomReduction);
    safeSerialPrintf("SCREENSAVER: Vertical bounce at Y=%d (bounds: %d to %d), logo height=%d\n", screensaverY, topPadding, screenHeight - totalHeight - bottomReduction, totalHeight);
  }
  
  // Position all logo elements relative to base position - fixed spacing to prevent overlaps
  if (screensaverP) {
    lv_obj_set_pos(screensaverP, screensaverX, screensaverY); // Base position
  }
  
  if (screensaverPickle) {
    lv_obj_set_pos(screensaverPickle, screensaverX + 35, screensaverY + 14); // After P, moved down 10px (4+10=14)
  }
  
  if (screensaverW) {
    lv_obj_set_pos(screensaverW, screensaverX + 110, screensaverY); // After "Pickle" with gap
  }
  
  if (screensaverWire) {
    lv_obj_set_pos(screensaverWire, screensaverX + 160, screensaverY + 14); // After W, moved down 10px (4+10=14)
  }
}

void exitScreensaver() {
  if (!screensaverActive) return;
  
  safeSerialPrintln("Exiting screensaver - simple overlay removal");
  screensaverActive = false;
  
  // Simply delete the overlay - the underlying screen remains intact
  if (screensaverScreen) {
    lv_obj_del(screensaverScreen);
    screensaverScreen = nullptr;
    screensaverP = nullptr;
    screensaverPickle = nullptr;
    screensaverW = nullptr;
    screensaverWire = nullptr;
  }
  
  safeSerialPrintln("Screensaver exit complete - overlay removed");
}

void resetIdleTimer() {
  uint32_t now = millis();
  uint32_t previousIdleTime = (now >= lastActivityTime) ? (now - lastActivityTime) : 0;
  lastActivityTime = now;
  
  // Debug: Track what's causing idle timer resets
  static uint32_t lastResetLog = 0;
  if (now - lastResetLog >= 1000) {  // Log resets at most once per second
    lastResetLog = now;
    safeSerialPrintf("IDLE RESET: Previous idle time was %lums\n", previousIdleTime);
  }
  
  // If screensaver is active, exit it from the main thread context
  if (screensaverActive) {
    safeSerialPrintln("Activity detected - scheduling screensaver exit");
    exitScreensaver();
    // Add a grace period to prevent immediate reactivation
    lastActivityTime = now + 2000; // 2 second grace period
  }
}
