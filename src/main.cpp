#include <Arduino.h>
#include <lvgl.h>
#include <ESP_Panel_Library.h>
#include <ESP_IOExpander_Library.h>
#include "ui.h"
#include "esp_log.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <Update.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <ArduinoJson.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <esp_ota_ops.h>  // Include ESP-IDF OTA operations
#include <ModbusRTU.h>    // Include emelianov's ModbusRTU library
#include <esp_system.h>
#include <esp_partition.h>
#include <mbedtls/sha256.h>

// Extend IO Pin define
#define TP_RST 1
#define LCD_BL 2
#define LCD_RST 3
#define SD_CS 4
#define USB_SEL 5

// I2C Pin define
#define I2C_MASTER_NUM 0
#define I2C_MASTER_SDA_IO 8
#define I2C_MASTER_SCL_IO 9

/* LVGL porting configurations */
#define LVGL_TICK_PERIOD_MS     (2)
#define LVGL_TASK_MAX_DELAY_MS  (500)
#define LVGL_TASK_MIN_DELAY_MS  (5) // Adjusted delay for stability
#define LVGL_TASK_STACK_SIZE    (10 * 1024) // Increased stack size for LVGL
#define LVGL_TASK_PRIORITY      (2)
#define LVGL_BUF_SIZE           (ESP_PANEL_LCD_H_RES * 40)



/* Define constants for the program data */
#define MAX_STEPS 10  // Adjust as needed
#define MAX_PROGRAMS 6  // Total number of programs
#define NAMESPACE "programs"  // NVS namespace for program data

#define MAX_CUSTOM_PERIODS 3  // Number of periods in custom mode

typedef struct {
    uint32_t times[MAX_CUSTOM_PERIODS];       // Time durations for each period in seconds
    uint16_t speeds[MAX_CUSTOM_PERIODS + 1];  // Speeds at the boundaries of periods
    bool rotations[MAX_CUSTOM_PERIODS];       // Rotation values for each period
} CustomProgramData;


// OTA update server details
#define CURRENT_FIRMWARE_VERSION "1.0.0"
const char* host = "https://schleiermacher34.pythonanywhere.com/firmware/check_update/";

/* Task and Semaphore Handles */
TaskHandle_t otaTaskHandle = NULL;
SemaphoreHandle_t lvgl_mux = NULL; // LVGL mutex
SemaphoreHandle_t wifi_connect_semaphore = NULL;
SemaphoreHandle_t wifi_scan_semaphore = NULL;
SemaphoreHandle_t otaSemaphore = NULL;
SemaphoreHandle_t save_program_semaphore = NULL; // Semaphore for saving program
SemaphoreHandle_t run_program_semaphore = NULL;
SemaphoreHandle_t nvs_mutex;
SemaphoreHandle_t wifi_mutex;
SemaphoreHandle_t modbus_mutex;

/* Mode definitions */
#define MODE_MANUAL 0
#define MODE_AUTO   1
#define MODE_CUSTOM 2

// Current mode variable
uint8_t currentMode = MODE_MANUAL;  // Start with Manual mode

/* Define StepData struct */
typedef struct {
    uint16_t speed;       // Speed value
    bool rotation;        // Rotation direction (true for forward, false for reverse)
    uint32_t time;        // Time in seconds
} StepData;

/* Define ProgramData struct */
typedef struct {
    uint8_t programNumber;    // Program number
    StepData steps[MAX_STEPS]; // Array of steps
    uint8_t stepCount;         // Number of steps in the program
} ProgramData;

/* Define other structs as needed */
typedef struct {
    uint16_t speed;    // Speed value
    bool rotation;     // Rotation direction (true for forward, false for reverse)
} ManualModeSettings;

typedef struct {
    uint8_t programNumber;  // Selected program number
    // Add other settings as needed
} AutoModeSettings;

typedef struct {
    uint8_t programNumber;
    // Add other custom settings as needed
} CustomModeSettings;

// Create instances of the settings structures
ManualModeSettings manualModeSettings;
AutoModeSettings autoModeSettings;
CustomModeSettings customModeSettings;

// Rest of your code...

ESP_Panel *panel = NULL;

/* Modbus Configuration */
#define MODBUS_TX_PIN 44        // Example GPIO pin for UART1 TX
#define MODBUS_RX_PIN 43      // Example GPIO pin for UART1 RX
#define MODBUS_DE_RE_PIN 4      // GPIO4 (DE/RE control pin)

HardwareSerial ModbusSerial(1); // Use UART1
ModbusRTU mb;                   // ModbusRTU instance

#define MAX_SPEED_RPM 60        // Maximum speed in RPM
#define MIN_SPEED_RPM 10        // Minimum speed in RPM
#define MOTOR_POLES 2           // Number of motor poles (adjust as per your motor)
#define BASE_FREQUENCY 50.0     // Base frequency in Hertz (adjust as per your motor)

volatile bool actualSpeedUpdated = false;
volatile uint16_t actualSpeedValue = 0;
uint16_t actualSpeedRegisterValue = 0;

// Global variables for UI updates
volatile bool modbusErrorFlag = false;
volatile uint8_t modbusErrorCode = 0;
volatile bool mototimeUpdated = false;
volatile uint16_t mototimeValue = 0;

#define MODBUS_SLAVE_ID 1
#define VFD_REG_CONTROL 0x2000
#define VFD_REG_SPEED   0x2001
#define VFD_REG_ROTATION 0x2002

/* Function to Convert RPM to Hertz */
float rpmToHertz(int rpm) {
    return (float)rpm * 60.0;
}

#if ESP_PANEL_LCD_BUS_TYPE == ESP_PANEL_BUS_TYPE_RGB
/* Display flushing */
void lvgl_port_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    panel->getLcd()->drawBitmap(area->x1, area->y1, area->x2 + 1, area->y2 + 1, color_p);
    lv_disp_flush_ready(disp);
}
#else
/* Display flushing */
void lvgl_port_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    panel->getLcd()->drawBitmap(area->x1, area->y1, area->x2 + 1, area->y2 + 1, color_p);
}

bool notify_lvgl_flush_ready(void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}
#endif /* ESP_PANEL_LCD_BUS_TYPE */

#if ESP_PANEL_USE_LCD_TOUCH
/* Read the touchpad */
void lvgl_port_tp_read(lv_indev_drv_t * indev, lv_indev_data_t * data)
{
    panel->getLcdTouch()->readData();

    bool touched = panel->getLcdTouch()->getTouchState();
    if(!touched) {
        data->state = LV_INDEV_STATE_REL;
    } else {
        TouchPoint point = panel->getLcdTouch()->getPoint();

        data->state = LV_INDEV_STATE_PR;
        /*Set the coordinates*/
        data->point.x = point.x;
        data->point.y = point.y;

        Serial.printf("Touch point: x %d, y %d\n", point.x, point.y);
    }
}
#endif

void lvgl_port_lock(int timeout_ms)
{
    const TickType_t timeout_ticks = (timeout_ms < 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    xSemaphoreTakeRecursive(lvgl_mux, timeout_ticks);
}

void lvgl_port_unlock(void)
{
    xSemaphoreGiveRecursive(lvgl_mux);
}

bool ota_in_progress = false;

void lvgl_port_task(void *arg) {
    Serial.println("Starting LVGL task");

    uint32_t task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
    while (1) {
        lvgl_port_lock(-1); // Lock LVGL due to non-thread-safe API

        // Handle LVGL timers
        task_delay_ms = lv_timer_handler();

        // Check and handle Modbus error updates
        if (modbusErrorFlag) {
            lv_label_set_text_fmt(ui_Label45, "Modbus error: 0x%02X", modbusErrorCode);
            modbusErrorFlag = false; // Reset the flag after handling
        }

        // Check and handle mototime updates
        if (mototimeUpdated) {
            lv_label_set_text_fmt(ui_Label45, " %u", mototimeValue);
            mototimeUpdated = false; // Reset the flag after handling
        }
        // Update actual speed label
        if (actualSpeedUpdated) {
            float actualFrequencyHz = (float)actualSpeedValue / 100.0; // Assuming the value is in 0.01 Hz units
            lv_label_set_text_fmt(ui_Label45, "Speed: %.2f Hz", actualFrequencyHz);
            actualSpeedUpdated = false;
        }

        lvgl_port_unlock();

        // Adjust task delay
        if (task_delay_ms > LVGL_TASK_MAX_DELAY_MS) {
            task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
        } else if (task_delay_ms < LVGL_TASK_MIN_DELAY_MS) {
            task_delay_ms = LVGL_TASK_MIN_DELAY_MS;
        }

        if (ota_in_progress) {
            vTaskDelay(pdMS_TO_TICKS(100)); // Increase delay during OTA to reduce screen updates
        } else {
            vTaskDelay(pdMS_TO_TICKS(LVGL_TASK_MIN_DELAY_MS));
        }

        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

/* UI Elements */
// Settings Screen (Screen1)
extern lv_obj_t *ui_starstopbutton;
extern lv_obj_t *ui_Label1;
extern lv_obj_t *ui_Label2;
extern lv_obj_t *ui_rotationbutton;
extern lv_obj_t *ui_Label4;
extern lv_obj_t *ui_Label13;

bool modbusCallback(Modbus::ResultCode event, uint16_t transactionId, void* data) {
    if (event == Modbus::EX_SUCCESS) {
        Serial.println("Modbus transaction successful.");
    } else {
        Serial.printf("Modbus error: %02X\n", event);
    }
    return true;
}
void stopMotor() {
    uint16_t result = mb.writeHreg(MODBUS_SLAVE_ID, VFD_REG_CONTROL, 0, modbusCallback);

    if (result == 0) {
        Serial.println("Motor stop command sent successfully.");
    } else {
        Serial.printf("Failed to send motor stop command. Modbus error: %d\n", result);
    }
}

#define CUSTOM_NAMESPACE "custom_program"  // NVS namespace for custom program data

esp_err_t saveCustomProgramToNVS(const CustomProgramData* customData) {
    nvs_handle_t nvsHandle;
    esp_err_t err;

    // Open NVS handle in read/write mode
    err = nvs_open(CUSTOM_NAMESPACE, NVS_READWRITE, &nvsHandle);
    if (err != ESP_OK) {
        Serial.printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return err;
    }

    // Save the custom program data as a blob
    size_t dataSize = sizeof(CustomProgramData);
    err = nvs_set_blob(nvsHandle, "custom_data", customData, dataSize);
    if (err != ESP_OK) {
        Serial.printf("Error (%s) saving custom program data!\n", esp_err_to_name(err));
    } else {
        // Commit changes
        err = nvs_commit(nvsHandle);
        if (err != ESP_OK) {
            Serial.printf("Error (%s) committing NVS changes!\n", esp_err_to_name(err));
        } else {
            Serial.println("Custom program saved to NVS.");
        }
    }

    // Close NVS handle
    nvs_close(nvsHandle);
    return err;
}




uint32_t getTimeFromUILabel(lv_obj_t* label) {
    const char* text = lv_label_get_text(label);
    return (uint32_t)atoi(text);  // Assuming time is in seconds
}

uint16_t getSpeedFromUILabel(lv_obj_t* label) {
    const char* text = lv_label_get_text(label);
    return (uint16_t)atoi(text);  // Speed in RPM
}

bool getRotationFromUI(lv_obj_t* rotationObj) {
    // Assuming rotationObj is a switch or button that represents rotation state
    // Return true for forward, false for reverse
    return lv_obj_has_state(rotationObj, LV_STATE_CHECKED);
}
void collectCustomProgramData(CustomProgramData* customData) {
    // Replace the ui_* variables with your actual UI element variables

    // Collect times
    customData->times[0] = getTimeFromUILabel(ui_time1);
    customData->times[1] = getTimeFromUILabel(ui_time3);
    customData->times[2] = getTimeFromUILabel(ui_time4);

    // Collect speeds
    customData->speeds[0] = getSpeedFromUILabel(ui_rpmvalue);
    customData->speeds[1] = getSpeedFromUILabel(ui_rpmvalue2);
    customData->speeds[2] = getSpeedFromUILabel(ui_rpmvalue3);
    customData->speeds[3] = getSpeedFromUILabel(ui_rpmvalue4);

    // Collect rotations
    customData->rotations[0] = getRotationFromUI(ui_rotationbutton1);
    customData->rotations[1] = getRotationFromUI(ui_rotationbutton3);
    customData->rotations[2] = getRotationFromUI(ui_rotationbutton4);
}

void event_handler_custom_value_changed(lv_event_t* e) {
    // Collect data and save
    CustomProgramData customData;
    collectCustomProgramData(&customData);
    esp_err_t err = saveCustomProgramToNVS(&customData);
    if (err == ESP_OK) {
        Serial.println("Custom program saved automatically.");
    } else {
        Serial.println("Failed to save custom program.");
    }
}

esp_err_t loadCustomProgramFromNVS(CustomProgramData* customData) {
    nvs_handle_t nvsHandle;
    esp_err_t err;

    // Open NVS handle in read-only mode
    err = nvs_open(CUSTOM_NAMESPACE, NVS_READONLY, &nvsHandle);
    if (err != ESP_OK) {
        Serial.printf("Error (%s) opening NVS handle for reading!\n", esp_err_to_name(err));
        return err;
    }

    // Determine the required size of the data
    size_t dataSize = sizeof(CustomProgramData);

    // Load the custom program data
    err = nvs_get_blob(nvsHandle, "custom_data", customData, &dataSize);
    if (err == ESP_OK) {
        Serial.println("Custom program loaded from NVS.");
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        Serial.println("Custom program not found in NVS.");
    } else {
        Serial.printf("Error (%s) loading custom program data!\n", esp_err_to_name(err));
    }

    // Close NVS handle
    nvs_close(nvsHandle);
    return err;
}


esp_err_t saveAutoProgramToNVS(uint8_t programNumber, const ProgramData* programData) {
    if (programNumber == 0 || programNumber > MAX_PROGRAMS) {
        Serial.println("Invalid program number.");
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvsHandle;
    esp_err_t err;

    // Open NVS handle in read/write mode
    err = nvs_open(NAMESPACE, NVS_READWRITE, &nvsHandle);
    if (err != ESP_OK) {
        Serial.printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return err;
    }

    // Create a key for the program number
    char key[16];
    snprintf(key, sizeof(key), "program_%d", programNumber);

    // Save the program data as a blob
    size_t dataSize = sizeof(ProgramData);
    err = nvs_set_blob(nvsHandle, key, programData, dataSize);
    if (err != ESP_OK) {
        Serial.printf("Error (%s) saving program data!\n", esp_err_to_name(err));
    } else {
        // Commit changes
        err = nvs_commit(nvsHandle);
        if (err != ESP_OK) {
            Serial.printf("Error (%s) committing NVS changes!\n", esp_err_to_name(err));
        } else {
            Serial.printf("Program %d saved to NVS.\n", programNumber);
        }
    }

    // Close NVS handle
    nvs_close(nvsHandle);
    return err;
}


esp_err_t loadProgramFromNVS(uint8_t programNumber, ProgramData* programData) {
    if (programNumber == 0 || programNumber > MAX_PROGRAMS) {
        Serial.println("Invalid program number.");
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvsHandle;
    esp_err_t err;

    // Open NVS handle in read-only mode
    err = nvs_open(NAMESPACE, NVS_READONLY, &nvsHandle);
    if (err != ESP_OK) {
        Serial.printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return err;
    }

    // Create a key for the program number
    char key[16];
    snprintf(key, sizeof(key), "program_%d", programNumber);

    // Determine the required size of the data
    size_t requiredSize = sizeof(ProgramData);

    // Load the program data
    err = nvs_get_blob(nvsHandle, key, programData, &requiredSize);
    if (err == ESP_OK) {
        Serial.printf("Program %d loaded from NVS.\n", programNumber);
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        Serial.printf("Program %d not found in NVS.\n", programNumber);
    } else {
        Serial.printf("Error (%s) loading program data!\n", esp_err_to_name(err));
    }

    // Close NVS handle
    nvs_close(nvsHandle);
    return err;
}

void saveCurrentProgramData() {
    // Get the current program number from ui_Label35
    const char* programNumberText = lv_label_get_text(ui_Label35);
    uint8_t programNumber = atoi(programNumberText);

    if (programNumber == 0 || programNumber > MAX_PROGRAMS) {
        Serial.println("Invalid program number.");
        return;
    }

    ProgramData programData;
    programData.programNumber = programNumber;
    programData.stepCount = 1; // Assuming one step for simplicity

    // Initialize the first step
    StepData step;

    // Get speed from ui_Label30
    const char* speedText = lv_label_get_text(ui_Label30);
    step.speed = atoi(speedText);

    // Get rotation from ui_Label29 and ui_Label32
    bool rotationForward = !lv_obj_has_flag(ui_Label32, LV_OBJ_FLAG_HIDDEN);
    step.rotation = rotationForward;

    // Get time from ui_Label31
    const char* timeText = lv_label_get_text(ui_Label31);
    step.time = atoi(timeText);

    // Add the step to the program data
    programData.steps[0] = step;

    // Save the program to NVS
    esp_err_t err = saveAutoProgramToNVS(programNumber, &programData);
    if (err == ESP_OK) {
        Serial.printf("Program %d saved successfully.\n", programNumber);
    } else {
        Serial.printf("Failed to save program %d.\n", programNumber);
    }
}

void runCustomProgram(const CustomProgramData* customData) {
    for (int i = 0; i < MAX_CUSTOM_PERIODS; i++) {
        uint16_t startSpeed = customData->speeds[i];
        uint16_t endSpeed = customData->speeds[i + 1];
        uint32_t periodTime = customData->times[i];
        bool rotation = customData->rotations[i];

        Serial.printf("Period %d: Start Speed=%d RPM, End Speed=%d RPM, Time=%d seconds, Rotation=%s\n",
                      i + 1, startSpeed, endSpeed, periodTime, rotation ? "Forward" : "Reverse");

        // Set rotation
        uint16_t rotationValue = rotation ? 0 : 1; // Adjust based on your rotation commands
        mb.writeHreg(MODBUS_SLAVE_ID, VFD_REG_ROTATION, rotationValue, modbusCallback);

        // Calculate speed increment per step
        const uint32_t rampSteps = 100; // Adjust for desired smoothness
        uint32_t stepDelay = (periodTime * 1000) / rampSteps; // Convert to milliseconds
        float speedIncrement = (float)(endSpeed - startSpeed) / rampSteps;

        // Start motor
        uint16_t commandValue = rotation ? 34 : 18; // Command to start motor
        mb.writeHreg(MODBUS_SLAVE_ID, VFD_REG_CONTROL, commandValue, modbusCallback);

        for (uint32_t step = 0; step <= rampSteps; step++) {
            uint16_t currentSpeed = startSpeed + (uint16_t)(speedIncrement * step);
            mb.writeHreg(MODBUS_SLAVE_ID, VFD_REG_SPEED, currentSpeed, modbusCallback);
            vTaskDelay(pdMS_TO_TICKS(stepDelay));
        }

        // Stop the motor between periods if necessary
        stopMotor();
    }
    Serial.println("Custom program execution completed.");
}

uint16_t getSpeedFromUI() {
    const char *speedText = lv_label_get_text(ui_Label4);
    return atoi(speedText);
}

bool getRotationFromUI() {
    // Check if ui_Label5 is hidden
    if (lv_obj_has_flag(ui_Label5, LV_OBJ_FLAG_HIDDEN)) {
        // If ui_Label5 is hidden, rotation is in reverse
        return false;  // Reverse rotation
    }

    // Check if ui_Label6 is hidden
    if (lv_obj_has_flag(ui_Label6, LV_OBJ_FLAG_HIDDEN)) {
        // If ui_Label6 is hidden, rotation is forward
        return true;  // Forward rotation
    }

    // Default to forward rotation if neither label is hidden (optional)
    return true;
}

void toggleRotation() {
    if (getRotationFromUI()) {
        // If current rotation is forward, set to reverse
        lv_obj_add_flag(ui_Label6, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(ui_Label5, LV_OBJ_FLAG_HIDDEN);
    } else {
        // If current rotation is reverse, set to forward
        lv_obj_add_flag(ui_Label5, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(ui_Label6, LV_OBJ_FLAG_HIDDEN);
    }
}


void setSpeedToUI(uint16_t speed) {
    char speedText[10];
    snprintf(speedText, sizeof(speedText), "%d", speed);
    lv_label_set_text(ui_Label4, speedText);
}

void setRotationToUI(bool rotation) {
    if (rotation) {
        // Forward rotation
        lv_obj_add_flag(ui_Label5, LV_OBJ_FLAG_HIDDEN);  // Hide "Reverse" label
        lv_obj_clear_flag(ui_Label6, LV_OBJ_FLAG_HIDDEN); // Show "Forward" label
    } else {
        // Reverse rotation
        lv_obj_add_flag(ui_Label6, LV_OBJ_FLAG_HIDDEN);  // Hide "Forward" label
        lv_obj_clear_flag(ui_Label5, LV_OBJ_FLAG_HIDDEN); // Show "Reverse" label
    }

    // Optionally update ui_rotationbutton or other UI elements if needed
    Serial.printf("Rotation set to %s\n", rotation ? "Forward" : "Reverse");
}

uint8_t getProgramNumberFromUI(bool isCustomMode) {
    const char *labelText = nullptr;

    if (isCustomMode) {
        // Get the text from ui_Label21 for custom mode
        labelText = lv_label_get_text(ui_Label21);
    } else {
        // Get the text from ui_Label11 for auto mode
        labelText = lv_label_get_text(ui_Label11);
    }

    if (labelText == nullptr) {
        Serial.println("Error: Label text is NULL");
        return 0;  // Default program number if text is invalid
    }

    // Extract the program number from the label text (assuming format like "0/1", "1/2", etc.)
    char programNumber[3] = {0};
    strncpy(programNumber, labelText, 1);  // Extract only the first character
    uint8_t selectedProgramNumber = atoi(programNumber);  // Convert to integer

    Serial.printf("Selected Program Number: %u (%s Mode)\n", selectedProgramNumber, isCustomMode ? "Custom" : "Auto");

    return selectedProgramNumber;
}


void setProgramNumberToUI(uint8_t programNumber, bool isCustomMode) {
    char programText[8];  // Buffer to hold the formatted program text
    snprintf(programText, sizeof(programText), "%u/1", programNumber);

    if (isCustomMode) {
        // Set the program number to ui_Label21 for custom mode
        if (ui_Label21 != nullptr) {
            lv_label_set_text(ui_Label21, programText);
            Serial.printf("Set program number to %u for Custom mode\n", programNumber);
        } else {
            Serial.println("Error: ui_Label21 is NULL");
        }
    } else {
        // Set the program number to ui_Label11 for auto mode
        if (ui_Label11 != nullptr) {
            lv_label_set_text(ui_Label11, programText);
            Serial.printf("Set program number to %u for Auto mode\n", programNumber);
        } else {
            Serial.println("Error: ui_Label11 is NULL");
        }
    }
}


void saveCurrentModeSettings() {
    switch (currentMode) {
        case MODE_MANUAL:
            manualModeSettings.speed = getSpeedFromUI();
            manualModeSettings.rotation = getRotationFromUI();
            break;
        case MODE_AUTO:
            autoModeSettings.programNumber = getProgramNumberFromUI(false);
            break;
        case MODE_CUSTOM:
            customModeSettings.programNumber = getProgramNumberFromUI(true);
            break;
        default:
            break;
    }
}






void runProgram(const ProgramData* program) {
    Serial.printf("Running Program %d with %d steps.\n", program->programNumber, program->stepCount);

    for (uint8_t i = 0; i < program->stepCount; i++) {
        const StepData* step = &program->steps[i];
        Serial.printf("Step %d: Speed=%d, Rotation=%s, Time=%d\n",
                      i + 1, step->speed, step->rotation ? "Forward" : "Backward", step->time);

        // Set speed
        mb.writeHreg(MODBUS_SLAVE_ID, VFD_REG_SPEED, step->speed, modbusCallback);

        // Set rotation and start motor
        uint16_t commandValue = step->rotation ? 34 : 18;
        mb.writeHreg(MODBUS_SLAVE_ID, VFD_REG_CONTROL, commandValue, modbusCallback);

        // Wait for the specified time
        vTaskDelay(pdMS_TO_TICKS(step->time * 1000));

        // Stop the motor between steps if necessary
        stopMotor();
    }
    Serial.println("Program execution completed.");
}




void event_handler_start_motor_button(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED) {

        _ui_flag_modify(ui_Label1, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_TOGGLE);
        _ui_flag_modify(ui_Image1, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_TOGGLE);
        _ui_flag_modify(ui_Label2, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_TOGGLE);
        _ui_flag_modify(ui_Image2, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_TOGGLE);
        _ui_flag_modify(ui_Spinner1, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_TOGGLE);

        Serial.println("Start/Stop button pressed...");

        // Determine the current mode based on UI elements
        uint8_t newMode;
        bool modebutton_hidden = lv_obj_has_flag(ui_modebutton, LV_OBJ_FLAG_HIDDEN);
        bool modebutton4_hidden = lv_obj_has_flag(ui_modebutton4, LV_OBJ_FLAG_HIDDEN);
        bool modebutton7_hidden = lv_obj_has_flag(ui_modebutton7, LV_OBJ_FLAG_HIDDEN);

        if (modebutton4_hidden && modebutton7_hidden && !modebutton_hidden) {
            newMode = MODE_MANUAL;
            Serial.println("Mode determined: Manual");
        } else if (modebutton_hidden && modebutton7_hidden && !modebutton4_hidden) {
            newMode = MODE_AUTO;
            Serial.println("Mode determined: Auto");
        } else if (modebutton_hidden && modebutton4_hidden && !modebutton7_hidden) {
            newMode = MODE_CUSTOM;
            Serial.println("Mode determined: Custom");
        } else {
            newMode = currentMode;
            Serial.println("Mode could not be determined based on UI elements. Keeping previous mode.");
        }

        // Stop motor if mode changed
        if (currentMode != newMode) {
            stopMotor();
            Serial.println("Motor stopped due to mode change.");
            currentMode = newMode;
        }

        if (currentMode == MODE_AUTO) {
            // Load and run the program
            const char* programNumberText = lv_label_get_text(ui_Label35);
            uint8_t programNumber = atoi(programNumberText);

            if (programNumber == 0 || programNumber > MAX_PROGRAMS) {
                Serial.println("Invalid program number.");
                return;
            }

            ProgramData program;
            esp_err_t err = loadProgramFromNVS(programNumber, &program);
            if (err == ESP_OK) {
                runProgram(&program);
            } else {
                Serial.printf("Failed to load program %d.\n", programNumber);
            }

        } else if (currentMode == MODE_MANUAL) {
            // Manual mode logic
            uint16_t speed = getSpeedFromUI();
            bool rotation = getRotationFromUI();

            Serial.printf("Manual Mode: Speed=%d, Rotation=%s\n", speed, rotation ? "Forward" : "Reverse");

            // Set rotation
            uint16_t rotationValue = rotation ? 0 : 1;
            uint16_t result = mb.writeHreg(MODBUS_SLAVE_ID, VFD_REG_ROTATION, rotationValue, modbusCallback);
            Serial.printf("Set rotation command result: %d\n", result);

            // Set speed
            result = mb.writeHreg(MODBUS_SLAVE_ID, VFD_REG_SPEED, speed, modbusCallback);
            Serial.printf("Set speed command result: %d\n", result);

            // Start motor
            uint16_t commandValue = rotation ? 34 : 18;
            result = mb.writeHreg(MODBUS_SLAVE_ID, VFD_REG_CONTROL, commandValue, modbusCallback);
            Serial.printf("Start motor command result: %d\n", result);

        } else if (currentMode == MODE_CUSTOM) {
            // Load and run the custom program
            CustomProgramData customData;
            esp_err_t err = loadCustomProgramFromNVS(&customData);
            if (err == ESP_OK) {
                runCustomProgram(&customData);
            } else {
                Serial.println("Failed to load custom program.");
            }
        }
    }
}



void rotationChangedEventHandler(lv_event_t * e) {
    if (currentMode == MODE_MANUAL) {
        bool currentRotation = getRotationFromUI();
        uint16_t rotationValue = currentRotation ? 0 : 1;
        xSemaphoreTake(modbus_mutex, portMAX_DELAY);
        uint16_t result = mb.writeHreg(MODBUS_SLAVE_ID, VFD_REG_ROTATION, rotationValue, modbusCallback);
        xSemaphoreGive(modbus_mutex);
        if (result == 0) {
            Serial.printf("Rotation changed to %s.\n", currentRotation ? "Forward" : "Reverse");
        } else {
            Serial.printf("Failed to change rotation. Error: %d\n", result);
        }
    }
}



void modbusLoopTask(void *pvParameters) {
    while (1) {
        xSemaphoreTake(modbus_mutex, portMAX_DELAY);
        mb.task();
        xSemaphoreGive(modbus_mutex);
        vTaskDelay(pdMS_TO_TICKS(10)); // Adjust delay as needed
    }
}








// Forward declarations
// void initializeSerial();
// void event_handler_serial_input_save(lv_event_t *e);
// bool validateSerialNumber(const char *serial);
// void saveSerialToNVS(const String& serial);
// String readSerialFromNVS();
// void pushSerialToServer(const String& serial);


// void event_handler_serial_input_save(lv_event_t *e);
// String collectLogs();
// void pushLogsToServer();
// void server_connection_task(void *pvParameters);
// bool validateSerialNumber(const char *serial);
// void initializeSerial();
// void saveSerialToNVS(const String& serial);
// String readSerialFromNVS();
// void pushSerialToServer(const String& serial);
// bool verifyChecksum(WiFiClient* stream, int contentLength, const char* expectedChecksum);
// void reduce_display_refresh_rate();
// void restore_display_refresh_rate();
// void hide_ui_elements_for_ota();
// void show_ui_elements_after_ota();
// void create_ota_progress_bar();
// void update_ota_progress_bar(int progress);
// void remove_ota_progress_bar();
// void performOTAUpdate(const String& firmwareUrl);
// void checkForUpdates();
// void otaUpdateTask(void* parameter);
// void event_handler_ota_update(lv_event_t * e);
// void event_handler_save_program_button(lv_event_t * e);
// bool actualSpeedCallback(Modbus::ResultCode event, uint16_t transactionId, void* data);
// void vfdActualSpeedReadTask(void *pvParameters);
// bool modbusCallback(Modbus::ResultCode event, uint16_t transactionId, void* data);
// esp_err_t saveProgramToNVS();





// esp_err_t loadProgramFromNVS(uint16_t programIndex, ProgramData *programData);
// void runProgram(const ProgramData *programData);
// void runProgramTask(void *pvParameters);
// esp_err_t saveMotorConfigToNVS(uint8_t motorID, const MotorConfig *config);
// void event_handler_save_motor_config(lv_event_t * e);
// esp_err_t loadMotorConfigFromNVS(uint8_t motorID, MotorConfig *config);
// void event_handler_motor_selection(lv_event_t * e);
// void event_handler_start_motor_button(lv_event_t * e);
// void event_handler_stop_motor_button(lv_event_t * e);
// void event_handler_change_speed_button(lv_event_t * e);
// void update_arc_values_from_label();
// void saveWifiCredentialsToNVS(const char* ssid, const char* password);
// bool readWifiCredentialsFromNVS(char* ssid, size_t ssid_size, char* password, size_t password_size);
void wifi_connect_task(void *pvParameters) {
    while (1) {
        // Wait for the semaphore to be given
        if (xSemaphoreTake(wifi_connect_semaphore, portMAX_DELAY)) {
            Serial.println("Wi-Fi connect task started.");

            char ssid[64] = {0};
            char password[64] = {0};

            // Lock LVGL before accessing UI elements
            lvgl_port_lock(portMAX_DELAY);

            // Get the selected SSID from the dropdown
            if (ui_Roller1 != NULL) {
                lv_roller_get_selected_str(ui_Roller1, ssid, sizeof(ssid));
                Serial.print("Selected SSID: ");
                Serial.println(ssid);
            } else {
                Serial.println("Error: ui_Roller1 is NULL");
                lvgl_port_unlock();
                continue; // Skip this iteration
            }

            // Get the password from the TextArea
            const char *password_tmp = NULL;
            if (ui_passwordarea != NULL) {
                password_tmp = lv_textarea_get_text(ui_passwordarea);
            } else {
                Serial.println("Error: ui_Screen2_TextArea2 is NULL");
                lvgl_port_unlock();
                continue; // Skip this iteration
            }

            // Unlock LVGL after accessing UI elements
            lvgl_port_unlock();

            // Check if password_tmp is NULL
            if (password_tmp == NULL) {
                Serial.println("Error: Password input is NULL");
                password[0] = '\0';
            } else {
                strncpy(password, password_tmp, sizeof(password) - 1);
                password[sizeof(password) - 1] = '\0';
            }

            // Check if SSID is empty
            if (strlen(ssid) == 0) {
                Serial.println("Error: SSID is empty");
                lvgl_port_lock(portMAX_DELAY);
                lv_label_set_text(ui_Label45, "SSID is empty. Please select a network.");
                lvgl_port_unlock();
                continue; // Skip this iteration
            }

            Serial.print("Attempting to connect to SSID: ");
            Serial.println(ssid);

            // Acquire the Wi-Fi mutex before accessing Wi-Fi functions
            if (xSemaphoreTake(wifi_mutex, portMAX_DELAY)) {
                WiFi.begin(ssid, password);

                // Wait for connection
                int attempts = 0;
                while (WiFi.status() != WL_CONNECTED && attempts < 20) {
                    delay(500);
                    Serial.print(".");
                    attempts++;
                }

                // Release the Wi-Fi mutex after Wi-Fi operations
                xSemaphoreGive(wifi_mutex);

                // Check if connected
                if (WiFi.status() == WL_CONNECTED) {
                    Serial.println("\nWi-Fi connected successfully!");
                    Serial.print("IP address: ");
                    Serial.println(WiFi.localIP());

                    // // Lock LVGL before accessing UI elements
                    // lvgl_port_lock(portMAX_DELAY);

                    // Check if the checkbox is checked
                    // bool savePassword = lv_obj_has_state(ui_Screen2_Checkbox1, LV_STATE_CHECKED);

                    // // Unlock LVGL
                    // lvgl_port_unlock();

                    // if (savePassword) {
                    //     Serial.println("Saving Wi-Fi credentials to NVS.");
                    //     xSemaphoreTake(nvs_mutex, portMAX_DELAY);
                    //     saveWifiCredentialsToNVS(ssid, password);
                    //     xSemaphoreGive(nvs_mutex);
                    // }

                    // Update the UI with connection success
                    lvgl_port_lock(portMAX_DELAY);
                    lv_label_set_text(ui_Label45, "Connected to network");
                    lvgl_port_unlock();

                } else {
                    Serial.println("\nWi-Fi connection failed.");
                    lvgl_port_lock(portMAX_DELAY);
                    lv_label_set_text(ui_Label45, "Failed to connect to network");
                    lvgl_port_unlock();
                }
            } else {
                Serial.println("Failed to acquire Wi-Fi mutex. Cannot perform Wi-Fi operations.");
                lvgl_port_lock(portMAX_DELAY);
                lv_label_set_text(ui_Label45, "Wi-Fi is busy. Try again later.");
                lvgl_port_unlock();
            }
        }
    }
}

void wifi_scan_task(void *pvParameters) {
    while (1) {
        if (xSemaphoreTake(wifi_scan_semaphore, portMAX_DELAY)) {
            if (xSemaphoreTake(wifi_mutex, portMAX_DELAY)) {
                Serial.println("Starting WiFi scan...");
                WiFi.mode(WIFI_STA);

                // Limit the number of networks to avoid memory issues
                int max_networks = 10;

                // Start Wi-Fi scan
                int n = WiFi.scanNetworks();
                Serial.printf("Scan completed, found %d networks\n", n);

                if (n == 0) {
                    Serial.println("No networks found");
                    lvgl_port_lock(-1);
                    lv_roller_set_options(ui_Roller1, "No networks found", LV_ROLLER_MODE_INFINITE);
                    lvgl_port_unlock();
                } else {
                    n = (n > max_networks) ? max_networks : n; // Limit to max_networks
                    String ssidList = "";

                    for (int i = 0; i < n; ++i) {
                        String ssid = WiFi.SSID(i);
                        Serial.printf("%d: %s (%d dBm)\n", i + 1, ssid.c_str(), WiFi.RSSI(i));
                        ssidList += ssid + "\n";
                    }

                    // Update roller options with available SSIDs
                    lvgl_port_lock(-1);
                    lv_roller_set_options(ui_Roller1, ssidList.c_str(), LV_ROLLER_MODE_INFINITE);
                    lvgl_port_unlock();
                }

                WiFi.scanDelete();
                xSemaphoreGive(wifi_mutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Yield to other tasks
    }
}



// void event_handler_scan_button(lv_event_t * e)
// {
//     if(switch)
// }
// void event_handler_connect_button(lv_event_t * e);
// void modbusTask(void *pvParameters);
// void vfdReadTask(void *pvParameters);
// void WiFiEvent(WiFiEvent_t event) {
//     switch (event.event_id) {
//         case ARDUINO_EVENT_WIFI_STA_GOT_IP:
//             Serial.println("Wi-Fi connected. IP address: ");
//             Serial.println(WiFi.localIP());
//             break;
//         case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
//             Serial.println("Wi-Fi lost connection");
//             // Try to reconnect
//             WiFi.reconnect();
//             break;
//         default:
//             break;
//     }
// }


// uint8_t getSelectedMotorID();

// Global variables
lv_obj_t* ota_progress_bar;
void event_handler_ui_Switch1(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);

    if (code == LV_EVENT_VALUE_CHANGED) {
        // Check if the switch is on (checked)
        if (lv_obj_has_state(obj, LV_STATE_CHECKED)) {
            Serial.println("Switch is ON, starting Wi-Fi scan...");
            xSemaphoreGive(wifi_scan_semaphore);  // Trigger the Wi-Fi scan task
        } else {
            Serial.println("Switch is OFF");
            // Optionally, you can handle the switch being turned off here
        }
    }
}
void event_enterwifi_button(lv_event_t * e){
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);

    if (code == LV_EVENT_PRESSED){
        xSemaphoreGive(wifi_connect_semaphore);
    }
}
void update_panel_colors(uint16_t min, uint16_t max) {
    // Get the value from ui_Label9
    const char *label_text = lv_label_get_text(ui_Label9);
    uint16_t prm = atoi(label_text); // Convert label text to an integer

    // Ensure the value is within the min and max range
    if (prm < min) prm = min;
    if (prm > max) prm = max;

    // Calculate the number of panels to activate
    uint16_t num_panels = ((prm - min) * 16) / (max - min);

    // Define colors
    lv_color_t default_color = lv_color_hex(0x26272C);
    lv_color_t active_color = lv_color_hex(0xE4322E);

    // Reset all panels to the default color
    lv_obj_set_style_bg_color(ui_Panel10, default_color, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panelline15, default_color, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panelline14, default_color, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panelline13, default_color, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panelline12, default_color, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panelline11, default_color, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panelline10, default_color, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panelline9, default_color, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panelline8, default_color, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panelline7, default_color, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panelline6, default_color, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panelline5, default_color, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panelline4, default_color, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panelline3, default_color, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panelline2, default_color, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel8, default_color, LV_PART_MAIN | LV_STATE_DEFAULT);

    // Activate panels based on the reversed sequence
    if (num_panels >= 1) lv_obj_set_style_bg_color(ui_Panel10, active_color, LV_PART_MAIN | LV_STATE_DEFAULT);
    if (num_panels >= 2) lv_obj_set_style_bg_color(ui_Panelline15, active_color, LV_PART_MAIN | LV_STATE_DEFAULT);
    if (num_panels >= 3) lv_obj_set_style_bg_color(ui_Panelline14, active_color, LV_PART_MAIN | LV_STATE_DEFAULT);
    if (num_panels >= 4) lv_obj_set_style_bg_color(ui_Panelline13, active_color, LV_PART_MAIN | LV_STATE_DEFAULT);
    if (num_panels >= 5) lv_obj_set_style_bg_color(ui_Panelline12, active_color, LV_PART_MAIN | LV_STATE_DEFAULT);
    if (num_panels >= 6) lv_obj_set_style_bg_color(ui_Panelline11, active_color, LV_PART_MAIN | LV_STATE_DEFAULT);
    if (num_panels >= 7) lv_obj_set_style_bg_color(ui_Panelline10, active_color, LV_PART_MAIN | LV_STATE_DEFAULT);
    if (num_panels >= 8) lv_obj_set_style_bg_color(ui_Panelline9, active_color, LV_PART_MAIN | LV_STATE_DEFAULT);
    if (num_panels >= 9) lv_obj_set_style_bg_color(ui_Panelline8, active_color, LV_PART_MAIN | LV_STATE_DEFAULT);
    if (num_panels >= 10) lv_obj_set_style_bg_color(ui_Panelline7, active_color, LV_PART_MAIN | LV_STATE_DEFAULT);
    if (num_panels >= 11) lv_obj_set_style_bg_color(ui_Panelline6, active_color, LV_PART_MAIN | LV_STATE_DEFAULT);
    if (num_panels >= 12) lv_obj_set_style_bg_color(ui_Panelline5, active_color, LV_PART_MAIN | LV_STATE_DEFAULT);
    if (num_panels >= 13) lv_obj_set_style_bg_color(ui_Panelline4, active_color, LV_PART_MAIN | LV_STATE_DEFAULT);
    if (num_panels >= 14) lv_obj_set_style_bg_color(ui_Panelline3, active_color, LV_PART_MAIN | LV_STATE_DEFAULT);
    if (num_panels >= 15) lv_obj_set_style_bg_color(ui_Panelline2, active_color, LV_PART_MAIN | LV_STATE_DEFAULT);
    if (num_panels >= 16) lv_obj_set_style_bg_color(ui_Panel8, active_color, LV_PART_MAIN | LV_STATE_DEFAULT);
}

void ui_event_Button5(lv_event_t *e) {
    // Get the current value of ui_Label9
    const char *label_text = lv_label_get_text(ui_Label35);
    int current_value = atoi(label_text);

    // Define the maximum value
    int max_value = 6;
    int min = 10;
    int max = 60;
    // Increment the value if it's below the maximum
    if (current_value < max_value) {
        current_value++;
        char new_value[10];
        snprintf(new_value, sizeof(new_value), "%d", current_value);
        lv_label_set_text(ui_Label35, new_value); // Update the label
        update_panel_colors(min, max);
    }
}
void ui_event_Button8(lv_event_t *e) {
    // Get the current value of ui_Label9
    const char *label_text = lv_label_get_text(ui_Label35);
    int current_value = atoi(label_text);

    // Define the minimum value
    int min_value = 1;
    int min = 10;
    int max = 60;
    // Decrement the value if it's above the minimum
    if (current_value > min_value) {
        current_value--;
        char new_value[10];
        snprintf(new_value, sizeof(new_value), "%d", current_value);
        lv_label_set_text(ui_Label35, new_value); // Update the label
        update_panel_colors(min, max);
    }
}
void ui_event_modebutton3(lv_event_t *e) {
    // Check if the event is a CLICKED event
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        // Lock LVGL since we are modifying UI elements
        lvgl_port_lock(-1);

        // Get the text from ui_TextArea1
        const char *textarea_text = lv_textarea_get_text(ui_TextArea1);

        // Check if the text is valid
        if (textarea_text != NULL) {
            Serial.printf("Text from TextArea: %s\n", textarea_text);

            // Set the text to ui_Label9
            lv_label_set_text(ui_Label9, textarea_text);

                        // Trigger the update_panel_colors function
            uint16_t min = 10;  // Example minimum value
            uint16_t max = 60; // Example maximum value
            update_panel_colors(min, max);
        
        } else {
            Serial.println("Error: TextArea text is NULL");
        }

        // Unlock LVGL after modifying UI
        lvgl_port_unlock();
    }
}



void ui_event_modebutton2(lv_event_t *e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_FADE_ON, 15, 0, &ui_Screen1_screen_init);
        lvgl_port_lock(-1);
        lv_label_set_text(ui_Label4,  lv_label_get_text(ui_Label9));
        lvgl_port_lock(1);
    }
}

void setup() {
    Serial.begin(115200);
    Serial.setDebugOutput(true);
    Serial.printf("Free heap: %u bytes\n", esp_get_free_heap_size());

    Serial.println("Setup starting...");
    esp_log_level_set("*", ESP_LOG_VERBOSE);

    panel = new ESP_Panel();

    /* Initialize LVGL core */
    lv_init();

    /* Create LVGL mutex */
    lvgl_mux = xSemaphoreCreateRecursiveMutex();

    static lv_disp_draw_buf_t draw_buf;
    // Use internal memory or PSRAM if available
    lv_color_t *buf = (lv_color_t *)heap_caps_malloc(LVGL_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!buf) {
        Serial.println("Error: Failed to allocate LVGL buffer");
        while (1);
    }
    lv_disp_draw_buf_init(&draw_buf, buf, NULL, LVGL_BUF_SIZE);

        // Initialize NVS
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);

    /* Initialize the display device */
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = ESP_PANEL_LCD_H_RES;
    disp_drv.ver_res = ESP_PANEL_LCD_V_RES;
    disp_drv.flush_cb = lvgl_port_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);


#if ESP_PANEL_USE_LCD_TOUCH
    /* Initialize the input device */
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = lvgl_port_tp_read;
    lv_indev_drv_register(&indev_drv);
#endif

    /* Initialize bus and device of panel */
    panel->init();

#if ESP_PANEL_LCD_BUS_TYPE != ESP_PANEL_BUS_TYPE_RGB
    /* Register a function to notify LVGL when the panel is ready to flush */
    panel->getLcd()->setCallback(notify_lvgl_flush_ready, &disp_drv);
#endif

    /* Initialize IO expander */
    Serial.println("Initialize IO expander");
    ESP_IOExpander *expander = new ESP_IOExpander_CH422G(I2C_MASTER_NUM, ESP_IO_EXPANDER_I2C_CH422G_ADDRESS_000);
    expander->init();
    expander->begin();
    expander->multiPinMode(TP_RST | LCD_BL | LCD_RST | SD_CS | USB_SEL, OUTPUT);
    expander->multiDigitalWrite(TP_RST | LCD_BL | LCD_RST | SD_CS, HIGH);
    expander->digitalWrite(USB_SEL, LOW);
    /* Add into panel */
    panel->addIOExpander(expander);

    /* Start panel */
    panel->begin();

    /* Create LVGL task */
    xTaskCreate(lvgl_port_task, "lvgl", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY + 1, NULL); // Set higher priority for LVGL task

    /* Initialize UI */
    ui_init();



    // Initialize Wi-Fi in station mode
    WiFi.mode(WIFI_STA);
    // WiFi.onEvent(WiFiEvent);
    WiFi.disconnect();

    delay(100); // Short delay to ensure Wi-Fi is initialized

    // Attempt to read Wi-Fi credentials from NVS
    char ssid[64];
    char password[64];
    // if (readWifiCredentialsFromNVS(ssid, sizeof(ssid), password, sizeof(password))) {
    //     Serial.printf("Connecting to saved Wi-Fi network: SSID: %s\n", ssid);
    //     WiFi.begin(ssid, password);

    //     // Wait for connection
    //     int attempts = 0;
    //     while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    //         delay(500);
    //         Serial.print(".");
    //         attempts++;
    //     }

    //     if (WiFi.status() == WL_CONNECTED) {
    //         Serial.println("\nWi-Fi connected successfully!");
    //         Serial.print("IP address: ");
    //         Serial.println(WiFi.localIP());
    //         // Optionally update UI
    //     } else {
    //         Serial.println("\nFailed to connect to saved Wi-Fi network.");
    //         // Optionally update UI
    //     }
    // } else {
    //     Serial.println("No saved Wi-Fi credentials found in NVS.");
    // }

    // Initialize the NVS mutex
    nvs_mutex = xSemaphoreCreateMutex();

    // // Check if the firmware is pending verification
    // esp_ota_img_states_t otaState;
    // const esp_partition_t* runningPartition = esp_ota_get_running_partition();
    // initializeSerial();

    // if (esp_ota_get_state_partition(runningPartition, &otaState) == ESP_OK) {
    //     if (otaState == ESP_OTA_IMG_PENDING_VERIFY) {
    //         Serial.println("Firmware is pending verification. Confirming now...");

    //         // Perform any additional checks here (e.g., sensor initialization)
    //         // If everything is okay, confirm the firmware
    //         esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    //         if (err == ESP_OK) {
    //             Serial.println("Firmware marked as valid.");
    //         } else {
    //             Serial.printf("Failed to mark firmware as valid: %s\n", esp_err_to_name(err));
    //             // Optionally, initiate a rollback
    //             // esp_ota_mark_app_invalid_rollback_and_reboot();
    //         }
    //     }
    // }

    if (ui_Switch1) {
        lv_obj_add_event_cb(ui_Switch1, event_handler_ui_Switch1, LV_EVENT_VALUE_CHANGED, NULL);
    } else {
        Serial.println("Error: ui_Switch1 is NULL");
    }
    lv_obj_add_event_cb(ui_rotationbutton, rotationChangedEventHandler, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(ui_starstopbutton, event_handler_start_motor_button, LV_EVENT_CLICKED, NULL);
    // Attach to time inputs
lv_obj_add_event_cb(ui_time1, event_handler_custom_value_changed, LV_EVENT_VALUE_CHANGED, NULL);
lv_obj_add_event_cb(ui_time4, event_handler_custom_value_changed, LV_EVENT_VALUE_CHANGED, NULL);
lv_obj_add_event_cb(ui_time3, event_handler_custom_value_changed, LV_EVENT_VALUE_CHANGED, NULL);

// Attach to speed inputs
lv_obj_add_event_cb(ui_rpmvalue, event_handler_custom_value_changed, LV_EVENT_VALUE_CHANGED, NULL);
lv_obj_add_event_cb(ui_rpmvalue2, event_handler_custom_value_changed, LV_EVENT_VALUE_CHANGED, NULL);
lv_obj_add_event_cb(ui_rpmvalue3, event_handler_custom_value_changed, LV_EVENT_VALUE_CHANGED, NULL);
lv_obj_add_event_cb(ui_rpmvalue4, event_handler_custom_value_changed, LV_EVENT_VALUE_CHANGED, NULL);

// Attach to rotation inputs
lv_obj_add_event_cb(ui_rotationbutton1, event_handler_custom_value_changed, LV_EVENT_VALUE_CHANGED, NULL);
lv_obj_add_event_cb(ui_rotationbutton3, event_handler_custom_value_changed, LV_EVENT_VALUE_CHANGED, NULL);
lv_obj_add_event_cb(ui_rotationbutton4, event_handler_custom_value_changed, LV_EVENT_VALUE_CHANGED, NULL);

    // /* Attach event handlers (check for null pointers) */
    // if (ui_Screen2_Button8) {
    //     lv_obj_add_event_cb(ui_Screen2_Button8, event_handler_scan_button, LV_EVENT_CLICKED, NULL);
    // } else {
    //     Serial.println("ui_Screen2_Button8 is NULL");
    // }

    // if (ui_Screen2_Button9) {
    //     lv_obj_add_event_cb(ui_Screen2_Button9, event_handler_connect_button, LV_EVENT_CLICKED, NULL);
    // } else {
    //     Serial.println("ui_Screen2_Button9 is NULL");
    // }

    // if (ui_Screen2_Button10) {
    //     lv_obj_add_event_cb(ui_Screen2_Button10, event_handler_ota_update, LV_EVENT_CLICKED, NULL);
    // } else {
    //     Serial.println("ui_Screen2_Button10 is NULL");
    // }

    // if (ui_Screen3_Button13) {
    //     lv_obj_add_event_cb(ui_Screen3_Button13, event_handler_start_motor_button, LV_EVENT_CLICKED, NULL);
    // } else {
    //     Serial.println("ui_Screen3_Button13 is NULL");
    // }

    // if (ui_Screen3_Button2) {
    //     lv_obj_add_event_cb(ui_Screen3_Button2, event_handler_stop_motor_button, LV_EVENT_CLICKED, NULL);
    // } else {
    //     Serial.println("ui_Screen3_Button2 is NULL");
    // }

    // if (ui_Screen3_Button3) {
    //     lv_obj_add_event_cb(ui_Screen3_Button3, event_handler_change_speed_button, LV_EVENT_CLICKED, NULL);
    // } else {
    //     Serial.println("ui_Screen3_Button3 is NULL");
    // }

    // if (ui_Screen5_Button3) {
    //     lv_obj_add_event_cb(ui_Screen5_Button3, event_handler_save_program_button, LV_EVENT_CLICKED, NULL);
    // } else {
    //     Serial.println("ui_Screen5_Button3 is NULL");
    // }

    // if (ui_Screen3_Button5) {
    //     lv_obj_add_event_cb(ui_Screen3_Button5, event_handler_save_motor_config, LV_EVENT_CLICKED, NULL);
    // } else {
    //     Serial.println("ui_Screen3_Button5 is NULL");
    // }

    // if (ui_Screen3_Dropdown1) {
    //     lv_obj_add_event_cb(ui_Screen3_Dropdown1, event_handler_motor_selection, LV_EVENT_VALUE_CHANGED, NULL);
    // } else {
    //     Serial.println("ui_Screen3_Dropdown1 is NULL");
    // }

    // if (ui_Screen1_Button4) {
    //     lv_obj_add_event_cb(ui_Screen1_Button4, event_handler_stop_motor_button, LV_EVENT_CLICKED, NULL);
    // } else {
    //     Serial.println("ui_Screen1_Button4 is NULL");
    // }

    // if (ui_Screen1_Button9) {
    //     Serial.println("Attaching event handler to ui_Screen1_Button9");
    //     lv_obj_add_event_cb(ui_Screen1_Button9, event_handler_serial_input_save, LV_EVENT_CLICKED, NULL);
    // } else {
    //     Serial.println("Error: ui_Screen1_Button9 is NULL");
    // }

    // if (ui_Screen1_Button8) {
    //     lv_obj_add_event_cb(ui_Screen1_Button8, event_handler_stop_motor_button, LV_EVENT_CLICKED, NULL);
    // } else {
    //     Serial.println("ui_Screen1_Button8 is NULL");
    // }



    save_program_semaphore = xSemaphoreCreateBinary();

    wifi_connect_semaphore = xSemaphoreCreateBinary();
    wifi_scan_semaphore = xSemaphoreCreateBinary();
    wifi_mutex = xSemaphoreCreateMutex();
    otaSemaphore = xSemaphoreCreateBinary();
        modbus_mutex = xSemaphoreCreateMutex();

    /* Initialize Modbus Serial */
    pinMode(MODBUS_DE_RE_PIN, OUTPUT);
    digitalWrite(MODBUS_DE_RE_PIN, LOW); // Receiver enabled by default

    // Initialize Modbus serial port with new TX and RX pins
    ModbusSerial.begin(9600, SERIAL_8N1, MODBUS_RX_PIN, MODBUS_TX_PIN);
    if (!ModbusSerial) {
        Serial.println("Modbus Serial initialization failed");
        while (1); // Halt if Modbus initialization fails
    }

    mb.begin(&ModbusSerial, MODBUS_DE_RE_PIN); // Initialize Modbus with DE/RE pin
    mb.master(); // Set as Modbus master
ModbusSerial.setTimeout(1000);
 nvs_mutex = xSemaphoreCreateMutex();
    /* Create tasks */
    xTaskCreate(wifi_connect_task, "WiFi Connect Task", 3072, NULL, 1, NULL);
    xTaskCreate(wifi_scan_task, "WiFi Scan Task", 3072, NULL, 1, NULL);
    xTaskCreate(modbusLoopTask, "Modbus Loop Task", 2048, NULL, 2, NULL);
    // xTaskCreate(server_connection_task, "Server Connection Task", 16384, NULL, 1, NULL);

    // // Create the OTA update task
    // xTaskCreate(otaUpdateTask, "OTA Update Task", 16384, NULL, 1, &otaTaskHandle);
    // xTaskCreate(saveProgramTask, "Save Program Task", 4096, NULL, 1, NULL);
    // xTaskCreate(vfdActualSpeedReadTask, "VFD Actual Speed Read Task", 4096, NULL, 1, NULL);
    // xTaskCreate(modbusTask, "Modbus Task", 4096, NULL, 2, NULL);
    // xTaskCreate(vfdReadTask, "VFD Read Task", 4096, NULL, 1, NULL);

    /* Initialize Modbus Serial */
    pinMode(MODBUS_DE_RE_PIN, OUTPUT);
    digitalWrite(MODBUS_DE_RE_PIN, LOW); // Receiver enabled by default

    // Initialize Modbus serial port with new TX and RX pins
    ModbusSerial.begin(9600, SERIAL_8N1, MODBUS_RX_PIN, MODBUS_TX_PIN);
    if (!ModbusSerial) {
        Serial.println("Modbus Serial initialization failed");
        while (1); // Halt if Modbus initialization fails
    }

    mb.begin(&ModbusSerial, MODBUS_DE_RE_PIN); // Initialize Modbus with DE/RE pin
    mb.master(); // Set as Modbus master

    Serial.println("Modbus initialized");
    Serial.println("Setup done");
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
