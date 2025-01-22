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

bool is_motor_running = false;

#define CUSTOM_NAMESPACE "custom_program"  // NVS namespace for custom program data


#define NAMESPACE "auto_mode" // NVS namespace for auto mode data
#define MAX_PROGRAMS 6 // Maximum number of auto programs

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

TaskHandle_t countdownTaskHandle = NULL; // Handle for the countdown task
bool isCountdownActive = false;         // Flag to indicate if the countdown is running
uint32_t countdownTime = 0;             // Time in seconds


// Current mode variable
uint8_t currentMode = MODE_MANUAL;  // Start with Manual mode

/* Define StepData struct */
typedef struct {
    uint16_t speed;    
    bool rotation;        
    uint32_t time;        
    uint8_t minutes;      
    uint8_t seconds; 
    uint8_t repeats;     
} StepData;

#define MAX_STEPS 5

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
// A simple helper function to show/hide your "running" vs. "stopped" UI elements.
void updateUIForMotorState(bool running)
{
    if (running) {
        // Motor is running: hide Label1/Image1, show Label2/Image2/Spinner
        _ui_flag_modify(ui_Label1,  LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_ADD);
        _ui_flag_modify(ui_Image1,  LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_ADD);

        _ui_flag_modify(ui_Label2,  LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_REMOVE);
        _ui_flag_modify(ui_Image2,  LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_REMOVE);
        _ui_flag_modify(ui_Spinner1, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_REMOVE);
    }
    else {
        // Motor is NOT running: show Label1/Image1, hide Label2/Image2/Spinner
        _ui_flag_modify(ui_Label1,  LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_REMOVE);
        _ui_flag_modify(ui_Image1,  LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_REMOVE);

        _ui_flag_modify(ui_Label2,  LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_ADD);
        _ui_flag_modify(ui_Image2,  LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_ADD);
        _ui_flag_modify(ui_Spinner1, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_ADD);
    }
}

void stopMotor() {
    // 1) Send Modbus command to stop the motor
    uint16_t result = mb.writeHreg(1, 0x1FFF, 0, modbusCallback);

    if (result == 0) {
        Serial.println("Motor stop command sent.");
        is_motor_running = false;
        updateUIForMotorState(false);  // <--- unify the UI here

        // 2) If countdown is active, stop it and reset labels to 00:00
        if (isCountdownActive) {
            isCountdownActive = false;

            lvgl_port_lock(-1);
            // lv_label_set_text(ui_Label107, "00");  // Reset minutes
            // lv_label_set_text(ui_Label108, "00");  // Reset seconds
            lvgl_port_unlock();

            Serial.println("Countdown was active, now stopped.");
        }

        // 3) (Optional) Hide “running” indicators, show “stopped” icons, etc.
        _ui_flag_modify(ui_Label1,  LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_ADD);
        _ui_flag_modify(ui_Spinner1, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_ADD);
    } 
    else {
        Serial.printf("Failed to send stop command. Modbus error: %d\n", result);
    }
}

void startCountdownFromUI() {
    // 1) Read minutes/seconds from the UI labels
    const char* minutesText = lv_label_get_text(ui_Label107);
    const char* secondsText = lv_label_get_text(ui_Label108);

    uint32_t minutes = atoi(minutesText);
    uint32_t seconds = atoi(secondsText);

    // 2) Compute total time in seconds
    countdownTime = (minutes * 60) + seconds;

    // 3) Only start if non-zero
    if (countdownTime > 0) {
        isCountdownActive = true;

        // Resume the countdown task (which is suspended otherwise)
        if (countdownTaskHandle != NULL) {
            vTaskResume(countdownTaskHandle);
        }

        Serial.printf("Countdown started for %u min, %u sec => %u total.\n", minutes, seconds, countdownTime);
    }
    else {
        Serial.println("Invalid time (00:00). Countdown not started.");
    }
}

void countdownTask(void *param) {
    while (true) {
        if (isCountdownActive) {
            // Count down if there's time left
            if (countdownTime > 0) {
                // 1) Convert to minutes/seconds for display
                uint32_t minutes = countdownTime / 60;
                uint32_t seconds = countdownTime % 60;

                // 2) Update the UI
                lvgl_port_lock(-1);
                char bufM[4], bufS[4];
                snprintf(bufM, sizeof(bufM), "%02u", minutes);
                snprintf(bufS, sizeof(bufS), "%02u", seconds);
                lv_label_set_text(ui_Label107, bufM);
                lv_label_set_text(ui_Label108, bufS);
                lvgl_port_unlock();

                // 3) Debug log
                Serial.printf("Countdown: %02u:%02u\n", minutes, seconds);

                // 4) Wait 1 second and decrement
                vTaskDelay(pdMS_TO_TICKS(1000));
                countdownTime--;

                // If motor was forcibly stopped from somewhere else
                if (!is_motor_running) {
                    // Cancel the countdown
                    isCountdownActive = false;
                    // Optionally reset UI to 00:00
                    Serial.println("Countdown stopped (motor forced off).");
                }
            }
            else {
                // Time reached zero => stop the motor
                Serial.println("Countdown finished => stopping motor.");
                //stopMotor(); // will set isCountdownActive=false inside
            }
        }
        else {
            // If not active, suspend ourselves to save CPU
            vTaskSuspend(NULL);
        }
    }
}





void startCountdown(uint32_t timeInSeconds) {
    countdownTime = timeInSeconds;
    isCountdownActive = true;

    // Resume the countdown task
    if (countdownTaskHandle != NULL) {
        vTaskResume(countdownTaskHandle);
    }
}



typedef struct {
    uint16_t speed;   // Speed value
    bool rotation;    // Rotation direction (true = forward, false = backward)
    uint8_t minutes;  // Time in minutes
    uint8_t seconds;  // Time in seconds
    uint8_t repeats;  // Repeat count
} AutoProgramData;

AutoProgramData currentAutoProgramData;
uint8_t currentProgramNumber = 1; // Default to program 1





bool loadAutoProgramFromNVS(uint8_t programNumber, AutoProgramData *programData) {
    if (programNumber < 1 || programNumber > MAX_PROGRAMS) {
        Serial.println("Invalid program number for loading.");
        return false;
    }

    char key[16];
    snprintf(key, sizeof(key), "prog_%d", programNumber);

    xSemaphoreTake(nvs_mutex, portMAX_DELAY);

    nvs_handle_t nvsHandle;
    esp_err_t err = nvs_open(NAMESPACE, NVS_READONLY, &nvsHandle);
    if (err != ESP_OK) {
        Serial.printf("Error opening NVS handle for loading: %s\n", esp_err_to_name(err));
        xSemaphoreGive(nvs_mutex);
        return false;
    }

    size_t dataSize = sizeof(AutoProgramData);
    err = nvs_get_blob(nvsHandle, key, programData, &dataSize);
    if (err == ESP_OK) {
        Serial.printf("Program %d loaded successfully.\n", programNumber);
        nvs_close(nvsHandle);
        xSemaphoreGive(nvs_mutex);
        return true;
    } else {
        Serial.printf("Failed to load program %d: %s\n", programNumber, esp_err_to_name(err));
        nvs_close(nvsHandle);
        xSemaphoreGive(nvs_mutex);
        return false;
    }
}

void loadAndApplyAutoProgram(uint8_t programNumber) {
    if (loadAutoProgramFromNVS(programNumber, &currentAutoProgramData)) {
        currentProgramNumber = programNumber;

        // Update UI elements with the loaded data
        char buffer[10];
        snprintf(buffer, sizeof(buffer), "%d", currentAutoProgramData.speed);
        lv_label_set_text(ui_Label30, buffer);

     //   lv_obj_set_hidden(ui_Image14, !currentAutoProgramData.rotation); // Hide or show based on rotation

        snprintf(buffer, sizeof(buffer), "%d", currentAutoProgramData.minutes);
        lv_label_set_text(ui_Label105, buffer);

        snprintf(buffer, sizeof(buffer), "%d", currentAutoProgramData.seconds);
        lv_label_set_text(ui_Label106, buffer);

        snprintf(buffer, sizeof(buffer), "%d", currentAutoProgramData.repeats);
        lv_label_set_text(ui_Label37, buffer);
    }
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



void saveCurrentProgramData(uint8_t currentProgramNumber) {
    if (currentProgramNumber == 0 || currentProgramNumber > MAX_PROGRAMS) {
        Serial.println("Invalid program number.");
        return;
    }

    ProgramData programData;
    programData.programNumber = currentProgramNumber;
    programData.stepCount = 1;  // Assuming one step for simplicity

    // Initialize the first step
    StepData step;

    // Get speed from UI
    const char *speedText = lv_label_get_text(ui_Label30);
    step.speed = atoi(speedText);

    // Get rotation from UI
    step.rotation = !lv_obj_has_flag(ui_Image18, LV_OBJ_FLAG_HIDDEN);

    // Get time from UI
    const char *timeText = lv_label_get_text(ui_Label106);
    step.time = atoi(timeText);

    // Assign the step to the program
    programData.steps[0] = step;

    // Save program data
    esp_err_t err = saveAutoProgramToNVS(currentProgramNumber, &programData);
    if (err == ESP_OK) {
        Serial.printf("Program %d saved successfully.\n", currentProgramNumber);
    } else {
        Serial.printf("Failed to save program %d.\n", currentProgramNumber);
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

//  esp_err_t saveAutoProgramToNVS(uint8_t programNumber, const ProgramData* programData) {
//     if (programNumber == 0 || programNumber > MAX_PROGRAMS) {
//         Serial.println("Invalid program number.");
//         return ESP_ERR_INVALID_ARG;
//     }
//}
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
    xSemaphoreTake(nvs_mutex, portMAX_DELAY); // Lock NVS access

    nvs_handle_t nvsHandle;
    esp_err_t err = nvs_open(NAMESPACE, NVS_READWRITE, &nvsHandle);
    if (err != ESP_OK) {
        Serial.printf("Error opening NVS handle: %s\n", esp_err_to_name(err));
        xSemaphoreGive(nvs_mutex);
        return;
    }

    switch (currentMode) {
        case MODE_MANUAL:
            // Save manual mode settings
            err = nvs_set_u16(nvsHandle, "manual_speed", manualModeSettings.speed);
            if (err != ESP_OK) {
                Serial.printf("Error saving manual speed: %s\n", esp_err_to_name(err));
            }

            err = nvs_set_u8(nvsHandle, "manual_rotation", manualModeSettings.rotation);
            if (err != ESP_OK) {
                Serial.printf("Error saving manual rotation: %s\n", esp_err_to_name(err));
            }
            Serial.println("Manual mode settings saved.");
            break;

        case MODE_AUTO:
            // Save auto mode settings
            err = nvs_set_u8(nvsHandle, "auto_program", autoModeSettings.programNumber);
            if (err != ESP_OK) {
                Serial.printf("Error saving auto program number: %s\n", esp_err_to_name(err));
            }
            Serial.println("Auto mode settings saved.");
            break;

        case MODE_CUSTOM: {
            // Save custom mode settings
            CustomProgramData customData;
           // collectCustomProgramData(&customData);
            //err = saveCustomProgramToNVS(&customData);
            if (err != ESP_OK) {
                Serial.println("Error saving custom program settings to NVS.");
            } else {
                Serial.println("Custom mode settings saved.");
            }
            break;
        }

        default:
            Serial.println("Unknown mode. No settings saved.");
            break;
    }

    // Commit changes and close NVS handle
    err = nvs_commit(nvsHandle);
    if (err != ESP_OK) {
        Serial.printf("Error committing NVS changes: %s\n", esp_err_to_name(err));
    }

    nvs_close(nvsHandle);
    xSemaphoreGive(nvs_mutex); // Release NVS access
}


void stopMotorOnModeChange() {
    if (is_motor_running) {
        // Stop the motor via Modbus
        stopMotor();

        // Update motor state
        is_motor_running = false;

        // Log the event
        Serial.println("Motor stopped due to mode change.");

        // Update relevant UI elements
        // Hide motor-running indicators
        _ui_flag_modify(ui_Label1, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_ADD);
        _ui_flag_modify(ui_Image1, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_ADD);

        // Show motor-stopped indicators
        _ui_flag_modify(ui_Label2, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_REMOVE);
        _ui_flag_modify(ui_Image2, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_REMOVE);

        // Stop any active spinner or loading indicators
        _ui_flag_modify(ui_Spinner1, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_ADD);
    } else {
        Serial.println("Motor is not running. No action required.");
    }
}

void updateModeBasedOnVisibility() {
    if (!lv_obj_has_flag(ui_modebutton, LV_OBJ_FLAG_HIDDEN)) {
        currentMode = MODE_MANUAL;
        Serial.println("Current mode: Manual");
    } else if (!lv_obj_has_flag(ui_modebutton4, LV_OBJ_FLAG_HIDDEN)) {
        currentMode = MODE_AUTO;
        Serial.println("Current mode: Auto");
    } else if (!lv_obj_has_flag(ui_modebutton7, LV_OBJ_FLAG_HIDDEN)) {
        currentMode = MODE_CUSTOM;
        Serial.println("Current mode: Custom");
    } else {
        Serial.println("No visible mode button found. Mode unchanged.");
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




void event_handler_start_motor_button(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code != LV_EVENT_CLICKED) {
        return; // Only handle clicks
    }

    Serial.println("Start button pressed...");

    // 1) Ensure we are in Manual Mode
    if (currentMode != MODE_MANUAL) {
        Serial.println("Error: Start button only works in manual mode.");
        return;
    }

    // 2) Lock Modbus to prevent conflicts
    xSemaphoreTake(modbus_mutex, portMAX_DELAY);

    // 3) Get speed from UI (e.g., 0–60)
    uint16_t speedVal = getSpeedFromUI(); // Your function to read from label/spinbox
    // 4) Determine direction => forward = 18, reverse = 34
    bool rotation = getRotationFromUI();  
    uint16_t commandValue = rotation ? 18 : 34;

    Serial.printf("Will write Speed=%u to 0x2001, then Start=0x%X to 0x2000\n",
                  speedVal, commandValue);

    // 5) Write speed first, if your drive requires a speed setpoint:
    if (!mb.writeHreg(MODBUS_SLAVE_ID, 0x2001, speedVal, modbusCallback)) {
        Serial.println("Failed to queue speed write command!");
        updateUIForMotorState(false);
        xSemaphoreGive(modbus_mutex);
        return;
    }

    // (Optional) a small delay so we don't flood the request queue:
    vTaskDelay(pdMS_TO_TICKS(50));

    // 6) Send start (direction) command to 0x2000
    uint16_t result = mb.writeHreg(MODBUS_SLAVE_ID, 0x2000, 34, modbusCallback);
    if (result == 0) {
        // '0' means queued successfully in Emelianov's library
        Serial.printf("Start command queued: 0x%X\n", commandValue);
        is_motor_running = true;
        updateUIForMotorState(true);
    } else {
        Serial.printf("Failed to queue start command: Error=%d\n", result);
        updateUIForMotorState(false);
    }

    // 7) Unlock Modbus
    xSemaphoreGive(modbus_mutex);
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
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


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



void ui_event_ui_eneterbutton(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED) {
        Serial.println("Enter button pressed.");

        uint8_t mode = currentMode; // Read currentMode directly
        Serial.printf("Current mode: %d\n", mode);

        switch (mode) {
            case MODE_MANUAL:
                Serial.println("Redirecting to Screen1.");
                _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_FADE_ON, 15, 0, &ui_Screen1_screen_init);
                lv_label_set_text(ui_Label4, lv_label_get_text(ui_Label9));
                break;

            case MODE_AUTO:
                Serial.println("Redirecting to Screen4.");
                _ui_screen_change(&ui_Screen4, LV_SCR_LOAD_ANIM_FADE_ON, 15, 0, &ui_Screen4_screen_init);
                lv_label_set_text(ui_Label30, lv_label_get_text(ui_Label9));
                break;

            case MODE_CUSTOM:
                Serial.println("Redirecting to Screen8.");
                _ui_screen_change(&ui_Screen8, LV_SCR_LOAD_ANIM_FADE_ON, 15, 0, &ui_Screen8_screen_init);
                break;

            default:
                Serial.println("Error: Undefined mode.");
                break;
        }
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
void setMode(uint8_t newMode) {
    Serial.printf("Setting mode from %d to %d\n", currentMode, newMode);
    if (currentMode == newMode) {
        Serial.println("Mode is already set. No action required.");
        return;
    }

    // Stop motor if running
    if (is_motor_running) {
        stopMotor();
        is_motor_running = false;
        Serial.println("Motor stopped due to mode change.");
    }

    // Update current mode
    currentMode = newMode;
    saveCurrentModeSettings(); // Save mode to NVS

    // Update UI visibility for modes
    Serial.printf("Updating UI for new mode: %d\n", newMode);
    lv_obj_add_flag(ui_modebutton, LV_OBJ_FLAG_HIDDEN);  // Hide Manual button by default
    lv_obj_add_flag(ui_modebutton4, LV_OBJ_FLAG_HIDDEN); // Hide Auto button by default
    lv_obj_add_flag(ui_modebutton7, LV_OBJ_FLAG_HIDDEN); // Hide Custom button by default

    switch (currentMode) {
        case MODE_MANUAL:
            lv_obj_clear_flag(ui_modebutton, LV_OBJ_FLAG_HIDDEN); // Show Manual button
            Serial.println("Switched to Manual Mode");
            break;
        case MODE_AUTO:
            lv_obj_clear_flag(ui_modebutton4, LV_OBJ_FLAG_HIDDEN); // Show Auto button
            Serial.println("Switched to Auto Mode");
            break;
        case MODE_CUSTOM:
            lv_obj_clear_flag(ui_modebutton7, LV_OBJ_FLAG_HIDDEN); // Show Custom button
            Serial.println("Switched to Custom Mode");
            break;
        default:
            Serial.println("Unknown mode.");
            break;
    }
}



#define DEBOUNCE_DELAY_MS 200  // Define a debounce delay in milliseconds

static uint32_t lastModeSwitchTime = 0;  // Track last mode switch time

// Function to cycle through modes
void cycleMode() {
    currentMode = (currentMode + 1) % 3; // Cycles through 0 -> 1 -> 2 -> 0
    setMode(currentMode); // Apply the new mode
}

void ui_event_modebutton(lv_event_t *e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        Serial.println("Mode button clicked. Testing direct mode setting.");
        setMode(MODE_AUTO); // Directly test setting Auto mode
        _ui_flag_modify(ui_modebutton6, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_REMOVE);
        _ui_flag_modify(ui_modebutton5, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_REMOVE);
        _ui_flag_modify(ui_modebutton4, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_REMOVE);
        _ui_flag_modify(ui_modebutton, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_ADD);
        _ui_flag_modify(ui_rotationbutton, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_ADD);
        _ui_flag_modify(ui_speedchange, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_ADD);
    }
}


void ui_event_modebutton4(lv_event_t *e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        setMode(MODE_CUSTOM); // Directly test setting Auto mode
        lv_obj_clear_flag(ui_modebutton4, LV_OBJ_FLAG_HIDDEN); // Show Auto button
        _ui_flag_modify(ui_modebutton4, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_ADD);
        _ui_flag_modify(ui_modebutton7, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_REMOVE);
    }
}

void ui_event_modebutton7(lv_event_t *e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        setMode(MODE_MANUAL); // Directly test setting Auto mode
        _ui_flag_modify(ui_modebutton6, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_ADD);
        _ui_flag_modify(ui_modebutton5, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_ADD);
        _ui_flag_modify(ui_modebutton4, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_ADD);
        _ui_flag_modify(ui_modebutton, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_REMOVE);
        _ui_flag_modify(ui_rotationbutton, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_REMOVE);
        _ui_flag_modify(ui_speedchange, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_REMOVE);
        _ui_flag_modify(ui_modebutton7, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_ADD);
    }
}






void ui_event_Buttonleftline(lv_event_t *e) {
    lv_event_code_t event_code = lv_event_get_code(e);
    if (event_code == LV_EVENT_CLICKED) {
        const char *label_text = lv_label_get_text(ui_Label9);
        int current_value = atoi(label_text);

        // Define the range
        const int min_value = 10;
        const int max_value = 60;

        // Decrement the value if it's above the minimum
        if (current_value > min_value) {
            current_value--;
            char new_value[10];
            snprintf(new_value, sizeof(new_value), "%d", current_value);
            lv_label_set_text(ui_Label9, new_value); // Update the label

            // (Optional) Call a function to update other UI elements or logic
            update_panel_colors(min_value, max_value);
        } else {
            Serial.println("Value is already at the minimum.");
        }
    }
}

void ui_event_Buttonrightline(lv_event_t *e) {
    lv_event_code_t event_code = lv_event_get_code(e);
    if (event_code == LV_EVENT_CLICKED) {
        const char *label_text = lv_label_get_text(ui_Label9);
        int current_value = atoi(label_text);

        // Define the range
        const int min_value = 10;
        const int max_value = 60;

        // Increment the value if it's below the maximum
        if (current_value < max_value) {
            current_value++;
            char new_value[10];
            snprintf(new_value, sizeof(new_value), "%d", current_value);
            lv_label_set_text(ui_Label9, new_value); // Update the label

            // (Optional) Call a function to update other UI elements or logic
            update_panel_colors(min_value, max_value);
        } else {
            Serial.println("Value is already at the maximum.");
        }
    }
}





bool loadProgramData(uint8_t programNumber, ProgramData *programData) {
    if (programNumber < 1 || programNumber > MAX_PROGRAMS) {
        Serial.println("Invalid program number.");
        return false;
    }

    nvs_handle_t nvsHandle;
    esp_err_t err;

    // Open NVS handle in read-only mode
    err = nvs_open(NAMESPACE, NVS_READONLY, &nvsHandle);
    if (err != ESP_OK) {
        Serial.printf("Error (%s) opening NVS handle for reading!\n", esp_err_to_name(err));
        return false;
    }

    // Create a key for the program number
    char key[16];
    snprintf(key, sizeof(key), "program_%d", programNumber);

    // Load the program data as a blob
    size_t dataSize = sizeof(ProgramData);
    err = nvs_get_blob(nvsHandle, key, programData, &dataSize);
    if (err == ESP_OK) {
        Serial.printf("Program %d loaded successfully from NVS.\n", programNumber);
    } else {
        Serial.printf("Error (%s) loading program data from NVS!\n", esp_err_to_name(err));
    }

    // Close NVS handle
    nvs_close(nvsHandle);
    return (err == ESP_OK);
}


void updateUIWithProgramData(const ProgramData *programData) {
    if (programData == nullptr || programData->stepCount == 0) {
        Serial.println("No valid program data to update UI.");
        return;
    }

    const StepData &step = programData->steps[0];

    // Update speed
    char speedText[10];
    snprintf(speedText, sizeof(speedText), "%d", step.speed);
    lv_label_set_text(ui_Label30, speedText);

    // Update time
    char timeText[10];
    snprintf(timeText, sizeof(timeText), "%d", step.time);
    lv_label_set_text(ui_Label106, timeText);

    // Update rotation
    if (step.rotation) {
        lv_obj_add_flag(ui_Image18, LV_OBJ_FLAG_HIDDEN); // Hide reverse indicator
        lv_obj_clear_flag(ui_Image19, LV_OBJ_FLAG_HIDDEN); // Show forward indicator
    } else {
        lv_obj_add_flag(ui_Image19, LV_OBJ_FLAG_HIDDEN); // Hide forward indicator
        lv_obj_clear_flag(ui_Image18, LV_OBJ_FLAG_HIDDEN); // Show reverse indicator
    }

    // Update repeat count
    char repeatText[10];
    snprintf(repeatText, sizeof(repeatText), "%d", step.repeats);
    lv_label_set_text(ui_Label37, repeatText);
}


void ui_event_Button5(lv_event_t *e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        const char *label_text = lv_label_get_text(ui_Label35);
        int currentProgramNumber = atoi(label_text);

        const int maxProgramNumber = MAX_PROGRAMS;

        // Save current program data
        saveCurrentProgramData(currentProgramNumber);

        if (currentProgramNumber < maxProgramNumber) {
            currentProgramNumber++;

            // Load the saved data for the new program
            ProgramData programData;
            if (loadProgramData(currentProgramNumber, &programData)) {
                updateUIWithProgramData(&programData);
            } else {
                Serial.printf("No data found for program %d. Using default values.\n", currentProgramNumber);
            }

            // Update program number in UI
            char newProgramNumber[10];
            snprintf(newProgramNumber, sizeof(newProgramNumber), "%d", currentProgramNumber);
            lv_label_set_text(ui_Label35, newProgramNumber);

            update_panel_colors(1, maxProgramNumber);
        } else {
            Serial.println("Program number is already at the maximum.");
        }
    }
}

void ui_event_Button8(lv_event_t *e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        const char *label_text = lv_label_get_text(ui_Label35);
        int currentProgramNumber = atoi(label_text);

        const int minProgramNumber = 1;

        // Save current program data
        saveCurrentProgramData(currentProgramNumber);

        if (currentProgramNumber > minProgramNumber) {
            currentProgramNumber--;

            // Load the saved data for the new program
            ProgramData programData;
            if (loadProgramData(currentProgramNumber, &programData)) {
                updateUIWithProgramData(&programData);
            } else {
                Serial.printf("No data found for program %d. Using default values.\n", currentProgramNumber);
            }

            // Update program number in UI
            char newProgramNumber[10];
            snprintf(newProgramNumber, sizeof(newProgramNumber), "%d", currentProgramNumber);
            lv_label_set_text(ui_Label35, newProgramNumber);

            update_panel_colors(minProgramNumber, MAX_PROGRAMS);
        } else {
            Serial.println("Program number is already at the minimum.");
        }
    }
}

// Store the original position of ui_passwordarea
static lv_coord_t original_x = 0;
static lv_coord_t original_y = -69;

static bool is_moved_to_center = false; // To track if it's currently moved

void ui_event_passwordarea(lv_event_t *e) {
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t *target = lv_event_get_target(e);

    if (event_code == LV_EVENT_CLICKED) {
        if (!is_moved_to_center) {
            // Save the original position only once
            original_x = lv_obj_get_x(target);
            original_y = lv_obj_get_y(target);

            // Move ui_passwordarea to the center top
            lv_obj_align(target, LV_ALIGN_TOP_MID, 0, 80); // Adjust y-offset as needed
            is_moved_to_center = true; // Mark as moved to the center

            // Ensure ui_passwordarea is visible
            lv_obj_clear_flag(target, LV_OBJ_FLAG_HIDDEN);

            // Toggle other UI elements
            _ui_flag_modify(ui_passwordboard, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_TOGGLE);
            _ui_flag_modify(ui_Checkbox1, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_TOGGLE);
            _ui_flag_modify(ui_Roller1, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_TOGGLE);
            _ui_flag_modify(ui_wifissid, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_TOGGLE);
            _ui_flag_modify(ui_wifilabel, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_TOGGLE);
            _ui_flag_modify(ui_Switch1, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_TOGGLE);
        } else {
            // Restore to original position if already moved to the center
            lv_obj_set_pos(target, original_x, original_y);
            is_moved_to_center = false; // Mark as restored

            // Ensure ui_passwordarea is visible
            lv_obj_clear_flag(target, LV_OBJ_FLAG_HIDDEN);

            // Toggle other UI elements back
            _ui_flag_modify(ui_passwordboard, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_TOGGLE);
        }
    }
}

void ui_event_okbutton(lv_event_t *e) {
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t *target = lv_event_get_target(e);

    if (event_code == LV_EVENT_CLICKED) {
        if (is_moved_to_center) {
            // Restore ui_passwordarea to its original position
            lv_obj_set_pos(ui_passwordarea, original_x, original_y);
            is_moved_to_center = false; // Mark as restored

            // Ensure ui_passwordarea is visible
            lv_obj_clear_flag(ui_passwordarea, LV_OBJ_FLAG_HIDDEN);
        }

        // Toggle visibility of ui_passwordboard
        _ui_flag_modify(ui_passwordboard, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_TOGGLE);
                _ui_flag_modify(ui_Checkbox1, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_TOGGLE);
        _ui_flag_modify(ui_Roller1, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_TOGGLE);
        _ui_flag_modify(ui_wifissid, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_TOGGLE);
        _ui_flag_modify(ui_wifilabel, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_TOGGLE);
        _ui_flag_modify(ui_Switch1, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_TOGGLE);
    }
}

static int serial_button_press_count = 0; // Counter for button presses
static uint32_t last_press_time = 0;      // Timestamp of the last button press
const uint32_t reset_timeout = 3000;      // 3 seconds timeout in milliseconds
const int required_presses = 10;         // Number of presses required to redirect

void ui_event_serialbutton(lv_event_t *e) {
    lv_event_code_t event_code = lv_event_get_code(e);
    if (event_code == LV_EVENT_CLICKED) {
        uint32_t current_time = millis();

        // Check if the last press was more than the timeout period
        if (current_time - last_press_time > reset_timeout) {
            serial_button_press_count = 0; // Reset counter
            Serial.println("Timeout reached. Resetting press count.");
        }

        // Update last press time and increment the counter
        last_press_time = current_time;
        serial_button_press_count++;
        Serial.printf("Button pressed %d times.\n", serial_button_press_count);

        // Check if the required number of presses is reached
        if (serial_button_press_count >= required_presses) {
            Serial.println("Button pressed 10 times! Redirecting to Screen11.");
            _ui_screen_change(&ui_Screen11, LV_SCR_LOAD_ANIM_FADE_ON, 15, 0, &ui_Screen11_screen_init);

            // Reset the counter after redirecting
            serial_button_press_count = 0;
        }
    }
}

// Global variables to store min and max RPM values
uint16_t minRPM = MIN_SPEED_RPM;
uint16_t maxRPM = MAX_SPEED_RPM;

// Save RPM values to NVS
void saveRPMValuesToNVS() {
    nvs_handle_t nvsHandle;
    esp_err_t err = nvs_open("rpm_data", NVS_READWRITE, &nvsHandle);
    if (err != ESP_OK) {
        Serial.printf("Error opening NVS handle: %s\n", esp_err_to_name(err));
        return;
    }

    err = nvs_set_u16(nvsHandle, "min_rpm", minRPM);
    if (err == ESP_OK) {
        err = nvs_set_u16(nvsHandle, "max_rpm", maxRPM);
    }

    if (err == ESP_OK) {
        nvs_commit(nvsHandle);
        Serial.println("RPM values saved to NVS.");
    } else {
        Serial.printf("Error saving RPM values to NVS: %s\n", esp_err_to_name(err));
    }

    nvs_close(nvsHandle);
}

// Load RPM values from NVS
void loadRPMValuesFromNVS() {
    nvs_handle_t nvsHandle;
    esp_err_t err = nvs_open("rpm_data", NVS_READONLY, &nvsHandle);
    if (err != ESP_OK) {
        Serial.printf("Error opening NVS handle: %s\n", esp_err_to_name(err));
        return;
    }

    err = nvs_get_u16(nvsHandle, "min_rpm", &minRPM);
    if (err != ESP_OK) minRPM = MIN_SPEED_RPM;

    err = nvs_get_u16(nvsHandle, "max_rpm", &maxRPM);
    if (err != ESP_OK) maxRPM = MAX_SPEED_RPM;

    nvs_close(nvsHandle);
    Serial.printf("Loaded RPM values: Min=%u, Max=%u\n", minRPM, maxRPM);
}

// Event handler for incrementing min RPM
void ui_event_Button85(lv_event_t *e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        if (minRPM < maxRPM - 1) {
            minRPM++;
            char buffer[10];
            snprintf(buffer, sizeof(buffer), "%u", minRPM);
            lv_label_set_text(ui_servicespeed, buffer);
        }
    }
}

// Event handler for decrementing min RPM
void ui_event_Button86(lv_event_t *e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        if (minRPM > MIN_SPEED_RPM) {
            minRPM--;
            char buffer[10];
            snprintf(buffer, sizeof(buffer), "%u", minRPM);
            lv_label_set_text(ui_servicespeed, buffer);
        }
    }
}

// Event handler for incrementing max RPM
void ui_event_Button55(lv_event_t *e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        if (maxRPM > minRPM + 1) {
            maxRPM++;
            char buffer[10];
            snprintf(buffer, sizeof(buffer), "%u", maxRPM);
            lv_label_set_text(ui_servicespeedmax, buffer);
        }
    }
}

// Event handler for decrementing max RPM
void ui_event_Button56(lv_event_t *e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        if (maxRPM > MIN_SPEED_RPM) {
            maxRPM--;
            char buffer[10];
            snprintf(buffer, sizeof(buffer), "%u", maxRPM);
            lv_label_set_text(ui_servicespeedmax, buffer);
        }
    }
}

// Update UI on another screen
void updateLabelsOnOtherScreen() {
    char buffer[20];
    snprintf(buffer, sizeof(buffer), "MIN\n  %u", minRPM);
    lv_label_set_text(ui_Label8, buffer);

    snprintf(buffer, sizeof(buffer), "MAX\n  %u", maxRPM);
    lv_label_set_text(ui_Label10, buffer);
}

// Event handler for saving values and switching screens
void ui_event_SaveAndSwitchButton(lv_event_t *e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        saveRPMValuesToNVS();
        updateLabelsOnOtherScreen();
        _ui_screen_change(&ui_Screen5, LV_SCR_LOAD_ANIM_FADE_ON, 15, 0, NULL);
    }
}

void ui_event_cleanerbutton(lv_event_t * e)
{
     lv_event_code_t event_code = lv_event_get_code(e);
    if (event_code == LV_EVENT_CLICKED) {
        lv_label_set_text(ui_Label9, "0");
        update_panel_colors(10, 60);
    }
}

void ui_event_modebutton5(lv_event_t *e) {
    lv_event_code_t event_code = lv_event_get_code(e);

    if (event_code == LV_EVENT_CLICKED) {
        Serial.println("Mode button 5 clicked. Changing Auto Mode program.");
        _ui_screen_change(&ui_Screen3, LV_SCR_LOAD_ANIM_FADE_ON, 15, 0, NULL);

        // Get the current program number from Auto Mode memory
        const char *programNumberText = lv_label_get_text(ui_Label35); // Assuming ui_Label35 shows the current program number
        uint8_t selectedProgramNumber = atoi(programNumberText);

        if (selectedProgramNumber < 1 || selectedProgramNumber > MAX_PROGRAMS) {
            Serial.printf("Invalid program number: %d\n", selectedProgramNumber);
            return;
        }

        // Load the program data from Auto mode memory
        ProgramData programData;
        if (loadProgramData(selectedProgramNumber, &programData)) {
            Serial.printf("Program %d loaded successfully.\n", selectedProgramNumber);

            // Update UI elements with the program data
            if (programData.stepCount > 0) {
                const StepData &step = programData.steps[0];

                // Update time in UI (minutes and seconds)
                char minutesText[4], secondsText[4];
                snprintf(minutesText, sizeof(minutesText), "%02u", step.time / 60);
                snprintf(secondsText, sizeof(secondsText), "%02u", step.time % 60);
                lv_label_set_text(ui_Label107, minutesText);
                lv_label_set_text(ui_Label108, secondsText);

                // Update speed value in UI
                char speedText[8];
                snprintf(speedText, sizeof(speedText), "%u", step.speed);
                lv_label_set_text(ui_Label99, speedText);

                // Update repeat count in UI
                char repeatText[4];
                snprintf(repeatText, sizeof(repeatText), "%u", step.repeats);
                lv_label_set_text(ui_Label2222, repeatText);

                Serial.printf("UI updated with Program %d data: Time=%02u:%02u, Speed=%u, Repeats=%u\n",
                              selectedProgramNumber, step.time / 60, step.time % 60, step.speed, step.repeats);
            } else {
                Serial.printf("Program %d has no steps.\n", selectedProgramNumber);
            }
        } else {
            Serial.printf("Failed to load program %d.\n", selectedProgramNumber);
        }
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
    currentMode = MODE_MANUAL; // Default mode
    setMode(MODE_MANUAL);      // Initialize the UI for Manual Mode

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

    // Create mutexes
    nvs_mutex = xSemaphoreCreateMutex();
    if (nvs_mutex == NULL) {
        Serial.println("Failed to create NVS mutex!");
        while (1);
    }

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
 
    nvs_mutex = xSemaphoreCreateMutex();


    if (ui_Switch1) {
        lv_obj_add_event_cb(ui_Switch1, event_handler_ui_Switch1, LV_EVENT_VALUE_CHANGED, NULL);
    } else {
        Serial.println("Error: ui_Switch1 is NULL");
    }
    lv_obj_add_event_cb(ui_rotationbutton, rotationChangedEventHandler, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(ui_starstopbutton, event_handler_start_motor_button, LV_EVENT_CLICKED, NULL);
    // Attach to time inputs
// lv_obj_add_event_cb(ui_time1, event_handler_custom_value_changed, LV_EVENT_VALUE_CHANGED, NULL);
// lv_obj_add_event_cb(ui_time4, event_handler_custom_value_changed, LV_EVENT_VALUE_CHANGED, NULL);
// lv_obj_add_event_cb(ui_time3, event_handler_custom_value_changed, LV_EVENT_VALUE_CHANGED, NULL);

// // Attach to speed inputs
// lv_obj_add_event_cb(ui_rpmvalue, event_handler_custom_value_changed, LV_EVENT_VALUE_CHANGED, NULL);
// lv_obj_add_event_cb(ui_rpmvalue2, event_handler_custom_value_changed, LV_EVENT_VALUE_CHANGED, NULL);
// lv_obj_add_event_cb(ui_rpmvalue3, event_handler_custom_value_changed, LV_EVENT_VALUE_CHANGED, NULL);
// lv_obj_add_event_cb(ui_rpmvalue4, event_handler_custom_value_changed, LV_EVENT_VALUE_CHANGED, NULL);

// // Attach to rotation inputs
// lv_obj_add_event_cb(ui_rotationbutton1, event_handler_custom_value_changed, LV_EVENT_VALUE_CHANGED, NULL);
// lv_obj_add_event_cb(ui_rotationbutton3, event_handler_custom_value_changed, LV_EVENT_VALUE_CHANGED, NULL);
// lv_obj_add_event_cb(ui_rotationbutton4, event_handler_custom_value_changed, LV_EVENT_VALUE_CHANGED, NULL);

lv_obj_add_event_cb(ui_modebutton, ui_event_modebutton, LV_EVENT_CLICKED, NULL);
lv_obj_add_event_cb(ui_modebutton4, ui_event_modebutton4, LV_EVENT_CLICKED, NULL);
lv_obj_add_event_cb(ui_modebutton7, ui_event_modebutton7, LV_EVENT_CLICKED, NULL);
lv_obj_add_event_cb(ui_serialbutton, ui_event_serialbutton, LV_EVENT_CLICKED, NULL);
lv_obj_add_event_cb(ui_Button85, ui_event_Button85, LV_EVENT_CLICKED, NULL);
lv_obj_add_event_cb(ui_Button86, ui_event_Button86, LV_EVENT_CLICKED, NULL);
lv_obj_add_event_cb(ui_Button55, ui_event_Button55, LV_EVENT_CLICKED, NULL);
lv_obj_add_event_cb(ui_Button56, ui_event_Button56, LV_EVENT_CLICKED, NULL);




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
ModbusSerial.setTimeout(100);
 nvs_mutex = xSemaphoreCreateMutex();
    /* Create tasks */
    xTaskCreate(wifi_connect_task, "WiFi Connect Task", 3072, NULL, 1, NULL);
    xTaskCreate(wifi_scan_task, "WiFi Scan Task", 3072, NULL, 1, NULL);
    xTaskCreate(modbusLoopTask, "Modbus Loop Task", 2048, NULL, 2, NULL);
    xTaskCreate(countdownTask, "Countdown Task", 4096, NULL, 1, &countdownTaskHandle);

    // xTaskCreate(server_connection_task, "Server Connection Task", 16384, NULL, 1, NULL);

    // // Create the OTA update task
    // xTaskCreate(otaUpdateTask, "OTA Update Task", 16384, NULL, 1, &otaTaskHandle);
    // xTaskCreate(saveProgramTask, "Save Program Task", 4096, NULL, 1, NULL);
    // xTaskCreate(vfdActualSpeedReadTask, "VFD Actual Speed Read Task", 4096, NULL, 1, NULL);
    // xTaskCreate(modbusTask, "Modbus Task", 4096, NULL, 2, NULL);
    // xTaskCreate(vfdReadTask, "VFD Read Task", 4096, NULL, 1, NULL);



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



    // if (result == ESP_OK) {
    //     Serial.println("Test start command sent successfully.");
    // } else {
    //     Serial.printf("Failed to send test start command. Error: %d\n", result);
    // }

    Serial.println("Modbus initialized");
    Serial.println("Setup done");
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}