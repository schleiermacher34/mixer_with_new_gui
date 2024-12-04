#ifndef MAIN_H
#define MAIN_H


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


void lvgl_port_lock(int timeout_ms);
void lvgl_port_unlock(void);
void lvgl_port_task(void *arg);
void wifi_connect_task(void *pvParameters);
void wifi_scan_task(void *pvParameters);
void saveCurrentProgramData();
void collectCustomProgramData(CustomProgramData* customData);
bool modbusCallback(Modbus::ResultCode event, uint16_t transactionId, void* data);
void stopMotor();



#endif