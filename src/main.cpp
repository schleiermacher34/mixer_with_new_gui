#include <Arduino.h>
#include <lvgl.h>
#include <ESP_Panel_Library.h>
#include <ESP_IOExpander_Library.h>
#include <ui.h>
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
#include "esp_ota_ops.h"  // Include ESP-IDF OTA operations
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

/**
/* To use the built-in examples and demos of LVGL uncomment the includes below respectively.
 * You also need to copy `lvgl/examples` to `lvgl/src/examples`. Similarly for the demos `lvgl/demos` to `lvgl/src/demos`.
 */
// #include <demos/lv_demos.h>
// #include <examples/lv_examples.h>

/* LVGL porting configurations */
#define LVGL_TICK_PERIOD_MS     (2)
#define LVGL_TASK_MAX_DELAY_MS  (500)
#define LVGL_TASK_MIN_DELAY_MS  (1)
#define LVGL_TASK_STACK_SIZE    (4 * 1024)
#define LVGL_TASK_PRIORITY      (2)
#define LVGL_BUF_SIZE           (ESP_PANEL_LCD_H_RES * ESP_PANEL_LCD_V_RES / 10)

ESP_Panel *panel = NULL;
SemaphoreHandle_t lvgl_mux = NULL;                  // LVGL mutex
SemaphoreHandle_t wifi_scan_semaphore = NULL;
SemaphoreHandle_t wifi_connect_semaphore = NULL;
SemaphoreHandle_t wifi_mutex;

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

void lvgl_port_task(void *arg)
{
    Serial.println("Starting LVGL task");

    uint32_t task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
    while (1) {
        // Lock the mutex due to the LVGL APIs are not thread-safe
        lvgl_port_lock(-1);
        task_delay_ms = lv_timer_handler();
        // Release the mutex
        lvgl_port_unlock();
        if (task_delay_ms > LVGL_TASK_MAX_DELAY_MS) {
            task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
        } else if (task_delay_ms < LVGL_TASK_MIN_DELAY_MS) {
            task_delay_ms = LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

void wifi_connect_task(void *pvParameters) {
    while (1) {
        // Wait for the semaphore to be given
        if (xSemaphoreTake(wifi_connect_semaphore, portMAX_DELAY)) {
            Serial.println("Wi-Fi connect task started.");

            // Initialize buffers
            char ssid[64] = {0};
            char password[64] = {0};

            // Lock LVGL before accessing UI elements
            lvgl_port_lock(portMAX_DELAY);

            // Get the selected SSID from the dropdown
            if (ui_Roller1 != NULL) {
                lv_dropdown_get_selected_str(ui_Roller1, ssid, sizeof(ssid));
                Serial.print("Selected SSID: ");
                Serial.println(ssid);
            } else {
                Serial.println("Error: ui_Screen2_Dropdown2 is NULL");
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
                // Copy the password safely
                strncpy(password, password_tmp, sizeof(password) - 1);
                password[sizeof(password) - 1] = '\0'; // Ensure null-termination
            }

            // Check if SSID is empty
            if (strlen(ssid) == 0) {
                Serial.println("Error: SSID is empty");
                lvgl_port_lock(portMAX_DELAY);
                //lv_label_set_text(ui_Screen2_Label7, "SSID is empty. Please select a network.");
                lvgl_port_unlock();
                continue; // Skip this iteration
            }

            // Safely print the SSID and password lengths
            Serial.print("Attempting to connect to SSID: ");
            Serial.println(ssid);
            Serial.print("Using password: ");
            Serial.println(password);
            Serial.printf("SSID length: %u\n", strlen(ssid));
            Serial.printf("Password length: %u\n", strlen(password));

            // Acquire the Wi-Fi mutex before accessing Wi-Fi functions
            if (xSemaphoreTake(wifi_mutex, portMAX_DELAY)) {
                // Disconnect before attempting new connection
                WiFi.disconnect(true); // The 'true' parameter tells WiFi to erase old connection data

                // Delay briefly to ensure disconnect completes
                delay(100);

                // Attempt to connect to the selected network
                if (strlen(password) == 0) {
                    Serial.println("No password provided, connecting without password.");
                    WiFi.begin(ssid); // Connect without password
                } else {
                    WiFi.begin(ssid, password);
                }

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

                    // Lock LVGL before accessing UI elements
                    lvgl_port_lock(portMAX_DELAY);

                //     // Check if the checkbox is checked
                //   //  bool savePassword = lv_obj_has_state(ui_Screen2_Checkbox1, LV_STATE_CHECKED);

                //     // Unlock LVGL
                //     lvgl_port_unlock();

                //     if (savePassword) {
                //         Serial.println("Saving Wi-Fi credentials to NVS.");
                //         // Acquire the NVS mutex before writing to NVS
                //         xSemaphoreTake(nvs_mutex, portMAX_DELAY);
                //         saveWifiCredentialsToNVS(ssid, password);
                //         xSemaphoreGive(nvs_mutex);
                //     }

                    // Update the UI with connection success
                    lvgl_port_lock(portMAX_DELAY);
                    lv_label_set_text(ui_backtosetupbutton2, "Connected to network");
                    lvgl_port_unlock();

                } else {
                    Serial.println("\nWi-Fi connection failed.");

                    // Update the UI with connection failure
                    lvgl_port_lock(portMAX_DELAY);
                    lv_label_set_text(ui_backtosetupbutton2, "Failed to connect to network");
                    lvgl_port_unlock();
                }
            } else {
                Serial.println("Failed to acquire Wi-Fi mutex. Cannot perform Wi-Fi operations.");
                // Optionally update UI to inform the user
                lvgl_port_lock(portMAX_DELAY);
                lv_label_set_text(ui_backtosetupbutton2, "Wi-Fi is busy. Try again later.");
                lvgl_port_unlock();
            }
        }
    }
}

void wifi_scan_task(void *pvParameters) {
    while (1) {
        // Wait for the semaphore to be given
        if (xSemaphoreTake(wifi_scan_semaphore, portMAX_DELAY)) {
            // Acquire the Wi-Fi mutex
            if (xSemaphoreTake(wifi_mutex, portMAX_DELAY)) {
                Serial.println("Starting WiFi scan...");

                // Clear existing dropdown items
                lvgl_port_lock(-1);
                lv_dropdown_clear_options(ui_Roller1);
                lvgl_port_unlock();

                // Set Wi-Fi mode to station mode
                WiFi.mode(WIFI_STA);

                // Start Wi-Fi scan
                int n = WiFi.scanNetworks();  // Scan for available networks
                Serial.println("Scan completed");

                // Check if any networks were found
                if (n == 0) {
                    Serial.println("No networks found");
                    lvgl_port_lock(-1);
                    lv_dropdown_set_options(ui_Roller1, "No networks found");
                    lvgl_port_unlock();
                } else {
                    Serial.printf("%d networks found:\n", n);
                    String ssidList = "";  // Initialize empty string for storing SSIDs

                    // Loop through the found networks and append them to ssidList
                    for (int i = 0; i < n; ++i) {
                        String ssid = WiFi.SSID(i);  // Get the SSID of the network
                        Serial.printf("%d: %s (%d dBm)\n", i + 1, ssid.c_str(), WiFi.RSSI(i));
                        ssidList += ssid + "\n";  // Add a newline for each SSID
                    }

                    // Update dropdown options with available SSIDs
                    lvgl_port_lock(-1);
                    lv_dropdown_set_options(ui_Roller1, ssidList.c_str());
                    lvgl_port_unlock();
                }

                WiFi.scanDelete();  // Clean up the scanned networks from memory

                // Release the Wi-Fi mutex
                xSemaphoreGive(wifi_mutex);
            }
        }
    }
}

/* Event Handlers for Wi-Fi Buttons */
void event_handler_scan_button(lv_event_t * e) {
    Serial.println("Scan button pressed, giving semaphore to WiFi scan task...");
    xSemaphoreGive(wifi_scan_semaphore);  // Unblock the Wi-Fi scan task
}

void event_handler_connect_button(lv_event_t * e) {
    Serial.println("Connect button pressed, giving semaphore to Wi-Fi connection task...");
    if (wifi_connect_semaphore != NULL) {
        xSemaphoreGive(wifi_connect_semaphore);  // Unblock the Wi-Fi connection task
    } else {
        Serial.println("Error: wifi_connect_semaphore is NULL");
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


void WiFiEvent(WiFiEvent_t event) {
    switch (event) {
        case SYSTEM_EVENT_STA_GOT_IP:
            Serial.println("Wi-Fi connected. IP address: ");
            Serial.println(WiFi.localIP());
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            Serial.println("Wi-Fi lost connection");
            // Try to reconnect
            WiFi.reconnect();
            break;
        default:
            break;
    }
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
void setup() {
    Serial.begin(115200); // Start serial communication for debugging
    Serial.println("Starting setup...");

    // Display LVGL version
    String LVGL_Arduino = "Hello LVGL! ";
    LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();
    Serial.println(LVGL_Arduino);

    // Initialize the panel object
    panel = new ESP_Panel();

    // Initialize LVGL core
    lv_init();

    static lv_disp_draw_buf_t draw_buf;

    // Allocate LVGL buffer
uint8_t *buf = (uint8_t *)heap_caps_malloc(LVGL_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
if (!buf) {
    // Fallback to internal memory if PSRAM allocation fails
    buf = (uint8_t *)heap_caps_calloc(1, LVGL_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_INTERNAL);
    if (!buf) {
        Serial.println("Failed to allocate display buffer!");
        while (true); // Halt execution for debugging
    }
}
if (ui_Switch1) {
        lv_obj_add_event_cb(ui_Switch1, event_handler_scan_button, LV_EVENT_CLICKED, NULL);
    } else {
        Serial.println("ui_Switch1 is NULL");
    }

    lv_disp_draw_buf_init(&draw_buf, buf, NULL, LVGL_BUF_SIZE);

    // Initialize the display device
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = ESP_PANEL_LCD_H_RES; // Set the horizontal resolution
    disp_drv.ver_res = ESP_PANEL_LCD_V_RES; // Set the vertical resolution
    disp_drv.flush_cb = lvgl_port_disp_flush; // Set the flush callback
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

#if ESP_PANEL_USE_LCD_TOUCH
    // Initialize touch input
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = lvgl_port_tp_read;
    lv_indev_drv_register(&indev_drv);
#endif

    // Initialize the panel
    panel->init();

#if ESP_PANEL_LCD_BUS_TYPE != ESP_PANEL_BUS_TYPE_RGB
    // Register callback for DMA flush ready
    panel->getLcd()->setCallback(notify_lvgl_flush_ready, &disp_drv);
#endif

    // Initialize IO expander if required by the board
    Serial.println("Initialize IO expander...");
    ESP_IOExpander *expander = new ESP_IOExpander_CH422G(I2C_MASTER_NUM, ESP_IO_EXPANDER_I2C_CH422G_ADDRESS_000);
    expander->init();
    expander->begin();
    expander->multiPinMode(TP_RST | LCD_BL | LCD_RST | SD_CS | USB_SEL, OUTPUT);
    expander->multiDigitalWrite(TP_RST | LCD_BL | LCD_RST | SD_CS, HIGH);
    expander->digitalWrite(USB_SEL, LOW); // Configure USB_SEL pin
    panel->addIOExpander(expander); // Add expander to the panel

    // Start the panel
    panel->begin();

    // Initialize Wi-Fi in station mode
    WiFi.mode(WIFI_STA);
    WiFi.onEvent(WiFiEvent);
    WiFi.disconnect();

    delay(100); // Short delay to ensure Wi-Fi is initialized

    // Create a mutex for LVGL to ensure thread safety
    lvgl_mux = xSemaphoreCreateRecursiveMutex();
    wifi_connect_semaphore = xSemaphoreCreateBinary();
    wifi_mutex = xSemaphoreCreateMutex();

    // Create a task for LVGL periodic handling
    xTaskCreate(lvgl_port_task, "lvgl", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL);
    // xTaskCreate(wifi_scan_task, "WiFi Scan Task", 4096 * 2, NULL, 1, NULL);
    // xTaskCreate(wifi_connect_task, "WiFi Connect Task", 2046, NULL, 1, NULL);

    // Lock LVGL while initializing the UI
    lvgl_port_lock(-1);
    ui_init(); // Initialize the UI (from your generated ui.h)
    lvgl_port_unlock();

    Serial.println("Setup done.");
}



void loop() {
    // LVGL periodic task handler
    lv_timer_handler();
    delay(5); // Small delay to avoid excessive CPU usage
}

