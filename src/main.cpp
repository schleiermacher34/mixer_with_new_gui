#include <Arduino.h>
#include <lvgl.h>
#include <ESP_Panel_Library.h>
#include <ESP_IOExpander_Library.h>
#include <ui.h>

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
#define LVGL_BUF_SIZE           (ESP_PANEL_LCD_H_RES * 20)

ESP_Panel *panel = NULL;
SemaphoreHandle_t lvgl_mux = NULL;                  // LVGL mutex

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





void setup()
{
    Serial.begin(115200); /* prepare for possible serial debug */

    String LVGL_Arduino = "Hello LVGL! ";
    LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

    Serial.println(LVGL_Arduino);
    Serial.println("I am ESP32_Display_Panel");

    panel = new ESP_Panel();

    /* Initialize LVGL core */
    lv_init();

    /* Initialize LVGL buffers */
    static lv_disp_draw_buf_t draw_buf;
    /* Using double buffers is more faster than single buffer */
    /* Using internal SRAM is more fast than PSRAM (Note: Memory allocated using `malloc` may be located in PSRAM.) */
    uint8_t *buf = (uint8_t *)heap_caps_calloc(1, LVGL_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_INTERNAL);
    assert(buf);
    lv_disp_draw_buf_init(&draw_buf, buf, NULL, LVGL_BUF_SIZE);

    /* Initialize the display device */
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    /* Change the following line to your display resolution */
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
    /* This is useful for refreshing the screen using DMA transfers */
    panel->getLcd()->setCallback(notify_lvgl_flush_ready, &disp_drv);
#endif

    /**
     * These development boards require the use of an IO expander to configure the screen,
     * so it needs to be initialized in advance and registered with the panel for use.
     *
     */
    Serial.println("Initialize IO expander");
    /* Initialize IO expander */
    // ESP_IOExpander *expander = new ESP_IOExpander_CH422G(I2C_MASTER_NUM, ESP_IO_EXPANDER_I2C_CH422G_ADDRESS_000, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO);
    ESP_IOExpander *expander = new ESP_IOExpander_CH422G(I2C_MASTER_NUM, ESP_IO_EXPANDER_I2C_CH422G_ADDRESS_000);
    expander->init();
    expander->begin();
    expander->multiPinMode(TP_RST | LCD_BL | LCD_RST | SD_CS | USB_SEL, OUTPUT);
    expander->multiDigitalWrite(TP_RST | LCD_BL | LCD_RST | SD_CS, HIGH);

    // Turn off backlight
    // expander->digitalWrite(USB_SEL, LOW);
    expander->digitalWrite(USB_SEL, LOW);
    /* Add into panel */
    panel->addIOExpander(expander);

    /* Start panel */
    panel->begin();

    /* Create a task to run the LVGL task periodically */
    lvgl_mux = xSemaphoreCreateRecursiveMutex();
    xTaskCreate(lvgl_port_task, "lvgl", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL);

    /* Lock the mutex due to the LVGL APIs are not thread-safe */
    lvgl_port_lock(-1);

    
    ui_init();

    /* Release the mutex */
    lvgl_port_unlock();

    Serial.println("Setup done");
}

void loop()
{
    // Serial.println("Loop");
    sleep(5);
}
