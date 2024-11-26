#ifndef MAIN_H
#define MAIN_H

void lvgl_port_lock(int timeout_ms);
void lvgl_port_unlock(void);
void lvgl_port_task(void *arg);
void wifi_connect_task(void *pvParameters);
void wifi_scan_task(void *pvParameters);



#endif