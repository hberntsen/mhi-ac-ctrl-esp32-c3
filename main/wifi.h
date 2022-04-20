#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define MAXIMUM_RETRY 6
#define STOP_AP_DELAY 20000         // how long to keep softAP running after obtaining an IP (in ms)

SemaphoreHandle_t* get_wifi_mutex();

void my_wifi_init();
void start_ap_prov();
void stop_ap_prov();

#ifdef __cplusplus
}
#endif