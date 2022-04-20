#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_AP_COUNT 10
#define LOG_BUF_MAX_LINE_SIZE 160
#define MAX_SSE_CLIENTS 3

esp_err_t start_webserver(void);
void stop_webserver(void);

#ifdef __cplusplus
}
#endif 