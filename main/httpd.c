#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <freertos/timers.h>
#include <sys/param.h>                          // MIN MAX

#include "esp_http_server.h"
#include "esp_wifi.h"
#include "cJSON.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"                       // SSE uses send()

#include "esp_ota_ops.h"
#include "esp_image_format.h"

#include "wifi.h"
#include "httpd.h"
#include <homekit/homekit.h>

#include "esp_log.h"
static const char *TAG = "myhttpd";

#define SCRATCH_BUFSIZE 1024

static httpd_handle_t server = NULL;

esp_event_handler_instance_t wifi_event_handler_instance;
esp_event_handler_instance_t ip_event_handler_instance;

char log_buf[LOG_BUF_MAX_LINE_SIZE];
static TaskHandle_t t_sse_task_handle;
static QueueHandle_t q_sse_message_queue;

// set volatile as a client could be removed at any time and the sse_logging_task 
// needs to make sure it has an updated copy
volatile int sse_sockets[MAX_SSE_CLIENTS];

int sse_logging_vprintf(const char *format, va_list arg) {
    vsprintf(log_buf, format, arg);
    xQueueSendToBack(q_sse_message_queue, log_buf, 0);

    // still send to console
    return vprintf(format, arg);
}

void send_sse_message (char* message, char* event) {
    const char *sse_data = "data: ";
    const char *sse_event = "\nevent: ";
    const char *sse_end_message = "\n\n";

    size_t event_len = 0;
    if (event != NULL) {
        event_len = strlen(sse_event) + strlen(event);
    }
    char send_buf[strlen(sse_data) + strlen(message) + event_len + strlen(sse_end_message)];
    strcpy(send_buf, sse_data);
    strcat(send_buf, message);
    if (event != NULL) {
        strcat(send_buf, sse_event);
        strcat(send_buf, event);
    }
    strcat(send_buf, sse_end_message);

    size_t message_len = strlen(send_buf);

    int return_code;
    for (int i = 0; i < MAX_SSE_CLIENTS; i++) {
        if (sse_sockets[i] != 0) {
            return_code = send(sse_sockets[i], send_buf, message_len, 0);
            if (return_code < 0) {
                httpd_sess_trigger_close(server, sse_sockets[i]);
            }
        }
    }
}

static void sse_logging_task(void * param)
{
    char recv_buf[LOG_BUF_MAX_LINE_SIZE];

    while(1) {
        if (xQueueReceive(q_sse_message_queue, recv_buf, portMAX_DELAY) == pdTRUE) {
            send_sse_message(recv_buf, NULL);
        } 
    }
}

static void status_json_sse_handler()
{
    char ip_buf[17];
    char *out;
    cJSON *root;
    root = cJSON_CreateObject();

    wifi_config_t wifi_cfg;
    esp_wifi_get_config(ESP_IF_WIFI_STA, &wifi_cfg);

    esp_netif_ip_info_t ip_info;
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_get_ip_info(netif, &ip_info);
    bool if_status = esp_netif_is_netif_up(netif);

    cJSON_AddItemToObject(root, "ssid", cJSON_CreateString((const char *)wifi_cfg.sta.ssid));
    snprintf(ip_buf, 17, IPSTR, IP2STR(&ip_info.ip));
    cJSON_AddItemToObject(root, "ip", cJSON_CreateString(ip_buf));
    snprintf(ip_buf, 17, IPSTR, IP2STR(&ip_info.netmask));
    cJSON_AddItemToObject(root, "netmask", cJSON_CreateString(ip_buf));
    snprintf(ip_buf, 17, IPSTR, IP2STR(&ip_info.gw));
    cJSON_AddItemToObject(root, "gw", cJSON_CreateString(ip_buf));
    cJSON_AddItemToObject(root, "if_status", cJSON_CreateBool(if_status));
 
    out = cJSON_PrintUnformatted(root);

    ESP_LOGI(TAG, "ssid: %s, ip:"IPSTR", netmask:"IPSTR", gw:"IPSTR", if_up? %s",
        wifi_cfg.sta.ssid, IP2STR(&ip_info.ip), IP2STR(&ip_info.netmask), IP2STR(&ip_info.gw), if_status?"TRUE":"FALSE");

    send_sse_message(out, "status");

    /* free all objects under root and root itself */
    cJSON_Delete(root);
    free(out);

}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    status_json_sse_handler();
}

void free_sse_ctx_func(void *ctx)
{
    int client_fd = *((int*) ctx);

    ESP_LOGD(TAG, "free_sse_context sse_socket: %d", client_fd);

    for (int i = 0; i < MAX_SSE_CLIENTS; i++) {
        if (sse_sockets[i] == client_fd) {
            sse_sockets[i] = 0;
        }
    } 
    free(ctx);
}

esp_err_t server_side_event_registration_handler(httpd_req_t *req)
{
    // disable sending to sse_socket until a proper HTTP 200 OK response has been sent back to client
    esp_log_set_vprintf(vprintf);
    
    int len;
    char buffer[150];
    int i = 0;

    /* Create session's context if not already available */
    if (!req->sess_ctx) {
        for (i = 0; i < MAX_SSE_CLIENTS; i++) {
            if (sse_sockets[i] == 0) {
                req->sess_ctx = malloc(sizeof(int)); 
                req->free_ctx = free_sse_ctx_func; 
                int client_fd = httpd_req_to_sockfd(req);  
                *(int *)req->sess_ctx = client_fd;
                sse_sockets[i] = client_fd;
                ESP_LOGD(TAG, "sse_socket: %d slot %d", sse_sockets[i], i);
                break;
            }
        }
        if (i == MAX_SSE_CLIENTS) {
            len = sprintf(buffer, "HTTP/1.1 503 Server Busy\r\nContent-Length: 0\r\n\r\n");
        }
        else {
            len = sprintf(buffer, "HTTP/1.1 200 OK\r\n"
                                    "Connection: Keep-Alive\r\n"
                                    "Content-Type: text/event-stream\r\n"
                                    "Cache-Control: no-cache\r\n\r\n");
        }
    } else {
        // Should never get here?
        len = sprintf(buffer, "HTTP/1.1 400 Session Already Active\r\n\r\n");
    }

    // need to use raw send function, as httpd_resp_send will add Content-Length: 0 which
    // will make the client disconnect
    httpd_send(req, buffer, len);

    // send first wi-fi status information;
    status_json_sse_handler();

    // send firmware details information;
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_app_desc_t app_desc;
    esp_ota_get_partition_description(running, &app_desc);
    sprintf(buffer, "{\"version\":\"%s\"}", app_desc.version);
    send_sse_message(buffer, "firmware");

    // enable sse logging again
    esp_log_set_vprintf(&sse_logging_vprintf);

    return ESP_OK;
}

esp_err_t root_handler(httpd_req_t *req)
{
    extern const char index_html_start[] asm("_binary_wifi_html_gz_start");
    extern const char index_html_end[] asm("_binary_wifi_html_gz_end");

    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    httpd_resp_send(req, index_html_start ,(index_html_end-index_html_start));

    return ESP_OK;
}

/* GET handler for /ap.json. Retrieves list of APs */
esp_err_t ap_json_handler(httpd_req_t *req)
{
    char *out;
    cJSON *root, *fld;
    root = cJSON_CreateArray();

    uint16_t ap_count = 0;
    esp_err_t err;

    if( xSemaphoreTake(*(get_wifi_mutex()), pdMS_TO_TICKS(500)) == pdTRUE) {
        esp_wifi_scan_start(NULL, true);
        xSemaphoreGive(*(get_wifi_mutex()));       // only if wifi_scan_start is blocking, otherwise, do it in SCAN_DONE event

        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));

        wifi_ap_record_t *ap_info = (wifi_ap_record_t *)malloc(sizeof(wifi_ap_record_t) * ap_count);
//        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_count, ap_info));
        err = esp_wifi_scan_get_ap_records(&ap_count, ap_info);

        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Total APs scanned = %u", ap_count);
            for (int i = 0; i < ap_count && i <= MAX_AP_COUNT; i++) {
                cJSON_AddItemToArray(root, fld = cJSON_CreateObject());
                cJSON_AddItemToObject(fld, "ssid", cJSON_CreateString((const char *)ap_info[i].ssid));
                cJSON_AddItemToObject(fld, "chan", cJSON_CreateNumber(ap_info[i].primary));
                cJSON_AddItemToObject(fld, "rssi", cJSON_CreateNumber(ap_info[i].rssi));
                cJSON_AddItemToObject(fld, "auth", cJSON_CreateNumber(ap_info[i].authmode));
            }
        }
        else {
            ESP_LOGW(TAG, "error in esp_wifi_scan_get_ap_records");
        }

        free(ap_info);
    }
    else {
       ESP_LOGW(TAG, "cannot take semaphore. wi-fi busy (ap_json_handler)");
    }

    out = cJSON_PrintUnformatted(root);
   
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    httpd_resp_set_hdr(req, "Pragma", "no-cache");
    httpd_resp_send(req, out, strlen(out));      

    /* free all objects under root and root itself */
    cJSON_Delete(root);
    free(out);

    return ESP_OK;
}

/* POST handler for /connect.json. Takes SSID and Password and connects to AP */
esp_err_t connect_json_handler(httpd_req_t *req)
{
    int total_len = req->content_len;
    int cur_len = 0;
    char buf[SCRATCH_BUFSIZE];
    int received = 0;

    if (total_len >= SCRATCH_BUFSIZE) {
        // Client will not receive response if it hasn't finished sending the POST data
        // Can't store to buffer (too big), so just close connection
        return ESP_FAIL;
    }
    while (cur_len < total_len) {
        received = httpd_req_recv(req, buf + cur_len, total_len);
        if (received <= 0) {
            if (received == HTTPD_SOCK_ERR_TIMEOUT) {
                    // Retry if timeout occurred
                    continue;
                }
                ESP_LOGE(TAG, "JSON reception failed!");
                return ESP_FAIL;
        }
        cur_len += received;
    }
    buf[total_len] = '\0';

    cJSON *root = cJSON_Parse(buf);
    cJSON *ssid = cJSON_GetObjectItem(root, "ssid");
    cJSON *pwd = cJSON_GetObjectItem(root, "password");

    wifi_config_t wifi_cfg;
    esp_wifi_get_config(ESP_IF_WIFI_STA, &wifi_cfg);

    snprintf((char *)wifi_cfg.sta.ssid, 32, "%s", ssid->valuestring);
    snprintf((char *)wifi_cfg.sta.password, 32, "%s", pwd->valuestring);

    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_cfg)); 

    esp_wifi_disconnect();  
    esp_wifi_connect();         // ignore mutex - we want this to take priority and immediately
    
    httpd_resp_send(req, NULL, 0);

    cJSON_Delete(root);
    return ESP_OK;
}

/* POST handler for /restart.json. Restarts ESP. Optionally resets NVS and/or HomeKit */
esp_err_t restart_json_handler(httpd_req_t *req)
{
    int total_len = req->content_len;
    int cur_len = 0;
    char buf[SCRATCH_BUFSIZE];
    int received = 0;

    esp_err_t err = ESP_OK;

    if (total_len >= SCRATCH_BUFSIZE) {
        // Client will not receive response if it hasn't finished sending the POST data
        // Can't store to buffer (too big), so just close connection
        return ESP_FAIL;
    }
    while (cur_len < total_len) {
        received = httpd_req_recv(req, buf + cur_len, total_len);
        if (received <= 0) {
            if (received == HTTPD_SOCK_ERR_TIMEOUT) {
                    // Retry if timeout occurred
                    continue;
                }
                ESP_LOGE(TAG, "JSON reception failed!");
                return ESP_FAIL;
        }
        cur_len += received;
    }
    buf[total_len] = '\0';

    cJSON *root = cJSON_Parse(buf);
    cJSON *reset_nvs = cJSON_GetObjectItem(root, "reset-nvs");
    cJSON *reset_homekit = cJSON_GetObjectItem(root, "reset-homekit");

    if (cJSON_IsTrue(reset_nvs)) {
        nvs_flash_erase();
    }
    if (cJSON_IsTrue(reset_homekit)) {
        homekit_server_reset();
    }

    if (err == ESP_OK) {
        ESP_LOGW(TAG, "Reset nvs = %d, homekit = %d. Restarting...", 
            cJSON_IsTrue(reset_nvs), cJSON_IsTrue(reset_homekit));

        // Just send an OK response. Client can monitor console logs.
        httpd_resp_send(req, NULL, 0);

        vTaskDelay(pdMS_TO_TICKS(2000));
        esp_restart();
    }
    else {
        ESP_LOGE(TAG, "Error %s", esp_err_to_name(err)); 

        httpd_resp_set_status(req, HTTPD_400);
        httpd_resp_send(req, NULL, 0);
    }

    cJSON_Delete(root);
    return ESP_OK;
}

/* POST handler for /update_ota. */
esp_err_t update_ota_handler(httpd_req_t *req)
{
    const esp_partition_t *configured = esp_ota_get_boot_partition();
    const esp_partition_t *running = esp_ota_get_running_partition();

    if (configured != running) {
        ESP_LOGW(TAG, "Configured OTA boot partition at offset 0x%08x, but running from offset 0x%08x",
                 configured->address, running->address);
        ESP_LOGW(TAG, "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)");
    }
    ESP_LOGI(TAG, "Running partition type %d subtype %d (offset 0x%08x)",
             running->type, running->subtype, running->address);


    esp_err_t err = ESP_OK;

    char buffer[SCRATCH_BUFSIZE];
    char sse_msg[90];
    int len = 0;
    uint8_t progress = 0;
    int remaining = req->content_len;
    bool is_image_header_checked = false;
    bool more_content = true;

    /* update handle : set by esp_ota_begin(), must be freed via esp_ota_end() */
    esp_ota_handle_t update_handle = 0 ;
    const esp_partition_t *update_partition = NULL;

    update_partition = esp_ota_get_next_update_partition(NULL);
    ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%08x",
             update_partition->subtype, update_partition->address);

    if (update_partition == NULL) {
        err = ESP_ERR_OTA_SELECT_INFO_INVALID;
    }

    // File cannot be larger than partition size
    if (update_partition != NULL && req->content_len > update_partition->size) {
        ESP_LOGE(TAG, "Content-Length of %d larger than partition size of %d", req->content_len, update_partition->size); 
        err = ESP_ERR_INVALID_SIZE;
    }

    sprintf(sse_msg, "{\"progress\":\"10\", \"status\":\"Sending File Size %dKB\"}", (int)req->content_len/1024);
    send_sse_message(sse_msg, "update");


    while (more_content) {
        if (err == ESP_OK && len > sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t) && !is_image_header_checked) {
            esp_app_desc_t new_app_info;
            // check current version with downloading
            memcpy(&new_app_info, &buffer[sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t)], sizeof(esp_app_desc_t) );
            ESP_LOGI(TAG, "New firmware version: %s", new_app_info.version);

            esp_app_desc_t running_app_info;
            if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK) {
                ESP_LOGI(TAG, "Running firmware version: %s", running_app_info.version);
            }

            if (err == ESP_OK) {
                // esp_ota_begin erases the partition, so only call it if the incoming file 
                //  looks OK.
                err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Error esp_ota_begin()"); 
                } 
                else {
                    ESP_LOGI(TAG, "Writing to partition '%s' at offset 0x%x",
                        update_partition->label, update_partition->address);
                }
            }
            is_image_header_checked = true;
        }

        // if no previous errors, continue to write. otherwise, just read the incoming data until it completes
        //  and send the HTTP error response back. stopping the connection early causes the client to
        //  display an ERROR CONNECTION CLOSED response instead of a meaningful error
        if (err == ESP_OK && is_image_header_checked) {
            err = esp_ota_write(update_handle, buffer, len);
        }

        // we use int, so no decimals. only send message on 1% change. progress from 10->95%
        if (req->content_len != 0 && progress != ((req->content_len-remaining)*85/req->content_len)+10) {
            progress = ((req->content_len-remaining)*85/req->content_len)+10;    
            sprintf(sse_msg, "{\"progress\":\"%d\", \"status\":\"Downloading..\"}", progress);
            send_sse_message(sse_msg, "update");
        }
        
        remaining -= len;

        if (remaining != 0) {
            if ((len = httpd_req_recv(req, buffer, MIN(remaining, SCRATCH_BUFSIZE))) <= 0) {
                if (len == HTTPD_SOCK_ERR_TIMEOUT) {
                    /* Retry if timeout occurred */
                    len = 0;
                    continue;
                }
                ESP_LOGE(TAG, "File reception failed!");
                err = ESP_FAIL;
            }
        }
        else {
            more_content = false;
        }
    } // end while(). no more content to read

    ESP_LOGI(TAG, "Binary transferred finished: %d bytes", req->content_len);

    if (update_handle) {
        err = esp_ota_end(update_handle);
    } 

    err = esp_ota_set_boot_partition(update_partition);

    const esp_partition_t *boot_partition = esp_ota_get_boot_partition();
    ESP_LOGW(TAG, "Next boot partition '%s' at offset 0x%x",
        boot_partition->label, boot_partition->address);

    if (err == ESP_OK) {
        httpd_resp_send(req, NULL, 0);
        sprintf(sse_msg, "{\"progress\":\"100\", \"status\":\"Firmware Installed. Restarting to '%s'...\"}", boot_partition->label);
        send_sse_message(sse_msg, "update");
    } else {
        httpd_resp_set_status(req, HTTPD_400);
        httpd_resp_send(req, NULL, 0);
        sprintf(sse_msg, "{\"progress\":\"100\", \"status\":\"Failed. Restarting to '%s'...\"}", boot_partition->label);
        send_sse_message(sse_msg, "update");
    }

    ESP_LOGI(TAG, "Prepare to restart system!");
    vTaskDelay(pdMS_TO_TICKS(4000));
    esp_restart();

    return ESP_OK;
}

esp_err_t start_webserver(void)
{
    if (server != NULL) {
        ESP_LOGE(TAG, "HTTPD already running");
        return ESP_FAIL;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 6072;
    config.max_open_sockets = 5;
    // kick off any old socket connections to allow new connections
    config.lru_purge_enable = true;

    if (httpd_start(&server, &config) == ESP_OK) {
        ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
        httpd_uri_t root_page = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = root_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &root_page);

        // Scan AP. Requested when the refresh button is pressed
        httpd_uri_t ap_json_page = {
            .uri       = "/ap.json",
            .method    = HTTP_GET,
            .handler   = ap_json_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &ap_json_page);

        // Sends SSID and password for AP to connect to
        httpd_uri_t connect_json_page = {
            .uri       = "/connect.json",
            .method    = HTTP_POST,
            .handler   = connect_json_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &connect_json_page);

        // Requests a restart - includes JSON with whether to reset NVS and/or HomeKit or perform Update
        httpd_uri_t restart_json_page = {
            .uri       = "/restart.json",
            .method    = HTTP_POST,
            .handler   = restart_json_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &restart_json_page);

        // Client requesting Server Side Events
        httpd_uri_t server_side_event_registration_page = {
            .uri       = "/event",
            .method    = HTTP_GET,
            .handler   = server_side_event_registration_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &server_side_event_registration_page);

        // Not shown on the webpage. Used to update boot partition
        httpd_uri_t update_ota_page = {
            .uri       = "/update_ota",
            .method    = HTTP_POST,
            .handler   = update_ota_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &update_ota_page);

        ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, &wifi_event_handler_instance));
        ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, &ip_event_handler_instance));
    
        // Task to accept messages from queue and send to SSE clients
        q_sse_message_queue = xQueueCreate( 10, sizeof(char)*LOG_BUF_MAX_LINE_SIZE );
        xTaskCreate(&sse_logging_task, "sse", 2048, NULL, 4, &t_sse_task_handle);

        esp_log_set_vprintf(&sse_logging_vprintf);
        
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Error starting server!");
    return ESP_FAIL;
}

void stop_webserver(void)
{
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler_instance));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, ESP_EVENT_ANY_ID, &ip_event_handler_instance));

    esp_log_set_vprintf(vprintf);

    vTaskDelete(t_sse_task_handle);
    vQueueDelete(q_sse_message_queue);

    // Stop the httpd server
    httpd_stop(server);
    server = NULL;
}


