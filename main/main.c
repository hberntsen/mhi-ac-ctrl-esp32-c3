/* 
    Version History
    ---------------
    1.0.1   20-Apr-2023     modified header check to include 0x6c and 0x6d
    1.0.5   20-Apr-2023     removed old update code from httpd.c
    1.1.0   22-Apr-2023     fixed errors in miso frame creation. 
                            sent message with value to queue.  by the time the queue was read, the characteristic 
                            had been reset/ovrwritten so the setting would not work
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"                                          // For EventGroupHandle_t
#include <string.h>                                                         // strcmp, memset

#include "driver/timer.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"

#include "nvs.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_ota_ops.h"
#include "esp_log.h"
static const char *TAG = "main";

#include "esp_wifi.h"
#include "esp_netif.h"                                                      // Must be included before esp_wifi_default.h
#include "esp_wifi_default.h"                                               // For esp_netif_create_default_wifi_sta

#include "wifi.h"

#include <homekit/homekit.h>
#include <homekit/characteristics.h>
ESP_EVENT_DEFINE_BASE(HOMEKIT_EVENT);                                       // Convert esp-homekit events into esp event system      


#include "mhi.h"

#define vTaskDelayMs(ms)            vTaskDelay((ms)/portTICK_PERIOD_MS)

#define ESP_INTR_FLAG_DEFAULT       0                                           // default to allocating a non-shared interrupt of level 1, 2 or 3.

#define TIMER_DIVIDER               (16)                                        // hardware timer clock divider
#define TIMER_SCALE_MS              (TIMER_BASE_CLK / TIMER_DIVIDER / 1000)     // convert counter value to milliseconds


static homekit_accessory_t *accessories[2];
static TaskHandle_t mhi_poll_task_handle = NULL;
static QueueHandle_t update_setting_queue;

typedef struct {
    const char *type;
    homekit_value_t value;
} homekit_setting_message_t;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
}

static bool IRAM_ATTR timer_group_isr_callback(void *args)
{
    spi_slave_transaction_t *t = (spi_slave_transaction_t *) args;
    spi_slave_queue_trans(RCV_HOST, t, 0);
    return false;
}


static void mhi_poll_task(void *arg)
{
    esp_err_t err;

    char *mosi_packet_homekit_char = malloc(61);

    uint16_t packet_cnt = 0;            // can be removed when diagnostic lines are removed

    uint8_t frame = 0;
    bool halfcycle = false;


    uint16_t rx_checksum = 0;
    uint16_t tx_checksum = 0;
    bool frame_diff = false;

    char *mode;                         // can be removed when diagnostic lines are removed
    char *state;                        // can be removed when diagnostic lines are removed
    uint8_t mhi_fan_speed;
    
    uint8_t active;                     // off, on
    uint8_t target_state;               // auto, heat, cool
    uint8_t current_state;              // inactive, idle, heating, cooling; look at compressor status in DB13
    float current_temp, set_temp;
    float rotation_speed;

    // use WORD_ALIGNED_ATTR when using DMA buffer
    WORD_ALIGNED_ATTR uint8_t recvbuf[20];
    WORD_ALIGNED_ATTR uint8_t sendbuf[20];

    //                        sb0   sb1   sb2   db0   db1   db2   db3   db4   db5   db6   db7   db8   db9  db10  db11  db12  db13  db14  chkH  chkL
    uint8_t miso_frame[] = { 0xA9, 0x00, 0x07, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0x0f, 0x00, 0x00, 0x00 };

    uint8_t mosi_frame[20];
//    uint8_t mosi_frame[] = { 0x6c, 0x80, 0x04, 0x49, 0x02, 0x2c, 0xa5, 0x00, 0x00, 0x88, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0x01, 0x00, 0x00, 0x00 };


    // map to homekit addresses
    homekit_accessory_t *accessory                  = accessories[0];
    homekit_service_t *heater_cooler_service        = homekit_service_by_type(accessory, HOMEKIT_SERVICE_HEATER_COOLER);
    homekit_characteristic_t *current_temp_ch       = homekit_service_characteristic_by_type(heater_cooler_service, HOMEKIT_CHARACTERISTIC_CURRENT_TEMPERATURE);
    homekit_characteristic_t *current_state_ch      = homekit_service_characteristic_by_type(heater_cooler_service, HOMEKIT_CHARACTERISTIC_CURRENT_HEATER_COOLER_STATE);
    homekit_characteristic_t *active_ch             = homekit_service_characteristic_by_type(heater_cooler_service, HOMEKIT_CHARACTERISTIC_ACTIVE);
    homekit_characteristic_t *target_state_ch       = homekit_service_characteristic_by_type(heater_cooler_service, HOMEKIT_CHARACTERISTIC_TARGET_HEATER_COOLER_STATE);
    homekit_characteristic_t *cooling_threshold_ch  = homekit_service_characteristic_by_type(heater_cooler_service, HOMEKIT_CHARACTERISTIC_COOLING_THRESHOLD_TEMPERATURE);
    homekit_characteristic_t *heating_threshold_ch  = homekit_service_characteristic_by_type(heater_cooler_service, HOMEKIT_CHARACTERISTIC_HEATING_THRESHOLD_TEMPERATURE);
    homekit_characteristic_t *rotation_speed_ch     = homekit_service_characteristic_by_type(heater_cooler_service, HOMEKIT_CHARACTERISTIC_ROTATION_SPEED);
    homekit_characteristic_t *custom_mosi_packet_ch = homekit_service_characteristic_by_type(heater_cooler_service, "02B77069-DA5D-493C-829D-F6C5DCFE5C28");
    homekit_characteristic_t *custom_crc_err_ch     = homekit_service_characteristic_by_type(heater_cooler_service, "02B77070-DA5D-493C-829D-F6C5DCFE5C28");
    homekit_characteristic_t *custom_header_err_ch  = homekit_service_characteristic_by_type(heater_cooler_service, "02B77071-DA5D-493C-829D-F6C5DCFE5C28");
   

    // configuration for the SPI bus
    spi_bus_config_t buscfg={
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
//            .flags = SPICOMMON_BUSFLAG_IOMUX_PINS
        .flags = SPICOMMON_BUSFLAG_GPIO_PINS
    };  

    // configuration for the SPI slave interface
    spi_slave_interface_config_t slvcfg={
        .mode = 3,                    //CPOL=1, CPHA=1
        .spics_io_num = -1,
        .queue_size = 3,
        .flags = SPI_SLAVE_BIT_LSBFIRST
    }; 

    spi_slave_transaction_t spi_slave_trans;
    spi_slave_transaction_t* spi_slave_trans_out;       // needed for spi_slave_get_trans_result which needs a pointer to a pointer

    // initialize SPI slave interface
    err = spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);    // can't disable DMA. no comms if you do...

    ESP_ERROR_CHECK(err);

    
    // Select and initialize basic parameters of the timer
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = TIMER_AUTORELOAD_DIS,
    }; // default clock source is APB
    timer_init(TIMER_GROUP_0, TIMER_0, &config);

    // Configure the alarm value (in milliseconds) and the interrupt on alarm. there is a delay between each frame of 40ms. 
    //  so we set the alarm to 20ms. once the alarm triggers, spi_slave_queue_trans is called which will get the data from the next spi packet
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 20 * TIMER_SCALE_MS);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, timer_group_isr_callback, &spi_slave_trans, 0);



    gpio_config_t io_conf = {};
    io_conf.mode = GPIO_MODE_INPUT;  
    io_conf.pin_bit_mask = (1ULL<<GPIO_SCLK); 
    io_conf.pull_down_en = 0;     
    io_conf.pull_up_en = 1;                                     // required to prevent abort() caused by floating pin when daughtboard not connected
    io_conf.intr_type = GPIO_INTR_LOW_LEVEL;                    // when this is set to NEGEDGE, DMA sometimes doesn't read the last 4 bytes
                                                                // if not connected to AC (plugged in) when starting, it will crash - probably because it 
                                                                // immediately calls an interrupt
    gpio_config(&io_conf); 

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_SCLK, gpio_isr_handler, NULL);
    gpio_intr_disable(GPIO_SCLK);


    while(1) {
        frame_diff = false;
        rx_checksum = 0;
        packet_cnt++;

        //Clear receive buffer
        memset(recvbuf, 0xAA, 20);

        if (frame++ >= MHI_NUM_FRAMES_PER_INTERVAL) {
            halfcycle = !halfcycle;                     // toggle
            miso_frame[DB14] = halfcycle << 2;          // MISO_frame[DB14] bit2 toggles periodically (about every 20 frames)
            frame = 1;                                  // 2 * MHI_NUM_FRAMES_PER_INTERVAL make a complete cycle. for half that, 
                                                        // MISO_frame[DB14] bit2 is 0, and the other half it is 1. When it is set to 1
                                                        // the MISO can be set with any new settings

            if (halfcycle) {
                // to set a setting, the same bits are set in the MISO frame that they are located in the MOSI frame. you also need to 
                // set a specific 'set bit'. this bit stays set (even in the MOSI frame) until the RC is used. at this point, _all_ set 
                // bits are reset to 0. if the RC is used, the updated setting will appear in the MOSI frame, which is updated every frame.
                // once every ~40 frames (2 * MHI_NUM_FRAMES_PER_INTERVAL), the MISO frame is set (in this loop) with any settings that have
                // changed in homekit

                miso_frame[DB0] = 0x00;
                miso_frame[DB1] = 0x00;
                miso_frame[DB2] = 0x00;
 
                // cycle through all queued messages and create the miso frame
                //  the message is simply the pointer to the characteristic that has requested a change

                homekit_setting_message_t message_ch;

                while (xQueueReceive(update_setting_queue, (void *)&message_ch, 0) == pdTRUE) {

                    if (strcmp(message_ch.type, HOMEKIT_CHARACTERISTIC_ACTIVE) == 0) {
                        miso_frame[DB0] &= ~PWR_MASK;                       // clear what is there first, in case multiple messages arrive quickly
                        miso_frame[DB0] |= message_ch.value.uint8_value;   // set the setting
                        miso_frame[DB0] |= 1 << 1;                          // DB0[1] 'set setting bit' 
                    }


                    if (strcmp(message_ch.type, HOMEKIT_CHARACTERISTIC_TARGET_HEATER_COOLER_STATE) == 0) {
                        miso_frame[DB0] &= ~MODE_MASK;                      // clear what is there first, in case multiple messages arrive quickly
                        switch (message_ch.value.uint8_value)
                        {
                            case 0: //auto
                                miso_frame[DB0] |= MODE_AUTO;
                                // no need to set temp, as when changing to auto, the heating and cooling thresholds are set either side of the
                                //  current temperature set point
                                break;
                            case 1: //heat
                                miso_frame[DB0] |= MODE_HEAT;
                                miso_frame[DB2]  = (int)(heating_threshold_ch->value.float_value * 2.0);
                                break; 
                            case 2: //cool
                                miso_frame[DB0] |= MODE_COOL;
                                miso_frame[DB2]  = (int)(cooling_threshold_ch->value.float_value * 2.0);
                                break;
                        }
                        miso_frame[DB0] |= 1 << 5;      // set bit for mode
                        miso_frame[DB2] |= 1 << 7;      // set bit for temp
                    }


                    if ( strcmp(message_ch.type, HOMEKIT_CHARACTERISTIC_COOLING_THRESHOLD_TEMPERATURE) == 0 || 
                         strcmp(message_ch.type, HOMEKIT_CHARACTERISTIC_HEATING_THRESHOLD_TEMPERATURE) == 0
                    ) {
                        float new_set_temp;
                        // in auto mode, both cooling (upper number) and heating (lower number) messages are sent
                        //  setter_ex ensures only one is sent
                        if (target_state_ch->value.uint8_value == 0) {
                            // get mid point of heating/cooling thresholds
                            new_set_temp = (cooling_threshold_ch->value.float_value + heating_threshold_ch->value.float_value)/2.0;
                        }
                        // if not auto mode, then message_ch will have the value that was changed
                        else {
                            new_set_temp = (message_ch.value.float_value);
                        }
                        miso_frame[DB2] =  (int)( new_set_temp * 2.0 );
                        miso_frame[DB2] |= 1 << 7;
                    }


                    if ( strcmp(message_ch.type, HOMEKIT_CHARACTERISTIC_ROTATION_SPEED) == 0 &&
                           message_ch.value.float_value != 0.0 
                    ) {
                        miso_frame[DB1] &= ~FAN_DB1_MASK;                      
                        miso_frame[DB6] &= ~FAN_DB6_MASK;                      // clear what is there first, in case multiple messages arrive quickly

                        switch ( (int)(message_ch.value.float_value/25) )
                        {
                            case 1:         // 25.0, fan speed 1
                                miso_frame[DB1] |= FAN_SPEED_1;
                                break;
                            case 2:         // 50.0, fan speed 2
                                miso_frame[DB1] |= FAN_SPEED_2;
                                break;
                            case 3:         // 75.0, fan speed 3
                                miso_frame[DB1] |= FAN_SPEED_3;
                                break;      
                            case 4:         // 100.0, fan speed 4
                                miso_frame[DB1] |= FAN_SPEED_3;
                                miso_frame[DB6] |= FAN_SPEED_4;
                                break;  
                            default:
                                // do nothing. 
                                break;
                        }

                        miso_frame[DB1] |= 1 << 3;              // set bit for fan speed on DB1
                    } 
                } 
            } 

            tx_checksum = 0;
            for (uint8_t byte_cnt = 0; byte_cnt < 20; byte_cnt++) { 
                // calculate checksum
                tx_checksum += miso_frame[byte_cnt];

                // copy bytes to sendbuf
                sendbuf[byte_cnt] = miso_frame[byte_cnt];
            }

            sendbuf[18] = tx_checksum>>8;
            sendbuf[19] = tx_checksum;

        }

        // reset timer and enable alarm (alarm disables after it has triggered once)
        timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
        timer_set_alarm(TIMER_GROUP_0, TIMER_0, TIMER_ALARM_EN);
        timer_start(TIMER_GROUP_0, TIMER_0);

        // enable the interrupt which will reset the timer counter to 0 every time it goes LOW
        gpio_intr_enable(GPIO_SCLK);
        
        //Set up a transaction of 20 bytes to send/receive
        spi_slave_trans.length = 20*8;

        spi_slave_trans.tx_buffer = sendbuf;
        spi_slave_trans.rx_buffer = recvbuf;
        
        // blocking function waiting for the spi results. the hardware timer must reach 20ms and perform an spi transaction
        //  we can get the data directly from 'spi_slave_trans' instead of 'spi_slave_trans_out->'
        spi_slave_get_trans_result(RCV_HOST, &spi_slave_trans_out, portMAX_DELAY);

        // timer and gpio interrupt are not required until next cycle
        timer_pause(TIMER_GROUP_0, TIMER_0);
        gpio_intr_disable(GPIO_SCLK);


        for (uint8_t byte_cnt = 0; byte_cnt < 20; byte_cnt++) { 
            // calculate checksum
            if (byte_cnt < 18){
                rx_checksum += recvbuf[byte_cnt];
            }

            // check if any bytes have changed
            if (mosi_frame[byte_cnt] != recvbuf[byte_cnt]) {
                mosi_frame[byte_cnt] = recvbuf[byte_cnt];
                frame_diff = true;
            }
        }
        
        bool frame_error = false;

        // check for errors. first byte should be 0x6c
        if ( ((mosi_frame[SB0] & 0xfe) != 0x6c) | (mosi_frame[SB1] != 0x80) | (mosi_frame[SB2] != 0x04) ) {
            ESP_LOGD("mhi", "packet: %5d wrong MOSI signature. 0x%02x 0x%02x 0x%02x", 
                        packet_cnt, mosi_frame[0], mosi_frame[1], mosi_frame[2]);

            frame_error = true;

            custom_header_err_ch->value.uint64_value++;
            homekit_characteristic_notify(custom_header_err_ch, custom_header_err_ch->value);

        } else if ( (mosi_frame[CBH] != (rx_checksum>>8 & 0xff)) | (mosi_frame[CBL] != (rx_checksum & 0xff)) ) {
            ESP_LOGD("mhi", "packet: %5d wrong MOSI checksum. calculated 0x%04x. MOSI[18]:0x%02x MOSI[19]:0x%02x", 
                        packet_cnt, rx_checksum, mosi_frame[18], mosi_frame[19]);

            frame_error = true;
            
            custom_crc_err_ch->value.uint64_value++;
            homekit_characteristic_notify(custom_crc_err_ch, custom_crc_err_ch->value);
        }
        
        if (frame_error) {
            // send packet to homekit custom characteristic
            snprintf(mosi_packet_homekit_char, 61, "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
                mosi_frame[0],  mosi_frame[1],  mosi_frame[2],  mosi_frame[3],  mosi_frame[4],  mosi_frame[5],  mosi_frame[6],  mosi_frame[7],  mosi_frame[8],  mosi_frame[9], 
                mosi_frame[10], mosi_frame[11], mosi_frame[12], mosi_frame[13], mosi_frame[14], mosi_frame[15], mosi_frame[16], mosi_frame[17], mosi_frame[18], mosi_frame[19]
                );

            custom_mosi_packet_ch->value = HOMEKIT_STRING(mosi_packet_homekit_char);
            homekit_characteristic_notify(custom_mosi_packet_ch, custom_mosi_packet_ch->value);

            // do not update homekit
            frame_diff = false;

            // wait a second before retrying communication
            vTaskDelayMs(1000);
        }


        
        // ********************** Diagnostics ************************
/*        
        if (frame == 1) {
            printf("miso packet: %5d    %02x %02x %02x   %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x   %02x %02x (calc %d)  tx_checksum: %d \n", 
                packet_cnt, 
            sendbuf[0],  sendbuf[1],  sendbuf[2],  sendbuf[3],  sendbuf[4],  sendbuf[5],  sendbuf[6],  sendbuf[7],  sendbuf[8],  sendbuf[9], 
            sendbuf[10], sendbuf[11], sendbuf[12], sendbuf[13], sendbuf[14], sendbuf[15], sendbuf[16], sendbuf[17], sendbuf[18], sendbuf[19], 
                (sendbuf[18]<<8)+(sendbuf[19]), 
                tx_checksum
            );

            printf("mosi packet: %5d    %02x %02x %02x   %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x   %02x %02x (calc %d)  rx_checksum: %d \n", 
                    packet_cnt, 
                mosi_frame[0],  mosi_frame[1],  mosi_frame[2],  mosi_frame[3],  mosi_frame[4],  mosi_frame[5],  mosi_frame[6],  mosi_frame[7],  mosi_frame[8],  mosi_frame[9], 
                mosi_frame[10], mosi_frame[11], mosi_frame[12], mosi_frame[13], mosi_frame[14], mosi_frame[15], mosi_frame[16], mosi_frame[17], mosi_frame[18], mosi_frame[19], 
                    (mosi_frame[18]<<8)+(mosi_frame[19]), 
                    rx_checksum
                );
        }

        if (frame == 2) {
            printf("mosi packet: %5d    %02x %02x %02x   %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x   %02x %02x (calc %d)  rx_checksum: %d \n", 
                    packet_cnt, 
                mosi_frame[0],  mosi_frame[1],  mosi_frame[2],  mosi_frame[3],  mosi_frame[4],  mosi_frame[5],  mosi_frame[6],  mosi_frame[7],  mosi_frame[8],  mosi_frame[9], 
                mosi_frame[10], mosi_frame[11], mosi_frame[12], mosi_frame[13], mosi_frame[14], mosi_frame[15], mosi_frame[16], mosi_frame[17], mosi_frame[18], mosi_frame[19], 
                    (mosi_frame[18]<<8)+(mosi_frame[19]), 
                    rx_checksum
                );

            printf("\n");
        }
*/        
        // ***********************************************************


        // only need to perform updates if there was a change (with no error) since last frame read
        if (frame_diff) {
            switch (mosi_frame[DB0] & MODE_MASK)
            {
                case MODE_AUTO:
                    target_state = 0;
                    mode = "auto";
                    break;
                case MODE_HEAT:
                    target_state = 1;
                    mode = "heat";
                    break; 
                case MODE_COOL:
                    target_state = 2;
                    mode = "cool";
                    break;

                case MODE_DRY:
                    target_state = 0;
                    mode = "dry";
                    break;
                case MODE_FAN:
                    target_state = 0;
                    mode = "fan";
                    break;         
                default:
                    target_state = 0;
                    mode = "unknown";
                    break;
            }

            if (target_state_ch->value.uint8_value != target_state) {
                target_state_ch->value = HOMEKIT_UINT8(target_state);
                homekit_characteristic_notify(target_state_ch, target_state_ch->value);
            }


            // this is the power state and the compressor state
            if (!(mosi_frame[DB0] & PWR_MASK)) {
                active = 0;
                current_state = 0;
                state = "inactive";
            } else if (!(mosi_frame[DB13] & COMP_ACTIVE_MASK)) {
                active = 1;
                current_state = 1;
                state = "idle";
            } else if (mosi_frame[DB13] & HEAT_COOL_MASK) {
                active = 1;
                current_state = 2;
                state = "heating";
            } else {
                active = 1;
                current_state = 3;
                state = "cooling";
            }

            if (active_ch->value.uint8_value != active) {
                active_ch->value = HOMEKIT_UINT8(active);
                homekit_characteristic_notify(active_ch, active_ch->value);
            }

            if (current_state_ch->value.uint8_value != current_state) {
                current_state_ch->value = HOMEKIT_UINT8(current_state);
                homekit_characteristic_notify(current_state_ch, current_state_ch->value);
            }


            current_temp = ((int)mosi_frame[DB3] - 61) / 4.0;

            if (current_temp_ch->value.float_value != current_temp) {
                current_temp_ch->value = HOMEKIT_FLOAT(current_temp);
                homekit_characteristic_notify(current_temp_ch, current_temp_ch->value);
            }


            // this needs to behave differently based on mode
            // save heating and cooling threshold separately
            // if auto, set cool and heat threshold around mid-point (determined by TEMP_THRESHOLD_DIFF)
            set_temp = (int)(mosi_frame[DB2] & 0x7F) / 2.0;

            if ( target_state == 2 && cooling_threshold_ch->value.float_value != set_temp ) {
                cooling_threshold_ch->value = HOMEKIT_FLOAT(set_temp);
                homekit_characteristic_notify(cooling_threshold_ch, cooling_threshold_ch->value);
            }
            if ( target_state == 1 && heating_threshold_ch->value.float_value != set_temp ) {
                heating_threshold_ch->value = HOMEKIT_FLOAT(set_temp);
                homekit_characteristic_notify(heating_threshold_ch, heating_threshold_ch->value);
            } 
            // if in auto mode, the temp setpoint should be between the heating and cooling thresholds
            if ( (target_state == 0) && ( (heating_threshold_ch->value.float_value + cooling_threshold_ch->value.float_value)/2.0 != set_temp) ) {
                heating_threshold_ch->value = HOMEKIT_FLOAT(set_temp - (TEMP_THRESHOLD_DIFF/2) );
                homekit_characteristic_notify(heating_threshold_ch, heating_threshold_ch->value);

                cooling_threshold_ch->value = HOMEKIT_FLOAT(set_temp + (TEMP_THRESHOLD_DIFF/2) );
                homekit_characteristic_notify(cooling_threshold_ch, cooling_threshold_ch->value);

                ESP_LOGD("mhi", "update set temp: %2.2f, cool_th: %2.2f, heat_th: %2.2f", 
                    set_temp, cooling_threshold_ch->value.float_value, heating_threshold_ch->value.float_value);

            }


            // note that, in homekit, rotation_speed = 0.0 will set active = 0 (turn the AC off)
            if (mosi_frame[DB6] & FAN_DB6_MASK) {
                mhi_fan_speed = 4;
                rotation_speed = 100.0;
            } 
            else {
                switch (mosi_frame[DB1] & FAN_DB1_MASK)
                {
                    case FAN_SPEED_1:
                        mhi_fan_speed = 1;
                        rotation_speed = 25.0;
                        break;
                    case FAN_SPEED_2:
                        mhi_fan_speed = 2;
                        rotation_speed = 50.0;
                        break;
                    case FAN_SPEED_3:
                        mhi_fan_speed = 3;
                        rotation_speed = 75.0;
                        break;      
                    default:
                        mhi_fan_speed = 0;
                        rotation_speed = 0.0;
                        break;
                }
            }

            if (rotation_speed_ch->value.float_value != rotation_speed) {
                rotation_speed_ch->value = HOMEKIT_FLOAT(rotation_speed);
                homekit_characteristic_notify(rotation_speed_ch, rotation_speed_ch->value);
            }

            // send packet to homekit custom characteristic
            snprintf(mosi_packet_homekit_char, 61, "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
                mosi_frame[0],  mosi_frame[1],  mosi_frame[2],  mosi_frame[3],  mosi_frame[4],  mosi_frame[5],  mosi_frame[6],  mosi_frame[7],  mosi_frame[8],  mosi_frame[9], 
                mosi_frame[10], mosi_frame[11], mosi_frame[12], mosi_frame[13], mosi_frame[14], mosi_frame[15], mosi_frame[16], mosi_frame[17], mosi_frame[18], mosi_frame[19]
                );

            custom_mosi_packet_ch->value = HOMEKIT_STRING(mosi_packet_homekit_char);
            homekit_characteristic_notify(custom_mosi_packet_ch, custom_mosi_packet_ch->value);


            // ********************** Diagnostics ************************

            ESP_LOGD("mhi", "packet: %5d    %02x %02x %02x   %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x   %02x %02x (calc %d)  rx_checksum: %d", 
                    packet_cnt, 
                mosi_frame[0],  mosi_frame[1],  mosi_frame[2],  mosi_frame[3],  mosi_frame[4],  mosi_frame[5],  mosi_frame[6],  mosi_frame[7],  mosi_frame[8],  mosi_frame[9], 
                mosi_frame[10], mosi_frame[11], mosi_frame[12], mosi_frame[13], mosi_frame[14], mosi_frame[15], mosi_frame[16], mosi_frame[17], mosi_frame[18], mosi_frame[19], 
                    (mosi_frame[18]<<8)+(mosi_frame[19]), 
                    rx_checksum
                );

            ESP_LOGD("mhi", "            power: %3s  mode: %5s  temp: %2.2f  set_temp: %2.2f  mhi_fan_speed: %d  state: %8s", 
                (mosi_frame[DB0] & PWR_MASK) ? "on" : "off", 
                mode, 
                current_temp, 
                set_temp,
                mhi_fan_speed,
                state
            );

            // ***********************************************************

        }
    }
}

void homekit_identify(homekit_value_t _value) {
    // no action
    // could turn on and off AC?
}

// use a switch in Eve to start and stop AP
void setter_ex_start_ap(homekit_characteristic_t *_ch, homekit_value_t value) {
    if (value.bool_value) {
        start_ap_prov();
    }
    else {
        stop_ap_prov();
    }
}

void setter_ex_update_settings(homekit_characteristic_t *_ch, homekit_value_t value) {

    // ESP_LOGW(TAG, "setter_ex: %s", _ch->description);

    homekit_accessory_t *accessory                  = accessories[0];
    homekit_service_t *heater_cooler_service        = homekit_service_by_type(accessory, HOMEKIT_SERVICE_HEATER_COOLER);
    homekit_characteristic_t *cooling_threshold_ch  = homekit_service_characteristic_by_type(heater_cooler_service, HOMEKIT_CHARACTERISTIC_COOLING_THRESHOLD_TEMPERATURE);
    homekit_characteristic_t *heating_threshold_ch  = homekit_service_characteristic_by_type(heater_cooler_service, HOMEKIT_CHARACTERISTIC_HEATING_THRESHOLD_TEMPERATURE);
    homekit_characteristic_t *target_state_ch       = homekit_service_characteristic_by_type(heater_cooler_service, HOMEKIT_CHARACTERISTIC_TARGET_HEATER_COOLER_STATE);

    static enum state_check {
        begin,
        check_next,
        complete
    } auto_threshold_set_state;

    homekit_setting_message_t update_setting_message;

    // if this is a target state change, and it is going to auto mode, then update heating and cooling thresholds 
    //  immediately for better user experience
    if ( strcmp(_ch->type, HOMEKIT_CHARACTERISTIC_TARGET_HEATER_COOLER_STATE) == 0  && 
         _ch->value.uint8_value != 0 && value.uint8_value == 0 )        
    {
        // get current target temp by looking at previous mode
        float temp_threshold = 0;
        if ( _ch->value.uint8_value == 1 ) {            // it was heat
            temp_threshold = heating_threshold_ch->value.float_value;
        }
        else if ( _ch->value.uint8_value == 2 ) {       // it was cool
            temp_threshold = cooling_threshold_ch->value.float_value;
        }

        // now set thresholds to either side of the previous mode/threshold temperature
        heating_threshold_ch->value = HOMEKIT_FLOAT(temp_threshold - TEMP_THRESHOLD_DIFF/2);
        cooling_threshold_ch->value = HOMEKIT_FLOAT(temp_threshold + TEMP_THRESHOLD_DIFF/2);
        homekit_characteristic_notify(heating_threshold_ch, heating_threshold_ch->value);
        homekit_characteristic_notify(cooling_threshold_ch, cooling_threshold_ch->value);

        ESP_LOGD("mhi", "setter_ex   temp_threshold: %2.2f, cool_th: %2.2f, heat_th: %2.2f", 
            temp_threshold, cooling_threshold_ch->value.float_value, heating_threshold_ch->value.float_value);
    }


    // in auto mode, if a threshold value is changed, immediately update the other one for better user experience 
    if ( target_state_ch->value.uint8_value == 0 ) {
        if ( strcmp(_ch->type, HOMEKIT_CHARACTERISTIC_HEATING_THRESHOLD_TEMPERATURE) == 0 ) {
                // if state is 'begin' or 'check_next'
                if (auto_threshold_set_state != complete ) {
                    // if it is begin, then heating_threshold is being checked before cooling_threshold. check if the heating_threshold has changed. 
                    // if it is 'check_next', cooling_threshold did not change, so heating_threshold must have changed
                    if ( _ch->value.float_value != value.float_value || auto_threshold_set_state == check_next ) {
                        cooling_threshold_ch->value = HOMEKIT_FLOAT(value.float_value + TEMP_THRESHOLD_DIFF);
                        homekit_characteristic_notify(cooling_threshold_ch, cooling_threshold_ch->value);

                        ESP_LOGD("mhi", "setter_ex    old_heat_th: %2.2f, new_heat_th: %2.2f, cool_th: %2.2f", 
                                _ch->value.float_value, value.float_value, cooling_threshold_ch->value.float_value);

                        _ch->value = value;

                        update_setting_message.type = _ch->type;
                        update_setting_message.value = _ch->value;

                        xQueueSendToBack(update_setting_queue, (void *)&update_setting_message, 0);

                        // if it is 'check_next', cooling_threshold was checked already, so reset back to begin
                        if ( auto_threshold_set_state == check_next ) {
                            auto_threshold_set_state = begin;
                        // else it is currently 'begin', so we need to flag as complete for cooling_threshold to skip checks
                        } else { 
                            auto_threshold_set_state = complete;
                        }
                    }
                    else {
                        auto_threshold_set_state = check_next;
                    }
                }
                else  {
                    auto_threshold_set_state = begin;
                }
                return;

        }
        else if ( strcmp(_ch->type, HOMEKIT_CHARACTERISTIC_COOLING_THRESHOLD_TEMPERATURE) == 0 ) {
                // if state is 'begin' or 'check_next'
                if (auto_threshold_set_state != complete ) {
                    // if it is begin, then cooling_threshold is being checked before heating_threshold. check if the cooling_threshold has changed. 
                    // if it is 'check_next', heating_threshold did not change, so cooling_threshold must have changed
                    if ( _ch->value.float_value != value.float_value || auto_threshold_set_state == check_next ) {

                        heating_threshold_ch->value = HOMEKIT_FLOAT(value.float_value - TEMP_THRESHOLD_DIFF);
                        homekit_characteristic_notify(heating_threshold_ch, heating_threshold_ch->value);

                        ESP_LOGD("mhi", "setter_ex    old_cool_th: %2.2f, new_cool_th: %2.2f, heat_th: %2.2f", 
                                _ch->value.float_value, value.float_value, heating_threshold_ch->value.float_value);

                        _ch->value = value;

                        update_setting_message.type = _ch->type;
                        update_setting_message.value = _ch->value;

                        xQueueSendToBack(update_setting_queue, (void *)&update_setting_message, 0);

                        // if it is 'check_next', cooling_threshold was checked already, so reset back to begin
                        if ( auto_threshold_set_state == check_next ) {
                            auto_threshold_set_state = begin;
                        // else it is currently 'begin', so we need to flag as complete for cooling_threshold to skip checks
                        } else { 
                            auto_threshold_set_state = complete;
                        }
                    }
                    else {
                        auto_threshold_set_state = check_next;
                    }
                }
                else  {
                    auto_threshold_set_state = begin;
                }
                return;
        }
    }

    // we are overriding the setter_ex function, so we must manually store value on the characteristic itself.
    //  this could be stored in any variable, but you would then need a function for getter_ex to retrieve that variable
    _ch->value = value;

    // This is not required. Tested with iPhone and iPad open. They are both updated instantly.
    //homekit_characteristic_notify(_ch, value);

    update_setting_message.type = _ch->type;
    update_setting_message.value = _ch->value;

    xQueueSendToBack(update_setting_queue, (void *)&update_setting_message, 0);

}


/* Event handler for Events */
static void main_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == HOMEKIT_EVENT) {
        if (event_id == HOMEKIT_EVENT_CLIENT_CONNECTED) {
            ESP_LOGI(TAG, "HOMEKIT_EVENT_CLIENT_CONNECTED");
        }
        else if (event_id == HOMEKIT_EVENT_CLIENT_DISCONNECTED) {
            ESP_LOGI(TAG, "HOMEKIT_EVENT_CLIENT_DISCONNECTED");
        }
        else if (event_id == HOMEKIT_EVENT_PAIRING_ADDED || event_id == HOMEKIT_EVENT_PAIRING_REMOVED) {
            ESP_LOGI(TAG, "HOMEKIT_EVENT_PAIRING_ADDED or HOMEKIT_EVENT_PAIRING_REMOVED");
        }
    } 
}

void homekit_on_event(homekit_event_t event) {
    esp_event_post(HOMEKIT_EVENT, event, NULL, sizeof(NULL), 10);
}

homekit_server_config_t config = {
    .accessories = accessories,
    .password = "111-11-111",
    .on_event = homekit_on_event,
};

void init_accessory() {
    uint8_t macaddr[6];
    esp_read_mac(macaddr, ESP_MAC_WIFI_SOFTAP);
    int name_len = snprintf( NULL, 0, "esp-%02x%02x%02x", macaddr[3], macaddr[4], macaddr[5] );
    char *name_value = malloc(name_len + 1);
    snprintf( name_value, name_len + 1, "esp-%02x%02x%02x", macaddr[3], macaddr[4], macaddr[5] ); 

    int conf_name_len = snprintf(NULL, 0, "no packets received");
    char *conf_name_val = malloc(conf_name_len + 1);
    snprintf(conf_name_val, conf_name_len + 1, "no packets received");


    esp_app_desc_t app_desc;
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_get_partition_description(running, &app_desc);

    // ACCESSORY_INFORMATION, HEATER_COOLER, and NULL
    homekit_service_t* services[3]; 
    homekit_service_t** s = services;

    *(s++) = NEW_HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]) {
        NEW_HOMEKIT_CHARACTERISTIC(NAME, name_value),
        NEW_HOMEKIT_CHARACTERISTIC(MANUFACTURER, "Riksman"),
        NEW_HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, name_value),
        NEW_HOMEKIT_CHARACTERISTIC(MODEL, "ESP32-Pico-Kit"),
        NEW_HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, app_desc.version),
        NEW_HOMEKIT_CHARACTERISTIC(IDENTIFY, homekit_identify),
        NULL
    });

    *(s++) = NEW_HOMEKIT_SERVICE(HEATER_COOLER, .primary=true, .characteristics=(homekit_characteristic_t*[]) {
        NEW_HOMEKIT_CHARACTERISTIC(
            CURRENT_TEMPERATURE, 20.25,
            .min_step = (float[]) {0.25}
        ),
        NEW_HOMEKIT_CHARACTERISTIC(
            CURRENT_HEATER_COOLER_STATE, 0
        ),
        NEW_HOMEKIT_CHARACTERISTIC(
            TEMPERATURE_DISPLAY_UNITS, 0
        ),
        NEW_HOMEKIT_CHARACTERISTIC(
            ACTIVE, false,
            .setter_ex = setter_ex_update_settings,
        ),
        NEW_HOMEKIT_CHARACTERISTIC(
            TARGET_HEATER_COOLER_STATE, 2,
            .setter_ex = setter_ex_update_settings,
        ),
        NEW_HOMEKIT_CHARACTERISTIC(
            ROTATION_SPEED, 25.0,               // Map this to {"1", "2", "3", "4"} 
            .setter_ex = setter_ex_update_settings,   
            .min_step = (float[]) {25.0},       // 0 automatically turns the AC off in HomeKit, and setting the fan on turns AC on
            .value = HOMEKIT_FLOAT_(25.0)       // .value is missing from the #define in characteristic.h
        ),
        NEW_HOMEKIT_CHARACTERISTIC(
            COOLING_THRESHOLD_TEMPERATURE, 24,
            .setter_ex = setter_ex_update_settings,
            .min_value = (float[]) {0},
            .max_value = (float[]) {35},
            .min_step = (float[]) {0.5}
        ),
        NEW_HOMEKIT_CHARACTERISTIC(
            HEATING_THRESHOLD_TEMPERATURE, 22,
            .setter_ex = setter_ex_update_settings,
            .min_value = (float[]) {0},
            .max_value = (float[]) {35},
            .min_step = (float[]) {0.5}
        ),
        NEW_HOMEKIT_CHARACTERISTIC(
            CUSTOM,
            .setter_ex = setter_ex_start_ap,
            .type = "02B77068-DA5D-493C-829D-F6C5DCFE5C28",
            .description = "Start AP",
            .format = homekit_format_bool,
            .permissions = homekit_permissions_paired_read
                         | homekit_permissions_paired_write
        ),
        NEW_HOMEKIT_CHARACTERISTIC(
            CUSTOM,
            .type = "02B77069-DA5D-493C-829D-F6C5DCFE5C28",
            .description = "MOSI",
            .format = homekit_format_string,
            .permissions = homekit_permissions_paired_read |
                           homekit_permissions_notify,
            .value = HOMEKIT_STRING_(conf_name_val),
        ),
        NEW_HOMEKIT_CHARACTERISTIC(
            CUSTOM,
            .type = "02B77070-DA5D-493C-829D-F6C5DCFE5C28",
            .description = "CRC Errors",
            .format = homekit_format_uint64,
            .permissions = homekit_permissions_paired_read |
                           homekit_permissions_notify,
            .value = HOMEKIT_UINT64_(0),
        ),
        NEW_HOMEKIT_CHARACTERISTIC(
            CUSTOM,
            .type = "02B77071-DA5D-493C-829D-F6C5DCFE5C28",
            .description = "Header Errors",
            .format = homekit_format_uint64,
            .permissions = homekit_permissions_paired_read |
                           homekit_permissions_notify,
            .value = HOMEKIT_UINT64_(0),
        ),
        NULL
    });


    *(s++) = NULL;

    accessories[0] = NEW_HOMEKIT_ACCESSORY(.category=homekit_accessory_category_air_conditioner, .services=services);
    accessories[1] = NULL;

}


void app_main(void)
{
    esp_err_t err;

    esp_log_level_set("spi", ESP_LOG_DEBUG);      
    esp_log_level_set("mhi", ESP_LOG_DEBUG);      


    // Initialize NVS. 
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {     // can happen if truncated/partition size changed
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(esp_event_loop_create_default());

   // esp_event_handler_register is being deprecated
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, main_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID, main_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(HOMEKIT_EVENT, ESP_EVENT_ANY_ID, main_event_handler, NULL, NULL));

    my_wifi_init();

    init_accessory();
    homekit_server_init(&config);

    update_setting_queue = xQueueCreate( 10, sizeof(homekit_setting_message_t) );

    xTaskCreatePinnedToCore(
                        mhi_poll_task,          // Function to implement the task 
                        "mhi_task",             // Name of the task 
                        4096,                   // Stack size in words 
                        NULL,                   // Task input parameter
                        10,                     // Priority of the task
                        &mhi_poll_task_handle,  // Task handle. 
                        1);                     // Core where the task should run 

    /*
    char buffer[600];
    vTaskList(buffer);
    ESP_LOGI(TAG, "\n%s", buffer);
    */

    vTaskDelay(pdMS_TO_TICKS(50));

    // App rollback ensures everything starts OK and sets the image valid. Otherwise, on the next reboot, it will rollback.
    esp_ota_mark_app_valid_cancel_rollback();



}
