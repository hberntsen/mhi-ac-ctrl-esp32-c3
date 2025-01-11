// MHI-AC-Ctrol-core
// implements the core functions (read & write SPI)

#include "MHI-AC-Ctrl-core.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

#include "driver/gptimer.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"

#include "esp_log.h"
static const char *TAG = "MHI-AC-CTRL-core";

using namespace mhi_ac;
using namespace mhi_ac::internal;


#define vTaskDelayMs(ms)            vTaskDelay((ms)/portTICK_PERIOD_MS)

#define ESP_INTR_FLAG_DEFAULT       0                                           // default to allocating a non-shared interrupt of level 1, 2 or 3.

gptimer_handle_t cs_timer = NULL;
static TaskHandle_t mhi_poll_task_handle = NULL;
//                              sb0                           sb1   sb2   db0   db1   db2   db3   db4   db5   db6   db7
static uint8_t miso_frame[] = { USE_LONG_FRAME ? 0xAA : 0xA9, 0x00, 0x07, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00,
//                              db8   db9   db10  db11  db12  db13  db14  chkH  chkL  db15  db16  db17  db18  db19  db20
                                0x00, 0x00, 0xff, 0xff, 0xff, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//                              db21  db22  db23  db24  db25  db26 (chk2L not needed)
                                0x00, 0x00, 0xff, 0xff, 0xff, 0xff };
static SemaphoreHandle_t miso_semaphore_handle;
static StaticSemaphore_t miso_semaphore_buffer;

static int ready = 0;
static bool active_mode = false;
static gpio_num_t gpio_cs_out;

static SemaphoreHandle_t snapshot_semaphore_handle;
static StaticSemaphore_t snapshot_semaphore_buffer;
static uint8_t mosi_frame_snapshot[DB14];
static uint8_t mosi_frame_snapshot_prev[DB14];
static uint32_t frame_errors = 0;

namespace mhi_ac {
MHIEnergy mhi_energy(230);


static void IRAM_ATTR gpio_isr_handler(void* arg)
{
  uint64_t current_timer_value;
  // We need to know whether the timer has already been started. If we start it when it is already started, we get the
  // following in the logs:
  // > gptimer: gptimer_start(348): timer is not enabled in the logs
  //
  // There is no direct API to check the status, so use the raw count instead
  ESP_ERROR_CHECK(gptimer_get_raw_count(cs_timer, &current_timer_value));
  if(current_timer_value == 0) {
    gptimer_start(cs_timer);
  }
}

static bool IRAM_ATTR gptimer_isr_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    // Stop timer, gpio isr will turn it back on
    gptimer_stop(timer);
    // Set to 0 to tell the GPIO isr that the timer is not running
    gptimer_set_raw_count(cs_timer, 0);

    // Trigger Chip Select
    gpio_set_level(gpio_cs_out, 1);
    if(ready) {
        spi_slave_transaction_t *t = (spi_slave_transaction_t *) user_ctx;
        spi_slave_queue_trans(RCV_HOST, t, 0);
    }
    gpio_set_level(gpio_cs_out, 0);
    return false;
}

bool mhi_ac_ctrl_core_snapshot(uint32_t wait_time_ms) {
    xSemaphoreTake(snapshot_semaphore_handle, 0);
    return xSemaphoreTake(snapshot_semaphore_handle, pdMS_TO_TICKS(wait_time_ms));
}

void mhi_ac_ctrl_core_active_mode_set(bool state) {
    active_mode = state;
    if(!active_mode) {
        // Ensure we stop counting power when active mode is off
        mhi_energy.set_current(0);
    }
}

bool mhi_ac_ctrl_core_active_mode_get() {
    return active_mode;
}

#define CLAMP(x, lower, upper) (MIN(upper, MAX(x, lower)))

void mhi_ac_ctrl_core_target_temperature_set(float target_temperature) {
    xSemaphoreTake(miso_semaphore_handle, portMAX_DELAY);
    //uint8_t tsetpoint = (uint8_t)roundf(CLAMP(target_temperature*2, 18.f*2, 30.f*2));
    uint8_t tsetpoint = (uint8_t)roundf(CLAMP(target_temperature*2, 0.f*2, 30.f*2));
    miso_frame[DB2] = tsetpoint;
    miso_frame[DB2] |= 1 << 7;      // set bit for temp
    ESP_LOGD(TAG, "Set temperature, new DB2: %x (from source temp %f)", miso_frame[DB2], target_temperature);
    xSemaphoreGive(miso_semaphore_handle);
}

bool mhi_ac_ctrl_core_target_temperature_changed() {
    return mosi_frame_snapshot[DB2] != mosi_frame_snapshot_prev[DB2];
}

float mhi_ac_ctrl_core_target_temperature_get() {
    float temp = mosi_frame_snapshot[DB2] &~ (1 << 7);
    return temp / 2.0f;
}

bool mhi_ac_ctrl_core_power_changed() {
    return (mosi_frame_snapshot[DB0] & PWR_MASK) != (mosi_frame_snapshot_prev[DB0] & PWR_MASK);
}

void mhi_ac_ctrl_core_power_set(ACPower power) {
    xSemaphoreTake(miso_semaphore_handle, portMAX_DELAY);
    miso_frame[DB0] &= ~PWR_MASK;                       // clear what is there first
    miso_frame[DB0] |= (uint8_t)power;                  // set the setting
    miso_frame[DB0] |= 1 << 1;                          // DB0[1] 'set setting bit'
    xSemaphoreGive(miso_semaphore_handle);
}

ACPower mhi_ac_ctrl_core_power_get() {
    if(mosi_frame_snapshot[DB0] & PWR_MASK)
        return ACPower::power_on;
    else
        return ACPower::power_off;
}

bool mhi_ac_ctrl_core_mode_changed() {
    return (mosi_frame_snapshot[DB0] & MODE_MASK) != (mosi_frame_snapshot_prev[DB0] & MODE_MASK);
}

void mhi_ac_ctrl_core_mode_set(ACMode mode) {
    if(mode == ACMode::mode_unknown) {
        return;
    }
    xSemaphoreTake(miso_semaphore_handle, portMAX_DELAY);
    miso_frame[DB0] &= ~MODE_MASK;                      // clear what is there first
    miso_frame[DB0] |= (uint8_t)mode;                   // set the setting
    miso_frame[DB0] |= 1 << 5;                          // 'set setting bit for mode'
    xSemaphoreGive(miso_semaphore_handle);
}

ACMode mhi_ac_ctrl_core_mode_get() {
    uint8_t current_mode = mosi_frame_snapshot[DB0] & MODE_MASK;
    switch(current_mode) {
        case (uint8_t)ACMode::mode_auto:
        case (uint8_t)ACMode::mode_dry:
        case (uint8_t)ACMode::mode_cool:
        case (uint8_t)ACMode::mode_fan:
        case (uint8_t)ACMode::mode_heat:
            return (ACMode) current_mode;
        default:
            return ACMode::mode_unknown;
    }
}

bool mhi_ac_ctrl_core_fan_changed() {
    return (mosi_frame_snapshot[DB1] & FAN_MASK) != (mosi_frame_snapshot_prev[DB1] & FAN_MASK);

}
void mhi_ac_ctrl_core_fan_set(ACFan fan) {
    xSemaphoreTake(miso_semaphore_handle, portMAX_DELAY);
    miso_frame[DB1] &= ~FAN_MASK;
    miso_frame[DB1] |= (uint8_t) fan;
    miso_frame[DB1] |= 1 << 3;              // set bit for fan speed on DB1
    xSemaphoreGive(miso_semaphore_handle);
}

ACFan mhi_ac_ctrl_core_fan_get() {
    uint8_t fan_value = mosi_frame_snapshot[DB1] & FAN_MASK;
    switch(fan_value) {
        case (uint8_t) ACFan::speed_1:
        case (uint8_t) ACFan::speed_2:
        case (uint8_t) ACFan::speed_3:
        case (uint8_t) ACFan::speed_4:
        case (uint8_t) ACFan::speed_auto:
            return (ACFan) fan_value;
        default:
            return ACFan::unknown;
    }
}

bool mhi_ac_ctrl_core_current_temperature_changed() {
    return mosi_frame_snapshot[DB3] != mosi_frame_snapshot_prev[DB3];
}

float mhi_ac_ctrl_core_current_temperature_get() {
    return ((int)mosi_frame_snapshot[DB3] - 61) / 4.0;
}

bool mhi_ac_ctrl_core_compressor_changed() {
    return (mosi_frame_snapshot[DB13] & COMP_ACTIVE_MASK) != (mosi_frame_snapshot_prev[DB13] & COMP_ACTIVE_MASK);
}

bool mhi_ac_ctrl_core_compressor_get() {
    return mosi_frame_snapshot[DB13] & COMP_ACTIVE_MASK;
}

bool mhi_ac_ctrl_core_heatcool_changed() {
    return (mosi_frame_snapshot[DB13] & HEAT_COOL_MASK) != (mosi_frame_snapshot_prev[DB13] & HEAT_COOL_MASK);
}

bool mhi_ac_ctrl_core_heatcool_get() {
    return mosi_frame_snapshot[DB13] & HEAT_COOL_MASK;
}

uint32_t mhi_ac_ctrl_core_frame_errors_get() {
    return frame_errors;
}

bool mhi_ac_ctrl_core_vanes_updown_changed() {
    return (mosi_frame_snapshot[DB0] & 0xC0) != (mosi_frame_snapshot_prev[DB0] & 0xC0) ||
        (mosi_frame_snapshot[DB1] & 0x30) != (mosi_frame_snapshot_prev[DB1] & 0x30);
}

ACVanes mhi_ac_ctrl_core_vanes_updown_get() {
    // Swing or up/down position was set using remote
    // meaning we can't know the position
    if((mosi_frame_snapshot[DB0] & 0x80) == 0 || (mosi_frame_snapshot[DB1] & 0x80) == 0) {
        return ACVanes::SeeIRRemote;
    }

    if(mosi_frame_snapshot[DB0] & 0x40) {
        return ACVanes::Swing;
    }
    switch (mosi_frame_snapshot[DB1] & 0x30) {
        case 0x00:
            return ACVanes::Up;
        case 0x10:
            return ACVanes::UpCenter;
        case 0x20:
            return ACVanes::CenterDown;
        case 0x30:
        default:
            return ACVanes::Down;
    }
}

void mhi_ac_ctrl_core_vanes_updown_set(ACVanes new_state) {
    if(new_state == ACVanes::SeeIRRemote)
        return;
    xSemaphoreTake(miso_semaphore_handle, portMAX_DELAY);

    miso_frame[DB0] |= 0x80; // Vanes set
    if(new_state == ACVanes::Swing) {
        miso_frame[DB0] |= 0x40; // Enable swing
    } else {
        miso_frame[DB0] &= ~0x40;
        miso_frame[DB1] |= 0x80; // Pos set
        miso_frame[DB1] |= (static_cast<uint8_t>(new_state)) << 4;
    }
    xSemaphoreGive(miso_semaphore_handle);
}

static int validate_frame_short(uint8_t* mosi_frame, uint16_t rx_checksum) {
  if ( ((mosi_frame[SB0] & 0xfe) != 0x6c) | (mosi_frame[SB1] != 0x80) | (mosi_frame[SB2] != 0x04) ) {
    ESP_LOGW(TAG, "wrong MOSI signature. 0x%02x 0x%02x 0x%02x",
                mosi_frame[0], mosi_frame[1], mosi_frame[2]);

    return -1;
  } else if ( (mosi_frame[CBH] != (rx_checksum>>8 & 0xff)) | (mosi_frame[CBL] != (rx_checksum & 0xff)) ) {
    ESP_LOGW(TAG, "wrong short MOSI checksum. calculated 0x%04x. MOSI[18]:0x%02x MOSI[19]:0x%02x",
                rx_checksum, mosi_frame[CBH], mosi_frame[CBL]);

    return -2;
  }
  return 0;
}

static int validate_frame_long(uint8_t* mosi_frame, uint8_t rx_checksum) {
  if(mosi_frame[CBL2] != rx_checksum) {
    ESP_LOGW(TAG, "wrong long MOSI checksum. calculated 0x%02x. MOSI[32]:0x%02x",
                rx_checksum, mosi_frame[CBL2]);
    return -3;
  }
  return 0;
}

static int validate_frame(uint8_t* mosi_frame, uint8_t frame_len) {
  int err = 0;;
  uint16_t rx_checksum = 0;
  // Frame len has been validated before to only be either MHI_FRAME_LEN_LONG or MHI_FRAME_LEN_SHORT
  for (uint8_t i = 0; i < frame_len; i++) {
    switch(i) {
      case CBH:
        // validate checksum short
        err = validate_frame_short(mosi_frame, rx_checksum);
        if(err) {
          return err;
        }
        rx_checksum += mosi_frame[CBH];
        rx_checksum += mosi_frame[CBL];
        // skip over CBL
        i++;
        break;
      case CBL2:
        err = validate_frame_long(mosi_frame, rx_checksum);
        if(err) {
          return err;
        }
        break;
      default:
        rx_checksum += mosi_frame[i];
    }
  }
  return err;
}

static void mhi_poll_task(void *arg)
{
    esp_err_t err = 0;

    uint8_t frame = 0;
    bool halfcycle = false;

    uint16_t rx_checksum = 0;
    uint16_t tx_checksum = 0;

    // use WORD_ALIGNED_ATTR when using DMA buffer
    // use 2 recv buffers to be able to check for differences
    WORD_ALIGNED_ATTR uint8_t sendbuf[MHI_FRAME_LEN_LONG];
    WORD_ALIGNED_ATTR uint8_t recvbuf[MHI_FRAME_LEN_LONG];
    WORD_ALIGNED_ATTR uint8_t recvbuf2[MHI_FRAME_LEN_LONG];
    uint8_t* mosi_frame_prev = recvbuf2;
    uint8_t* mosi_frame = recvbuf;

    spi_slave_transaction_t spi_slave_trans;
    spi_slave_transaction_t* spi_slave_trans_out;       // needed for spi_slave_get_trans_result which needs a pointer to a pointer

    gptimer_event_callbacks_t timer_callbacks = {
      .on_alarm = gptimer_isr_callback
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(cs_timer, &timer_callbacks, &spi_slave_trans));
    ESP_ERROR_CHECK(gptimer_enable(cs_timer));
    // Set the count to 0, so the gpio ISR will start the timer
    ESP_ERROR_CHECK(gptimer_set_raw_count(cs_timer, 0));

    //Set up a transaction of MHI_FRAME_LEN bytes to send/receive
    spi_slave_trans.length = MHI_FRAME_LEN_LONG*8;
    spi_slave_trans.tx_buffer = sendbuf;
    spi_slave_trans.rx_buffer = mosi_frame;

    while(1) {
        if (err) {
            ESP_LOGW(TAG, "error %i . trans len: %i", err, spi_slave_trans.trans_len);
            frame_errors++;

            // wait a second before retrying communication
            vTaskDelayMs(1000);
        }
        err = 0;

        if(!active_mode) {
            memset(sendbuf, 0xff, sizeof sendbuf);
        } else if (frame++ >= MHI_NUM_FRAMES_PER_INTERVAL) {
            halfcycle = !halfcycle;                     // toggle
            frame = 1;                                  // 2 * MHI_NUM_FRAMES_PER_INTERVAL make a complete cycle. for half that,
            // MISO_frame[DB14] bit2 is 0, and the other half it is 1. When it is set to 1
            // the MISO can be set with any new settings
            if (halfcycle && xSemaphoreTake(miso_semaphore_handle, 0) == pdTRUE) {
                memcpy(sendbuf, miso_frame, sizeof(miso_frame));
                //request current
                sendbuf[DB6] = 0x40;
                sendbuf[DB9] = 0x90; //current
                // we never change those, right?
                //sendbuf[DB10] = 0xff;
                //sendbuf[DB11] = 0xff;
                //sendbuf[DB12] = 0xff;

                // to set a setting, the same bits are set in the MISO frame
                // that they are located in the MOSI frame. you also need to
                // set a specific 'set bit'. this bit stays set (even in the
                // MOSI frame) until the RC is used. at this point, _all_ set
                // bits are reset to 0 since we just copied all the new
                // configuration to the sendbuf. if the RC is used, the
                // updated setting will appear in the MOSI frame, which is
                // updated every frame. once every ~40 frames (2 *
                // MHI_NUM_FRAMES_PER_INTERVAL), the MISO frame is set (in
                // this loop) with any settings that have changed in the ESP

                miso_frame[DB0] = 0x00;
                miso_frame[DB1] = 0x00;
                miso_frame[DB2] = 0x00;
                xSemaphoreGive(miso_semaphore_handle);
            }

            // DB14 bit2 toggles periodically (about every 20 frames)
            sendbuf[DB14] = halfcycle << 2;

            // calculate checksum for the short frame
            tx_checksum = 0;
            for (uint8_t byte_cnt = 0; byte_cnt < CBH; byte_cnt++) {
                tx_checksum += sendbuf[byte_cnt];
            }
            sendbuf[CBH] = tx_checksum>>8;
            sendbuf[CBL] = tx_checksum;

            // Continue calculating for the long frame
            for (uint8_t byte_cnt = CBH; byte_cnt < CBL2; byte_cnt++) {
              tx_checksum += sendbuf[byte_cnt];
            }
            sendbuf[CBL2] = tx_checksum;
        }


        // Do an SPI transaction
        // blocking function waiting for the spi results. the hardware timer must reach 20ms and perform an spi transaction
        //  we can get the data directly from 'spi_slave_trans' instead of 'spi_slave_trans_out->'
        ready = true;
        err = spi_slave_get_trans_result(RCV_HOST, &spi_slave_trans_out, portMAX_DELAY);
        ready = false;
        if(err) {
            ESP_LOGE(TAG, "get_trans_result error: %i", err);
            continue;
        }
        // swap buffers
        if(spi_slave_trans.rx_buffer == recvbuf) {
          mosi_frame = recvbuf;
          mosi_frame_prev = recvbuf2;
          spi_slave_trans.rx_buffer = recvbuf2;
        } else {
          mosi_frame = recvbuf2;
          mosi_frame_prev = recvbuf;
          spi_slave_trans.rx_buffer = recvbuf;
        }
        // Transaction must be of a supported length
        if(spi_slave_trans_out->trans_len != MHI_FRAME_LEN_LONG * 8
            && spi_slave_trans_out->trans_len != MHI_FRAME_LEN_SHORT * 8) {
          err = true;
          continue;
        }
        const size_t trans_len_bytes = spi_slave_trans_out->trans_len / 8;

        // Validate SPI transaction
        err = validate_frame(mosi_frame, trans_len_bytes);
        if(err != 0) {
          continue;
        }

        // Make snapshot if requested
        if(xSemaphoreTake(snapshot_semaphore_handle, 0) != pdTRUE) {
            memcpy(mosi_frame_snapshot_prev, mosi_frame_snapshot, sizeof(mosi_frame_snapshot_prev));
            memcpy(mosi_frame_snapshot, mosi_frame, sizeof(mosi_frame_snapshot));
        }
        xSemaphoreGive(snapshot_semaphore_handle);



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
        const int frame_diff = memcmp(mosi_frame, mosi_frame_prev, trans_len_bytes);
        if (frame_diff) {
            // Evaluate Operating Data and Error Operating Data
            bool MOSI_type_opdata = (mosi_frame[DB10] & 0x30) == 0x10;

            if(mosi_frame[DB9] == 0x90) {
                if ((mosi_frame[DB6] & 0x80) == 0) {  // 29 CT
                    if (MOSI_type_opdata) {
                        mhi_energy.set_current(mosi_frame[DB11]);
                        //float current = ((int)mosi_frame[DB11] * 14) / 51.0f;
                        //ESP_LOGI(TAG, "Current: %f, raw: %i, *230: %f", current, mosi_frame[DB11], current*230);
                    }
                    else {
                        ESP_LOGW(TAG, "Current: error: %i", mosi_frame[DB11]);
                    }
                }
            }
        }
    }
}

void mhi_ac_ctrl_core_init(const Config& config) {
    esp_err_t err;

    miso_semaphore_handle = xSemaphoreCreateMutexStatic( &miso_semaphore_buffer );
    snapshot_semaphore_handle = xSemaphoreCreateBinaryStatic( &snapshot_semaphore_buffer );
    gpio_cs_out = config.cs_out;

    // configuration for the SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = config.mosi,
        .miso_io_num = config.miso,
        .sclk_io_num = config.sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .max_transfer_sz = 0,
        .flags = SPICOMMON_BUSFLAG_GPIO_PINS | SPICOMMON_BUSFLAG_SLAVE,
        .intr_flags = 0,
    };

    // configuration for the SPI slave interface
    spi_slave_interface_config_t slvcfg = {
        .spics_io_num = config.cs_in,
        .flags = SPI_SLAVE_BIT_LSBFIRST,
        .queue_size = 1,
        .mode = 3,                    //CPOL=1, CPHA=1
        .post_setup_cb = 0,
        .post_trans_cb = 0,
    };

    // initialize SPI slave interface
    err = spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);    // can't disable DMA. no comms if you do...
    ESP_ERROR_CHECK(err);

    // Set up timer
    gptimer_config_t timer_config = {
      .clk_src = GPTIMER_CLK_SRC_APB,
      .direction = GPTIMER_COUNT_UP,
      .resolution_hz = 1000000, // Plenty of resolution to encode a rough 20ms ;)
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &cs_timer));
    gptimer_alarm_config_t timer_alarm_config = {
      // Configure the alarm value (in milliseconds) and the interrupt on alarm. there is a gap between each frame of 40ms.
      // So we set the alarm to 20ms. once the alarm triggers, spi_slave_queue_trans is called which will get the data
      // from the hardware spi. This is also the point to toggle the CS line to mark the end/start of a SPI
      // transaction to the hardware, as it won't work without.
      //
      .alarm_count = 20 * (timer_config.resolution_hz / 1000),
      .reload_count = 0,
      .flags = {
        // We manually reload based on the clock signal we get from the AC
        .auto_reload_on_alarm = false,
      }
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(cs_timer, &timer_alarm_config));
    // The ISR uses the counter value to determine whether it is already running. The ISR will probably trigger before
    // we are ready in the mhi_poll_task, so set a non-zero value so it assumes it has already been started. The
    // mhi_poll_task will prepare the timer with a 0 count.
    ESP_ERROR_CHECK(gptimer_set_raw_count(cs_timer, timer_config.resolution_hz));

    gpio_config_t io_conf = {};
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL<<config.sclk);
io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;                                     // required to prevent abort() caused by floating pin when daughtboard not connected
    io_conf.intr_type = GPIO_INTR_LOW_LEVEL;                    // when this is set to NEGEDGE, DMA sometimes doesn't read the last 4 bytes
                                                                // if not connected to AC (plugged in) when starting, it will crash - probably because it
                                                                // immediately calls an interrupt
    gpio_config(&io_conf);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(config.sclk, gpio_isr_handler, NULL);
    gpio_intr_enable(config.sclk);

    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<gpio_cs_out);
    io_conf.intr_type =GPIO_INTR_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    gpio_set_level(gpio_cs_out, 1);

    xTaskCreatePinnedToCore(
                        mhi_poll_task,          // Function to implement the task
                        "mhi_task",             // Name of the task
                        4096,                   // Stack size in words
                        NULL,                   // Task input parameter
                        10,                     // Priority of the task
                        &mhi_poll_task_handle,  // Task handle.
                        1);                     // Core where the task should run

}
} //namespace mhi_ac
