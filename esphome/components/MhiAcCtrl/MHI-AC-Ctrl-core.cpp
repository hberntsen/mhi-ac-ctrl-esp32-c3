// MHI-AC-Ctrol-core
// implements the core functions (read & write SPI)

#include "MHI-AC-Ctrl-core.h"

#include <math.h>
#include <string.h>
#include <algorithm>
#include <ranges>

#include "driver/gptimer.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"

#include "esp_log.h"

#include "MHI-AC-CTRL-operation-data.h"
static const char *TAG = "MHI-AC-CTRL-core";

using namespace mhi_ac;
using namespace mhi_ac::internal;


#define vTaskDelayMs(ms)            vTaskDelay((ms)/portTICK_PERIOD_MS)

#define ESP_INTR_FLAG_DEFAULT       0                                           // default to allocating a non-shared interrupt of level 1, 2 or 3.

gptimer_handle_t cs_timer = NULL;
static TaskHandle_t mhi_poll_task_handle = NULL;

static bool active_mode = false;
static gpio_num_t gpio_cs_out;

static DMA_ATTR std::array<uint8_t, MHI_FRAME_LEN_LONG> sendbuf;
static DMA_ATTR std::array<uint8_t, MHI_FRAME_LEN_LONG> recvbuf;
static DMA_ATTR std::array<uint8_t, MHI_FRAME_LEN_LONG> recvbuf2;

static uint32_t frame_errors = 0;

namespace mhi_ac {
Energy energy(230);
SpiState spi_state;
operation_data::State operation_data_state;

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
  } else {
    ESP_ERROR_CHECK(gptimer_set_raw_count(cs_timer, 0));
  }
}

static bool IRAM_ATTR gptimer_isr_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Stop timer, gpio isr will turn it back on
    gptimer_stop(timer);
    // Set to 0 to tell the GPIO isr that the timer is not running
    gptimer_set_raw_count(cs_timer, 0);

    // Trigger Chip Select low->high->low
    gpio_set_level(gpio_cs_out, 1);
    gpio_set_level(gpio_cs_out, 0);

    vTaskNotifyGiveFromISR(mhi_poll_task_handle, &xHigherPriorityTaskWoken);

    return xHigherPriorityTaskWoken;
}

bool SpiState::snapshot_semaphore_take() {
  return xSemaphoreTake(this->snapshot_semaphore_handle_, 0) == pdTRUE;
}

void SpiState::snapshot_semaphore_give() {
  xSemaphoreGive(this->snapshot_semaphore_handle_);
}

void SpiState::set_snapshot_as_previous() {
  this->mosi_frame_snapshot_prev_ = this->mosi_frame_snapshot_;
}

static bool validate_signature(uint8_t sb0, uint8_t sb1, uint8_t sb2) {
  return (sb0 & 0xfe) == 0x6c && sb1 == 0x80 && sb2 == 0x04;
}

bool SpiState::has_received_data() {
  return validate_signature(this->mosi_frame_snapshot_[SB0], this->mosi_frame_snapshot_[SB1], this->mosi_frame_snapshot_[SB2]);
}

void SpiState::use_long_frame(bool long_frame_enabled) {
  xSemaphoreTake(this->miso_semaphore_handle_, portMAX_DELAY);
  this->miso_frame_[SB0] = long_frame_enabled ? 0xAA : 0xA9;
  xSemaphoreGive(this->miso_semaphore_handle_);
}

void active_mode_set(bool state) {
    active_mode = state;
    if(!active_mode) {
        // Ensure we stop counting power when active mode is off
        energy.set_current(0);
    }
}

bool active_mode_get() {
    return active_mode;
}

#define CLAMP(x, lower, upper) (MIN(upper, MAX(x, lower)))

void SpiState::target_temperature_set(float target_temperature) {
  xSemaphoreTake(this->miso_semaphore_handle_, portMAX_DELAY);
  uint8_t tsetpoint = (uint8_t)roundf(CLAMP(target_temperature*2, 0.f*2, 30.f*2));
  this->miso_frame_[DB2] = tsetpoint;
  this->miso_frame_[DB2] |= 1 << 7;      // set bit for temp
  ESP_LOGD(TAG, "Set temperature, new DB2: %x (from source temp %f)", this->miso_frame_[DB2], target_temperature);
  xSemaphoreGive(this->miso_semaphore_handle_);
}

bool SpiState::target_temperature_changed() const {
  return (this->mosi_frame_snapshot_[DB2] &~ (1 << 7)) != (this->mosi_frame_snapshot_prev_[DB2] &~ (1 << 7));
}

float SpiState::target_temperature_get() const {
  float temp = this->mosi_frame_snapshot_[DB2] &~ (1 << 7);
  return temp / 2.0f;
}

bool SpiState::power_changed() const {
  return (this->mosi_frame_snapshot_[DB0] & PWR_MASK) != (this->mosi_frame_snapshot_prev_[DB0] & PWR_MASK);
}

void SpiState::power_set(ACPower power) {
  xSemaphoreTake(this->miso_semaphore_handle_, portMAX_DELAY);
  this->miso_frame_[DB0] &= ~PWR_MASK;                       // clear what is there first
  this->miso_frame_[DB0] |= (uint8_t)power;                  // set the setting
  this->miso_frame_[DB0] |= 1 << 1;                          // DB0[1] 'set setting bit'
  xSemaphoreGive(this->miso_semaphore_handle_);
}

ACPower SpiState::power_get() const {
  if(this->mosi_frame_snapshot_[DB0] & PWR_MASK)
    return ACPower::power_on;
  else
    return ACPower::power_off;
}

bool SpiState::mode_changed() const {
  return (this->mosi_frame_snapshot_[DB0] & MODE_MASK) != (this->mosi_frame_snapshot_prev_[DB0] & MODE_MASK);
}

void SpiState::mode_set(ACMode mode) {
  if(mode == ACMode::mode_unknown) {
    return;
  }
  xSemaphoreTake(this->miso_semaphore_handle_, portMAX_DELAY);
  this->miso_frame_[DB0] &= ~MODE_MASK;                      // clear what is there first
  this->miso_frame_[DB0] |= (uint8_t)mode;                   // set the setting
  this->miso_frame_[DB0] |= 1 << 5;                          // 'set setting bit for mode'
  xSemaphoreGive(this->miso_semaphore_handle_);
}

ACMode SpiState::mode_get() const {
  uint8_t current_mode = this->mosi_frame_snapshot_[DB0] & MODE_MASK;
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

bool SpiState::fan_changed() const {
  return (this->mosi_frame_snapshot_[DB1] & FAN_MASK) != (this->mosi_frame_snapshot_prev_[DB1] & FAN_MASK);
}
void SpiState::fan_set(ACFan fan) {
  xSemaphoreTake(this->miso_semaphore_handle_, portMAX_DELAY);
  this->miso_frame_[DB1] &= ~FAN_MASK;
  this->miso_frame_[DB1] |= (uint8_t) fan;
  this->miso_frame_[DB1] |= 1 << 3;              // set bit for fan speed on DB1
  xSemaphoreGive(this->miso_semaphore_handle_);
}

ACFan SpiState::fan_get() const {
  uint8_t fan_value = this->mosi_frame_snapshot_[DB1] & FAN_MASK;
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

bool SpiState::current_temperature_changed() const {
  return this->mosi_frame_snapshot_[DB3] != this->mosi_frame_snapshot_prev_[DB3];
}

float SpiState::current_temperature_get() const {
  return ((int)this->mosi_frame_snapshot_[DB3] - 61) / 4.0;
}

bool SpiState::compressor_changed() const{
  return (this->mosi_frame_snapshot_[DB13] & COMP_ACTIVE_MASK) != (this->mosi_frame_snapshot_prev_[DB13] & COMP_ACTIVE_MASK);
}

bool SpiState::compressor_get() const {
  return this->mosi_frame_snapshot_[DB13] & COMP_ACTIVE_MASK;
}

bool SpiState::heatcool_changed() const {
  return (this->mosi_frame_snapshot_[DB13] & HEAT_COOL_MASK) != (this->mosi_frame_snapshot_prev_[DB13] & HEAT_COOL_MASK);
}

bool SpiState::heatcool_get() const {
  return this->mosi_frame_snapshot_[DB13] & HEAT_COOL_MASK;
}

uint32_t frame_errors_get() {
    return frame_errors;
}

bool SpiState::vanes_updown_changed() const {
  return (this->mosi_frame_snapshot_[DB0] & 0xC0) != (this->mosi_frame_snapshot_prev_[DB0] & 0xC0) ||
    (this->mosi_frame_snapshot_[DB1] & 0x30) != (this->mosi_frame_snapshot_prev_[DB1] & 0x30);
}

ACVanesUD SpiState::vanes_updown_get() const {
  // Swing or up/down position was set using remote
  // meaning we can't know the position
  if((this->mosi_frame_snapshot_[DB0] & 0x80) == 0 || (this->mosi_frame_snapshot_[DB1] & 0x80) == 0) {
    return ACVanesUD::SeeIRRemote;
  }

  if(this->mosi_frame_snapshot_[DB0] & 0x40) {
    return ACVanesUD::Swing;
  }
  switch (this->mosi_frame_snapshot_[DB1] & 0x30) {
    case 0x00:
      return ACVanesUD::Up;
    case 0x10:
      return ACVanesUD::UpCenter;
    case 0x20:
      return ACVanesUD::CenterDown;
    case 0x30:
    default:
      return ACVanesUD::Down;
  }
}

void SpiState::vanes_updown_set(ACVanesUD new_state) {
  if(new_state == ACVanesUD::SeeIRRemote)
    return;
  xSemaphoreTake(this->miso_semaphore_handle_, portMAX_DELAY);

  this->miso_frame_[DB0] |= 0x80; // Vanes set
  if(new_state == ACVanesUD::Swing) {
      this->miso_frame_[DB0] |= 0x40; // Enable swing
  } else {
      this->miso_frame_[DB0] &= ~0x40;
      this->miso_frame_[DB1] |= 0x80; // Pos set
      this->miso_frame_[DB1] |= (static_cast<uint8_t>(new_state)) << 4;
  }
  xSemaphoreGive(miso_semaphore_handle_);
}

bool SpiState::vanes_leftright_changed() const {
  return (this->mosi_frame_snapshot_[DB16] & 0x07) != (this->mosi_frame_snapshot_prev_[DB16] & 0x07) ||
    (this->mosi_frame_snapshot_[DB17] & 0x01) != (this->mosi_frame_snapshot_[DB17] & 0x01);
}

ACVanesLR SpiState::vanes_leftright_get() const {
  if(this->mosi_frame_snapshot_[DB17] & 0x01) {
    return ACVanesLR::Swing;
  }
  return static_cast<ACVanesLR>(this->mosi_frame_snapshot_[DB16] & 0x07);
}

void SpiState::vanes_leftright_set(ACVanesLR new_state) {
  xSemaphoreTake(this->miso_semaphore_handle_, portMAX_DELAY);
  this->miso_frame_[DB17] |= 0b00000010; // swing set

  if(new_state == ACVanesLR::Swing) {
    this->miso_frame_[DB17] |= 1;
  } else {
    this->miso_frame_[DB17] &= ~1; // Disable swing

    this->miso_frame_[DB16] |= 0b00010000; // LR set
    this->miso_frame_[DB16] &= ~0b00000111; // Unset previously set direction
    this->miso_frame_[DB16] |= static_cast<uint8_t>(new_state) & 0x07; // Set direction
  }

  xSemaphoreGive(this->miso_semaphore_handle_);
}

bool SpiState::three_d_auto_changed() const {
  return (this->mosi_frame_snapshot_[DB17] & 0x04) != (this->mosi_frame_snapshot_prev_[DB17] & 0x04);
}

bool SpiState::three_d_auto_get() const {
  return this->mosi_frame_snapshot_[DB17] & 0x04;
}

void SpiState::three_d_auto_set(bool new_state) {
  xSemaphoreTake(this->miso_semaphore_handle_, portMAX_DELAY);
  this->miso_frame_[DB17] |= 0b00001000; // 3dauto set
  if(new_state) {
    this->miso_frame_[DB17] |= 0b00000100;
  } else {
    this->miso_frame_[DB17] &= ~0b00000100;
  }
  xSemaphoreGive(miso_semaphore_handle_);
}

static int validate_frame_short(std::array<uint8_t, MHI_FRAME_LEN_LONG>& mosi_frame, uint16_t rx_checksum) {
  if (! validate_signature(mosi_frame[SB0], mosi_frame[SB1], mosi_frame[SB2])) {
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

static int validate_frame_long(std::array<uint8_t, MHI_FRAME_LEN_LONG>& mosi_frame, uint8_t rx_checksum) {
  if(mosi_frame[CBL2] != rx_checksum) {
    ESP_LOGW(TAG, "wrong long MOSI checksum. calculated 0x%02x. MOSI[32]:0x%02x",
                rx_checksum, mosi_frame[CBL2]);
    return -3;
  }
  return 0;
}

static int validate_frame(std::array<uint8_t, MHI_FRAME_LEN_LONG>& mosi_frame, uint8_t frame_len) {
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
    bool double_frame = false;

    // use 2 recv buffers to be able to check for differences
    std::array<uint8_t, MHI_FRAME_LEN_LONG>& mosi_frame_prev = recvbuf2;
    std::array<uint8_t, MHI_FRAME_LEN_LONG>& mosi_frame = recvbuf;

    spi_slave_transaction_t spi_slave_trans;

    gptimer_event_callbacks_t timer_callbacks = {
      .on_alarm = gptimer_isr_callback
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(cs_timer, &timer_callbacks, NULL));
    ESP_ERROR_CHECK(gptimer_enable(cs_timer));
    // Set the count to 0, so the gpio ISR will start the timer
    ESP_ERROR_CHECK(gptimer_set_raw_count(cs_timer, 0));

    //Set up a transaction of MHI_FRAME_LEN bytes to send/receive
    spi_slave_trans.length = MHI_FRAME_LEN_LONG*8;
    spi_slave_trans.tx_buffer = &sendbuf;
    spi_slave_trans.rx_buffer = &mosi_frame;

    while(1) {
        if (err) {
            ESP_LOGW(TAG, "error %i . trans len: %i", err, spi_slave_trans.trans_len);
            frame_errors++;
        }
        err = 0;

        double_frame = !double_frame;

        if(!active_mode) {
          std::fill(sendbuf.begin(), sendbuf.end(), 0xff);
        } else {
          if (double_frame && xSemaphoreTake(spi_state.miso_semaphore_handle_, 0) == pdTRUE) {
            std::copy(spi_state.miso_frame_.begin(), spi_state.miso_frame_.end(), sendbuf.begin());

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
            // updated setting will appear in the MOSI frame
            spi_state.miso_frame_[DB0] = 0x00;
            spi_state.miso_frame_[DB1] = 0x00;
            spi_state.miso_frame_[DB2] = 0x00;
            spi_state.miso_frame_[DB16] = 0x00;
            spi_state.miso_frame_[DB17] = 0x00;
            xSemaphoreGive(spi_state.miso_semaphore_handle_);
          }

          sendbuf[DB14] = double_frame ? 0x04 : 0;
          if(double_frame) {
            operation_data_state.on_miso(sendbuf);
          }

          // calculate checksum for the short frame
          uint16_t tx_checksum = 0;
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

        // The GPIO CLK triggered timer alarm will clear right after a frame. Wait for it so we don't transmit the
        // transaction mid-frame. There is a very small chance that it will still happen in between the check here and
        // the actual transaction starting.
        uint64_t current_timer_value;
        do {
          // Count missed frames as error in active mode
          uint32_t missed_frames = ulTaskNotifyTake( pdTRUE, portMAX_DELAY) - 1;
          if(active_mode && missed_frames) {
            frame_errors += missed_frames;
            ESP_LOGE(TAG, "Missed %u frames", missed_frames);
          }
          ESP_ERROR_CHECK(gptimer_get_raw_count(cs_timer, &current_timer_value));
        } while(current_timer_value != 0);
        // We're ready, wait on the SPI transaction to happen
        spi_slave_transmit(RCV_HOST, &spi_slave_trans, pdMS_TO_TICKS(10000));
        if(err) {
          if(err == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "SPI transaction timeout. Is sclk_pin connected?");
          } else {
            ESP_LOGE(TAG, "get_trans_result error: %i", err);
          }
          continue;
        }
        // swap buffers
        if(spi_slave_trans.rx_buffer == &recvbuf) {
          mosi_frame = recvbuf;
          mosi_frame_prev = recvbuf2;
          spi_slave_trans.rx_buffer = &recvbuf2;
        } else {
          mosi_frame = recvbuf2;
          mosi_frame_prev = recvbuf;
          spi_slave_trans.rx_buffer = &recvbuf;
        }
        // Transaction must be of a supported length
        if(spi_slave_trans.trans_len != MHI_FRAME_LEN_LONG * 8
            && spi_slave_trans.trans_len != MHI_FRAME_LEN_SHORT * 8) {
          err = true;
          continue;
        }
        const size_t trans_len_bytes = spi_slave_trans.trans_len / 8;

        // Validate SPI transaction
        err = validate_frame(mosi_frame, trans_len_bytes);
        if(err != 0) {
          continue;
        }

        // Snapshot data when not in use
        if(xSemaphoreTake(spi_state.snapshot_semaphore_handle_, 0) == pdTRUE) {
          std::ranges::copy(
              mosi_frame | std::views::take(spi_state.mosi_frame_snapshot_.size()),
              spi_state.mosi_frame_snapshot_.begin());
          xSemaphoreGive(spi_state.snapshot_semaphore_handle_);
        }

        // We only seem get operation data when double_frame is false
        if(!double_frame){
          // DB4 becomes 1 after a while when active mode is turned off, ignore that
          if(mosi_frame[DB4] > 0 && !(mosi_frame[DB4] == 1 && !active_mode)) {
            ESP_LOGW(TAG, "DB4 error %i", mosi_frame[DB4]);
          }

          operation_data_state.on_mosi(mosi_frame);

          if(mosi_frame[DB9] == 0x90 && (mosi_frame[DB6] & 0x80) == 0 && (mosi_frame[DB10] & 0x30) == 0x10) {
            // 29 CT
            energy.set_current(mosi_frame[DB11]);
            float current = ((int)mosi_frame[DB11] * 14) / 51.0f;
            ESP_LOGD(TAG, "Current: %f, raw: %i, *230: %f", current, mosi_frame[DB11], current*230);
          }
        }
    }
}

/**
 * Checks whether the GPIO CS loopback works by toggling it a few times.
 */
static bool check_gpio_cs_loopback(gpio_num_t cs_in_pin) {
  int on = 0;
  for(int i = 0; i < 42; i++) {
    gpio_set_level(gpio_cs_out, on);
    if(gpio_get_level(cs_in_pin) != on) {
      return false;
    }
    on ^= 1;
  }
  return true;
}

InitError init(const Config& config) {
    esp_err_t err;

    gpio_cs_out = static_cast<gpio_num_t>(config.cs_out_pin);
    spi_state.use_long_frame(config.use_long_frame);

    // configuration for the SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = config.mosi_pin,
        .miso_io_num = config.miso_pin,
        .sclk_io_num = config.sclk_pin,
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
        .spics_io_num = config.cs_in_pin,
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
      // Configure the alarm value (in milliseconds) and the interrupt on alarm. there is a gap between each frame of
      // 40ms with short frames and about 33ms with long frames.
      // We set the alarm to 1ms, so it is triggering after 1ms of no clock. We receive about 2 bytes per millisecond.
      // This tolerance should be enough and will give us maximum time to process this frame and prepare the next.
      //
      // Once the alarm triggers, we toggle the CS line to mark the end/start of a SPI transaction to the hardware, as
      // it won't work without. The GPIO interrupt below resets this timer every time the clock signal is low. When the
      // SPI transaction is complete, the master leaves the clock pin high, setting of the alarm and us toggling the
      // pin.
      .alarm_count = 1 * (timer_config.resolution_hz / 1000),
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
    io_conf.pin_bit_mask = (1ULL<<config.sclk_pin);
io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;                                     // required to prevent abort() caused by floating pin when daughtboard not connected
    io_conf.intr_type = GPIO_INTR_LOW_LEVEL;                    // when this is set to NEGEDGE, DMA sometimes doesn't read the last 4 bytes
                                                                // if not connected to AC (plugged in) when starting, it will crash - probably because it
                                                                // immediately calls an interrupt
    gpio_config(&io_conf);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(static_cast<gpio_num_t>(config.sclk_pin), gpio_isr_handler, NULL);
    gpio_intr_enable(static_cast<gpio_num_t>(config.sclk_pin));

    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<gpio_cs_out);
    io_conf.intr_type =GPIO_INTR_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    if(!check_gpio_cs_loopback(static_cast<gpio_num_t>(config.cs_in_pin))) {
      return InitError::CSLoopbackFail;
    }

    gpio_set_level(gpio_cs_out, 1);

    xTaskCreatePinnedToCore(
                        mhi_poll_task,          // Function to implement the task
                        "mhi_task",             // Name of the task
                        4096,                   // Stack size in words
                        NULL,                   // Task input parameter
                        10,                     // Priority of the task
                        &mhi_poll_task_handle,  // Task handle.
                        1);                     // Core where the task should run

    return InitError::Ok;
}
} //namespace mhi_ac
