// MHI-AC-Ctrol-core
// implements the core functions (read & write SPI)

#include "MHI-AC-Ctrl-core.h"

#include <math.h>
#include <string.h>
#include <algorithm>
#include <ranges>
#include <span>

#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "driver/gpio_filter.h"
#include "soc/spi_periph.h"
#include "driver/rmt_rx.h"

//#undef LOG_LOCAL_LEVEL
//#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"

#include "MHI-AC-CTRL-operation-data.h"
static const char *TAG = "MHI-AC-CTRL-core";

using namespace mhi_ac;
using namespace mhi_ac::internal;


#define vTaskDelayMs(ms)            vTaskDelay((ms)/portTICK_PERIOD_MS)

#define ESP_INTR_FLAG_DEFAULT       0                                           // default to allocating a non-shared interrupt of level 1, 2 or 3.
#define STACK_SIZE 2560

static StaticTask_t xTaskBuffer;
static StackType_t xStack[ STACK_SIZE ];
static TaskHandle_t mhi_comm_task_handle = NULL;

static bool active_mode = false;

namespace mhi_ac {

// Needs length that is a multiple of 4
using spi_dma_buf_t = std::array<uint8_t, MHI_FRAME_LEN_LONG + 4 - (MHI_FRAME_LEN_LONG % 4)>;

static DMA_ATTR spi_dma_buf_t miso_buf;
static DMA_ATTR spi_dma_buf_t mosi_buf;

static rmt_channel_handle_t rmt_rx_chan;
static rmt_receive_config_t rmt_recv_cfg = {};
static rmt_symbol_word_t rmt_buf[SOC_RMT_MEM_WORDS_PER_CHANNEL];

static uint32_t frame_errors = 0;

Energy energy(230);
SpiState spi_state;
operation_data::State operation_data_state;

static bool IRAM_ATTR rmt_on_recv_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_ctx)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Trigger Chip Select low->high->low
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, spi_periph_signal[RCV_HOST].spics_in, false);
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ZERO_INPUT, spi_periph_signal[RCV_HOST].spics_in, false);

    // Start detecting next SPI clock idle
    rmt_receive(rmt_rx_chan, rmt_buf, sizeof(rmt_buf), &rmt_recv_cfg);

    vTaskNotifyGiveFromISR(mhi_comm_task_handle, &xHigherPriorityTaskWoken);

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
  return (this->mosi_frame_snapshot_[DB1] & FAN_MASK) != (this->mosi_frame_snapshot_prev_[DB1] & FAN_MASK) || \
    (this->mosi_frame_snapshot_[DB6] & FAN_DB6_MASK) != (this->mosi_frame_snapshot_prev_[DB6] & FAN_DB6_MASK);
}
void SpiState::fan_set(ACFan fan) {
  xSemaphoreTake(this->miso_semaphore_handle_, portMAX_DELAY);
  this->miso_frame_[DB1] &= ~FAN_MASK;
  this->miso_frame_[DB1] |= (uint8_t) fan;
  this->miso_frame_[DB1] |= 1 << 3;              // set bit for fan speed on DB1
  xSemaphoreGive(this->miso_semaphore_handle_);
}

ACFan SpiState::fan_get() const {
  uint8_t fan_value = (this->mosi_frame_snapshot_[DB1] & FAN_MASK) |
    (this->mosi_frame_snapshot_[DB6] & FAN_DB6_MASK) >> 4;
  switch(fan_value) {
    case (uint8_t) ACFan::speed_1:
    case (uint8_t) ACFan::speed_2:
    case (uint8_t) ACFan::speed_3:
    case (uint8_t) ACFan::speed_4:
    case (uint8_t) ACFan::speed_auto:
      return (ACFan) fan_value;
    default:
      ESP_LOGW(TAG, "Unknown fan speed: %i", fan_value);
      return ACFan::unknown;
  }
}

bool SpiState::current_temperature_changed() const {
  return this->mosi_frame_snapshot_[DB3] != this->mosi_frame_snapshot_prev_[DB3];
}

float SpiState::current_temperature_get() const {
  return ((int)this->mosi_frame_snapshot_[DB3] - 61) / 4.0;
}

void SpiState::external_room_temperature_set(float value) {
  xSemaphoreTake(this->miso_semaphore_handle_, portMAX_DELAY);
  if(std::isnan(value)) {
    this->miso_frame_[DB3] = 0xff;
    ESP_LOGD(TAG, "Disabled external room temperature sensor");
  } else {
    uint8_t troom = (uint8_t)roundf(CLAMP(value * 4 + 61, 0, 0xfe));
    this->miso_frame_[DB3] = troom;
    ESP_LOGD(TAG, "Set external room temperature, new DB3: %x (from source temp %f)", this->miso_frame_[DB3], value);
  }
  xSemaphoreGive(this->miso_semaphore_handle_);
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
  // Clear previous settings
  this->miso_frame_[DB0] &= ~0x40;
  this->miso_frame_[DB1] &= ~0xb0;

  this->miso_frame_[DB0] |= 0x80; // Vanes set
  if(new_state == ACVanesUD::Swing) {
      this->miso_frame_[DB0] |= 0x40; // Enable swing
  } else {
      this->miso_frame_[DB1] |= 0x80; // Pos set
      this->miso_frame_[DB1] |= (static_cast<uint8_t>(new_state)) << 4;
  }
  xSemaphoreGive(miso_semaphore_handle_);
}

bool SpiState::vanes_leftright_changed() const {
  return (this->mosi_frame_snapshot_[DB16] & 0x07) != (this->mosi_frame_snapshot_prev_[DB16] & 0x07) ||
    (this->mosi_frame_snapshot_[DB17] & 0x01) != (this->mosi_frame_snapshot_prev_[DB17] & 0x01);
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

static int validate_frame_short(std::span<uint8_t, MHI_FRAME_LEN_SHORT> mosi_frame, uint16_t rx_checksum) {
  if (! validate_signature(mosi_frame[SB0], mosi_frame[SB1], mosi_frame[SB2])) {
    ESP_LOGW(TAG, "wrong MOSI signature. 0x%02x 0x%02x 0x%02x",
                mosi_frame[0], mosi_frame[1], mosi_frame[2]);

    return -1;
  } else if ( (mosi_frame[CBH] != (rx_checksum>>8 & 0xff)) || (mosi_frame[CBL] != (rx_checksum & 0xff)) ) {
    ESP_LOGW(TAG, "wrong short MOSI checksum. calculated 0x%04x. MOSI[18]:0x%02x MOSI[19]:0x%02x",
                rx_checksum, mosi_frame[CBH], mosi_frame[CBL]);

    return -2;
  }
  return 0;
}

static int validate_frame_long(std::span<uint8_t, MHI_FRAME_LEN_LONG> mosi_frame, uint8_t rx_checksum) {
  if(mosi_frame[CBL2] != rx_checksum) {
    ESP_LOGW(TAG, "wrong long MOSI checksum. calculated 0x%02x. MOSI[32]:0x%02x",
                rx_checksum, mosi_frame[CBL2]);
    return -3;
  }
  return 0;
}

static int validate_frame(std::span<uint8_t, MHI_FRAME_LEN_LONG> mosi_frame, uint8_t frame_len) {
  int err = 0;;
  uint16_t rx_checksum = 0;
  // Frame len has been validated before to only be either MHI_FRAME_LEN_LONG or MHI_FRAME_LEN_SHORT
  for (uint8_t i = 0; i < frame_len; i++) {
    switch(i) {
      case CBH:
        // validate checksum short
        err = validate_frame_short(std::span{mosi_frame}.first<MHI_FRAME_LEN_SHORT>(), rx_checksum);
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

static void mhi_comm_task(void *arg)
{
    esp_err_t err = 0;
    bool double_frame = false;

    spi_slave_transaction_t spi_slave_trans;

    //Set up a transaction of MHI_FRAME_LEN bytes to send/receive
    spi_slave_trans.length = mosi_buf.size() * 8;
    spi_slave_trans.tx_buffer = &miso_buf;
    spi_slave_trans.rx_buffer = &mosi_buf;

    // Start RMT receiver. Missed frames from now on will be counted as frame_errors
    rmt_receive(rmt_rx_chan, rmt_buf, sizeof(rmt_buf), &rmt_recv_cfg);

    while(1) {
        if (err) {
            ESP_LOGW(TAG, "error %i . trans len: %i", err, spi_slave_trans.trans_len);
            frame_errors++;
        }
        err = 0;

        double_frame = !double_frame;

        if(!active_mode) {
          std::fill(miso_buf.begin(), miso_buf.end(), 0xff);
        } else {
          if (double_frame && xSemaphoreTake(spi_state.miso_semaphore_handle_, 0) == pdTRUE) {
            // Copy the changed settings into the miso_buf. Will be cleared from the spi_state on a successful
            // transaction
            operation_data_state.on_miso(std::span{spi_state.miso_frame_}.first<DB10>());

            std::copy(spi_state.miso_frame_.begin(), spi_state.miso_frame_.end(), miso_buf.begin());
            xSemaphoreGive(spi_state.miso_semaphore_handle_);
          }

          miso_buf[DB14] = double_frame ? 0x04 : 0;

          // calculate checksum for the short frame
          uint16_t tx_checksum = 0;
          for (uint8_t byte_cnt = 0; byte_cnt < CBH; byte_cnt++) {
              tx_checksum += miso_buf[byte_cnt];
          }
          miso_buf[CBH] = tx_checksum>>8;
          miso_buf[CBL] = tx_checksum;

          // Continue calculating for the long frame
          for (uint8_t byte_cnt = CBH; byte_cnt < CBL2; byte_cnt++) {
            tx_checksum += miso_buf[byte_cnt];
          }
          miso_buf[CBL2] = tx_checksum;
        }

        // Wait for clock idle via RMT
        uint32_t frames_received = ulTaskNotifyTake( pdTRUE, pdMS_TO_TICKS(10000));
        if(frames_received == 0) {
          // ulTaskNotifyTake timed out
          ESP_LOGE(TAG, "No SPI clock detected");
          frame_errors += 1;
          continue;
        } else if(active_mode && frames_received > 1) {
          // Add missed frames to frame_errors
          frame_errors += frames_received - 1;
          ESP_LOGE(TAG, "Missed %u frames", frames_received - 1);
        }

        // We're ready, wait on the SPI transaction to happen
        err = spi_slave_transmit(RCV_HOST, &spi_slave_trans, pdMS_TO_TICKS(10000));
        if(err) {
          if(err == ESP_ERR_TIMEOUT) {
            frame_errors++;
            ESP_LOGE(TAG, "SPI transaction timeout.");
          } else {
            frame_errors++;
            ESP_LOGE(TAG, "get_trans_result error: %i", err);
          }
          continue;
        }

        // Transaction must be of a supported length
        if(spi_slave_trans.trans_len != MHI_FRAME_LEN_LONG * 8
            && spi_slave_trans.trans_len != MHI_FRAME_LEN_SHORT * 8) {
          err = true;
          continue;
        }
        const size_t trans_len_bytes = spi_slave_trans.trans_len / 8;

        // Validate SPI transaction
        err = validate_frame(std::span{mosi_buf}.first<MHI_FRAME_LEN_LONG>(), trans_len_bytes);
        if(err != 0) {
          frame_errors++;
          continue;
        }

        if (double_frame && xSemaphoreTake(spi_state.miso_semaphore_handle_, 0) == pdTRUE) {
          // Successful SPI transaction. reset changes

          // Reset all indices we use to set settings, except DB3 (external temperature sensor)
          constexpr std::array<size_t, 7> indices_to_erase = {DB0, DB1, DB2, DB6, DB9, DB16, DB17};

          // When active_mode is off, always clear settings to prevent stale settings being applied when active mode
          // is activated later on. Otherwise, only reset when nothing has changed since we've copied it into miso_buf.
          // It it is possible that settings have changed in the meantime and we want to cover that.
          const bool erase = !active_mode || !std::any_of(
              indices_to_erase.begin(), indices_to_erase.end(),
              [&](size_t i) { return spi_state.miso_frame_[i] != miso_buf[i]; }
          );

          if(erase) {
            for(size_t i : indices_to_erase) {
              spi_state.miso_frame_[i] = 0x00;
            }
          }

          xSemaphoreGive(spi_state.miso_semaphore_handle_);
        }

        // Snapshot data when not in use
        if(xSemaphoreTake(spi_state.snapshot_semaphore_handle_, 0) == pdTRUE) {
          std::ranges::copy(
              mosi_buf | std::views::take(spi_state.mosi_frame_snapshot_.size()),
              spi_state.mosi_frame_snapshot_.begin());
          xSemaphoreGive(spi_state.snapshot_semaphore_handle_);
        }

        // We only seem get operation data when double_frame is false
        if(!double_frame){
          // DB4 becomes 1 after a while when active mode is turned off, ignore that
          if(mosi_buf[DB4] > 0 && !(mosi_buf[DB4] == 1 && !active_mode)) {
            ESP_LOGW(TAG, "DB4 error %i", mosi_buf[DB4]);
          }

          operation_data_state.on_mosi(std::span{mosi_buf}.first<DB12>());

          if(mosi_buf[DB9] == 0x90 && (mosi_buf[DB6] & 0x80) == 0 && (mosi_buf[DB10] & 0x30) == 0x10) {
            // 29 CT
            energy.set_current(mosi_buf[DB11]);
            float current = ((int)mosi_buf[DB11] * 14) / 51.0f;
            ESP_LOGD(TAG, "Current: %f, raw: %i, *230: %f", current, mosi_buf[DB11], current*230);
          }
        }
    }
}

void init(const Config& config) {
    esp_err_t err;

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
        .data_io_default_level = false,
        .max_transfer_sz = 0,
        .flags = SPICOMMON_BUSFLAG_GPIO_PINS | SPICOMMON_BUSFLAG_SLAVE,
        .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
        .intr_flags = 0,
    };

    // configuration for the SPI slave interface
    spi_slave_interface_config_t slvcfg = {
        .spics_io_num = -1,
        .flags = SPI_SLAVE_BIT_LSBFIRST,
        .queue_size = 1,
        .mode = 3,                    //CPOL=1, CPHA=1
        .post_setup_cb = 0,
        .post_trans_cb = 0,
    };

    // initialize SPI slave interface
    err = spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);    // can't disable DMA. no comms if you do...
    ESP_ERROR_CHECK(err);

    // Initialise CS to 1 (deselect ourselves as peripheral). RMT interrupt will handle CS after this
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, spi_periph_signal[RCV_HOST].spics_in, false);

    // Set up RMT, used to detect clock idle for generating our own chip select signal
    rmt_rx_channel_config_t rmt_rx_cfg = {};
    rmt_rx_cfg.gpio_num          = static_cast<gpio_num_t>(config.sclk_pin);
    rmt_rx_cfg.clk_src           = RMT_CLK_SRC_DEFAULT;
    rmt_rx_cfg.resolution_hz     = 1000000;
    rmt_rx_cfg.mem_block_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL;
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rmt_rx_cfg, &rmt_rx_chan));

    rmt_rx_event_callbacks_t rmt_cbs = {
        .on_recv_done = rmt_on_recv_done_callback,
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rmt_rx_chan, &rmt_cbs, NULL));

    ESP_ERROR_CHECK(rmt_enable(rmt_rx_chan));

    // Configure the RMT to trigger an interrupt after the clock is idle. The clock is idle between each frame of 40 ms
    // with short frames and about 33ms with long frames.
    // We set the max signal range to 1ms, so it is triggering after 1ms of no clock. We receive about 2 bytes per
    // millisecond. This tolerance should be enough and will give us maximum time to process this frame and prepare the
    // next.
    // In the interrupt of rmt.cbs_on_recv_done, we toggle the CS line internally to mark the end/start of an SPI
    // transaction to the hardware, as it won't work without.
    rmt_recv_cfg.signal_range_max_ns = 1000000;

    gpio_config_t io_conf = {};
    // Make sure the pin is configured as input only
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL<<config.sclk_pin);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

#if SOC_GPIO_SUPPORT_PIN_GLITCH_FILTER
    // No known cases where this made a difference.
    // Shouldn't hurt to enable either
    gpio_pin_glitch_filter_config_t clk_glitch_filter = {
        .clk_src = GLITCH_FILTER_CLK_SRC_DEFAULT,
        .gpio_num = static_cast<gpio_num_t>(config.sclk_pin),
    };
    gpio_glitch_filter_handle_t clk_glitch_filter_handle;
    ESP_ERROR_CHECK(gpio_new_pin_glitch_filter(&clk_glitch_filter, &clk_glitch_filter_handle));
#endif

    mhi_comm_task_handle = xTaskCreateStatic(mhi_comm_task, "mhi_comm_task", STACK_SIZE, NULL, 10, xStack, &xTaskBuffer);
}
} //namespace mhi_ac
