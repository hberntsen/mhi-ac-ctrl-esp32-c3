#pragma once
#include <stdint.h>
#include <array>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "MHI-AC-Ctrl-internal.h"

#define OPERATION_DATA_REQUEST_TIMEOUT_CYCLES 100

#define DEFINE_OPERATION_DATA_CLASS(ClassName, MISO_DB6, MISO_DB9, Tinternal, Texternal, MatchExpr, UpdateExpr, GetExpr) \
  class ClassName final : public OperationDataHelper<MISO_DB6, MISO_DB9, Tinternal, Texternal> { \
  public: \
    ClassName() : OperationDataHelper() {} \
    const char* name() const override { return #ClassName; } \
    bool matches(const std::array<uint8_t, MHI_FRAME_LEN_LONG>& mosi_frame) override { return MatchExpr; } \
    void update(const std::array<uint8_t, MHI_FRAME_LEN_LONG>& mosi_frame) override { this->set_internal(UpdateExpr); } \
    Texternal get() const override { return GetExpr; } \
  };

namespace mhi_ac {
namespace operation_data {
  using namespace mhi_ac::internal;

  class OperationData {
  public:
    virtual const char* name() const = 0;

    virtual void request(std::array<uint8_t, MHI_FRAME_LEN_LONG>& miso_frame);

    /// Returns true when this operationdata matches this mosi frame
    virtual bool matches(const std::array<uint8_t, MHI_FRAME_LEN_LONG>& mosi_frame) = 0;

    /// When matched, store the updated value
    virtual void update(const std::array<uint8_t, MHI_FRAME_LEN_LONG>& mosi_frame) = 0;

    bool has_value() const {
      return this->age != -1;
    }

    bool enabled = false;
    /// Age in terms of esp_timer_get_time() time. -1 for no value yet
    int64_t age = -1;
  protected:
    OperationData() {}
  };

  template<uint8_t MISO_DB6, uint8_t MISO_DB9, typename Tinternal, typename Texternal>
  class OperationDataHelper: public OperationData {
  public:

    void request(std::array<uint8_t, MHI_FRAME_LEN_LONG>& miso_frame) override {
      miso_frame[DB6] = MISO_DB6;
      miso_frame[DB9] = MISO_DB9;
    }

    bool was_changed() const {
      return changed;
    }

    virtual Texternal get() const;

    Texternal get_reset_changed() {
      changed = false;
      return this->get();
    }

  protected:
    OperationDataHelper(): changed(false) {}

    /// Called when the request was answered
    void set_internal(Tinternal new_value) {
      int64_t new_age = esp_timer_get_time();
      ESP_LOGD("MHI-AC-CTRL-Operation-Data", "Operdata %s request fulfilled after %u", this->name(), new_age - this->age);
      if(new_value != last_value || !this->has_value()) {
        this->changed = true;
        this->last_value = new_value;
      }
      this->age = new_age;
    }

    bool changed;
    Tinternal last_value;
  };

  DEFINE_OPERATION_DATA_CLASS(SetTemperature, 0xc0, 0x05, uint8_t, float,
    mosi_frame[DB9] == 0x05 && (mosi_frame[DB6] & 0x80) != 0 && mosi_frame[DB10] == 0x13,
    mosi_frame[DB11] & 0x7f,
    this->last_value / 2.f)

  DEFINE_OPERATION_DATA_CLASS(ReturnAirTemperature, 0xc0, 0x80, uint8_t, float,
    mosi_frame[DB9] == 0x80 && (mosi_frame[DB6] & 0x80) != 0 && (mosi_frame[DB10] & 0x30) == 0x20,
    mosi_frame[DB11],
    (this->last_value - 61.f) / 4.f)

  /// a.k.a THI-R1
  DEFINE_OPERATION_DATA_CLASS(IndoorUBendTemperature, 0xc0, 0x81, uint8_t, float,
    mosi_frame[DB9] == 0x81 && (mosi_frame[DB6] & 0x80) != 0 && (mosi_frame[DB10] & 0x30) == 0x20,
    mosi_frame[DB11],
    this->last_value * 0.327f - 11.4f) // only rough approximation

  /// a.k.a THI-R2
  DEFINE_OPERATION_DATA_CLASS(IndoorCapillaryTemperature, 0x40, 0x81, uint8_t, uint8_t,
    mosi_frame[DB9] == 0x81 && (mosi_frame[DB6] & 0x80) == 0 && (mosi_frame[DB10] & 0x30) == 0x10,
    mosi_frame[DB11],
    this->last_value) // formula for calculation not known

  /// a.k.a THI-R3
  DEFINE_OPERATION_DATA_CLASS(IndoorSuctionHeaderTemperature, 0xc0, 0x87, uint8_t, float,
    mosi_frame[DB9] == 0x87 && (mosi_frame[DB6] & 0x80) != 0 && (mosi_frame[DB10] & 0x30) == 0x10,
    mosi_frame[DB11],
    this->last_value * 0.327f - 11.4f) // only rough approximation

  DEFINE_OPERATION_DATA_CLASS(IndoorFanSpeed, 0xc0, 0x1f, uint8_t, uint8_t,
    mosi_frame[DB9] == 0x1f && (mosi_frame[DB6] & 0x80) != 0 && (mosi_frame[DB10] & 0x30) == 0x10,
    mosi_frame[DB10] & 0x0f,
    this->last_value)

  DEFINE_OPERATION_DATA_CLASS(IndoorTotalRunHours, 0xc0, 0x1e, uint8_t, uint16_t,
    mosi_frame[DB9] == 0x1e && (mosi_frame[DB6] & 0x80) != 0 && (mosi_frame[DB10] & 0x30) == 0x10,
    mosi_frame[DB11],
    this->last_value * 100)

  DEFINE_OPERATION_DATA_CLASS(OutdoorAirTemperature, 0x40, 0x80, uint8_t, float,
    mosi_frame[DB9] == 0x80 && (mosi_frame[DB10] & 0x30) == 0x10,
    mosi_frame[DB11],
    (this->last_value - 94.f) * 0.25f)

  DEFINE_OPERATION_DATA_CLASS(OutdoorHeatExchangerTemperature1, 0x40, 0x82, uint8_t, float,
    mosi_frame[DB9] == 0x82 && (mosi_frame[DB6] & 0x80) == 0 && (mosi_frame[DB10] & 0x30) == 0x10,
    mosi_frame[DB11],
    this->last_value * 0.327f - 11.4f) // Formula for calculation not known

  DEFINE_OPERATION_DATA_CLASS(CompressorFrequency, 0x40, 0x11, uint16_t, float,
    mosi_frame[DB9] == 0x11 && (mosi_frame[DB6] & 0x80) == 0 && (mosi_frame[DB10] & 0x30) == 0x10,
    (mosi_frame[DB10] << 8 | mosi_frame[DB11]) & 0x0fff,
    (this->last_value >> 8) * 25.6f + 0.1f * (this->last_value & 0xff))

  DEFINE_OPERATION_DATA_CLASS(DischargePipeTemperature, 0x40, 0x85, uint8_t, float,
    mosi_frame[DB9] == 0x85 && (mosi_frame[DB6] & 0x80) == 0 && (mosi_frame[DB10] & 0x30) == 0x10,
    mosi_frame[DB11],
    this->last_value < 0x12 ? 30 : this->last_value / 2.f + 32.f) // We cannot express <=30 in a float, so clamp to 30

  DEFINE_OPERATION_DATA_CLASS(Current, 0x40, 0x90, uint8_t, float,
    mosi_frame[DB9] == 0x90 && (mosi_frame[DB6] & 0x80) == 0 && (mosi_frame[DB10] & 0x30) == 0x10,
    mosi_frame[DB11],
    ((int)this->last_value * 14) / 51.0f)

  DEFINE_OPERATION_DATA_CLASS(CompressorDischargePipeSuperHeatTemperature, 0x40, 0xb1, uint8_t, uint8_t,
    mosi_frame[DB9] == 0xb1 && (mosi_frame[DB6] & 0x80) == 0 && (mosi_frame[DB10] & 0x30) == 0x10,
    mosi_frame[DB11],
    this->last_value) // Formula not known

  DEFINE_OPERATION_DATA_CLASS(CompressorProtectionStateNumber, 0x40, 0x7c, uint8_t, uint8_t,
    mosi_frame[DB9] == 0x7c && (mosi_frame[DB6] & 0x80) == 0 && (mosi_frame[DB10] & 0x30) == 0x10,
    mosi_frame[DB11],
    this->last_value)

  DEFINE_OPERATION_DATA_CLASS(OutdoorFanSpeed, 0x40, 0x1f, uint8_t, uint8_t,
    mosi_frame[DB9] == 0x1f && (mosi_frame[DB6] & 0x80) == 0 && (mosi_frame[DB10] & 0x30) == 0x10,
    mosi_frame[DB10] & 0x0f,
    this->last_value)

  DEFINE_OPERATION_DATA_CLASS(Defrosting, 0x40, 0x0c, uint8_t, bool,
    mosi_frame[DB9] == 0x0c && (mosi_frame[DB6] & 0x80) == 0 && (mosi_frame[DB10] & 0x30) == 0x10,
    mosi_frame[DB10] & 0b1,
    this->last_value)

  DEFINE_OPERATION_DATA_CLASS(CompressorTotalRunHours, 0x40, 0x1e, uint8_t, uint16_t,
    mosi_frame[DB9] == 0x1e && (mosi_frame[DB6] & 0x80) == 0,
    mosi_frame[DB11],
    this->last_value * 100)

  DEFINE_OPERATION_DATA_CLASS(OutdoorExpansionValvePulseRate, 0x40, 0x13, uint16_t, uint16_t,
    mosi_frame[DB9] == 0x13 && (mosi_frame[DB6] & 0x80) == 0 && (mosi_frame[DB10] & 0x30) == 0x10,
    mosi_frame[DB12] << 8 | mosi_frame[DB11],
    this->last_value)

  DEFINE_OPERATION_DATA_CLASS(EnergyUsed, 0xc0, 0x94, uint16_t, float,
    mosi_frame[DB9] == 0x94 && (mosi_frame[DB6] & 0x80) != 0 && (mosi_frame[DB10] & 0x30) == 0x10,
    mosi_frame[DB12] << 8 | mosi_frame[DB11],
    this->last_value / 4.f)


  class State {
  public:
    State() {
      value_semaphore_handle_ = xSemaphoreCreateBinaryStatic( &this->value_semaphore_buffer_ );
      xSemaphoreGive(this->value_semaphore_handle_);
    }

    std::array<OperationData*, 22> get_all() {
      // Not all OperationData is equally fast to retrieve. Interleave slow and instant ones, might help?
      return {
        &this->current_, // instant
        &this->set_temperature_, // instant
        &this->compressor_protection_state_number_, /// 16 times no match on
                                                    /// single, 26 on split
        &this->current_, // Listed more often for higher update freqency
        &this->return_air_temperature_, // instant
        &this->indoor_u_bend_temperature_, //instant
        &this->indoor_capillary_temperature_, //instant
        &this->compressor_total_run_hours_, /// 38 times no match on single,
                                            /// 17 on split
        &this->indoor_suction_header_temperature_,
        &this->indoor_fan_speed_, // instant
        &this->indoor_total_run_hours_, // instant
        &this->current_, // Listed more often for higher update freqency
        &this->outdoor_fan_speed_, // 16 times no match on single, 26 on split
        &this->outdoor_air_temperature_, // instant
        &this->outdoor_heat_exchanger_temperature_1_, // instant
        &this->compressor_frequency_, //instant
        &this->current_, // Listed more so we can request it more often for higher update freqency
        &this->defrosting_, /// 17 times no match on single, 15 on split
        &this->discharge_pipe_temperature_, // instant
        &this->compressor_discharge_pipe_super_heat_temperature_, // 18 times no match
        &this->outdoor_expansion_valve_pulse_rate_, // 39 times no match
        &this->energy_used_ // instant
      };
    }

    std::array<OperationData*, 19> get_all_unique() {
      return {
        &this->current_,
        &this->set_temperature_,
        &this->compressor_protection_state_number_,
        &this->return_air_temperature_,
        &this->indoor_u_bend_temperature_,
        &this->indoor_capillary_temperature_,
        &this->compressor_total_run_hours_,
        &this->indoor_suction_header_temperature_,
        &this->indoor_fan_speed_,
        &this->indoor_total_run_hours_,
        &this->outdoor_fan_speed_,
        &this->outdoor_air_temperature_,
        &this->outdoor_heat_exchanger_temperature_1_,
        &this->compressor_frequency_,
        &this->defrosting_,
        &this->discharge_pipe_temperature_,
        &this->compressor_discharge_pipe_super_heat_temperature_,
        &this->outdoor_expansion_valve_pulse_rate_,
        &this->energy_used_
      };
    }


    bool value_semaphore_take() {
      return xSemaphoreTake(this->value_semaphore_handle_, 0) == pdTRUE;
    }

    void value_semaphore_give() {
      xSemaphoreGive(this->value_semaphore_handle_);
    }

    void on_miso(std::array<uint8_t, MHI_FRAME_LEN_LONG>& miso_frame) {
      // Reset by default
      miso_frame[DB6] = 0x80;
      miso_frame[DB9] = 0xff;

      auto all = this->get_all();

      this->request_cycles++;
      if(this->request_cycles >= OPERATION_DATA_REQUEST_TIMEOUT_CYCLES) {
        this->request_next = true;
        this->timeouts++;
        ESP_LOGW("MHI-AC-CTRL-Operation-Data", "Timeout when requesting Operation Data %s", all[this->cycle_index]->name());
      }

      if(this->request_next) {
        // Jump to the next enabled index after the current one
        for(unsigned i = 0; i < all.size(); i++) {
          auto all_index = (i + this->cycle_index + 1) % all.size();
          auto x = all[all_index];
          if(x->enabled) {
            x->request(miso_frame);
            this->cycle_index = all_index;
            this->request_cycles = 0;
            ESP_LOGD("MHI-AC-CTRL-Operation-Data", "Requested %s", x->name());
            break;
          }
        }
        this->request_next = false;
      }
    }

    void on_mosi(const std::array<uint8_t, MHI_FRAME_LEN_LONG>& mosi_frame) {
      if(!this->value_semaphore_take()) {
        return;
      }

      bool match_found = false;
      for(auto x: this->get_all_unique()) {
        if(x->matches(mosi_frame)) {
          ESP_LOGD("MHI-AC-CTRL-Operation-Data", "Got: %s", x->name());
          x->update(mosi_frame);
          match_found = true;
          // Assumes we received the thing we requested an only one thing per request
          this->request_next = true;
          break;
        }
      }

      if(!match_found) {
        if(mosi_frame[DB9] == 0x45 && (mosi_frame[DB6] & 0x80) != 0 && mosi_frame[DB10] == 0x11) {
          ESP_LOGW("MHI-AC-CTRL-Operation-Data", "Last error number: %u", mosi_frame[DB11]);
        } else if(mosi_frame[DB9] == 0x45 && (mosi_frame[DB6] & 0x80) != 0 && mosi_frame[DB10] == 0x12) {
          ESP_LOGW("MHI-AC-CTRL-Operation-Data", "Count of following error operation data: %u", mosi_frame[DB11] + 4);
        } else if (mosi_frame[DB9] != 0xff) {
          ESP_LOGW("MHI-AC-CTRL-Operation-Data", "Error: Unknown opdata: %02x %02x %02x", mosi_frame[DB9], mosi_frame[DB6], mosi_frame[DB10]);
        } else {
          ESP_LOGD("MHI-AC-CTRL-Operation-Data", "Got: nothing");
        }
      }

      this->value_semaphore_give();
    }

    SetTemperature set_temperature_;
    ReturnAirTemperature return_air_temperature_;
    IndoorUBendTemperature indoor_u_bend_temperature_;
    IndoorCapillaryTemperature indoor_capillary_temperature_;
    IndoorSuctionHeaderTemperature indoor_suction_header_temperature_;
    IndoorFanSpeed indoor_fan_speed_;
    IndoorTotalRunHours indoor_total_run_hours_;
    OutdoorAirTemperature outdoor_air_temperature_;
    OutdoorHeatExchangerTemperature1 outdoor_heat_exchanger_temperature_1_;
    CompressorFrequency compressor_frequency_;
    DischargePipeTemperature discharge_pipe_temperature_;
    Current current_;
    CompressorDischargePipeSuperHeatTemperature compressor_discharge_pipe_super_heat_temperature_;
    CompressorProtectionStateNumber compressor_protection_state_number_;
    OutdoorFanSpeed outdoor_fan_speed_;
    Defrosting defrosting_;
    CompressorTotalRunHours compressor_total_run_hours_;
    OutdoorExpansionValvePulseRate outdoor_expansion_valve_pulse_rate_;
    EnergyUsed energy_used_;

    uint32_t timeouts = 0;

  protected:
    /// The index of the current operation data we requested
    unsigned cycle_index = 0;
    /// Whether to go to the next operation data in the next on_miso
    bool request_next = true;
    /// How many on_miso cycles we are busy with a request
    uint8_t request_cycles = 0;

    /// Semaphore to lock updating the values of the operation data
    SemaphoreHandle_t value_semaphore_handle_;
    StaticSemaphore_t value_semaphore_buffer_;
  };
}
}
