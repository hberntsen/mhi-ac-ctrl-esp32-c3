#pragma once
#include <stdint.h>
#include <array>
#include "esp_log.h"
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

    bool is_requested() const {
      return this->request_age != 0xff;
    }

    /// Returns true when timed out
    bool increment_age() {
      if(request_age < 0xff) {
        request_age++;

        if(request_age == OPERATION_DATA_REQUEST_TIMEOUT_CYCLES) {
          ESP_LOGW("MHI-AC-CTRL-Operation-Data", "Timeout when requesting Operation Data %s", this->name());
          request_age = 0xff;
          this->has_value = false;
          return true;
        }
      }
      return false;
    }

    bool enabled = false;
    bool has_value = false;
  protected:
    OperationData() {}
    /// 0xff -> not requested. Else: frame cycle counter how old the request is
    uint8_t request_age = 0xff;
  };

  template<uint8_t MISO_DB6, uint8_t MISO_DB9, typename Tinternal, typename Texternal>
  class OperationDataHelper: public OperationData {
  public:

    void request(std::array<uint8_t, MHI_FRAME_LEN_LONG>& miso_frame) override {
      miso_frame[DB6] = MISO_DB6;
      miso_frame[DB9] = MISO_DB9;
      request_age = 0;
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
      ESP_LOGD("MHI-AC-CTRL-Operation-Data", "Operdata %s request fulfilled after %u", this->name(), this->request_age);
      this->request_age = 0xff;
      if(new_value != last_value || !this->has_value) {
        this->has_value = true;
        this->changed = true;
        this->last_value = new_value;
      }
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
    this->last_value < 0x2 ? 30 : this->last_value / 2.f * 32.f)

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

    std::array<OperationData*, 20> get_all() {
      // Not all OperationData is equally fast to retrieve. Interleave slow and instant ones, might help?
      return {
        &this->current_,
        &this->set_temperature_,
        &this->compressor_protection_state_number_, /// 7 cycles
        &this->return_air_temperature_,
        &this->indoor_u_bend_temperature_,
        &this->indoor_capillary_temperature_,
        &this->compressor_total_run_hours_, /// 14 cycles
        &this->indoor_suction_header_temperature_,
        &this->indoor_fan_speed_,
        &this->indoor_total_run_hours_,
        &this->current_, /// Listed more so we can request it more often for higher update freqency
        &this->outdoor_fan_speed_, /// 19 cycles
        &this->outdoor_air_temperature_,
        &this->outdoor_heat_exchanger_temperature_1_,
        &this->compressor_frequency_,
        &this->defrosting_, /// 9 cycles
        &this->discharge_pipe_temperature_,
        &this->compressor_discharge_pipe_super_heat_temperature_,//22
        &this->outdoor_expansion_valve_pulse_rate_, // 4 - 19 cycles
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
      unsigned active_requests = 0;

      for(auto operation_data: all) {
        if(operation_data->increment_age()) {
          this->timeouts++;
        }
        if(operation_data->enabled && operation_data->is_requested()) {
          active_requests++;
        }
      }

      // With set to "<1" and all OperationData enabled, we get a current reading every 15 seconds
      // With set to "<2", this is every 10 seconds. But: after a day or so, requests are answered but their value stays
      // the same
      // With set to "<3", this is a few seconds quicker but we also start getting timeouts on stuff
      if(active_requests < 1) {
        for(unsigned i = 0; i < all.size(); i++) {
          auto x = all[(i + this->cycle_index) % all.size()];

          if(x->enabled && !x->is_requested()) {
            x->request(miso_frame);
            this->cycle_index = (i + this->cycle_index + 1) % all.size();
            ESP_LOGD("MHI-AC-CTRL-Operation-Data", "Requested %s, Active requests=%u", x->name(), active_requests+1);
            break;
          } else {
            ESP_LOGD("MHI-AC-CTRL-Operation-Data", "%s, enabled %i, requested %i", x->name(), x->enabled, x->is_requested());
          }
        }
      }
    }

    void on_mosi(const std::array<uint8_t, MHI_FRAME_LEN_LONG>& mosi_frame) {
      if(!this->value_semaphore_take()) {
        return;
      }

      bool match_found = false;
      for(auto x: this->get_all()) {
        if(x->matches(mosi_frame)) {
          x->update(mosi_frame);
          match_found = true;
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
    /// The index of the next operation data to try to request
    unsigned cycle_index = 0;

    /// Semaphore to lock updating the values of the operation data
    SemaphoreHandle_t value_semaphore_handle_;
    StaticSemaphore_t value_semaphore_buffer_;
  };
}
}
