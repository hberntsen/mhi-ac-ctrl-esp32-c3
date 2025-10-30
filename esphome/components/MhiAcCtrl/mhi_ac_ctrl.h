#include "MHI-AC-Ctrl-core.h"
#include "esphome/core/gpio.h"

#include "esphome/components/climate/climate.h"
#include "esphome/components/sensor/sensor.h"
#ifdef USE_SWITCH
#include "esphome/components/switch/switch.h"
#endif
#ifdef USE_SELECT
#include "esphome/components/select/select.h"
#endif
#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif


using namespace esphome;
using namespace esphome::climate;
using namespace esphome::sensor;
#ifdef USE_SWITCH
using namespace esphome::switch_;
#endif

static const char* TAG = "mhi_ac_ctrl";

class MhiFrameErrors : public Sensor {
public:
    MhiFrameErrors() {
        this->publish_state(mhi_ac::frame_errors_get());
    }

    void loop() {
        if(mhi_ac::frame_errors_get() != this->get_raw_state()) {
            this->publish_state(mhi_ac::frame_errors_get());
        }
    }
};

class MhiIntegratedTotalEnergy : public Sensor {
public:
    MhiIntegratedTotalEnergy() {
        this->total_energy_ = mhi_ac::energy.total_energy.load();
        publish_total();
    }

    void loop() {
        uint64_t new_energy = mhi_ac::energy.total_energy.load();
        if(new_energy != total_energy_) {
            total_energy_ = new_energy;
            publish_total();
        }
    }

    void publish_total() {
        this->publish_state(total_energy_ * (14.0f/(51000000.0f*3600)));
    }

protected:
    uint64_t total_energy_;
};

class MhiPower : public Sensor {
public:
    MhiPower() {
        if(mhi_ac::active_mode_get()) {
            last_power = mhi_ac::energy.get_power();
            this->publish_power();
        }
    }

    void loop() {
        if(mhi_ac::active_mode_get()) {
            uint32_t new_power = mhi_ac::energy.get_power();
            if(new_power != last_power || std::isnan(get_raw_state())) {
                last_power = new_power;
                publish_power();
            }
        } else if(!std::isnan(get_raw_state()) && !mhi_ac::active_mode_get()) {
            this->publish_state(std::numeric_limits<float>::quiet_NaN());
        }
    }

    void publish_power() {
        this->publish_state(last_power * (14.0f/51.0f));
    }

protected:
    uint32_t last_power;
};

#ifdef USE_SWITCH
class MhiActiveMode : public switch_::Switch {
public:
  void setup() {
    ESP_LOGCONFIG(TAG, "Setting up MHI Active Mode Switch '%s'...", this->name_.c_str());

    bool initial_state = this->get_initial_state_with_restore_mode().value_or(false);

    if (initial_state) {
      this->turn_on();
    } else {
      this->turn_off();
    }
  }

protected:
    virtual void write_state(bool state) {
        mhi_ac::active_mode_set(state);
        this->publish_state(state);
    }
};
#endif

#ifdef USE_SELECT
class MhiVanesUD : public select::Select {
public:
  // Select options must match the ones in the select.py Python code
  virtual void control(const std::string &value) {
    if(value == "3D Auto") {
      mhi_ac::spi_state.three_d_auto_set(true);
    } else {
      mhi_ac::spi_state.three_d_auto_set(false);
      if(value == "Swing") {
        mhi_ac::spi_state.vanes_updown_set(mhi_ac::ACVanesUD::Swing);
      } else if(value == "Up") {
        mhi_ac::spi_state.vanes_updown_set(mhi_ac::ACVanesUD::Up);
      } else if(value == "Up/Center") {
        mhi_ac::spi_state.vanes_updown_set(mhi_ac::ACVanesUD::UpCenter);
      } else if(value == "Center/Down") {
        mhi_ac::spi_state.vanes_updown_set(mhi_ac::ACVanesUD::CenterDown);
      } else if(value == "Down") {
        mhi_ac::spi_state.vanes_updown_set(mhi_ac::ACVanesUD::Down);
      } else {
        ESP_LOGW(TAG, "Unknown vanes_ud mode received: %s", value.c_str());
      }
    }
  }

  void loop() {
    if(!mhi_ac::spi_state.three_d_auto_changed() && !mhi_ac::spi_state.vanes_updown_changed() && has_state()) {
      return;
    }
    if(mhi_ac::spi_state.three_d_auto_get()) {
      publish_state("3D Auto");
    } else {
      switch (mhi_ac::spi_state.vanes_updown_get()) {
        case mhi_ac::ACVanesUD::Swing:
          publish_state("Swing");
          break;
        case mhi_ac::ACVanesUD::Up:
          publish_state("Up");
          break;
        case mhi_ac::ACVanesUD::UpCenter:
          publish_state("Up/Center");
          break;
        case mhi_ac::ACVanesUD::CenterDown:
          publish_state("Center/Down");
          break;
        case mhi_ac::ACVanesUD::Down:
          publish_state("Down");
          break;
        case mhi_ac::ACVanesUD::SeeIRRemote:
          publish_state("See IR Remote");
          break;
      }
    }
  }
};
class MhiVanesLR : public select::Select {
public:
  // Select options must match the ones in the select.py Python code
  virtual void control(const std::string &value) {
    if(value == "3D Auto") {
      mhi_ac::spi_state.three_d_auto_set(true);
    } else {
      mhi_ac::spi_state.three_d_auto_set(false);
      if(value == "Left") {
        mhi_ac::spi_state.vanes_leftright_set(mhi_ac::ACVanesLR::Left);
      } else if(value == "Left/Center") {
        mhi_ac::spi_state.vanes_leftright_set(mhi_ac::ACVanesLR::LeftCenter);
      } else if(value == "Center") {
        mhi_ac::spi_state.vanes_leftright_set(mhi_ac::ACVanesLR::Center);
      } else if(value == "Center/Right") {
        mhi_ac::spi_state.vanes_leftright_set(mhi_ac::ACVanesLR::CenterRight);
      } else if(value == "Right") {
        mhi_ac::spi_state.vanes_leftright_set(mhi_ac::ACVanesLR::Right);
      } else if(value == "Wide") {
        mhi_ac::spi_state.vanes_leftright_set(mhi_ac::ACVanesLR::Wide);
      } else if(value == "Spot") {
        mhi_ac::spi_state.vanes_leftright_set(mhi_ac::ACVanesLR::Spot);
      } else if(value == "Swing") {
        mhi_ac::spi_state.vanes_leftright_set(mhi_ac::ACVanesLR::Swing);
      } else {
        ESP_LOGW(TAG, "Unknown vanes_lr mode received: %s", value.c_str());
      }
    }
  }

  void loop() {
    if(!mhi_ac::spi_state.three_d_auto_changed() && !mhi_ac::spi_state.vanes_leftright_changed() && has_state()) {
      return;
    }
    if(mhi_ac::spi_state.three_d_auto_get()) {
      publish_state("3D Auto");
    } else {
      switch (mhi_ac::spi_state.vanes_leftright_get()) {
        case mhi_ac::ACVanesLR::Left:
          publish_state("Left");
          break;
        case mhi_ac::ACVanesLR::LeftCenter:
          publish_state("Left/Center");
          break;
        case mhi_ac::ACVanesLR::Center:
          publish_state("Center");
          break;
        case mhi_ac::ACVanesLR::CenterRight:
          publish_state("Center/Right");
          break;
        case mhi_ac::ACVanesLR::Right:
          publish_state("Right");
          break;
        case mhi_ac::ACVanesLR::Wide:
          publish_state("Wide");
          break;
        case mhi_ac::ACVanesLR::Spot:
          publish_state("Spot");
          break;
        case mhi_ac::ACVanesLR::Swing:
          publish_state("Swing");
          break;
      }
    }
  }
};
#endif

class MhiAcCtrl : public climate::Climate,
                  public Component {
public:
  MhiAcCtrl(const mhi_ac::Config &config) {
    this->ac_config_ = config;
  }

    void setup() override
    {
        if(this->operation_data_visible_timeouts_sensor_) {
          this->operation_data_visible_timeouts_sensor_->publish_state(0);
        }
        mhi_ac::InitError err = mhi_ac::init(this->ac_config_);
        switch(err) {
          case mhi_ac::InitError::CSLoopbackFail:
            this->status_set_error("CS loopback check failed. Are cs_in_pin and cs_out_pin connected with each other?");
            return;
        }

        constexpr auto opdatas = &mhi_ac::operation_data_state;
        opdatas->set_temperature_.enabled = this->set_temperature_sensor_;
        opdatas->return_air_temperature_.enabled = this->return_air_temperature_sensor_;
        opdatas->indoor_u_bend_temperature_.enabled = this->indoor_u_bend_temperature_sensor_;
        opdatas->indoor_capillary_temperature_.enabled = this->indoor_capillary_temperature_sensor_;
        opdatas->indoor_suction_header_temperature_.enabled = this->indoor_suction_header_temperature_sensor_;
        opdatas->indoor_fan_speed_.enabled = this->indoor_fan_speed_sensor_;
        opdatas->indoor_total_run_hours_.enabled = this->indoor_total_run_hours_sensor_;
        opdatas->outdoor_air_temperature_.enabled = this->outdoor_air_temperature_sensor_;
        opdatas->outdoor_heat_exchanger_temperature_1_.enabled = this->outdoor_heat_exchanger_temperature_1_sensor_;
        opdatas->compressor_frequency_.enabled = this->compressor_frequency_sensor_;
        opdatas->discharge_pipe_temperature_.enabled = this->discharge_pipe_temperature_sensor_;
        opdatas->current_.enabled = this->current_sensor_ || this->integrated_total_energy_sensor_ || this->power_sensor_;
        opdatas->compressor_discharge_pipe_super_heat_temperature_.enabled = this->compressor_discharge_pipe_super_heat_temperature_sensor_;
        opdatas->compressor_protection_state_number_.enabled = this->compressor_protection_state_number_sensor_;
        opdatas->outdoor_fan_speed_.enabled = this->outdoor_fan_speed_sensor_;
        opdatas->compressor_total_run_hours_.enabled = this->compressor_total_run_hours_sensor_;
        opdatas->outdoor_expansion_valve_pulse_rate_.enabled = this->outdoor_expansion_valve_pulse_rate_sensor_;
        opdatas->energy_used_.enabled = this->energy_used_sensor_;
#ifdef USE_BINARY_SENSOR
        opdatas->defrosting_.enabled = this->defrosting_binary_sensor_;
#endif

        if (this->external_room_temperature_sensor_ != nullptr) {
          this->external_room_temperature_sensor_->add_on_state_callback([](float state) {
            mhi_ac::spi_state.external_room_temperature_set(state);
          });
        }

#ifdef USE_SWITCH
        if(this->active_mode_switch_) {
          this->active_mode_switch_->setup();
        }
#endif
    }

    void loop() override {
        bool publish_self_state = false;
        if(!mhi_ac::spi_state.has_received_data()) {
          this->status_set_warning("No MHI AC communication");
          return;
        }
        this->status_clear_warning();

        if(!mhi_ac::spi_state.snapshot_semaphore_take()) {
          return;
        }

        if(integrated_total_energy_sensor_)
            integrated_total_energy_sensor_->loop();
        if(power_sensor_)
            power_sensor_->loop();
        if(frame_errors_sensor_)
            frame_errors_sensor_->loop();
#ifdef USE_SELECT
        if(vanes_ud_select_)
            vanes_ud_select_->loop();
        if(vanes_lr_select_)
            vanes_lr_select_->loop();
#endif
        bool first_time = std::isnan(this->target_temperature);

        if(mhi_ac::spi_state.target_temperature_changed() || first_time) {
          publish_self_state = true;
          this->target_temperature = mhi_ac::spi_state.target_temperature_get();
        }

        if(mhi_ac::spi_state.power_changed()
                || mhi_ac::spi_state.mode_changed()
                || mhi_ac::spi_state.compressor_changed()
                || mhi_ac::spi_state.heatcool_changed()
                || first_time) {
            publish_self_state = true;

          if(mhi_ac::spi_state.power_get() == mhi_ac::ACPower::power_off) {
            this->mode = climate::CLIMATE_MODE_OFF;
            this->action = climate::CLIMATE_ACTION_OFF;
          } else {
            if(!mhi_ac::spi_state.compressor_get()) {
                this->action = climate::CLIMATE_ACTION_IDLE;
            }

            switch(mhi_ac::spi_state.mode_get()) {
              case mhi_ac::ACMode::mode_cool:
                this->mode = climate::CLIMATE_MODE_COOL;
                if(mhi_ac::spi_state.compressor_get())
                    this->action = climate::CLIMATE_ACTION_COOLING;
                break;
              case mhi_ac::ACMode::mode_heat:
                this->mode = climate::CLIMATE_MODE_HEAT;
                if(mhi_ac::spi_state.compressor_get())
                    this->action = climate::CLIMATE_ACTION_HEATING;
                break;
              case mhi_ac::ACMode::mode_dry:
                this->mode = climate::CLIMATE_MODE_DRY;
                if(mhi_ac::spi_state.compressor_get())
                    this->action = climate::CLIMATE_ACTION_DRYING;
                break;
              case mhi_ac::ACMode::mode_fan:
                this->mode = climate::CLIMATE_MODE_FAN_ONLY;
                this->action = climate::CLIMATE_ACTION_FAN;
                break;
              case mhi_ac::ACMode::mode_auto:
                this->mode = climate::CLIMATE_MODE_HEAT_COOL;
                if(mhi_ac::spi_state.compressor_get()) {
                    if(mhi_ac::spi_state.heatcool_get())
                        this->action = climate::CLIMATE_ACTION_HEATING;
                    else
                        this->action = climate::CLIMATE_ACTION_COOLING;
                }
                break;
              default:
                break;
            }
          }
        }

        if(mhi_ac::spi_state.fan_changed() || first_time) {
          publish_self_state = true;
          switch(mhi_ac::spi_state.fan_get()) {
            case mhi_ac::ACFan::speed_1:
              this->set_custom_fan_mode_(custom_fan_ultra_low);
              break;
            case mhi_ac::ACFan::speed_2:
              this->set_fan_mode_(climate::CLIMATE_FAN_LOW);
              break;
            case mhi_ac::ACFan::speed_3:
              this->set_fan_mode_(climate::CLIMATE_FAN_MEDIUM);
              break;
            case mhi_ac::ACFan::speed_4:
              this->set_fan_mode_(climate::CLIMATE_FAN_HIGH);
              break;
            case mhi_ac::ACFan::speed_auto:
              this->set_fan_mode_(climate::CLIMATE_FAN_AUTO);
              break;
            default:
                break;
          }
        }

        if(climate_current_temperature_sensor_) {
          if(mhi_ac::spi_state.current_temperature_changed() || std::isnan(climate_current_temperature_sensor_->get_raw_state())) {
            climate_current_temperature_sensor_->publish_state(mhi_ac::spi_state.current_temperature_get());
          }

          const float new_temperature = climate_current_temperature_sensor_->get_state();
          publish_self_state |= current_temperature != new_temperature && !(std::isnan(current_temperature) && std::isnan(new_temperature));
          current_temperature = new_temperature;
        } else if(mhi_ac::spi_state.current_temperature_changed() || std::isnan(current_temperature)) {
          // Use raw value directly
          current_temperature = mhi_ac::spi_state.current_temperature_get();
          publish_self_state |= !std::isnan(current_temperature);
        }

        if(this->mode != climate::CLIMATE_MODE_HEAT && this->target_temperature < 18) {
            mhi_ac::spi_state.target_temperature_set(18);
            this->target_temperature = 18;
        }

        if(publish_self_state) {
            this->publish_state();
        }

        mhi_ac::spi_state.set_snapshot_as_previous();
        mhi_ac::spi_state.snapshot_semaphore_give();

        this->loop_operation_data(first_time);
    }

protected:
    void loop_operation_data(bool first_time) {
      constexpr auto opdatas = &mhi_ac::operation_data_state;
      if(!opdatas ->value_semaphore_take()) {
        return;
      }

      if(this->operation_data_timeouts_sensor_
          && this->operation_data_timeouts_sensor_->get_raw_state() != opdatas->timeouts) {
        this->operation_data_timeouts_sensor_->publish_state(opdatas->timeouts);
      }

      int64_t current_time = esp_timer_get_time();
      // One minute after which we set values to NaN. Should be plenty
      const int64_t expiry_time = current_time - (1e6 * 60);

      auto update_sensor = [first_time, expiry_time, this](Sensor* sensor, auto operation_data) {
        if(sensor) {
          bool timed_out = operation_data->age < expiry_time;
          bool sensor_has_value = !first_time && !std::isnan(sensor->get_raw_state());

          if(timed_out || !operation_data->has_value()) {
            if(sensor_has_value) {
              sensor->publish_state(NAN);
              // Log this transition to NAN as a visible timeout
              if(timed_out && this->operation_data_visible_timeouts_sensor_) {
                auto visible_timeouts_sensor = this->operation_data_visible_timeouts_sensor_;
                visible_timeouts_sensor->publish_state(visible_timeouts_sensor->get_raw_state()+1);
              }
            }
          } else if(operation_data->was_changed() || !sensor_has_value) {
            // has_value() is true and we assume the value is not NAN
            sensor->publish_state(operation_data->get_reset_changed());
          }
        }
      };

#ifdef USE_BINARY_SENSOR
      auto update_binary_sensor = [first_time](binary_sensor::BinarySensor* sensor, auto operation_data) {
        // A boolean sensor does not have get_raw_state and cannot be set to NAN to indicate we don't have a proper
        // value
        if(sensor) {
          bool sensor_has_value = !first_time;
          if(operation_data->has_value() && (operation_data->was_changed() || !sensor_has_value)) {
            sensor->publish_state(operation_data->get_reset_changed());
          }
        }
      };
#endif

      update_sensor(this->set_temperature_sensor_, &opdatas->set_temperature_);
      update_sensor(this->return_air_temperature_sensor_, &opdatas->return_air_temperature_);
      update_sensor(this->indoor_u_bend_temperature_sensor_, &opdatas->indoor_u_bend_temperature_);
      update_sensor(this->indoor_capillary_temperature_sensor_, &opdatas->indoor_capillary_temperature_);
      update_sensor(this->indoor_suction_header_temperature_sensor_, &opdatas->indoor_suction_header_temperature_);
      update_sensor(this->indoor_fan_speed_sensor_, &opdatas->indoor_fan_speed_);
      update_sensor(this->indoor_total_run_hours_sensor_, &opdatas->indoor_total_run_hours_);
      update_sensor(this->outdoor_air_temperature_sensor_, &opdatas->outdoor_air_temperature_);
      update_sensor(this->outdoor_heat_exchanger_temperature_1_sensor_, &opdatas->outdoor_heat_exchanger_temperature_1_);
      update_sensor(this->compressor_frequency_sensor_, &opdatas->compressor_frequency_);
      update_sensor(this->discharge_pipe_temperature_sensor_, &opdatas->discharge_pipe_temperature_);
      update_sensor(this->current_sensor_, &opdatas->current_);
      update_sensor(this->compressor_discharge_pipe_super_heat_temperature_sensor_, &opdatas->compressor_discharge_pipe_super_heat_temperature_);
      update_sensor(this->compressor_protection_state_number_sensor_, &opdatas->compressor_protection_state_number_);
      update_sensor(this->outdoor_fan_speed_sensor_, &opdatas->outdoor_fan_speed_);
      update_sensor(this->compressor_total_run_hours_sensor_, &opdatas->compressor_total_run_hours_);
      update_sensor(this->outdoor_expansion_valve_pulse_rate_sensor_, &opdatas->outdoor_expansion_valve_pulse_rate_);
      update_sensor(this->energy_used_sensor_, &opdatas->energy_used_);
#ifdef USE_BINARY_SENSOR
      update_binary_sensor(this->defrosting_binary_sensor_, &opdatas->defrosting_);
#endif

      opdatas->value_semaphore_give();
    }

    /// Transmit the state of this climate controller.
    void control(const climate::ClimateCall& call) override
    {
        if (call.get_target_temperature().has_value()) {
          mhi_ac::spi_state.target_temperature_set(*call.get_target_temperature());
        }

        if (call.get_mode().has_value()) {
          auto mode = *call.get_mode();

          if(mode == climate::CLIMATE_MODE_OFF) {
            mhi_ac::spi_state.power_set(mhi_ac::ACPower::power_off);
          } else {
            mhi_ac::spi_state.power_set(mhi_ac::ACPower::power_on);
          }

          switch (mode) {
          case climate::CLIMATE_MODE_COOL:
            mhi_ac::spi_state.mode_set(mhi_ac::ACMode::mode_cool);
            break;
          case climate::CLIMATE_MODE_HEAT:
            mhi_ac::spi_state.mode_set(mhi_ac::ACMode::mode_heat);
            break;
          case climate::CLIMATE_MODE_DRY:
            mhi_ac::spi_state.mode_set(mhi_ac::ACMode::mode_dry);
            break;
          case climate::CLIMATE_MODE_FAN_ONLY:
            mhi_ac::spi_state.mode_set(mhi_ac::ACMode::mode_fan);
            break;
          case climate::CLIMATE_MODE_HEAT_COOL:
            mhi_ac::spi_state.mode_set(mhi_ac::ACMode::mode_auto);
            break;
          default:
            break;
          }
        }

        if (call.get_custom_fan_mode().has_value()) {
          auto fan_mode = *call.get_custom_fan_mode();
          if(fan_mode == custom_fan_ultra_low) {
            mhi_ac::spi_state.fan_set(mhi_ac::ACFan::speed_1);
          } else {
              ESP_LOGW(TAG, "Unsupported custom fan mode: %s", fan_mode.c_str());
          }
        } else if (call.get_fan_mode().has_value()) {
          auto fan_mode = *call.get_fan_mode();
          switch(fan_mode) {
            case climate::CLIMATE_FAN_AUTO:
              mhi_ac::spi_state.fan_set(mhi_ac::ACFan::speed_auto);
              break;
            case climate::CLIMATE_FAN_HIGH:
              mhi_ac::spi_state.fan_set(mhi_ac::ACFan::speed_4);
              break;
            case climate::CLIMATE_FAN_MEDIUM:
              mhi_ac::spi_state.fan_set(mhi_ac::ACFan::speed_3);
              break;
            case climate::CLIMATE_FAN_LOW:
              mhi_ac::spi_state.fan_set(mhi_ac::ACFan::speed_2);
              break;
            default:
              ESP_LOGW(TAG, "Unrecognised fan mode received");
          }
        }
    }

    /// Return the traits of this controller.
    climate::ClimateTraits traits() override
    {
        auto traits = climate::ClimateTraits();
        traits.set_supports_current_temperature(true);
        traits.set_supported_modes({ CLIMATE_MODE_OFF, CLIMATE_MODE_HEAT_COOL, CLIMATE_MODE_COOL, CLIMATE_MODE_HEAT, CLIMATE_MODE_DRY, CLIMATE_MODE_FAN_ONLY });
        traits.set_supports_action(true);
        traits.set_supports_two_point_target_temperature(false);
        traits.set_visual_min_temperature(this->minimum_temperature_);
        traits.set_visual_max_temperature(this->maximum_temperature_);
        traits.set_visual_temperature_step(this->temperature_step_);
        traits.set_visual_current_temperature_step(0.25);
        traits.add_supported_custom_fan_mode(custom_fan_ultra_low);
        traits.set_supported_fan_modes({ CLIMATE_FAN_LOW, CLIMATE_FAN_MEDIUM, CLIMATE_FAN_HIGH, CLIMATE_FAN_AUTO });
        //traits.set_supported_swing_modes({ CLIMATE_SWING_VERTICAL });
        return traits;
    }

    const float minimum_temperature_ { 18.0f };
    const float maximum_temperature_ { 30.0f };
    // Although the hardware accepts temperatures in steps of 0.5, it
    // effectively is per 1 degree on most units:
    // https://github.com/absalom-muc/MHI-AC-Ctrl/issues/81
    // But not all: https://github.com/hberntsen/mhi-ac-ctrl-esp32-c3/issues/14#issue-2828862442, so you an override it
    // if needed in the esphome yaml
    const float temperature_step_ { 1.0f };
    const std::string custom_fan_ultra_low = std::string("Ultra Low");

    SUB_SENSOR(climate_current_temperature)

    SUB_SENSOR(operation_data_timeouts)
    SUB_SENSOR(operation_data_visible_timeouts)
    SUB_SENSOR(set_temperature)
    SUB_SENSOR(return_air_temperature)
    SUB_SENSOR(indoor_u_bend_temperature)
    SUB_SENSOR(indoor_capillary_temperature)
    SUB_SENSOR(indoor_suction_header_temperature)
    SUB_SENSOR(indoor_fan_speed)
    SUB_SENSOR(indoor_total_run_hours)
    SUB_SENSOR(outdoor_air_temperature)
    SUB_SENSOR(outdoor_heat_exchanger_temperature_1)
    SUB_SENSOR(compressor_frequency)
    SUB_SENSOR(discharge_pipe_temperature)
    SUB_SENSOR(current)
    SUB_SENSOR(compressor_discharge_pipe_super_heat_temperature)
    SUB_SENSOR(compressor_protection_state_number)
    SUB_SENSOR(outdoor_fan_speed)
    SUB_SENSOR(compressor_total_run_hours)
    SUB_SENSOR(outdoor_expansion_valve_pulse_rate)
    SUB_SENSOR(energy_used)
#ifdef USE_BINARY_SENSOR
    SUB_BINARY_SENSOR(defrosting)
#endif
    SUB_SENSOR(external_room_temperature)

protected:
#ifdef USE_SELECT
    MhiVanesUD *vanes_ud_select_ = nullptr;
    MhiVanesLR *vanes_lr_select_ = nullptr;
#endif
    MhiFrameErrors *frame_errors_sensor_ = nullptr;
    MhiIntegratedTotalEnergy *integrated_total_energy_sensor_ = nullptr;
    MhiPower *power_sensor_ = nullptr;
    mhi_ac::Config ac_config_;
#ifdef USE_SWITCH
    MhiActiveMode *active_mode_switch_ = nullptr;
#endif

public:
#ifdef USE_SELECT
    void set_vanes_ud_select(MhiVanesUD *select) {
        this->vanes_ud_select_ = select;
    }

    void set_vanes_lr_select(MhiVanesLR *select) {
        this->vanes_lr_select_ = select;
    }
#endif

    void set_frame_errors_sensor(MhiFrameErrors *sensor) {
        this->frame_errors_sensor_ = sensor;
    }

    void set_integrated_total_energy_sensor(MhiIntegratedTotalEnergy *sensor) {
        this->integrated_total_energy_sensor_ = sensor;
    }

    void set_power_sensor(MhiPower *sensor) {
        this->power_sensor_ = sensor;
    }
#ifdef USE_SWITCH
    void set_active_mode_switch(MhiActiveMode *swi) {
      this->active_mode_switch_ = swi;
    }
#endif


};
