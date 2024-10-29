#include "MHI-AC-Ctrl-core.h"

#include "esphome/components/climate/climate.h"
#include "esphome/components/sensor/sensor.h"
#ifdef USE_SWITCH
#include "esphome/components/switch/switch.h"
#endif


using namespace esphome;
using namespace esphome::climate;
using namespace esphome::sensor;
#ifdef USE_SWITCH
using namespace esphome::switch_;
#endif
using namespace mhi_ac;

static const char* TAG = "mhi_ac_ctrl";

class MhiFrameErrors : public Sensor {
public:
    MhiFrameErrors() {
        this->publish_state(mhi_ac_ctrl_core_frame_errors_get());
    }

    void loop() {
        if(mhi_ac_ctrl_core_frame_errors_get() != this->get_raw_state()) {
            this->publish_state(mhi_ac_ctrl_core_frame_errors_get());
        }
    }
};

class MhiTotalEnergy : public Sensor {
public:
    MhiTotalEnergy() {
        this->total_energy_ = 0;
#if STORE_POWER_IN_PREFERENCES
        this->pref_ = global_preferences->make_preference<uint64_t>(this->get_object_id_hash());
        this->pref_.load(&this->total_energy_);
#endif
        mhi_energy.add_energy(this->total_energy_);

        this->total_energy_ = mhi_energy.total_energy.load();
        publish_total();
    }

    void loop() {
        uint64_t new_energy = mhi_energy.total_energy.load();
        if(new_energy != total_energy_) {
            total_energy_ = new_energy;
            publish_total();
        }
    }

    void publish_total() {
        this->publish_state(total_energy_ * (14.0f/(51000000.0f*3600)));
#if STORE_POWER_IN_PREFERENCES
        pref_.save(&total_energy_);
#endif
    }

protected:
    uint64_t total_energy_;
#if STORE_POWER_IN_PREFERENCES
    ESPPreferenceObject pref_;
#endif
};

class MhiPower : public Sensor {
public:
    MhiPower() {
        if(mhi_ac_ctrl_core_active_mode_get()) {
            last_power = mhi_energy.get_power();
            this->publish_power();
        }
    }

    void loop() {
        if(mhi_ac_ctrl_core_active_mode_get()) {
            uint32_t new_power = mhi_energy.get_power();
            if(new_power != last_power || std::isnan(get_raw_state())) {
                last_power = new_power;
                publish_power();
            }
        } else if(!std::isnan(get_raw_state()) && !mhi_ac_ctrl_core_active_mode_get()) {
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
protected:
    virtual void write_state(bool state) {
        mhi_ac_ctrl_core_active_mode_set(state);
        this->publish_state(state);
    }
};
#endif

#ifdef USE_SELECT
class MhiVanes : public select::Select {
public:
    MhiSwingMode() {
        traits.set_options({ "Swing", "1", "2", "3", "4" });
    }

    virtual void control(const std::string &value) {
        if(value == traits.get_options()[0]) {
            mhi_ac_ctrl_core_vanes_updown_set(ACVanes::swing);
        } else if(vanes == traits.get_options()[1]) {
            mhi_ac_ctrl_core_vanes_updown_set(ACVanes::vanes_1);
        } else if(vanes == traits.get_options()[2]) {
            mhi_ac_ctrl_core_vanes_updown_set(ACVanes::vanes_2);
        } else if(vanes == traits.get_options()[3]) {
            mhi_ac_ctrl_core_vanes_updown_set(ACVanes::vanes_3);
        } else if(vanes == traits.get_options()[4]) {
            mhi_ac_ctrl_core_vanes_updown_set(ACVanes::vanes_4);
        } else {
            ESP_LOGW(TAG, "Unknown vanes mode received: %s", value.c_str());
        }
    }

    void loop() {
        if(mhi_ac_ctrl_core_vanes_updown_changed()) {
            switch (mhi_ac_ctrl_core_vanes_updown_get()) {
                case ACVanes::swing:
                    publish_state(traits.get_options()[0]);
                    break;
                case ACVanes::vanes_1:
                    publish_state(traits.get_options()[1]);
                    break;
                case ACVanes::vanes_2:
                    publish_state(traits.get_options()[2]);
                    break;
                case ACVanes::vanes_3:
                    publish_state(traits.get_options()[3]);
                    break;
                case ACVanes::vanes_4:
                    publish_state(traits.get_options()[4]);
                    break;
            }
        }
    }
}
#endif

class MhiAcCtrl : public climate::Climate,
                  public Component {
public:
    void setup() override
    {
        auto restore = this->restore_state_();
        if (restore.has_value()) {
            restore->apply(this);
        } else {
            // restore from defaults
            //this->mode = climate::CLIMATE_MODE_OFF;
            //// initialize target temperature to some value so that it's not NAN
            //this->target_temperature = roundf(clamp(
                //this->current_temperature, this->minimum_temperature_, this->maximum_temperature_));
            //this->fan_mode = climate::CLIMATE_FAN_AUTO;
            //this->swing_mode = climate::CLIMATE_SWING_OFF;
        }

        //current_power.set_icon("mdi:current-ac");
        //current_power.set_unit_of_measurement("A");
        //current_power.set_accuracy_decimals(2);

        mhi_ac_ctrl_core_init();
    }

    void loop() override
    {
        bool publish_self_state = false;
        if(!mhi_ac_ctrl_core_snapshot(100)) {
            ESP_LOGW(TAG, "Snapshot timeout");
            return;
        }

        if(total_energy_sensor_)
            total_energy_sensor_->loop();
        if(power_sensor_)
            power_sensor_->loop();
        if(frame_errors_sensor_)
            frame_errors_sensor_->loop();
#ifdef USE_SELECT
        vanes.loop();
#endif

        if(mhi_ac_ctrl_core_target_temperature_changed()) {
            publish_self_state = true;
            this->target_temperature = mhi_ac_ctrl_core_target_temperature_get();
        }

        if(mhi_ac_ctrl_core_power_changed()
                || mhi_ac_ctrl_core_mode_changed()
                || mhi_ac_ctrl_core_compressor_changed()
                || mhi_ac_ctrl_core_heatcool_changed()) {
            publish_self_state = true;

            if(mhi_ac_ctrl_core_power_get() == ACPower::power_off) {
                this->mode = climate::CLIMATE_MODE_OFF;
                this->action = climate::CLIMATE_ACTION_OFF;
            } else {
                if(!mhi_ac_ctrl_core_compressor_get()) {
                    this->action = climate::CLIMATE_ACTION_IDLE;
                }

                switch(mhi_ac_ctrl_core_mode_get()) {
                case ACMode::mode_cool:
                    this->mode = climate::CLIMATE_MODE_COOL;
                    if(mhi_ac_ctrl_core_compressor_get())
                        this->action = climate::CLIMATE_ACTION_COOLING;
                    break;
                case ACMode::mode_heat:
                    this->mode = climate::CLIMATE_MODE_HEAT;
                    if(mhi_ac_ctrl_core_compressor_get())
                        this->action = climate::CLIMATE_ACTION_HEATING;
                    break;
                case ACMode::mode_dry:
                    this->mode = climate::CLIMATE_MODE_DRY;
                    if(mhi_ac_ctrl_core_compressor_get())
                        this->action = climate::CLIMATE_ACTION_DRYING;
                    break;
                case ACMode::mode_fan:
                    this->mode = climate::CLIMATE_MODE_FAN_ONLY;
                    this->action = climate::CLIMATE_ACTION_FAN;
                    break;
                case ACMode::mode_auto:
                    this->mode = climate::CLIMATE_MODE_HEAT_COOL;
                    if(mhi_ac_ctrl_core_compressor_get()) {
                        if(mhi_ac_ctrl_core_heatcool_get())
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

        if(mhi_ac_ctrl_core_fan_changed()) {
            publish_self_state = true;
            switch(mhi_ac_ctrl_core_fan_get()) {
                case ACFan::speed_1:
                    this->set_custom_fan_mode_(custom_fan_ultra_low);
                    break;
                case ACFan::speed_2:
                    this->set_fan_mode_(climate::CLIMATE_FAN_LOW);
                    break;
                case ACFan::speed_3:
                    this->set_fan_mode_(climate::CLIMATE_FAN_MEDIUM);
                    break;
                case ACFan::speed_4:
                    this->set_fan_mode_(climate::CLIMATE_FAN_HIGH);
                    break;
                case ACFan::speed_auto:
                    this->set_fan_mode_(climate::CLIMATE_FAN_AUTO);
                    break;
                default:
                    break;
            }
            if(fan_raw_sensor_)
                fan_raw_sensor_->publish_state(mhi_ac_ctrl_core_fan_get_raw());
        }

        if(fan_old_sensor_ && mhi_ac_ctrl_core_fan_old_changed()) {
            fan_old_sensor_->publish_state((uint8_t)mhi_ac_ctrl_core_fan_old_get());
        }

        if(mhi_ac_ctrl_core_current_temperature_changed()) {
            publish_self_state = true;
            this->current_temperature = mhi_ac_ctrl_core_current_temperature_get();
        }

        if(this->mode != climate::CLIMATE_MODE_HEAT && this->target_temperature < 18) {
            mhi_ac_ctrl_core_target_temperature_set(18);
            this->target_temperature = 18;
        }

        if(publish_self_state) {
            this->publish_state();
        }
    }

    void dump_config() override
    {
    }

#ifdef USE_SELECT
    std::vector<Select *> get_selects() {
        return {
            &vanes,
        };
    }
#endif

protected:
    /// Transmit the state of this climate controller.
    void control(const climate::ClimateCall& call) override
    {
        if (call.get_target_temperature().has_value()) {
            //auto target_temperature = *call.get_target_temperature();
            //auto mode = call.get_mode().value_or(this->mode);

            //if(mode != climate::CLIMATE_MODE_HEAT && target_temperature < 18) {
                //target_temperature = 18;
            //}
            //todo: do not immediately update here, do later?
            //
            //this->target_temperature = *call.get_target_temperature();
            //mhi_ac_ctrl_core_target_temperature_set(this->target_temperature);
            mhi_ac_ctrl_core_target_temperature_set(*call.get_target_temperature());
        }

        if (call.get_mode().has_value()) {
            //this->mode = *call.get_mode();
            auto mode = *call.get_mode();

            if(mode == climate::CLIMATE_MODE_OFF) {
                mhi_ac_ctrl_core_power_set(ACPower::power_off);
            } else {
                mhi_ac_ctrl_core_power_set(ACPower::power_on);
            }

            switch (mode) {
            case climate::CLIMATE_MODE_COOL:
                mhi_ac_ctrl_core_mode_set(ACMode::mode_cool);
                break;
            case climate::CLIMATE_MODE_HEAT:
                mhi_ac_ctrl_core_mode_set(ACMode::mode_heat);
                break;
            case climate::CLIMATE_MODE_DRY:
                mhi_ac_ctrl_core_mode_set(ACMode::mode_dry);
                break;
            case climate::CLIMATE_MODE_FAN_ONLY:
                mhi_ac_ctrl_core_mode_set(ACMode::mode_fan);
                break;
            case climate::CLIMATE_MODE_HEAT_COOL:
                mhi_ac_ctrl_core_mode_set(ACMode::mode_auto);
                break;
            default:
                break;
            }

        }

        if (call.get_custom_fan_mode().has_value()) {
            auto fan_mode = *call.get_custom_fan_mode();
            if(fan_mode == custom_fan_ultra_low) {
                mhi_ac_ctrl_core_fan_set(ACFan::speed_1);
            } else {
                ESP_LOGW(TAG, "Unsupported custom fan mode: %s", fan_mode.c_str());
            }
        } else if (call.get_fan_mode().has_value()) {
            auto fan_mode = *call.get_fan_mode();
            switch(fan_mode) {
                case climate::CLIMATE_FAN_AUTO:
                    mhi_ac_ctrl_core_fan_set(ACFan::speed_auto);
                    break;
                case climate::CLIMATE_FAN_HIGH:
                    mhi_ac_ctrl_core_fan_set(ACFan::speed_4);
                    break;
                case climate::CLIMATE_FAN_MEDIUM:
                    mhi_ac_ctrl_core_fan_set(ACFan::speed_3);
                    break;
                case climate::CLIMATE_FAN_LOW:
                    mhi_ac_ctrl_core_fan_set(ACFan::speed_2);
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
        traits.add_supported_custom_fan_mode(custom_fan_ultra_low);
        traits.set_supported_fan_modes({ CLIMATE_FAN_LOW, CLIMATE_FAN_MEDIUM, CLIMATE_FAN_HIGH, CLIMATE_FAN_AUTO });
        //traits.set_supported_swing_modes({ CLIMATE_SWING_VERTICAL });
        return traits;
    }

    const float minimum_temperature_ { 10.0f };
    const float maximum_temperature_ { 30.0f };
    // Although the hardware accepts temperatures in steps of 0.5, it
    // effectively is per 1 degree:
    // https://github.com/absalom-muc/MHI-AC-Ctrl/issues/81
    const float temperature_step_ { 1.0f };
    const std::string custom_fan_ultra_low = std::string("Ultra Low");

    SUB_SENSOR(fan_raw)
    SUB_SENSOR(fan_old)

protected:
#ifdef USE_SWITCH
    MhiActiveMode *active_mode_switch_;
#endif
    MhiFrameErrors *frame_errors_sensor_;
    MhiTotalEnergy *total_energy_sensor_;
    MhiPower *power_sensor_;

public:
#ifdef USE_SWITCH
    void set_active_mode_switch(MhiActiveMode *s) {
        this->active_mode_switch_ = s;
    }
#endif

    void set_frame_errors_sensor(MhiFrameErrors *sensor) {
        this->frame_errors_sensor_ = sensor;
    }

    void set_total_energy_sensor(MhiTotalEnergy *sensor) {
        this->total_energy_sensor_ = sensor;
    }

    void set_power_sensor(MhiPower *sensor) {
        this->power_sensor_ = sensor;
    }


#ifdef USE_SELECT
    MhiVanes vanes;
#endif
};
