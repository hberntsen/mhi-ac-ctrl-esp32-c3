#pragma once
#include <stdint.h>
#include <atomic>
#include "esp_timer.h"

// # Config
// ## pin defintions
#define GPIO_MOSI                   7
#define GPIO_MISO                   2
#define GPIO_SCLK                   GPIO_NUM_6
#define GPIO_CS_OUT                 GPIO_NUM_9
#define GPIO_CS_IN                  10

#define RCV_HOST                    SPI2_HOST

#define MHI_FRAME_LEN               20

// constants for the frame
#define MODE_MASK                   0x1C    // auto=0 in homekit        //DB0
#define MODE_AUTO                   0x00
#define MODE_DRY                    0x04
#define MODE_COOL                   0x08    // cool=2 in homekit
#define MODE_FAN                    0x0C
#define MODE_HEAT                   0x10    // heat=1 in homekit

#define PWR_MASK                    0x01    //DB0

#define FAN_DB1_MASK                 0x03    //DB1
#define FAN_DB6_MASK                0x10    //DB6 (for fan speed 4)
#define FAN_MASK                    7

#define FAN_SPEED_1                 0x00
#define FAN_SPEED_2                 0x01
#define FAN_SPEED_3                 0x02
#define FAN_SPEED_4                 0x10

#define HEAT_COOL_MASK              0x02            // DB13 0=Cooling, 1=Heating
#define COMP_ACTIVE_MASK            0x04            // DB13 0=Compressor Idle, 1=Compressor Running

#define MHI_NUM_FRAMES_PER_INTERVAL 20              // every 20 frames, MISO_frame[DB14] bit2 toggles

namespace mhi_ac {

enum class ACPower {
   power_off = 0,
   power_on = 1
};

enum class ACMode {
   mode_auto = 0b00000000,
   mode_dry = 0b00000100,
   mode_cool = 0b00001000,
   mode_fan = 0b00001100,
   mode_heat = 0b00010000,
   mode_unknown = 0xff
};

enum class ACFan {
   speed_1 = 0,
   speed_2 = 1,
   speed_3 = 2,
   speed_4 = 6,
   speed_auto = 7,
   unknown = 0xff,
};

enum class ACVanes {  // Vanes enum
   vanes_1 = 0, vanes_2 = 1, vanes_3 = 2, vanes_4 = 3, swing = 4
};

//class CallbackInterface_Status {
   //public: virtual void cbiStatusFunction(ACStatus status, int value) = 0;
//};

void mhi_ac_ctrl_core_init();
bool mhi_ac_ctrl_core_snapshot(uint32_t wait_time_ms);

void mhi_ac_ctrl_core_active_mode_set(bool state);
bool mhi_ac_ctrl_core_active_mode_get();

bool mhi_ac_ctrl_core_target_temperature_changed();
void mhi_ac_ctrl_core_target_temperature_set(float target_temperature);
float mhi_ac_ctrl_core_target_temperature_get();

bool mhi_ac_ctrl_core_power_changed();
void mhi_ac_ctrl_core_power_set(ACPower power);
ACPower mhi_ac_ctrl_core_power_get();

bool mhi_ac_ctrl_core_mode_changed();
void mhi_ac_ctrl_core_mode_set(ACMode mode);
ACMode mhi_ac_ctrl_core_mode_get();

bool mhi_ac_ctrl_core_fan_changed();
void mhi_ac_ctrl_core_fan_set(ACFan fan);
ACFan mhi_ac_ctrl_core_fan_get();
uint8_t mhi_ac_ctrl_core_fan_get_raw();

bool mhi_ac_ctrl_core_fan_old_changed();
uint8_t mhi_ac_ctrl_core_fan_old_get();

bool mhi_ac_ctrl_core_current_temperature_changed();
float mhi_ac_ctrl_core_current_temperature_get();

bool mhi_ac_ctrl_core_compressor_changed();
bool mhi_ac_ctrl_core_compressor_get();

bool mhi_ac_ctrl_core_heatcool_changed();
bool mhi_ac_ctrl_core_heatcool_get();

bool mhi_ac_ctrl_core_vanes_updown_changed();
ACVanes mhi_ac_ctrl_core_vanes_updown_get();
void mhi_ac_ctrl_core_vanes_updown_set(ACVanes new_state);

uint32_t mhi_ac_ctrl_core_frame_errors_get();

class MHIEnergy {
public:
    // Multiply by 14/51 * 10^-6 / 3600 for Wh
    // Should survive at least 32 years with 5kW without rolling over
    std::atomic_uint_least64_t total_energy;

    MHIEnergy(uint16_t new_voltage) {
        last_update = 0;
        voltage = new_voltage;
        current = 0;
        total_energy = 0;
    }

    // Multiply by 14/51 for current in A
    uint8_t get_current() {
        return current;
    }

    // Multiply by 14/51 for power in W
    uint32_t get_power() {
        if(current == 0) {
            // standby useage of 5W
            // x* 14/51 = 5
            return 19;
        }
        return current * voltage;
    }

    void set_current(uint8_t new_current) {
        update_total_energy();
        current = new_current;
    }

    void set_voltage(uint16_t new_voltage) {
        update_total_energy();
        voltage = new_voltage;
    }

    void update_total_energy() {
        add_energy(update_time() * get_power());
    }

    void add_energy(uint64_t additional_energy) {
        bool exchanged;
        uint64_t new_total;
        uint64_t current_total;
        // avoid huge spikes of 500Wh: 500 /( 14/51 * 10**(-6) / 3600)
        if(additional_energy > 6557142857142) {
            return;
        }
        do {
            uint64_t current_total = total_energy.load();
            new_total = current_total + additional_energy;
            exchanged = total_energy.compare_exchange_weak(current_total, new_total);
        } while(!exchanged);
    }

protected:
    uint64_t update_time() {
        uint64_t current_time = esp_timer_get_time();
        uint64_t time_diff = current_time - last_update;
        last_update = current_time;
        return time_diff;
    }

    uint64_t last_update;
    uint16_t voltage;
    uint8_t current;
};

extern MHIEnergy mhi_energy;

namespace internal {
    enum FrameIndices {
        SB0 = 0,
        SB1 = 1,
        SB2 = 2,
        DB0 = 3, ///< mode DB0[4:2]
        DB1 = 4, ///< fan speed [1-3]
        DB2 = 5, ///< set room temp DB2[6:0]. T[°C]=DB2[6:0]/2 The resolution is 0.50°C
        DB3 = 6, ///< room temp DB3[7:0]. T[°C]=(DB3[7:0]-61)/4 The resolution is 0.25°C
        DB6 = 9, ///< fan speed 4 DB6[6]
        DB9 = 12,
        DB10 = 13,
        DB11 = 14,
        DB12 = 15,
        DB13 = 16, ///< compressor status. DB13[0] AC is on, DB13[1] AC is in heat mode, DB13[2]  compressor running/idle
        DB14 = 17, ///< used on MISO toggle clock bit every 20 frames
        CBH = 18,
        CBL = 19,
        FRAME_LEN = MHI_FRAME_LEN
    };

} // namespace internal

} // namespace mhi_ac
