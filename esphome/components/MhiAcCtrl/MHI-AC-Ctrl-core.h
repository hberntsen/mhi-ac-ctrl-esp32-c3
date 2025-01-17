#pragma once
#include <stdint.h>
#include <atomic>
#include <array>
#include "esp_timer.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// # Config
#define RCV_HOST                    SPI2_HOST

#define MHI_FRAME_LEN_SHORT         20
#define MHI_FRAME_LEN_LONG          33

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

#ifndef MIN
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#endif
#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

namespace mhi_ac {
struct Config {
  bool use_long_frame;
  uint8_t mosi_pin;
  uint8_t miso_pin;
  uint8_t sclk_pin;
  uint8_t cs_in_pin;
  uint8_t cs_out_pin;
};

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

enum class ACVanesUD {  // Vanes enum
   Up = 0, UpCenter = 1, CenterDown = 2, Down = 3, Swing = 4, SeeIRRemote = 255
};

enum class ACVanesLR {
  Left = 0, LeftCenter = 1, Center = 2, CenterRight = 3, Right = 4, Wide = 5, Spot = 6, Swing = 8
};

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
    DB15,
    DB16,
    DB17,
    DB18,
    DB19,
    DB20,
    DB21,
    DB22,
    DB23,
    DB24,
    DB25,
    DB26,
    CBL2,
    FRAME_LEN = MHI_FRAME_LEN_LONG
  };
} // namespace internal



void init(const Config& config);

void active_mode_set(bool state);
bool active_mode_get();


uint32_t frame_errors_get();

class SpiState {
public:
//                           sb0   sb1   sb2   db0   db1   db2   db3   db4   db5   db6   db7   db8   db9   db10  db11  
  SpiState():  miso_frame_{ 0xAA, 0x00, 0x07, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 
//                          db12  db13  db14  chkH  chkL  db15  db16  db17  db18  db19  db20  db21  db22  db23  db24  
                            0xff, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 
//                          db25  db26 (chk2L not needed)
                            0xff, 0xff } {
    miso_semaphore_handle_ = xSemaphoreCreateMutexStatic( &this->miso_semaphore_buffer_ );
    snapshot_semaphore_handle_ = xSemaphoreCreateBinaryStatic( &this->snapshot_semaphore_buffer_ );
  }

  bool update_snapshot(uint32_t wait_time_ms);
  void use_long_frame(bool long_frame_enabled);

  bool target_temperature_changed() const;
  void target_temperature_set(float target_temperature);
  float target_temperature_get() const;

  bool power_changed() const;
  void power_set(ACPower power);
  ACPower power_get() const;

  bool mode_changed() const;
  void mode_set(ACMode mode);
  ACMode mode_get() const;

  bool fan_changed() const;
  void fan_set(ACFan fan);
  ACFan fan_get() const;

  bool current_temperature_changed() const;
  float current_temperature_get() const;

  bool compressor_changed() const;
  bool compressor_get() const;

  bool heatcool_changed() const;
  bool heatcool_get() const;

  bool vanes_updown_changed() const;
  ACVanesUD vanes_updown_get() const;
  void vanes_updown_set(ACVanesUD new_state);

  bool vanes_leftright_changed() const;
  ACVanesLR vanes_leftright_get() const;
  void vanes_leftright_set(ACVanesLR new_state);

  bool three_d_auto_changed() const;
  bool three_d_auto_get() const;
  void three_d_auto_set(bool new_state);

  std::array<uint8_t, internal::CBL2> miso_frame_;
  std::array<uint8_t, internal::DB26> mosi_frame_snapshot_;
  std::array<uint8_t, internal::DB26> mosi_frame_snapshot_prev_;
  SemaphoreHandle_t miso_semaphore_handle_;
  StaticSemaphore_t miso_semaphore_buffer_;
  SemaphoreHandle_t snapshot_semaphore_handle_;
  StaticSemaphore_t snapshot_semaphore_buffer_;
};

class Energy {
public:
    // Multiply by 14/51 * 10^-6 / 3600 for Wh
    // Should survive at least 32 years with 5kW without rolling over
    std::atomic_uint_least64_t total_energy;

    Energy(uint16_t new_voltage) {
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

extern Energy energy;
extern SpiState spi_state;

} // namespace mhi_ac
