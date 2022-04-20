#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define SB0                         0
#define SB1                         1
#define SB2                         2
#define DB0                         3     // mode DB0[4:2]
#define DB1                         4     // fan speed [1-3]
#define DB2                         5     // set room temp DB2[6:0]. T[°C]=DB2[6:0]/2 The resolution is 0.50°C 
#define DB3                         6     // room temp DB3[7:0]. T[°C]=(DB3[7:0]-61)/4 The resolution is 0.25°C   
#define DB6                         9     // fan speed 4 DB6[6]
#define DB9                         12    
#define DB11                        14
#define DB13                        16    // compressor status. DB13[0] AC is on, DB13[1] AC is in heat mode, DB13[2]  compressor running/idle
#define DB14                        17    // used on MISO toggle clock bit every 20 frames
#define CBH                         18
#define CBL                         19

#define MODE_MASK                   0x1C    // auto=0 in homekit        //DB0
#define MODE_AUTO                   0x00
#define MODE_DRY                    0x04
#define MODE_COOL                   0x08    // cool=2 in homekit
#define MODE_FAN                    0x0C
#define MODE_HEAT                   0x10    // heat=1 in homekit

#define PWR_MASK                    0x01    //DB0

#define FAN_DB1_MASK                 0x03    //DB1
#define FAN_DB6_MASK                0x10    //DB6 (for fan speed 4) 

#define FAN_SPEED_1                 0x00
#define FAN_SPEED_2                 0x01
#define FAN_SPEED_3                 0x02
#define FAN_SPEED_4                 0x10    

#define HEAT_COOL_MASK              0x02            // DB13 0=Cooling, 1=Heating
#define COMP_ACTIVE_MASK            0x04            // DB13 0=Compressor Idle, 1=Compressor Running



#define GPIO_MOSI                   23
#define GPIO_MISO                   19   
#define GPIO_SCLK                   18

#define RCV_HOST                    SPI3_HOST



#define MHI_NUM_FRAMES_PER_INTERVAL 20              // every 20 frames, MISO_frame[DB14] bit2 toggles

#define TEMP_THRESHOLD_DIFF         2.0             // separates Heat and Cool thresholds by 2 degC in auto mode


#ifdef __cplusplus
}
#endif 