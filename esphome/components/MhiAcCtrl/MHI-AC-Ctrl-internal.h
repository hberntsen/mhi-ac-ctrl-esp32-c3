#pragma once

#define MHI_FRAME_LEN_SHORT         20
#define MHI_FRAME_LEN_LONG          33

#ifndef MIN
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#endif
#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

namespace mhi_ac {
namespace internal {
  enum FrameIndices {
    SB0 = 0,
    SB1 = 1,
    SB2 = 2,
    DB0 = 3, ///< mode DB0[4:2]
    DB1 = 4, ///< fan speed [1-3]
    DB2 = 5, ///< set room temp DB2[6:0]. T[°C]=DB2[6:0]/2 The resolution is 0.50°C
    DB3 = 6, ///< room temp DB3[7:0]. T[°C]=(DB3[7:0]-61)/4 The resolution is 0.25°C
    DB4 = 7, ///< error code
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
} // namespace mhi_ac
