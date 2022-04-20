set(wolfssl_VERSION "4.1.0")
set(wolfssl_THIRDPARTY_ROOT "wolfssl-${wolfssl_VERSION}")
 
set(COMPONENT_SRCDIRS "${wolfssl_THIRDPARTY_ROOT}/src/"
                      "${wolfssl_THIRDPARTY_ROOT}/wolfcrypt/src/"
                      "${wolfssl_THIRDPARTY_ROOT}/wolfcrypt/src/port/Espressif/"
                      "${wolfssl_THIRDPARTY_ROOT}/wolfcrypt/src/port/atmel/"
                      )
  
set(COMPONENT_SRCEXCLUDE
  "${wolfssl_THIRDPARTY_ROOT}/wolfcrypt/src/aes.c"
  "${wolfssl_THIRDPARTY_ROOT}/wolfcrypt/src/aes_asm.S"  
  "${wolfssl_THIRDPARTY_ROOT}/wolfcrypt/src/aes_gcm_asm.S"
  "${wolfssl_THIRDPARTY_ROOT}/wolfcrypt/src/chacha_asm.S"
  "${wolfssl_THIRDPARTY_ROOT}/wolfcrypt/src/fe_x25519_asm.S"
  "${wolfssl_THIRDPARTY_ROOT}/wolfcrypt/src/poly1305_asm.S"
  "${wolfssl_THIRDPARTY_ROOT}/wolfcrypt/src/sha256_asm.S"
  "${wolfssl_THIRDPARTY_ROOT}/wolfcrypt/src/sha512_asm.S"
  "${wolfssl_THIRDPARTY_ROOT}/wolfcrypt/src/sp_x86_64_asm.S"    
  "${wolfssl_THIRDPARTY_ROOT}/wolfcrypt/src/evp.c"
  "${wolfssl_THIRDPARTY_ROOT}/wolfcrypt/src/misc.c"
  "${wolfssl_THIRDPARTY_ROOT}/src/bio.c"
)

set(COMPONENT_ADD_INCLUDEDIRS
  "."
  "include/"
  "${wolfssl_THIRDPARTY_ROOT}"
)

set(COMPONENT_REQUIRES lwip)

idf_component_register(
  SRC_DIRS ${COMPONENT_SRCDIRS} EXCLUDE_SRCS ${COMPONENT_SRCEXCLUDE}
  INCLUDE_DIRS ${COMPONENT_ADD_INCLUDEDIRS}
  REQUIRES ${COMPONENT_REQUIRES}
)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} \
  -DWOLFSSL_USER_SETTINGS"
)

if(CONFIG_HOMEKIT_SMALL)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} \
    -DCURVE25519_SMALL \
    -DED25519_SMALL"
  )
endif()

if(${IDF_TARGET} STREQUAL "esp8266")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} \
    -DIDF_TARGET_ESP8266"
  )
elseif(${IDF_TARGET} STREQUAL "esp32")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} \
    -DIDF_TARGET_ESP32"
  )
else()
  message(FATAL_ERROR "user_settings.h only supports esp8266 and esp32 targets")
endif()