#ifndef wolfcrypt_user_settings_h
#define wolfcrypt_user_settings_h

#ifdef IDF_TARGET_ESP32
    //From maximkulkin/esp-wolfssl/tree/master
    // From WolfSSL CMakeLists and HomeKit CMakeLists
    #define WOLFSSL_SHA512
    #define WOLFCRYPT_HAVE_SRP
    #define WOLFSSL_BASE64_ENCODE
    #define NO_SHA
    #define NO_MD5
    #define HAVE_CURVE25519
    #define HAVE_HKDF
    #define HAVE_CHACHA
    #define HAVE_POLY1305
    #define HAVE_ED25519
    #define NO_SESSION_CACHE
    #define USE_WOLFSSL_MEMORY
    #define RSA_LOW_MEMORY
    #define GCM_SMALL
    #define USE_SLOW_SHA512
    #define WOLFCRYPT_ONLY
//    #define CURVE25519_SMALL          // set with CONFIG_HOMEKIT_SMALL
//    #define ED25519_SMALL             //
    
    #define WOLFSSL_ESPIDF

    #include <esp_system.h>

    static inline int hwrand_generate_block(uint8_t *buf, size_t len) {
        int i;
        for (i=0; i+4 < len; i+=4) {
            *((uint32_t*)buf) = esp_random();
            buf += 4;
        }
        if (i < len) {
            uint32_t r = esp_random();
            while (i < len) {
                *buf++ = r;
                r >>= 8;
                i++;
            }
        }
        return 0;
    }

    #define CUSTOM_RAND_GENERATE_BLOCK hwrand_generate_block

#elif defined(IDF_TARGET_ESP8266)

    #include "esp_libc.h"

    static inline int hwrand_generate_block(uint8_t *buf, size_t len) {
        os_get_random(buf, len);
        return 0;
    }

    #define CUSTOM_RAND_GENERATE_BLOCK hwrand_generate_block
    
    // From Mixiaoxiao/Arduino-HomeKit-ESP8266/blob/master/src
    // Originally from RavenSystem/esp-homekit-devices/blob/master/external_libs/wolfssl/user_settings.h
//    #define FREERTOS                      // defined in WOLFSSL_ESPIDF
//    #define WOLFSSL_LWIP                  // *
//    #define NO_WOLFSSL_DIR                // *
    #define WC_NO_HARDEN                    // removes the 'For timing resistance / side-channel attack prevention' warning
//    #define SINGLE_THREADED
//    #define NO_INLINE                     // increased bin size
//    #define NO_WOLFSSL_MEMORY             // if WOLFSSL_ESPIDF is defined, do not use this 
                                            // it causes a redefinition of XREALLOC
//    #define NO_WOLFSSL_SMALL_STACK        // increases size by 440 bytes

    // *********  Which he added the following ***********

    #define WOLFSSL_SHA512
    #define WOLFCRYPT_HAVE_SRP   
    #define WOLFSSL_BASE64_ENCODE
    #define NO_SHA
    #define NO_MD5
    #define HAVE_CURVE25519
    #define HAVE_HKDF
    #define HAVE_CHACHA
    #define HAVE_POLY1305
    #define HAVE_ED25519
    #define NO_SESSION_CACHE                    // These are extra in the ESP-IDF
    #define USE_WOLFSSL_MEMORY                  // that esp-homekit had defined (above)
    #define RSA_LOW_MEMORY                      //
    #define GCM_SMALL                           // Uses 9KB less flash, and no noticeable
    #define USE_SLOW_SHA512                     // difference in pairing
    #define WOLFCRYPT_ONLY
//    #define CURVE25519_SMALL                  // set with CONFIG_HOMEKIT_SMALL
//    #define ED25519_SMALL                     //  decreases size by 70KB, but pairing time almost doubles

    #define WOLFSSL_ESPIDF                      // This was also in the ESP-IDF defines
    /* Note; if defined WOLFSSL_ESPIDF, then
        #define FREERTOS
        #define WOLFSSL_LWIP
        #define NO_WRITEV
        #define SIZEOF_LONG_LONG 8
        #define NO_WOLFSSL_DIR
        #define WOLFSSL_NO_CURRDIR
        #define TFM_TIMING_RESISTANT
        #define ECC_TIMING_RESISTANT
        #define WC_RSA_BLINDING
    */

    #define NO_ASN                              // Reduces size by 2200 bytes
    #define NO_AES                              // *
    #define NO_RC4                              // *
    #define NO_RSA                              // *
    #define NO_SHA256                           // *
    #define NO_DH                               // *
    #define NO_DSA                              // *

//    #define NO_CODING                         // Causes undefined reference to `Base64_Encode_NoNl'
//    #define WC_NO_HASHDRBG                    // no effect as CUSTOM_RAND_GENERATE_BLOCK is defined

//    #define MP_LOW_MEM
    //see integer.c
    //default winsize=5(MP_LOW_MEM), but ram(heap) is not sufficient!
    //winsize of {2,3,4,5} are same performance
    //lower winsize, lower ram required
//  #define ESP_INTEGER_WINSIZE 2

    //winsize=3 & mp_exptmod_fast : ram(heap) is not sufficient
    //force use s_mp_exptmod (lower memory), and smiller performance with mp_exptmod_fast
//  #define ESP_FORCE_S_MP_EXPTMOD
    //winsize = 5 & mp_exptmod_fast 最快，Pair Verify Step 2/2 = 10s左右
    //winsize = 6 heap不够

//  #define MP_16BIT 



    /*
    // From ESP8266 RTOS SDK
    #define WOLFSSL_LWIP
    #define NO_WRITEV
    #define NO_WOLFSSL_DIR
    #define NO_INLINE
    #define NO_WOLFSSL_MEMORY
    #define HAVE_PK_CALLBACKS
    #define WOLFSSL_KEY_GEN
    #define WOLFSSL_RIPEMD
    #define USE_WOLFSSL_IO
    #define WOLFSSL_STATIC_RSA
    #define NO_DH
    #define NO_MD4
    #define NO_DES3
    #define NO_DSA
    #define NO_RC4
    #define NO_RABBIT
    #define HAVE_ECC
    #define HAVE_AES_ECB
    #define WOLFSSL_AES_DIRECT
    #define WC_NO_HARDEN
    #define FREERTOS
    #define WOLFSSL_TYPES
    #define NO_FILESYSTEM
    #define WOLFSSL_ALT_CERT_CHAINS
    #define WOLFSSL_ALLOW_TLSV10
    #define WOLFSSL_SMALL_STACK
    #define SMALL_SESSION_CACHE
    #define OPENSSL_EXTRA

    #define SSL_CTX_use_certificate_ASN1(ctx,len,buf) wolfSSL_CTX_use_certificate_buffer(ctx,buf,len,WOLFSSL_FILETYPE_PEM)
    #define SSL_CTX_use_PrivateKey_ASN1(type,ctx,buf,len) wolfSSL_CTX_use_PrivateKey_buffer(ctx,buf,len, WOLFSSL_FILETYPE_PEM)
    #define SSL_CTX_load_verify_buffer(ctx,buf,len) wolfSSL_CTX_load_verify_buffer(ctx,buf,len, WOLFSSL_FILETYPE_PEM)

    #ifdef WOLFSSL_TYPES
        #ifndef byte
            typedef unsigned char  byte;
        #endif
        typedef unsigned short word16;
        typedef unsigned int   word32;
        typedef byte           word24[3];
    #endif

    #ifndef CUSTOM_RAND_GENERATE_BLOCK
    #include "esp_libc.h"
        // To use define the following:
        #define CUSTOM_RAND_GENERATE_BLOCK os_get_random
    #endif
    */


#endif


#endif // wolfcrypt_user_settings_h