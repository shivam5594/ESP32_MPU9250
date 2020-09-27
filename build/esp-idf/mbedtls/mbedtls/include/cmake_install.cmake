# Install script for directory: /Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/mbedtls" TYPE FILE PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ FILES
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/aes.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/aesni.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/arc4.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/aria.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/asn1.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/asn1write.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/base64.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/bignum.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/blowfish.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/bn_mul.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/camellia.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/ccm.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/certs.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/chacha20.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/chachapoly.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/check_config.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/cipher.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/cipher_internal.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/cmac.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/compat-1.3.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/config.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/ctr_drbg.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/debug.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/des.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/dhm.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/ecdh.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/ecdsa.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/ecjpake.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/ecp.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/ecp_internal.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/entropy.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/entropy_poll.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/error.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/gcm.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/havege.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/hkdf.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/hmac_drbg.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/md.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/md2.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/md4.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/md5.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/md_internal.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/memory_buffer_alloc.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/net.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/net_sockets.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/nist_kw.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/oid.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/padlock.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/pem.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/pk.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/pk_internal.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/pkcs11.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/pkcs12.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/pkcs5.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/platform.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/platform_time.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/platform_util.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/poly1305.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/ripemd160.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/rsa.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/rsa_internal.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/sha1.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/sha256.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/sha512.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/ssl.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/ssl_cache.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/ssl_ciphersuites.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/ssl_cookie.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/ssl_internal.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/ssl_ticket.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/threading.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/timing.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/version.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/x509.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/x509_crl.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/x509_crt.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/x509_csr.h"
    "/Users/shivamchauhan/Documents/esp/esp-idf/components/mbedtls/mbedtls/include/mbedtls/xtea.h"
    )
endif()

