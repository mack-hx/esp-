# This is an ESP-IDF component - do NOT use cmake_minimum_required or project() commands here
# Those commands must only be in the top-level CMakeLists.txt

include(${CMAKE_SOURCE_DIR}/cmake/vars.cmake)
set(TARGET ${CMAKE_PROJECT_NAME})

# Note:
# If the device created on the platform uses the MQTT 5.0 protocol, this must be ON.
# If it uses the MQTT protocol, this must be OFF.
# Otherwise, a protocol mismatch will prevent connection.
option(CONFIG_MQTT_VERSION_5 "Enable MQTT 5.0 support" ON)

include_directories(
    common
    3rd/cJSON
    onenet/platforms/include
    onenet/utils
    3rd/wolfssl/wolfssl-3.15.3
    onenet/security/tls/wolfssl
    onenet/tm
)

file(GLOB src
    common/*.c
    3rd/cJSON/cJSON.c
    onenet/platforms/linux/osl_linux.c
    onenet/platforms/linux/uart_linux.c
    onenet/platforms/linux/time_linux.c
    onenet/platforms/linux/tcp_linux.c
    onenet/platforms/linux/udp_linux.c
    onenet/utils/dev_token.c
    3rd/wolfssl/wolfssl-3.15.3/wolfcrypt/src/coding.c
    3rd/wolfssl/wolfssl-3.15.3/wolfcrypt/src/hash.c
    3rd/wolfssl/wolfssl-3.15.3/wolfcrypt/src/hmac.c
    3rd/wolfssl/wolfssl-3.15.3/wolfcrypt/src/logging.c
    3rd/wolfssl/wolfssl-3.15.3/wolfcrypt/src/md5.c
    3rd/wolfssl/wolfssl-3.15.3/wolfcrypt/src/misc.c
    3rd/wolfssl/wolfssl-3.15.3/wolfcrypt/src/sha.c
    3rd/wolfssl/wolfssl-3.15.3/wolfcrypt/src/sha256.c
    onenet/tm/aiot_tm_api.c
    onenet/tm/tm_data.c
    onenet/tm/tm_onejson.c
    examples/things_model/tm_user.c
    examples/things_model/main.c
    onenet/tm/tm_mqtt.c
    onenet/tm/tm_subdev.c
    onenet/tm/dev_discov.c
    3rd/wolfssl/wolfssl-3.15.3/wolfcrypt/src/aes.c
    3rd/wolfssl/wolfssl-3.15.3/wolfcrypt/src/asn.c
    3rd/wolfssl/wolfssl-3.15.3/wolfcrypt/src/integer.c
    3rd/wolfssl/wolfssl-3.15.3/wolfcrypt/src/random.c
    3rd/wolfssl/wolfssl-3.15.3/wolfcrypt/src/rsa.c
    3rd/wolfssl/wolfssl-3.15.3/wolfcrypt/src/ecc.c
    3rd/wolfssl/wolfssl-3.15.3/wolfcrypt/src/curve25519.c
    3rd/wolfssl/wolfssl-3.15.3/wolfcrypt/src/fe_operations.c
    3rd/wolfssl/wolfssl-3.15.3/wolfcrypt/src/wolfmath.c
    3rd/wolfssl/wolfssl-3.15.3/wolfcrypt/src/wc_port.c
    3rd/wolfssl/wolfssl-3.15.3/src/internal.c
    3rd/wolfssl/wolfssl-3.15.3/src/keys.c
    3rd/wolfssl/wolfssl-3.15.3/src/ssl.c
    3rd/wolfssl/wolfssl-3.15.3/src/tls.c
    3rd/wolfssl/wolfssl-3.15.3/src/wolfio.c
    onenet/security/tls/tls.c
)

add_definitions(
   -w # hide warnings
   # -g # add debug info
   -DPLAT_HAVE_STDINT=1
   -DSDK_PAYLOAD_LEN=4096
   -DSDK_SEND_BUF_LEN=4096
   -DSDK_RECV_BUF_LEN=4096
   -DSDK_REQUEST_TIMEOUT=4096
   -DSDK_ACCESS_LIFE_TIME=120
   # -DLOG_LEVEL=LOG_LEVEL_INFO
   -DGET_ICCID
   -DSTATIC_ICCID="898604293620C0000970"
   -DCONFIG_PLAT_ARCH_64BIT=1
   -DSDK_TM_VERSION="1.0"
   -DCONFIG_TM_GATEWAY=1
   -DSDK_USE_MQTTS
   # -DCONFIG_NETWORK_TLS=1
   -DSUBDEV_PRODUCT_ID=""
   -DSUBDEV_DEVICE_NAME=""
   -DSUBDEV_ACCESS_KEY=""
   -DSDK_MQTTS_TLS_CERT="${SDK_MQTTS_TLS_CERT}"
   -DIOT_MQTT5_SERVER_ADDR_TLS="mqttv5tls.heclouds.com"
   -DIOT_MQTT5_SERVER_PORT_TLS=8883
   -DIOT_MQTT5_SERVER_ADDR="mqttv5.heclouds.com"
   -DIOT_MQTT5_SERVER_PORT=1883
   -DIOT_MQTT_SERVER_ADDR_TLS="mqttstls.heclouds.com"
   -DIOT_MQTT_SERVER_PORT_TLS=8883
   -DIOT_MQTT_SERVER_ADDR="mqtts.heclouds.com"
   -DIOT_MQTT_SERVER_PORT=1883
)

if(CONFIG_MQTT_VERSION_5)
include_directories(
    onenet/protocols/mqtt
    3rd/coreMQTT/v5/include
    3rd/coreMQTT/v5/interface
    onenet/security/tls
)
file(GLOB mqttsrc
    3rd/coreMQTT/v5/core_mqtt.c
    3rd/coreMQTT/v5/core_mqtt_serializer.c
    3rd/coreMQTT/v5/core_mqtt_serializer_private.c
    3rd/coreMQTT/v5/core_mqtt_state.c
    3rd/coreMQTT/v5/core_mqtt_prop_serializer.c
    3rd/coreMQTT/v5/core_mqtt_prop_deserializer.c
    onenet/protocols/mqtt/coreMQTT/mqtt_clientv5.c
)
list(APPEND src ${mqttsrc})
add_definitions(
   # -DMQTT5_AUTO_TOPIC_ALIAS
   -DCONFIG_MQTT_VERSION_5
)
else()
include_directories(
    onenet/protocols/mqtt
    3rd/coreMQTT/v3/include
    3rd/coreMQTT/v3/interface
    onenet/security/tls
)
file(GLOB mqttsrc
    3rd/coreMQTT/v3/core_mqtt.c
    3rd/coreMQTT/v3/core_mqtt_serializer.c
    3rd/coreMQTT/v3/core_mqtt_state.c
    onenet/protocols/mqtt/coreMQTT/mqtt_clientv3.c
)
list(APPEND src ${mqttsrc})
endif()

add_executable(${TARGET} ${src})
target_link_libraries(${TARGET} pthread ${libs})
