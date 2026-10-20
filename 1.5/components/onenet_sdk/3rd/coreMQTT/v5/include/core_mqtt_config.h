/**
 * Copyright (c), 2012~2024 iot.10086.cn All Rights Reserved
 *
 * @file core_mqtt_config.h
 * @brief Custom configuration for coreMQTT library used in OneNET SDK
 */

#ifndef CORE_MQTT_CONFIG_H_
#define CORE_MQTT_CONFIG_H_

#include <stdio.h>
#include "log.h"

/*-------------------- Protocol / buffer sizing --------------------*/

/**
 * @brief Maximum number of vectors in subscribe and unsubscribe packet.
 */
#define MQTT_SUB_UNSUB_MAX_VECTORS    ( 4U )

/*-------------------- Connect handshake --------------------*/

/**
 * @brief Number of retries for receiving CONNACK when timeoutMs is 0.
 */
#define MQTT_MAX_CONNACK_RECEIVE_RETRY_COUNT    ( 10U )

/*-------------------- Keep-alive and idle timing --------------------*/

/**
 * @brief Maximum ms to wait for PINGRESP after PINGREQ.
 */
#define MQTT_PINGRESP_TIMEOUT_MS    ( 5000U )

/**
 * @brief Maximum ms of TX inactivity before sending PINGREQ.
 */
#define PACKET_TX_TIMEOUT_MS    ( 30000U )

/**
 * @brief Maximum ms of RX inactivity before sending PINGREQ.
 */
#define PACKET_RX_TIMEOUT_MS    ( 30000U )

/*-------------------- Transport I/O timing --------------------*/

/**
 * @brief Maximum ms to wait between partial reads of an incoming packet.
 */
#define MQTT_RECV_POLLING_TIMEOUT_MS    ( 100U )

/**
 * @brief Maximum ms to spend sending a single MQTT packet.
 */
#define MQTT_SEND_TIMEOUT_MS    ( 20000U )

/*-------------------- Logging --------------------*/

#define LogError( message )   do { loge message; } while( 0 )
#define LogWarn(  message )   do { logi message; } while( 0 )
#define LogInfo(  message )   do { logi message; } while( 0 )
// #define LogDebug( message )   do { logd message; } while( 0 )
// #define LogTrace( message )   do { logd message; } while( 0 )

#endif /* ifndef CORE_MQTT_CONFIG_H_ */
