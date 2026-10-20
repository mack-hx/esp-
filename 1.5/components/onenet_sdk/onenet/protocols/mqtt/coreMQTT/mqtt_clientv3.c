/**
 * Copyright (c), 2012~2024 iot.10086.cn All Rights Reserved
 *
 * @file mqtt_clientv3.c
 * @brief MQTT 3.1.1 client implementation based on coreMQTT v3 backend
 *
 * This file implements the same mqtt_api.h interface as mqtt_client.c (paho-mqtt),
 * but uses Amazon's coreMQTT v3 library for MQTT 3.1.1 protocol.
 * It allows the OneNET SDK data-stream layer (iot_mqtts.c) to work
 * seamlessly with the lightweight coreMQTT library.
 */

/*****************************************************************************/
/* Includes                                                                  */
/*****************************************************************************/
#include "mqtt_api.h"
#include "data_types.h"
#include "log.h"
#include "plat_osl.h"
#include "plat_tcp.h"
#include "plat_time.h"

#include <stdlib.h>
#include <string.h>

#if CONFIG_NETWORK_TLS
#include "tls.h"
#endif

/* coreMQTT v3 headers */
#include "core_mqtt.h"
#include "core_mqtt_state.h"

/*****************************************************************************/
/* Local Definitions ( Constant and Macro )                                  */
/*****************************************************************************/
#define MQTT_NETWORK_BUFFER_SIZE    SDK_RECV_BUF_LEN
#define MQTT_KEEP_ALIVE_INTERVAL    120   /* seconds */
#define OUTGOING_PUB_RECORD_COUNT   10
#define INCOMING_PUB_RECORD_COUNT   10
#define TRANSPORT_DEFAULT_SEND_TIMEOUT_MS   5000
#define TRANSPORT_DEFAULT_RECV_TIMEOUT_MS   5000

/*****************************************************************************/
/* Structures, Enum and Typedefs                                             */
/*****************************************************************************/

/**
 * @brief Network context wrapping the platform connection handle.
 */
struct NetworkContext
{
    osl_handle_t conn_handle;
    uint32_t     send_timeout_ms;
    uint32_t     recv_timeout_ms;
};

/**
 * @brief Internal MQTT object holding all state for the coreMQTT v3 backend.
 */
struct mqtt_core_obj_t
{
    MQTTContext_t       mqtt_context;
    MQTTFixedBuffer_t   network_buffer;
    uint8_t             mqtt_buf[ MQTT_NETWORK_BUFFER_SIZE ];
    size_t              mqtt_buf_size;
    NetworkContext_t    net_ctx;
    MQTTPubAckInfo_t    outgoing_records[ OUTGOING_PUB_RECORD_COUNT ];
    MQTTPubAckInfo_t    incoming_records[ INCOMING_PUB_RECORD_COUNT ];

    /* Message handler (from mqtt_subscribe) */
    char               *subed_topic;
    mqtt_message_handler user_handler;
    void               *user_handler_arg;

    uint8_t             connected;

    /* Keep net_cb for disconnect */
    struct mqtt_network  net_cb;
};

/*****************************************************************************/
/* Local Variables                                                           */
/*****************************************************************************/

/*****************************************************************************/
/* Function Implementation                                                   */
/*****************************************************************************/

/* ── Time function ──────────────────────────────────────────────────────── */

static uint32_t get_time_ms( void )
{
    return ( uint32_t ) time_count_ms();
}

/* ── Transport interface implementations ───────────────────────────────── */

static int32_t transport_send( NetworkContext_t * pNetworkContext,
                               const void * pBuffer,
                               size_t bytesToSend )
{
    int32_t total_sent = 0;
    int32_t rc = 0;
    osl_handle_t handle = pNetworkContext->conn_handle;
    uint32_t send_timeout_ms = pNetworkContext->send_timeout_ms;

    while( ( size_t ) total_sent < bytesToSend )
    {
#if CONFIG_NETWORK_TLS
        rc = tls_send( handle, ( void * )( ( const uint8_t * ) pBuffer + total_sent ),
                       ( uint32_t )( bytesToSend - total_sent ), send_timeout_ms );
#else
        rc = plat_tcp_send( handle, ( void * )( ( const uint8_t * ) pBuffer + total_sent ),
                            ( uint32_t )( bytesToSend - total_sent ), send_timeout_ms );
#endif

        if( rc < 0 )
        {
            return rc;
        }

        if( rc == 0 )
        {
            return 0;
        }

        total_sent += rc;
    }

    return total_sent;
}

static int32_t transport_recv( NetworkContext_t * pNetworkContext,
                               void * pBuffer,
                               size_t bytesToRecv )
{
    int32_t rc;
    uint32_t recv_timeout_ms = pNetworkContext->recv_timeout_ms;

#if CONFIG_NETWORK_TLS
    rc = tls_recv( pNetworkContext->conn_handle, pBuffer,
                   ( uint32_t ) bytesToRecv, recv_timeout_ms );
#else
    rc = plat_tcp_recv( pNetworkContext->conn_handle, pBuffer,
                        ( uint32_t ) bytesToRecv, recv_timeout_ms );
#endif

    if( rc < 0 )
    {
        return rc;
    }

    return rc;
}

/* ── MQTT event callback ───────────────────────────────────────────────── */

static void mqtt_event_callback( MQTTContext_t * pContext,
                                 MQTTPacketInfo_t * pPacketInfo,
                                 MQTTDeserializedInfo_t * pDeserializedInfo )
{
    struct mqtt_core_obj_t * obj = ( struct mqtt_core_obj_t * ) pContext;

    if( pPacketInfo == NULL || pDeserializedInfo == NULL )
    {
        return;
    }

    switch( pPacketInfo->type )
    {
        case MQTT_PACKET_TYPE_CONNACK:
        {
            logi( "CONNACK received" );
            break;
        }

        case MQTT_PACKET_TYPE_SUBACK:
        {
            logi( "SUBACK received" );
            break;
        }

        case MQTT_PACKET_TYPE_UNSUBACK:
        {
            logi( "UNSUBACK received" );
            break;
        }

        case MQTT_PACKET_TYPE_PUBLISH:
        {
            MQTTPublishInfo_t * pubInfo = pDeserializedInfo->pPublishInfo;
            if( pubInfo != NULL && obj != NULL && obj->user_handler )
            {
                /* Convert coreMQTT publish to mqtt_message_t */
                struct mqtt_message_t msg;
                osl_memset( &msg, 0, sizeof( msg ) );
                msg.qos          = ( enum mqtt_qos_e ) pubInfo->qos;
                msg.retained     = pubInfo->retain;
                msg.dup          = pubInfo->dup;
                msg.payload      = ( uint8_t * ) pubInfo->pPayload;
                msg.payload_len  = ( uint32_t ) pubInfo->payloadLength;

                /* Duplicate topic string to null-terminate it */
                int data_len = pubInfo->topicNameLength;
                char * data  = NULL;
                if( NULL != ( data = osl_malloc( data_len + 1 ) ) )
                {
                    osl_memset( data, 0, data_len + 1 );
                    osl_memcpy( data, pubInfo->pTopicName, data_len );
                    obj->user_handler( obj->user_handler_arg, ( const uint8_t * ) data, &msg );
                    osl_free( data );
                }
            }
            break;
        }

        case MQTT_PACKET_TYPE_PUBACK:
        {
            logi( "PUBACK received for packetId=%u",
                  pDeserializedInfo->packetIdentifier );
            break;
        }

        case MQTT_PACKET_TYPE_PINGRESP:
        {
            logi( "PINGRESP received" );
            break;
        }

        default:
        {
            logi( "Unhandled packet type: 0x%02X", pPacketInfo->type );
            break;
        }
    }
}

/* ── Public API: mqtt_connect ──────────────────────────────────────────── */

void *mqtt_connect( const uint8_t *remote_addr, uint16_t remote_port,
                    const uint8_t *ca_cert, uint16_t ca_cert_len,
                    struct mqtt_param_t *mqtt_param, uint32_t timeout_ms )
{
    logd("Login Info:"
        "\n\t  Product ID : %s"
        "\n\t       Token : %s"
        "\n\t Server Addr : %s:%d",
        mqtt_param->username , mqtt_param->password, remote_addr, remote_port);

    struct mqtt_core_obj_t * obj = NULL;
    MQTTStatus_t status;
    bool session_present = false;
    osl_handle_t cd_hdl = 0;

    /* Allocate internal object */
    if( NULL == ( obj = osl_malloc( sizeof( struct mqtt_core_obj_t ) ) ) )
    {
        return NULL;
    }
    osl_memset( obj, 0, sizeof( struct mqtt_core_obj_t ) );
    obj->net_ctx.conn_handle      = -1;
    obj->net_ctx.send_timeout_ms  = TRANSPORT_DEFAULT_SEND_TIMEOUT_MS;
    obj->net_ctx.recv_timeout_ms  = TRANSPORT_DEFAULT_RECV_TIMEOUT_MS;

    obj->network_buffer.pBuffer = obj->mqtt_buf;
    obj->mqtt_buf_size          = MQTT_NETWORK_BUFFER_SIZE;
    obj->network_buffer.size    = MQTT_NETWORK_BUFFER_SIZE;

    cd_hdl = countdown_start( timeout_ms );

    /* ── Step 1: Network connect ─────────────────────────────────────── */
#if CONFIG_NETWORK_TLS
    obj->net_ctx.conn_handle = tls_connect( remote_addr, remote_port,
                                            ca_cert, ca_cert_len,
                                            countdown_left( cd_hdl ) );
#else
    obj->net_ctx.conn_handle = plat_tcp_connect( remote_addr, remote_port,
                                                  countdown_left( cd_hdl ) );
#endif

    if( obj->net_ctx.conn_handle < 0 )
    {
        logi( "connection to %s:%u failed!", remote_addr, remote_port );
        goto exit;
    }
    logi( "connected to %s:%u", remote_addr, remote_port );

    /* ── Step 2: MQTT_Init ───────────────────────────────────────────── */
    TransportInterface_t transport;
    transport.pNetworkContext = &obj->net_ctx;
    transport.send            = transport_send;
    transport.recv            = transport_recv;
    transport.writev          = NULL;

    status = MQTT_Init( &obj->mqtt_context, &transport, get_time_ms,
                        mqtt_event_callback, &obj->network_buffer );
    if( status != MQTTSuccess )
    {
        logi( "MQTT_Init failed: %s", MQTT_Status_strerror( status ) );
        goto exit1;
    }

    status = MQTT_InitStatefulQoS( &obj->mqtt_context,
                                   obj->outgoing_records, OUTGOING_PUB_RECORD_COUNT,
                                   obj->incoming_records, INCOMING_PUB_RECORD_COUNT );
    if( status != MQTTSuccess )
    {
        logi( "MQTT_InitStatefulQoS failed: %s", MQTT_Status_strerror( status ) );
        goto exit1;
    }

    /* ── Step 3: MQTT 3.1.1 CONNECT ─────────────────────────────────── */
    MQTTConnectInfo_t connectInfo = { 0 };
    connectInfo.cleanSession           = true;
    connectInfo.keepAliveSeconds       = MQTT_KEEP_ALIVE_INTERVAL;
    connectInfo.pClientIdentifier      = ( const char * ) mqtt_param->client_id;
    connectInfo.clientIdentifierLength = mqtt_param->client_id ? osl_strlen( mqtt_param->client_id ) : 0;

    if( mqtt_param->connect_flag & MQTT_CONNECT_FLAG_USERNAME )
    {
        connectInfo.pUserName      = ( const char * ) mqtt_param->username;
        connectInfo.userNameLength = mqtt_param->username ? osl_strlen( mqtt_param->username ) : 0;
    }
    if( mqtt_param->connect_flag & MQTT_CONNECT_FLAG_PASSWORD )
    {
        connectInfo.pPassword      = ( const char * ) mqtt_param->password;
        connectInfo.passwordLength = mqtt_param->password ? osl_strlen( mqtt_param->password ) : 0;
    }

    /* Will message support */
    MQTTPublishInfo_t * pWillInfo = NULL;
    MQTTPublishInfo_t willInfo = { 0 };
    if( mqtt_param->connect_flag & MQTT_CONNECT_FLAG_WILL )
    {
        pWillInfo = &willInfo;
        willInfo.pTopicName      = ( const char * ) mqtt_param->will_msg.will_topic;
        willInfo.topicNameLength = mqtt_param->will_msg.will_topic ? osl_strlen( mqtt_param->will_msg.will_topic ) : 0;
        willInfo.pPayload        = mqtt_param->will_msg.will_message;
        willInfo.payloadLength   = mqtt_param->will_msg.will_message_len;
        willInfo.qos             = mqtt_param->will_msg.qos;
        willInfo.retain          = ( mqtt_param->connect_flag & MQTT_CONNECT_FLAG_WILL_RETAIN ) ? true : false;
    }

    status = MQTT_Connect( &obj->mqtt_context, &connectInfo, pWillInfo,
                           countdown_left( cd_hdl ), &session_present );

    if( status != MQTTSuccess )
    {
        logi( "MQTT_Connect failed: %s", MQTT_Status_strerror( status ) );
        goto exit1;
    }

    obj->connected = 1;
    logi( "MQTT 3.1.1 CONNECT successful! session_present=%d", session_present );

    countdown_stop( cd_hdl );
    return obj;

exit1:
#if CONFIG_NETWORK_TLS
    tls_disconnect( obj->net_ctx.conn_handle );
#else
    plat_tcp_disconnect( obj->net_ctx.conn_handle );
#endif
exit:
    countdown_stop( cd_hdl );
    osl_free( obj );
    return NULL;
}

/* ── Public API: mqtt_yield ────────────────────────────────────────────── */

int32_t mqtt_yield( void *client, uint32_t timeout_ms )
{
    struct mqtt_core_obj_t * obj = ( struct mqtt_core_obj_t * ) client;

    if( obj == NULL || !obj->connected )
    {
        return -1;
    }

    if( timeout_ms > 0 )
    {
        obj->net_ctx.recv_timeout_ms = timeout_ms;
    }

    MQTTStatus_t status = MQTT_ProcessLoop( &obj->mqtt_context );

    if( status != MQTTSuccess && status != MQTTNeedMoreBytes && status != MQTTNoDataAvailable )
    {
        logi( "MQTT_ProcessLoop error: %s", MQTT_Status_strerror( status ) );
        obj->connected = 0;
        return -1;
    }

    return 0;
}

/* ── Public API: mqtt_publish ──────────────────────────────────────────── */

int32_t mqtt_publish( void *client, const uint8_t *topic,
                      struct mqtt_message_t *message, uint32_t timeout_ms )
{
    struct mqtt_core_obj_t * obj = ( struct mqtt_core_obj_t * ) client;

    if( obj == NULL || !obj->connected )
    {
        return -1;
    }

    ( void ) timeout_ms;
    MQTTStatus_t status;

    MQTTPublishInfo_t publishInfo = { 0 };
    publishInfo.qos             = message->qos;
    publishInfo.pTopicName      = ( const char * ) topic;
    publishInfo.topicNameLength = osl_strlen( topic );
    publishInfo.pPayload        = message->payload;
    publishInfo.payloadLength   = message->payload_len;
    publishInfo.retain          = message->retained ? true : false;

    uint16_t packet_id = ( publishInfo.qos > MQTTQoS0 ) ? MQTT_GetPacketId( &obj->mqtt_context ) : 0;

    status = MQTT_Publish( &obj->mqtt_context, &publishInfo, packet_id );

    if( status != MQTTSuccess )
    {
        logi( "MQTT_Publish failed: %s", MQTT_Status_strerror( status ) );
        return -1;
    }

    return 0;
}

/* ── Public API: mqtt_subscribe ────────────────────────────────────────── */

int32_t mqtt_subscribe( void *client, const uint8_t *topic, enum mqtt_qos_e qos,
                        mqtt_message_handler msg_handler, void *arg, uint32_t timeout_ms )
{
    struct mqtt_core_obj_t * obj = ( struct mqtt_core_obj_t * ) client;

    if( obj == NULL || !obj->connected )
    {
        return -1;
    }

    ( void ) timeout_ms;
    MQTTStatus_t status;

    /* Store handler for incoming messages */
    if( obj->subed_topic )
    {
        osl_free( obj->subed_topic );
    }
    obj->subed_topic = ( char * ) osl_malloc( osl_strlen( topic ) + 1 );
    if( obj->subed_topic )
    {
        osl_strcpy( ( uint8_t * ) obj->subed_topic, topic );
    }
    obj->user_handler     = msg_handler;
    obj->user_handler_arg = arg;

    MQTTSubscribeInfo_t subInfo = { 0 };
    subInfo.qos               = qos;
    subInfo.pTopicFilter      = ( const char * ) topic;
    subInfo.topicFilterLength = osl_strlen( topic );

    uint16_t packet_id = MQTT_GetPacketId( &obj->mqtt_context );

    status = MQTT_Subscribe( &obj->mqtt_context, &subInfo, 1, packet_id );
    if( status != MQTTSuccess )
    {
        logi( "MQTT_Subscribe failed: %s", MQTT_Status_strerror( status ) );
        return -1;
    }

    /* Wait for SUBACK with timeout protection */
    osl_handle_t sub_cd = countdown_start( timeout_ms );
    do
    {
        status = MQTT_ProcessLoop( &obj->mqtt_context );
        if( countdown_is_expired( sub_cd ) )
        {
            logi( "Subscribe timeout waiting for SUBACK" );
            countdown_stop( sub_cd );
            return -1;
        }
    } while( status == MQTTNeedMoreBytes );
    countdown_stop( sub_cd );

    if( status != MQTTSuccess )
    {
        logi( "ProcessLoop after subscribe failed: %s", MQTT_Status_strerror( status ) );
        return -1;
    }

    logi( "Subscribe confirmed, topic=%s", topic );
    return 0;
}

/* ── Public API: mqtt_unsubscribe ──────────────────────────────────────── */

int32_t mqtt_unsubscribe( void *client, const uint8_t *topic, uint32_t timeout_ms )
{
    struct mqtt_core_obj_t * obj = ( struct mqtt_core_obj_t * ) client;

    if( obj == NULL || !obj->connected )
    {
        return -1;
    }

    ( void ) timeout_ms;
    MQTTStatus_t status;

    MQTTSubscribeInfo_t unsubInfo = { 0 };
    unsubInfo.qos               = MQTTQoS0;
    unsubInfo.pTopicFilter      = ( const char * ) topic;
    unsubInfo.topicFilterLength = osl_strlen( topic );

    uint16_t packet_id = MQTT_GetPacketId( &obj->mqtt_context );

    status = MQTT_Unsubscribe( &obj->mqtt_context, &unsubInfo, 1, packet_id );
    if( status != MQTTSuccess )
    {
        logi( "MQTT_Unsubscribe failed: %s", MQTT_Status_strerror( status ) );
        return -1;
    }

    /* Wait for UNSUBACK with timeout protection */
    osl_handle_t unsub_cd = countdown_start( timeout_ms );
    do
    {
        status = MQTT_ProcessLoop( &obj->mqtt_context );
        if( countdown_is_expired( unsub_cd ) )
        {
            logi( "Unsubscribe timeout waiting for UNSUBACK" );
            countdown_stop( unsub_cd );
            return -1;
        }
    } while( status == MQTTNeedMoreBytes );
    countdown_stop( unsub_cd );

    if( status != MQTTSuccess )
    {
        logi( "ProcessLoop after unsubscribe failed: %s", MQTT_Status_strerror( status ) );
        return -1;
    }

    /* Clear handler if it matches */
    if( obj->subed_topic && osl_strcmp( ( const uint8_t * ) obj->subed_topic, topic ) == 0 )
    {
        osl_free( obj->subed_topic );
        obj->subed_topic   = NULL;
        obj->user_handler  = NULL;
        obj->user_handler_arg = NULL;
    }

    return 0;
}

/* ── Public API: mqtt_disconnect ───────────────────────────────────────── */

int32_t mqtt_disconnect( void *client, uint32_t timeout_ms )
{
    struct mqtt_core_obj_t * obj = ( struct mqtt_core_obj_t * ) client;

    ( void ) timeout_ms;

    if( obj )
    {
        if( obj->connected )
        {
            MQTT_Disconnect( &obj->mqtt_context );
            obj->connected = 0;
            logi( "MQTT 3.1.1 DISCONNECT sent" );
        }

        /* Disconnect network */
        if( obj->net_ctx.conn_handle >= 0 )
        {
#if CONFIG_NETWORK_TLS
            tls_disconnect( obj->net_ctx.conn_handle );
#else
            plat_tcp_disconnect( obj->net_ctx.conn_handle );
#endif
            obj->net_ctx.conn_handle = -1;
        }

        if( obj->subed_topic )
        {
            osl_free( obj->subed_topic );
            obj->subed_topic = NULL;
        }

        osl_free( obj );
    }

    return 0;
}

/* ── Public API: mqtt_set_default_message_handler ──────────────────────── */

int32_t mqtt_set_default_message_handler( void *client, mqtt_message_handler msg_handler, void *arg )
{
    struct mqtt_core_obj_t * obj = ( struct mqtt_core_obj_t * ) client;

    if( obj )
    {
        obj->user_handler     = msg_handler;
        obj->user_handler_arg = arg;
    }

    return 0;
}
