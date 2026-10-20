/**
 * Copyright (c), 2012~2018 iot.10086.cn All Rights Reserved
 * @file        tm_lwm2m.h
 * @brief       Thing model LwM2M Protocol
 */
#ifndef _TM_LWM2M_H_
#define _TM_LWM2M_H_

#include "plat_time.h"
#include "plat_osl.h"
#include "iot_api.h"
#include "log.h"
#include "nbiot_m2m.h"
#include "utils.h"

extern int tm_lwm2m_step(int timeout_ms);
extern int tm_lwm2m_login();
extern int32_t tm_lwm2m_send_packet(uint8_t *topic,uint8_t *payload,int payload_len,uint32_t timeout_ms);
extern int tm_lwm2m_deinit();
extern int tm_lwm2m_logout(uint32_t timeout_ms);
extern int tm_lwm2m_init(int32_t (*callback)(const uint8_t * /** data_name*/, uint8_t * /** data*/, uint32_t /** data_len*/));

#endif