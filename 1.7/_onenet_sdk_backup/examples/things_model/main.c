/**
 * Copyright (c), 2012~2024 iot.10086.cn All Rights Reserved
 *
 * @file main.c
 * @brief Thing model example
 */

/*****************************************************************************/
/* Includes                                                                  */
/*****************************************************************************/
#include <stdio.h>
#include <string.h>

#include "common.h"
#include "data_types.h"
#include "plat_time.h"
#include "aiot_tm_api.h"
#include "tm_data.h"
#include "tm_user.h"
#include "log.h"

// Note:
// If the device created on the platform uses the MQTT 5.0 protocol, CONFIG_MQTT_VERSION_5 in CMakeLists.txt must be ON.
// If it uses the MQTT protocol, CONFIG_MQTT_VERSION_5 must be OFF.
// Otherwise, a protocol mismatch will prevent connection.

#ifndef PRODUCT_ID
#define PRODUCT_ID ""
#endif

#ifndef DEVICE_NAME
#define DEVICE_NAME ""
#endif

#ifndef ACCESS_KEY
#define ACCESS_KEY ""
#endif

/** token Effective time，The default is 2030.12*/
#define TM_EXPIRE_TIME 1924833600

#ifdef SDK_USE_MQTTS
#include "tm_subdev.h"

int32_t cb_subdev_props_get(const uint8_t* product_id, const uint8_t* dev_name, const uint8_t* props_list, uint8_t** props_data) {
    return 0;
}
int32_t cb_subdev_props_set(const uint8_t* product_id, const uint8_t* dev_name, uint8_t* props_data) {
    return 0;
}
int32_t cb_subdev_service_invoke(const uint8_t* product_id, const uint8_t* dev_name, const uint8_t* svc_id, uint8_t* in_data, uint8_t** out_data) {
    return 0;
}
int32_t cb_subdev_topo(uint8_t* topo_data) {
    logs(topo_data);
    return 0;
}
#endif

char* g_server_ip=NULL;
char* g_server_port=NULL;

// extern int led(int num,int stat);


int main(int argc, char *argv[])
{

    struct tm_downlink_tbl_t dl_tbl;
    int                      ret = 0;
    osl_handle_t                  timer_handle = 0;
    uint32_t                  timer_period = 120;

    dl_tbl.prop_tbl      = tm_prop_list;
    dl_tbl.prop_tbl_size = tm_prop_list_size;
    dl_tbl.svc_tbl       = tm_svc_list;
    dl_tbl.svc_tbl_size  = tm_svc_list_size;

    char* product_id = (uint8_t *)PRODUCT_ID;
    char* device_sn  = (uint8_t *)DEVICE_NAME;
    char* auth_code  = (uint8_t *)ACCESS_KEY;

    osl_init();
    // osl_set_config(argc,argv,&product_id,&device_sn,&auth_code);

    int timeout_ms=60*1000;
#ifdef CELLULAR_MODULE
    timeout_ms=10*60*1000; //Mobile network connection is slow
#endif

    /** Device Initialization*/
    ret = tm_init(&dl_tbl);
    CHECK_EXPR_GOTO(ERR_OK != ret, _END, "ThingModel init failed!\n");
    logi("ThingModel init ok");

    /** Device Login*/
    ret = tm_login(product_id, device_sn, auth_code, TM_EXPIRE_TIME, timeout_ms);
    CHECK_EXPR_GOTO(ERR_OK != ret, _END, "ThingModel login failed!");
    logi("ThingModel login ok");

    // #define SDK_USE_SUBDEV
    #ifdef SDK_USE_SUBDEV
    struct tm_subdev_cbs callbacks={
        .subdev_props_get       =cb_subdev_props_get,
        .subdev_props_set       =cb_subdev_props_set,
        .subdev_service_invoke  =cb_subdev_service_invoke,
        .subdev_topo            =cb_subdev_topo,
    };
    ret = tm_subdev_init(callbacks);
    tm_subdev_topo_get(5000);
    CHECK_EXPR_GOTO(ERR_OK != ret, SUBDEV_END, "tm_subdev_init failed!");

    ret = tm_subdev_add(SUBDEV_PRODUCT_ID, SUBDEV_DEVICE_NAME, SUBDEV_ACCESS_KEY, timeout_ms);
    CHECK_EXPR_GOTO(ERR_OK != ret, SUBDEV_END, "tm_subdev_add failed!");

    ret = tm_subdev_login(SUBDEV_PRODUCT_ID, SUBDEV_DEVICE_NAME, timeout_ms);
    CHECK_EXPR_GOTO(ERR_OK != ret, SUBDEV_END, "tm_subdev_login failed!");

SUBDEV_END:
    #endif

    int i=0;

    while (1) {
        // logi("tm_step, i=%d",i);
        if (i == 5 * 10) { // 200ms * 5 * 10 = 10s
            tm_prop_ErrorCode_notify(NULL, i, NULL, 3000);
            i = 0;
        }
        i++;
        if (0 != (ret = tm_step(200))) {
            logi("ThingModel tm_step failed,ret is %d",ret);
            // break;
        }
    }

    /** device deregistration*/
    tm_logout(3000);
_END:
    return 0;
}
