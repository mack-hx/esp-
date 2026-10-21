/**
 * @file tm_user.h
 * @brief 物模型用户接口定义文件
 * @details 该文件由物联网平台根据用户配置的物模型自动生成，包含设备服务、属性、命令和事件的接口定义。
 *          用户需实现文件中声明的回调函数，以处理物模型数据的读写、命令执行和事件上报等操作。
 * @author 中移物联网有限公司
 * @date 2025-06-13
 * @version V1.0
 * @copyright Copyright © 2024 中移物联网有限公司. All rights reserved.
 */
 
#ifndef __TM_USER_H__
#define __TM_USER_H__

/*****************************************************************************/
/* Includes                                                                  */
/*****************************************************************************/
#include "aiot_tm_api.h"

#ifdef __cplusplus
extern "C"
{
#endif

/*****************************************************************************/
/* External Definition ( Constant and Macro )                                */
/*****************************************************************************/

/*****************************************************************************/
/* External Structures, Enum and Typedefs                                    */
/*****************************************************************************/
/****************************** Structure type *******************************/
struct event_armed_t
{
    int64_t time;
};

struct event_fallen_t
{
    float32_t pitch;
    int64_t time;
};

struct svc_calibrate_in_t
{
};
struct svc_calibrate_out_t
{
	boolean ok;
};
struct svc_param_get_in_t
{
};
struct svc_param_get_out_t
{
	int8_t *values;
};
struct svc_param_save_in_t
{
};
struct svc_param_save_out_t
{
	boolean ok;
};
struct svc_param_set_in_t
{
	int8_t *values;
};
struct svc_param_set_out_t
{
};
struct svc_stand_in_t
{
};
struct svc_stand_out_t
{
	boolean ok;
};
struct svc_stop_in_t
{
};
struct svc_stop_out_t
{
	boolean ok;
};
struct prop_$OneNET_OneOS_VERSION_t
{
    int8_t *OneNET_OneOS_VERSION;
};


/****************************** Auto Generated *******************************/

/*****************************************************************************/
/* External Variables and Functions                                          */
/*****************************************************************************/
/*************************** Property Func List ******************************/
extern struct tm_prop_tbl_t tm_prop_list[];
extern uint16_t tm_prop_list_size;
/****************************** Auto Generated *******************************/

/**************************** Service Func List ******************************/
extern struct tm_svc_tbl_t tm_svc_list[];
extern uint16_t tm_svc_list_size;
/****************************** Auto Generated *******************************/

/**************************** Property Func Read ****************************/
int32_t tm_prop_$OneNET_OneOS_VERSION_rd_cb(void *data);
int32_t tm_prop_enc_l_rd_cb(void *data);
int32_t tm_prop_enc_r_rd_cb(void *data);
int32_t tm_prop_gz_rd_cb(void *data);
int32_t tm_prop_motor_l_rd_cb(void *data);
int32_t tm_prop_motor_r_rd_cb(void *data);
int32_t tm_prop_pitch_rd_cb(void *data);
int32_t tm_prop_roll_rd_cb(void *data);
int32_t tm_prop_speed_rd_cb(void *data);
int32_t tm_prop_state_rd_cb(void *data);
int32_t tm_prop_uptime_rd_cb(void *data);
int32_t tm_prop_yaw_rd_cb(void *data);

/****************************** Auto Generated *******************************/

/**************************** Service Func Invoke ****************************/
int32_t tm_svc_calibrate_cb(void *in_data, void *out_data);
int32_t tm_svc_param_get_cb(void *in_data, void *out_data);
int32_t tm_svc_param_save_cb(void *in_data, void *out_data);
int32_t tm_svc_param_set_cb(void *in_data, void *out_data);
int32_t tm_svc_stand_cb(void *in_data, void *out_data);
int32_t tm_svc_stop_cb(void *in_data, void *out_data);

/****************************** Auto Generated *******************************/

/**************************** Property Func Write ****************************/
int32_t tm_prop_$OneNET_OneOS_VERSION_wr_cb(void *data);
int32_t tm_prop_enc_l_wr_cb(void *data);
int32_t tm_prop_enc_r_wr_cb(void *data);
int32_t tm_prop_gz_wr_cb(void *data);
int32_t tm_prop_motor_r_wr_cb(void *data);
int32_t tm_prop_pitch_wr_cb(void *data);
int32_t tm_prop_roll_wr_cb(void *data);
int32_t tm_prop_speed_wr_cb(void *data);
int32_t tm_prop_state_wr_cb(void *data);
int32_t tm_prop_uptime_wr_cb(void *data);
int32_t tm_prop_yaw_wr_cb(void *data);

/****************************** Auto Generated *******************************/

/**************************** Property Func Notify ***************************/
int32_t tm_prop_$OneNET_OneOS_VERSION_notify(void *data, struct prop_$OneNET_OneOS_VERSION_t val, uint64_t timestamp, uint32_t timeout_ms);
int32_t tm_prop_enc_l_notify(void *data, int32_t val, uint64_t timestamp, uint32_t timeout_ms);
int32_t tm_prop_enc_r_notify(void *data, int32_t val, uint64_t timestamp, uint32_t timeout_ms);
int32_t tm_prop_gz_notify(void *data, float32_t val, uint64_t timestamp, uint32_t timeout_ms);
int32_t tm_prop_motor_l_notify(void *data, int32_t val, uint64_t timestamp, uint32_t timeout_ms);
int32_t tm_prop_motor_r_notify(void *data, int32_t val, uint64_t timestamp, uint32_t timeout_ms);
int32_t tm_prop_pitch_notify(void *data, float32_t val, uint64_t timestamp, uint32_t timeout_ms);
int32_t tm_prop_roll_notify(void *data, float32_t val, uint64_t timestamp, uint32_t timeout_ms);
int32_t tm_prop_speed_notify(void *data, float32_t val, uint64_t timestamp, uint32_t timeout_ms);
int32_t tm_prop_state_notify(void *data, int8_t *val, uint64_t timestamp, uint32_t timeout_ms);
int32_t tm_prop_uptime_notify(void *data, int32_t val, uint64_t timestamp, uint32_t timeout_ms);
int32_t tm_prop_yaw_notify(void *data, float32_t val, uint64_t timestamp, uint32_t timeout_ms);

/****************************** Auto Generated *******************************/

/***************************** Event Func Notify *****************************/
int32_t tm_event_armed_notify(void *data, struct event_armed_t val, uint64_t timestamp, uint32_t timeout_ms);
int32_t tm_event_fallen_notify(void *data, struct event_fallen_t val, uint64_t timestamp, uint32_t timeout_ms);

/****************************** Auto Generated *******************************/

#ifdef __cplusplus
}
#endif

#endif
