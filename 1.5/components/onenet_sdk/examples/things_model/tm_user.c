 /**
 * @file tm_user.c
 * @brief 物模型用户接口定义文件
 * @details 该文件由物联网平台根据用户配置的物模型自动生成，包含设备服务、属性、命令和事件的接口定义。
 *          用户需实现文件中声明的回调函数，以处理物模型数据的读写、命令执行和事件上报等操作。
 * @author 中移物联网有限公司
 * @date 2025-06-13
 * @version V1.0
 * @copyright Copyright © 2024 中移物联网有限公司. All rights reserved.
 */

/*****************************************************************************/
/* Includes                                                                  */
/*****************************************************************************/
#include "tm_data.h"
#include "tm_user.h"

/*****************************************************************************/
/* Local Definitions ( Constant and Macro )                                  */
/*****************************************************************************/

/*****************************************************************************/
/* Structures, Enum and Typedefs                                             */
/*****************************************************************************/
/*************************** Property Func List ******************************/
struct tm_prop_tbl_t tm_prop_list[] = {
    TM_PROPERTY_RW($OneNET_OneOS_VERSION),
    TM_PROPERTY_RW(enc_l),
    TM_PROPERTY_RW(enc_r),
    TM_PROPERTY_RW(gz),
    TM_PROPERTY_RO(motor_l),
    TM_PROPERTY_RW(motor_r),
    TM_PROPERTY_RW(pitch),
    TM_PROPERTY_RW(roll),
    TM_PROPERTY_RW(speed),
    TM_PROPERTY_RW(state),
    TM_PROPERTY_RW(uptime),
    TM_PROPERTY_RW(yaw)
};
uint16_t tm_prop_list_size = ARRAY_SIZE(tm_prop_list);
/****************************** Auto Generated *******************************/

/***************************** Service Func List *******************************/
struct tm_svc_tbl_t tm_svc_list[] = {
	TM_SERVICE(calibrate),
	TM_SERVICE(param_get),
	TM_SERVICE(param_save),
	TM_SERVICE(param_set),
	TM_SERVICE(stand),
	TM_SERVICE(stop)};
uint16_t tm_svc_list_size = ARRAY_SIZE(tm_svc_list);
/****************************** Auto Generated *******************************/

/*****************************************************************************/
/* Local Function Prototype                                                  */
/*****************************************************************************/

/*****************************************************************************/
/* Local Variables                                                           */
/*****************************************************************************/

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

/*****************************************************************************/
/* Function Implementation                                                   */
/*****************************************************************************/
/**************************** Property Func Read *****************************/
int32_t tm_prop_$OneNET_OneOS_VERSION_rd_cb(void *data)
{
    void *structure = tm_data_struct_create();
    struct prop_$OneNET_OneOS_VERSION_t val = {0};
    /** 根据业务逻辑获取功能点值，设置到val */

    tm_data_struct_set_string(structure, "OneNET_OneOS_VERSION", val.OneNET_OneOS_VERSION);
    tm_data_struct_set_data(data, "$OneNET_OneOS_VERSION", structure);
   return 0;
}

int32_t tm_prop_enc_l_rd_cb(void *data)
{
	int32_t val = 0;

	/** 根据业务逻辑获取功能点值，设置到val */


	tm_data_struct_set_int32(data, "enc_l", val);

	return 0;
}

int32_t tm_prop_enc_r_rd_cb(void *data)
{
	int32_t val = 0;

	/** 根据业务逻辑获取功能点值，设置到val */


	tm_data_struct_set_int32(data, "enc_r", val);

	return 0;
}

int32_t tm_prop_gz_rd_cb(void *data)
{
	float32_t val = 0;

	/** 根据业务逻辑获取功能点值，设置到val */


	tm_data_struct_set_float(data, "gz", val);

	return 0;
}

int32_t tm_prop_motor_l_rd_cb(void *data)
{
	int32_t val = 0;

	/** 根据业务逻辑获取功能点值，设置到val */


	tm_data_struct_set_int32(data, "motor_l", val);

	return 0;
}

int32_t tm_prop_motor_r_rd_cb(void *data)
{
	int32_t val = 0;

	/** 根据业务逻辑获取功能点值，设置到val */


	tm_data_struct_set_int32(data, "motor_r", val);

	return 0;
}

int32_t tm_prop_pitch_rd_cb(void *data)
{
	float32_t val = 0;

	/** 根据业务逻辑获取功能点值，设置到val */


	tm_data_struct_set_float(data, "pitch", val);

	return 0;
}

int32_t tm_prop_roll_rd_cb(void *data)
{
	float32_t val = 0;

	/** 根据业务逻辑获取功能点值，设置到val */


	tm_data_struct_set_float(data, "roll", val);

	return 0;
}

int32_t tm_prop_speed_rd_cb(void *data)
{
	float32_t val = 0;

	/** 根据业务逻辑获取功能点值，设置到val */


	tm_data_struct_set_float(data, "speed", val);

	return 0;
}

int32_t tm_prop_state_rd_cb(void *data)
{
	int8_t *val = 0;
	/** 根据业务逻辑获取功能点值，设置到val */


	tm_data_struct_set_string(data, "state", val);

	return 0;
}

int32_t tm_prop_uptime_rd_cb(void *data)
{
	int32_t val = 0;

	/** 根据业务逻辑获取功能点值，设置到val */


	tm_data_struct_set_int32(data, "uptime", val);

	return 0;
}

int32_t tm_prop_yaw_rd_cb(void *data)
{
	float32_t val = 0;

	/** 根据业务逻辑获取功能点值，设置到val */


	tm_data_struct_set_float(data, "yaw", val);

	return 0;
}


/****************************** Auto Generated *******************************/

/**************************** Property Func Write ****************************/
int32_t tm_prop_$OneNET_OneOS_VERSION_wr_cb(void *data)
{
    struct prop_$OneNET_OneOS_VERSION_t val;

    tm_data_struct_get_string(data, "OneNET_OneOS_VERSION", &(val.OneNET_OneOS_VERSION));
    /** 根据变量val的值，填入下发控制逻辑 */

    /***/
    return 0;
}

int32_t tm_prop_enc_l_wr_cb(void *data)
{
    int32_t val = 0;
    tm_data_get_int32(data, &val);
    /** 根据变量val的值，填入下发控制逻辑 */

    /***/
    return 0;
}

int32_t tm_prop_enc_r_wr_cb(void *data)
{
    int32_t val = 0;
    tm_data_get_int32(data, &val);
    /** 根据变量val的值，填入下发控制逻辑 */

    /***/
    return 0;
}

int32_t tm_prop_gz_wr_cb(void *data)
{
    float32_t val = 0;
    tm_data_get_float(data, &val);
    /** 根据变量val的值，填入下发控制逻辑 */

    /***/
    return 0;
}

int32_t tm_prop_motor_r_wr_cb(void *data)
{
    int32_t val = 0;
    tm_data_get_int32(data, &val);
    /** 根据变量val的值，填入下发控制逻辑 */

    /***/
    return 0;
}

int32_t tm_prop_pitch_wr_cb(void *data)
{
    float32_t val = 0;
    tm_data_get_float(data, &val);
    /** 根据变量val的值，填入下发控制逻辑 */

    /***/
    return 0;
}

int32_t tm_prop_roll_wr_cb(void *data)
{
    float32_t val = 0;
    tm_data_get_float(data, &val);
    /** 根据变量val的值，填入下发控制逻辑 */

    /***/
    return 0;
}

int32_t tm_prop_speed_wr_cb(void *data)
{
    float32_t val = 0;
    tm_data_get_float(data, &val);
    /** 根据变量val的值，填入下发控制逻辑 */

    /***/
    return 0;
}

int32_t tm_prop_state_wr_cb(void *data)
{
    int8_t *val = 0;
    tm_data_get_string(data, &val);
    /** 根据变量val的值，填入下发控制逻辑 */

    /***/
    return 0;
}

int32_t tm_prop_uptime_wr_cb(void *data)
{
    int32_t val = 0;
    tm_data_get_int32(data, &val);
    /** 根据变量val的值，填入下发控制逻辑 */

    /***/
    return 0;
}

int32_t tm_prop_yaw_wr_cb(void *data)
{
    float32_t val = 0;
    tm_data_get_float(data, &val);
    /** 根据变量val的值，填入下发控制逻辑 */

    /***/
    return 0;
}


/****************************** Auto Generated *******************************/

/**************************** Property Func Notify ***************************/
int32_t tm_prop_$OneNET_OneOS_VERSION_notify(void *data, struct prop_$OneNET_OneOS_VERSION_t val, uint64_t timestamp, uint32_t timeout_ms)
{
    void *resource = NULL;
    void *structure = tm_data_struct_create();
	int32_t ret = 0;

    if(NULL == data)
    {
        resource = tm_data_create();
    }
    else
    {
        resource = data;
    }

	tm_data_struct_set_string(structure, "OneNET_OneOS_VERSION", val.OneNET_OneOS_VERSION);

    tm_data_set_struct(resource, "$OneNET_OneOS_VERSION", structure, timestamp);

    if(NULL == data)
    {
        ret = tm_post_property(resource, timeout_ms);
    }

    return ret;
}

int32_t tm_prop_enc_l_notify(void *data, int32_t val, uint64_t timestamp, uint32_t timeout_ms)
{
	void *resource = NULL;
    int32_t ret = 0;

    if(NULL == data)
    {
        resource = tm_data_create();
    }
    else
    {
        resource = data;
    }

    tm_data_set_int32(resource, "enc_l", val, timestamp);

    if(NULL == data)
    {
        ret = tm_post_property(resource, timeout_ms);
    }

    return ret;
}

int32_t tm_prop_enc_r_notify(void *data, int32_t val, uint64_t timestamp, uint32_t timeout_ms)
{
	void *resource = NULL;
    int32_t ret = 0;

    if(NULL == data)
    {
        resource = tm_data_create();
    }
    else
    {
        resource = data;
    }

    tm_data_set_int32(resource, "enc_r", val, timestamp);

    if(NULL == data)
    {
        ret = tm_post_property(resource, timeout_ms);
    }

    return ret;
}

int32_t tm_prop_gz_notify(void *data, float32_t val, uint64_t timestamp, uint32_t timeout_ms)
{
	void *resource = NULL;
    int32_t ret = 0;

    if(NULL == data)
    {
        resource = tm_data_create();
    }
    else
    {
        resource = data;
    }

    tm_data_set_float(resource, "gz", val, timestamp);

    if(NULL == data)
    {
        ret = tm_post_property(resource, timeout_ms);
    }

    return ret;
}

int32_t tm_prop_motor_l_notify(void *data, int32_t val, uint64_t timestamp, uint32_t timeout_ms)
{
	void *resource = NULL;
    int32_t ret = 0;

    if(NULL == data)
    {
        resource = tm_data_create();
    }
    else
    {
        resource = data;
    }

    tm_data_set_int32(resource, "motor_l", val, timestamp);

    if(NULL == data)
    {
        ret = tm_post_property(resource, timeout_ms);
    }

    return ret;
}

int32_t tm_prop_motor_r_notify(void *data, int32_t val, uint64_t timestamp, uint32_t timeout_ms)
{
	void *resource = NULL;
    int32_t ret = 0;

    if(NULL == data)
    {
        resource = tm_data_create();
    }
    else
    {
        resource = data;
    }

    tm_data_set_int32(resource, "motor_r", val, timestamp);

    if(NULL == data)
    {
        ret = tm_post_property(resource, timeout_ms);
    }

    return ret;
}

int32_t tm_prop_pitch_notify(void *data, float32_t val, uint64_t timestamp, uint32_t timeout_ms)
{
	void *resource = NULL;
    int32_t ret = 0;

    if(NULL == data)
    {
        resource = tm_data_create();
    }
    else
    {
        resource = data;
    }

    tm_data_set_float(resource, "pitch", val, timestamp);

    if(NULL == data)
    {
        ret = tm_post_property(resource, timeout_ms);
    }

    return ret;
}

int32_t tm_prop_roll_notify(void *data, float32_t val, uint64_t timestamp, uint32_t timeout_ms)
{
	void *resource = NULL;
    int32_t ret = 0;

    if(NULL == data)
    {
        resource = tm_data_create();
    }
    else
    {
        resource = data;
    }

    tm_data_set_float(resource, "roll", val, timestamp);

    if(NULL == data)
    {
        ret = tm_post_property(resource, timeout_ms);
    }

    return ret;
}

int32_t tm_prop_speed_notify(void *data, float32_t val, uint64_t timestamp, uint32_t timeout_ms)
{
	void *resource = NULL;
    int32_t ret = 0;

    if(NULL == data)
    {
        resource = tm_data_create();
    }
    else
    {
        resource = data;
    }

    tm_data_set_float(resource, "speed", val, timestamp);

    if(NULL == data)
    {
        ret = tm_post_property(resource, timeout_ms);
    }

    return ret;
}

int32_t tm_prop_state_notify(void *data, int8_t *val, uint64_t timestamp, uint32_t timeout_ms)
{
	void *resource = NULL;
    int32_t ret = 0;

    if(NULL == data)
    {
        resource = tm_data_create();
    }
    else
    {
        resource = data;
    }

    tm_data_set_string(resource, "state", val, timestamp);

    if(NULL == data)
    {
        ret = tm_post_property(resource, timeout_ms);
    }

    return ret;
}

int32_t tm_prop_uptime_notify(void *data, int32_t val, uint64_t timestamp, uint32_t timeout_ms)
{
	void *resource = NULL;
    int32_t ret = 0;

    if(NULL == data)
    {
        resource = tm_data_create();
    }
    else
    {
        resource = data;
    }

    tm_data_set_int32(resource, "uptime", val, timestamp);

    if(NULL == data)
    {
        ret = tm_post_property(resource, timeout_ms);
    }

    return ret;
}

int32_t tm_prop_yaw_notify(void *data, float32_t val, uint64_t timestamp, uint32_t timeout_ms)
{
	void *resource = NULL;
    int32_t ret = 0;

    if(NULL == data)
    {
        resource = tm_data_create();
    }
    else
    {
        resource = data;
    }

    tm_data_set_float(resource, "yaw", val, timestamp);

    if(NULL == data)
    {
        ret = tm_post_property(resource, timeout_ms);
    }

    return ret;
}


/****************************** Auto Generated *******************************/

/***************************** Event Func Notify *****************************/
int32_t tm_event_armed_notify(void *data, struct event_armed_t val, uint64_t timestamp, uint32_t timeout_ms)
{
    void *resource = NULL;
    void *structure = tm_data_struct_create();
    int32_t ret = 0;

    if(NULL == data)
    {
        resource = tm_data_create();
    }
    else
    {
        resource = data;
    }

    tm_data_struct_set_int64(structure, "time", val.time);

    tm_data_set_struct(resource, "armed", structure, timestamp);

    if(NULL == data)
    {
        ret = tm_post_event(resource, timeout_ms);
    }

    return ret;
}

int32_t tm_event_fallen_notify(void *data, struct event_fallen_t val, uint64_t timestamp, uint32_t timeout_ms)
{
    void *resource = NULL;
    void *structure = tm_data_struct_create();
    int32_t ret = 0;

    if(NULL == data)
    {
        resource = tm_data_create();
    }
    else
    {
        resource = data;
    }

    tm_data_struct_set_float(structure, "pitch", val.pitch);
    tm_data_struct_set_int64(structure, "time", val.time);

    tm_data_set_struct(resource, "fallen", structure, timestamp);

    if(NULL == data)
    {
        ret = tm_post_event(resource, timeout_ms);
    }

    return ret;
}


/****************************** Auto Generated *******************************/

/**************************** Service Func Invoke ****************************/
int32_t tm_svc_calibrate_cb(void *in_data, void *out_data)
{
	struct svc_calibrate_in_t in_param = {0};
	struct svc_calibrate_out_t out_param = {0};
	int32_t ret = 0;

	/** 根据输入参数in_param，生成输出参数out_param */

	tm_data_struct_set_bool(out_data, "ok", out_param.ok);

	return ret;
}

int32_t tm_svc_param_get_cb(void *in_data, void *out_data)
{
	struct svc_param_get_in_t in_param = {0};
	struct svc_param_get_out_t out_param = {0};
	int32_t ret = 0;

	/** 根据输入参数in_param，生成输出参数out_param */

	tm_data_struct_set_string(out_data, "values", out_param.values);

	return ret;
}

int32_t tm_svc_param_save_cb(void *in_data, void *out_data)
{
	struct svc_param_save_in_t in_param = {0};
	struct svc_param_save_out_t out_param = {0};
	int32_t ret = 0;

	/** 根据输入参数in_param，生成输出参数out_param */

	tm_data_struct_set_bool(out_data, "ok", out_param.ok);

	return ret;
}

int32_t tm_svc_param_set_cb(void *in_data, void *out_data)
{
	struct svc_param_set_in_t in_param = {0};
	struct svc_param_set_out_t out_param = {0};
	int32_t ret = 0;

	tm_data_struct_get_string(in_data, "values", &(in_param.values));

	/** 根据输入参数in_param，生成输出参数out_param */

	return ret;
}

int32_t tm_svc_stand_cb(void *in_data, void *out_data)
{
	struct svc_stand_in_t in_param = {0};
	struct svc_stand_out_t out_param = {0};
	int32_t ret = 0;

	/** 根据输入参数in_param，生成输出参数out_param */

	tm_data_struct_set_bool(out_data, "ok", out_param.ok);

	return ret;
}

int32_t tm_svc_stop_cb(void *in_data, void *out_data)
{
	struct svc_stop_in_t in_param = {0};
	struct svc_stop_out_t out_param = {0};
	int32_t ret = 0;

	/** 根据输入参数in_param，生成输出参数out_param */

	tm_data_struct_set_bool(out_data, "ok", out_param.ok);

	return ret;
}


/****************************** Auto Generated *******************************/
