/**
 * Copyright (c), 2012~2018 iot.10086.cn All Rights Reserved
 * @file        utils.c
 * @brief       Some common used data processing function.
 */

/*****************************************************************************/
/* Includes                                                                  */
/*****************************************************************************/
#include "plat_osl.h"
#include "plat_time.h"
#include "log.h"
#include "utils.h"
#include "err_def.h"
/*****************************************************************************/
/* Local Definitions ( Constant and Macro )                                  */
/*****************************************************************************/

/*****************************************************************************/
/* Structures, Enum and Typedefs                                             */
/*****************************************************************************/

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
uint16_t set_16bit_le(uint8_t* buf, uint16_t val)
{
    buf[0] = val & 0xFF;
    buf[1] = (val >> 8) & 0xFF;
    return 2;
}

uint16_t get_16bit_le(uint8_t* buf, uint16_t* val)
{
    *val = (buf[1] << 8) | buf[0];
    return 2;
}

uint16_t set_16bit_be(uint8_t* buf, uint16_t val)
{
    buf[0] = (val >> 8) & 0xFF;
    buf[1] = val & 0xFF;
    return 2;
}

uint16_t get_16bit_be(uint8_t* buf, uint16_t* val)
{
    *val = (buf[0] << 8) | buf[1];
    return 2;
}

uint16_t set_32bit_le(uint8_t* buf, uint32_t val)
{
    buf[0] = val & 0xFF;
    buf[1] = (val >> 8) & 0xFF;
    buf[2] = (val >> 16) & 0xFF;
    buf[3] = (val >> 24) & 0xFF;
    return 4;
}

uint16_t get_32bit_le(uint8_t* buf, uint32_t* val)
{
    *val = (buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0];
    return 4;
}

uint16_t set_32bit_be(uint8_t* buf, uint32_t val)
{
    buf[0] = (val >> 24) & 0xFF;
    buf[1] = (val >> 16) & 0xFF;
    buf[2] = (val >> 8) & 0xFF;
    buf[3] = val & 0xFF;
    return 4;
}

uint16_t get_32bit_be(uint8_t* buf, uint32_t* val)
{
    *val = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
    return 4;
}

// Refer to example_hex_to_str
int32_t hex_to_str(uint8_t* str_hex, uint8_t* str, uint16_t len)
{
    uint16_t i, j, k;
    uint8_t  data_buf[2] = { 0 };

    for (i = 0, j = 0; i < len; i++) {
        for (k = 0; k < 2; k++) {
            if ((str_hex[j] >= '0') && (str_hex[j] <= '9'))
                data_buf[k] = str_hex[j] - '0';
            else if ((str_hex[j] >= 'a') && (str_hex[j] <= 'f'))
                data_buf[k] = str_hex[j] - 'a' + 10;
            else if ((str_hex[j] >= 'A') && (str_hex[j] <= 'F'))
                data_buf[k] = str_hex[j] - 'A' + 10;
            else
                return -1;
            j++;
        }
        str[i] = ((data_buf[0] << 4) | data_buf[1]);
    }

    return 0;
}

int example_hex_to_str() {
    char str_hex[]="4141";
    char str[64]={0};
    logs(str_hex);
    hex_to_str(str_hex,str,strlen(str_hex));
    logs(str);
    // exit(0);
    return 0;
}

void str_to_hex(uint8_t* str_hex, uint8_t* str, uint16_t len)
{
    char ddl, ddh;
    int  i;

    for (i = 0; i < len; i++) {
        ddh = 48 + str[i] / 16;
        ddl = 48 + str[i] % 16;
        if (ddh > 57) ddh = ddh + 7;
        if (ddl > 57) ddl = ddl + 7;
        str_hex[i * 2]     = ddh;
        str_hex[i * 2 + 1] = ddl;
    }
    str_hex[len * 2] = '\0';
}

int int_pair_get(struct int_pair_t* pair,int pair_len,int in,int out,int in_value,int* out_value) {
    uint32_t i = 0;
    for (i = 0; i < pair_len; i++) {
        if (in_value == pair[i].arr[in]) {
            *out_value=pair[i].arr[out];
            return ERR_OK;
        }
    }
    return ERR_INVALID_DATA;
}

int log_port_config(struct uart_config_t *port_config) {
    logs(port_config->port_name);
    logn(port_config->baudrate);
    logn(port_config->data_bits);
    logn(port_config->stop_bits);
    logn(port_config->flow_ctrl);
    logn(port_config->parity);
    return 0;
}

void osl_common(void) {
    #if defined(__GNUC__) // GCC, GNU Compiler
    logd("__GNUC__ is %d",__GNUC__);
    #elif defined(__CC_ARM) // Keil MDK, Arm Compiler 4/5
    logd("__CC_ARM is %d",__CC_ARM);
    #elif defined(__ARMCC_VERSION) // Keil MDK, Arm Compiler 6 (armclang)
    logd("__ARMCC_VERSION is %d",__ARMCC_VERSION);
    #endif

    logd("Build Date: %s,%s",__DATE__ ,__TIME__);

    #if defined(USE_MODULE) && (!defined(SDK_USE_NBIOT)) //Initialize communication module
        if(ERR_OK!=module_init(NULL,NULL)) {
            loge("module_init failed!");
            abort();
        }
    #endif

    logd("osl_init ok");
}

uint8_t* osl_random_string(uint8_t* buf, int len)
{
    static char s_characters[] = {
        'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U',
        'V', 'W', 'X', 'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p',
        'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
    };
    int i = 0;
    for (; i < len; i++) {
        buf[i] = s_characters[osl_rand(0, sizeof(s_characters) - 1)];
    }
    buf[i] = '\0';
    return buf;
}

/*****************************************************************************/
/* Thread Synchronization Implementation                                      */
/*****************************************************************************/

/**
 * @brief Initialize a mutex lock
 */
int osl_mutex_init(osl_mutex_t* mutex)
{
    if (mutex == NULL) {
        return -1;
    }

    OSL_ATOMIC_STORE(&mutex->lock, OSL_SYNC_UNLOCKED);
    return 0;
}

/**
 * @brief Destroy a mutex lock
 */
int osl_mutex_destroy(osl_mutex_t* mutex)
{
    if (mutex == NULL) {
        return -1;
    }

    OSL_ATOMIC_STORE(&mutex->lock, OSL_SYNC_UNLOCKED);
    return 0;
}

/**
 * @brief Lock a mutex (blocking)
 */
int osl_mutex_lock(osl_mutex_t* mutex)
{
    if (mutex == NULL) {
        return -1;
    }

    for (;;) {
        if (OSL_ATOMIC_CAS(&mutex->lock, OSL_SYNC_UNLOCKED, OSL_SYNC_LOCKED)) {
            return 0;  /* Lock acquired */
        }
        time_delay_ms(1);
    }
}

/**
 * @brief Try to lock a mutex (non-blocking)
 *
 * @return 0 on success (lock acquired), -1 if already locked
 */
int osl_mutex_trylock(osl_mutex_t* mutex)
{
    if (mutex == NULL) {
        return -1;
    }

    return OSL_ATOMIC_CAS(&mutex->lock, OSL_SYNC_UNLOCKED, OSL_SYNC_LOCKED) ? 0 : -1;
}

/**
 * @brief Unlock a mutex
 */
int osl_mutex_unlock(osl_mutex_t* mutex)
{
    if (mutex == NULL) {
        return -1;
    }

    OSL_ATOMIC_STORE(&mutex->lock, OSL_SYNC_UNLOCKED);
    return 0;
}
