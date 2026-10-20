/**
 * Copyright (c), 2012~2024 iot.10086.cn All Rights Reserved
 *
 * @file plat_osl.h
 * @brief Standard Library Interface
 */

#ifndef __PLAT_OSL_H__
#define __PLAT_OSL_H__

/*****************************************************************************/
/* Includes                                                                  */
/*****************************************************************************/
#include "data_types.h"
#include "plat_time.h"

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************/
/* External Definition（Constant and Macro )                                 */
/*****************************************************************************/

/*****************************************************************************/
/* External Structures, Enum and Typedefs                                    */
/*****************************************************************************/

/*****************************************************************************/
/* External Variables and Functions                                          */
/*****************************************************************************/
extern char* g_server_ip;
extern char* g_server_port;

void *osl_malloc(size_t size);
void *osl_calloc(size_t num, size_t size);
void  osl_free(void *ptr);

void *osl_memcpy(void *dst, const void *src, size_t n);
void *osl_memmove(void *dst, const void *src, size_t n);
void *osl_memset(void *dst, int32_t val, size_t n);

uint8_t *osl_strdup(const uint8_t *s);
uint8_t *osl_strndup(const uint8_t *s, size_t n);
uint8_t *osl_strcpy(uint8_t *s1, const uint8_t *s2);
uint8_t *osl_strncpy(uint8_t *s1, const uint8_t *s2, size_t n);
uint8_t *osl_strcat(uint8_t *dst, const uint8_t *src);
uint8_t *osl_strstr(const uint8_t *s1, const uint8_t *s2);

uint32_t osl_strlen(const uint8_t *s);
int32_t  osl_strcmp(const uint8_t *s1, const uint8_t *s2);
int32_t  osl_strncmp(const uint8_t *s1, const uint8_t *s2, size_t n);
int32_t  osl_sprintf(uint8_t *str, const uint8_t *format, ...);
int32_t  osl_sscanf(const uint8_t *str, const uint8_t *format, ...);
void     osl_assert(bool_t expression);
int32_t  osl_get_random(unsigned char *buf, size_t len);
int32_t osl_atoi(const uint8_t *nptr);

/// @brief  Generation a ramdom number located in a closed interval[min,max]
/// @param  min Minimum Value
/// @param  max Maximum value
/// @return  Random number
int32_t osl_rand(int32_t min, int32_t max);

/// @brief  Generation a ramdom string with length len
/// @param  buf String cached address
/// @param  len Length
/// @return  Random String
uint8_t *osl_random_string(uint8_t *buf, int len);

int osl_init(void);
int osl_set_config(int argc, char* argv[],char** pp_product_id,char** pp_device_sn,char** pp_auth_code);
void osl_common(void);
int32_t module_init(void *arg,void* callback);
int32_t module_deinit(void);

#if defined(_MSC_VER) /* Microsoft Visual C++ */
    #include <intrin.h>
    /* _InterlockedCompareExchange operates on LONG*; cast is safe since
       uint32_t and long are both 32-bit on Windows. Returns the old value. */
    #define OSL_ATOMIC_CAS(ptr, expected_val, desired_val) \
        (_InterlockedCompareExchange((volatile LONG*)(ptr), \
            (LONG)(desired_val), (LONG)(expected_val)) == (LONG)(expected_val))

    #define OSL_ATOMIC_STORE(ptr, val) \
        (_InterlockedExchange((volatile LONG*)(ptr), (LONG)(val)))

#elif defined(__GNUC__) || defined(__clang__) || defined(__ARMCC_VERSION)
    /* GCC 4.7+, Clang 3.0+, ARM Compiler 6+ all support __atomic builtins.
       ARM Compiler 5 (__CC_ARM) only supports __sync builtins — handled below. */
    #if defined(__CC_ARM) && !defined(__ARMCC_VERSION)
        /* ARM Compiler 5 (armcc): use legacy __sync builtins */
        #define OSL_ATOMIC_CAS(ptr, expected_val, desired_val) \
            (__sync_val_compare_and_swap(ptr, expected_val, desired_val) == expected_val)

        #define OSL_ATOMIC_STORE(ptr, val) \
            __sync_lock_release(ptr)
    #else
        /* GCC, Clang, ARM Compiler 6 (armclang): use __atomic builtins
           with proper memory ordering (ACQUIRE for lock, RELEASE for unlock) */
        #define OSL_ATOMIC_CAS(ptr, expected_val, desired_val) \
            ({ uint32_t _exp = (expected_val); \
              __atomic_compare_exchange_n(ptr, &_exp, (desired_val), \
                  0, __ATOMIC_ACQUIRE, __ATOMIC_RELAXED); })

        #define OSL_ATOMIC_STORE(ptr, val) \
            __atomic_store_n(ptr, (val), __ATOMIC_RELEASE)
    #endif
#endif

#define OSL_SYNC_UNLOCKED    0
#define OSL_SYNC_LOCKED      1

/**
 * @brief mutex lock structure
 */
typedef struct {
    volatile uint32_t lock;  /**< Lock state: 0=unlocked, 1=locked */
} osl_mutex_t;

/**
 * @brief Initialize a mutex lock
 * @param mutex Pointer to the mutex structure to initialize
 * @return 0 on success, -1 on error
 */
int osl_mutex_init(osl_mutex_t* mutex);

/**
 * @brief Destroy a mutex lock
 * @param mutex Pointer to the mutex structure to destroy
 * @return 0 on success, -1 on error
 */
int osl_mutex_destroy(osl_mutex_t* mutex);

/**
 * @brief Lock a mutex (blocking)
 * @param mutex Pointer to the mutex structure to lock
 * @return 0 on success, -1 on error
 */
int osl_mutex_lock(osl_mutex_t* mutex);

/**
 * @brief Try to lock a mutex (non-blocking)
 * @param mutex Pointer to the mutex structure to try lock
 * @return 0 on success (locked), -1 if already locked
 */
int osl_mutex_trylock(osl_mutex_t* mutex);

/**
 * @brief Unlock a mutex
 * @param mutex Pointer to the mutex structure to unlock
 * @return 0 on success, -1 on error
 */
int osl_mutex_unlock(osl_mutex_t* mutex);

#ifdef __cplusplus
}
#endif

#endif