/**
 * Copyright (c), 2012~2024 iot.10086.cn All Rights Reserved
 *
 * @file osl_linux.c
 * @brief base API for linux
 */

/*****************************************************************************/
/* Includes                                                                  */
/*****************************************************************************/
#include "plat_osl.h"
#include "plat_time.h"

#include "err_def.h"
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <fcntl.h>
#include <unistd.h>
/*****************************************************************************/
/* Local Definitions ( Constant and Macro )                                  */
/*****************************************************************************/
#define GPIO_PATH "/sys/class/gpio/"

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
void* osl_malloc(size_t size)
{
    return malloc(size);
}

void* osl_calloc(size_t num, size_t size)
{
    return calloc(num, size);
}

void osl_free(void* ptr)
{
    if (ptr) {
        free(ptr);
        ptr = NULL;
    }
}

void* osl_memcpy(void* dst, const void* src, size_t n)
{
    return memcpy(dst, src, n);
}

void* osl_memmove(void* dst, const void* src, size_t n)
{
    return memmove(dst, src, n);
}

void* osl_memset(void* dst, int32_t val, size_t n)
{
    return memset(dst, val, n);
}

uint8_t* osl_strdup(const uint8_t* s)
{
    return strdup(s);
}

uint8_t* osl_strndup(const uint8_t* s, size_t n)
{
    return strndup(s, n);
}

uint8_t* osl_strcpy(uint8_t* s1, const uint8_t* s2)
{
    return (uint8_t*)strcpy((char*)s1, (const char*)s2);
}

uint32_t osl_strlen(const uint8_t* s)
{
    return s == NULL ? 0 : strlen((const char*)s);
}

uint8_t* osl_strcat(uint8_t* dst, const uint8_t* src)
{
    return (uint8_t*)strcat((char*)dst, (const char*)src);
}

int32_t osl_strcmp(const uint8_t* s1, const uint8_t* s2)
{
    return strcmp((const char*)s1, (const char*)s2);
}

int32_t osl_strncmp(const uint8_t* s1, const uint8_t* s2, size_t n)
{
    return strncmp((const char*)s1, (const char*)s2, n);
}

uint8_t* osl_strstr(const uint8_t* s1, const uint8_t* s2)
{
    return (uint8_t*)strstr((const char*)s1, (const char*)s2);
}

int32_t osl_sprintf(uint8_t* str, const uint8_t* format, ...)
{
    va_list ap;
    int     ret = 0;

    va_start(ap, format);
    ret = vsprintf((char*)str, (char*)format, ap);
    va_end(ap);

    return ret;
}

int32_t osl_sscanf(const uint8_t* str, const uint8_t* format, ...)
{
    va_list ap;
    int     ret = 0;

    va_start(ap, format);
    ret = vsscanf((const char*)str, (const char*)format, ap);
    va_end(ap);

    return ret;
}

void osl_assert(bool_t expression)
{
    assert(expression);
}

int32_t osl_get_random(unsigned char* buf, size_t len)
{
    size_t i = 0;

    srand(time_count());
    for (i = 0; i < len; i++) {
        buf[i] = (rand() & 0xFF);
    }
    return 0;
}

int32_t osl_atoi(const uint8_t* nptr)
{
    return atoi((const char*)nptr);
}

int32_t osl_rand(int32_t min, int32_t max)
{
    static int s_seed = 0;
    osl_assert(max > min);

    if (s_seed == 0) {
        s_seed = time_count_ms();
        srand(s_seed);
    }

    int rand_num = rand();
    rand_num     = min + (int)((double)((double)(max) - (min) + 1.0) * ((rand_num) / ((RAND_MAX) + 1.0)));
    return rand_num;
}

int osl_init(void)
{
    // uart_stdio_init(); //Redirectable serial communication
    setbuf(stdout,NULL);
    osl_common();
}

int osl_set_config(int argc, char* argv[],char** pp_product_id,char** pp_device_sn,char** pp_auth_code) {
    return 0;
    //TODO
}

void export_gpio(int gpio_num) {
    char path[64];
    char buf[10];
    int fd;

    snprintf(path, sizeof(path), GPIO_PATH "export");
    fd = open(path, O_WRONLY);
    if (fd < 0) {
        perror("Failed to open export for writing");
        exit(EXIT_FAILURE);
    }

    snprintf(buf, sizeof(buf), "%d", gpio_num);
    if (write(fd, buf, strlen(buf)) < 0) {
        close(fd);
        perror("Failed to export GPIO");
        exit(EXIT_FAILURE);
    }

    close(fd);
}

void set_gpio_direction(int gpio_num, const char *direction) {
    char path[64];
    int fd;

    snprintf(path, sizeof(path), GPIO_PATH "gpio%d/direction", gpio_num);
    fd = open(path, O_WRONLY);
    if (fd < 0) {
        perror("Failed to open direction for writing");
        exit(EXIT_FAILURE);
    }

    if (write(fd, direction, strlen(direction)) < 0) {
        close(fd);
        perror("Failed to set GPIO direction");
        exit(EXIT_FAILURE);
    }

    close(fd);
}

void set_gpio_value(int gpio_num, int value) {
    char path[64];
    char buf[2];
    int fd;

    snprintf(path, sizeof(path), GPIO_PATH "gpio%d/value", gpio_num);
    fd = open(path, O_WRONLY);
    if (fd < 0) {
        perror("Failed to open value for writing");
        exit(EXIT_FAILURE);
    }

    snprintf(buf, sizeof(buf), "%d", value);
    if (write(fd, buf, 1) < 0) {
        close(fd);
        perror("Failed to set GPIO value");
        exit(EXIT_FAILURE);
    }

    close(fd);
}

int led(int num,int stat) {
    int gpio[4]={1,2,3,4};
    int on_off[2]={0,1};

    export_gpio(gpio[num]);
    set_gpio_direction(gpio[num], "out");
    set_gpio_value(gpio[num], on_off[stat]);
}