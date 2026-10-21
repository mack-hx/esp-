/**
 * Copyright (c), 2012~2018 iot.10086.cn All Rights Reserved
 * @file        common.h
 * @brief       Generic macro definitions
 */
#ifndef _COMMON_H_
#define _COMMON_H_

#include <stdio.h>
#include <stdlib.h>

#include "err_def.h"
#include "log.h"

#ifndef SAFE_ALLOC
#define SAFE_ALLOC(ptr, n)                                                                                                                                     \
    do {                                                                                                                                                       \
        ptr = malloc(n);                                                                                                                                       \
        if (!ptr) {                                                                                                                                            \
            fprintf(stderr, "malloc failed!\n");                                                                                                               \
            exit(-1);                                                                                                                                          \
        }                                                                                                                                                      \
        memset(ptr, '\0', n);                                                                                                                                  \
    } while (0)
#endif

#ifndef SAFE_FREE
#define SAFE_FREE(ptr)                                                                                                                                         \
    do {                                                                                                                                                       \
        if (ptr) {                                                                                                                                             \
            free(ptr);                                                                                                                                         \
            ptr = NULL;                                                                                                                                        \
        }                                                                                                                                                      \
    } while (0)
#endif

typedef struct data_s
{
    unsigned int size;
    char*        val;
} data_t;

#define data_alloc(d, n)                                                                                                                                       \
    do {                                                                                                                                                       \
        if (d == NULL) {                                                                                                                                       \
            SAFE_ALLOC(d, sizeof(data_t));                                                                                                                     \
            if (n > 0) SAFE_ALLOC(d->val, n);                                                                                                                  \
            d->size = n;                                                                                                                                       \
        }                                                                                                                                                      \
    } while (0)

#define data_free(d)                                                                                                                                           \
    do {                                                                                                                                                       \
        if (d != NULL) {                                                                                                                                       \
            SAFE_FREE(d->val);                                                                                                                                 \
            SAFE_FREE(d);                                                                                                                                      \
        }                                                                                                                                                      \
    } while (0)

#endif