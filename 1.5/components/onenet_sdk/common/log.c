/**
 * Copyright (c), 2012~2018 iot.10086.cn All Rights Reserved
 * @file        log.c
 * @brief       Log library
 */

#include "plat_time.h"
#include "log.h"

#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <time.h>

//__unix__ defined by gcc，armcc define it？
//__APPLE__ defined in mac OS
#if !(defined(__unix__) || defined(__APPLE__))
    #define STYLE_DEBUG_L "    "
    #define STYLE_DEBUG_R "    "
    #define STYLE_INFO_L  "  [["
    #define STYLE_INFO_R  "]]]  "
    #define STYLE_ERROR_L "!!!!"
    #define STYLE_ERROR_R "!!!!"
    #define STYLE_TIME_L  ""
    #define STYLE_TIME_R  ""
#else
    //ANSI_ESCAPE_CODE,With ESC(ASCII Code 27,Hexadecimal 0x1B,Octal 033)Control character at the beginning，Primary supported by Unix Platform's Xterm Standard terminal
    // windows 10 and later also support it
    #define ANSI_ESC_NONE       "\033[0m" //Reset colors and styles
    #define ANSI_ESC_HILIGHT    "\033[1m" //Settings highlighting
    #define ANSI_ESC_UNDERLINE  "\033[4m" //Underline
    #define ANSI_ESC_BLINK      "\033[5m" //Flicker
    #define ANSI_ESC_HIDE       "\033[8m" //Blanking,Text will be hidden from display,But can be copied
    #define ANSI_ESC_RED        "\033[31m" //Red
    #define ANSI_ESC_GREEN      "\033[32m" //Green
    #define ANSI_ESC_YELLOW     "\033[33m" //Yellow
    #define ANSI_ESC_BLUE       "\033[34m" //Blue
    #define ANSI_ESC_PURPLE     "\033[35m" //Purple，Magenta
    #define ANSI_ESC_CYAN       "\033[36m" //Cyan，Light blue
    #define ANSI_ESC_WHITE      "\033[37m" //White

    #define STYLE_DEBUG_L ANSI_ESC_NONE
    #define STYLE_DEBUG_R ANSI_ESC_NONE
    #define STYLE_INFO_L  ANSI_ESC_GREEN""ANSI_ESC_HILIGHT
    #define STYLE_INFO_R  ANSI_ESC_NONE" "
    #define STYLE_ERROR_L ANSI_ESC_RED
    #define STYLE_ERROR_R ANSI_ESC_NONE
    #define STYLE_TIME_L  ANSI_ESC_PURPLE
    #define STYLE_TIME_R  ANSI_ESC_NONE
#endif

/**************************************************************************************/
typedef struct logger_s
{
    logger_handler handler;
    unsigned int bufsize;
    char buf[LOG_MAX_BUFSIZE];
    int level;
    int enable_color;
} logger_t;

static logger_t *s_logger = NULL;

#define logger_usr_enable() \
    do \
    { \
        printf("Custom logger is enabled,%s() is invalid!", __FUNCTION__); \
        return; \
    } while(0)
/**************************************************************************************/

/// @brief Create OneNET Logger
/// @return OneNET Logger handle
static logger_t *logger_create();

/// @brief Destroy OneNET Logger
/// @param logger
static void logger_destroy(logger_t *logger);

/// @brief Initialize OneNET Logger
/// @param logger OneNET Logger handle
static void logger_init(logger_t *logger);

/// @brief Create a default logger(printf Output)
/// @return OneNET Logger handle
static logger_t *default_logger();

/// @brief Destroy Default OneNET Logger
static void destroy_default_logger(void);
/**************************************************************************************/

void logger_set_level(int level)
{
    default_logger();
    if(s_logger->handler)
        logger_usr_enable();
    s_logger->level = (level < LOG_LEVEL_VERBOSE || level > LOG_LEVEL_SILENT) ? LOG_LEVEL : level;
}

void logger_set_level_by_str(const char *level)
{
    default_logger();
    if(s_logger->handler)
        logger_usr_enable();
    int loglevel = LOG_LEVEL;
    if(osl_strcmp(level, "VERBOSE") == 0)
    {
        loglevel = LOG_LEVEL_VERBOSE;
    }
    else if(osl_strcmp(level, "DEBUG") == 0)
    {
        loglevel = LOG_LEVEL_DEBUG;
    }
    else if(osl_strcmp(level, "INFO") == 0)
    {
        loglevel = LOG_LEVEL_INFO;
    }
    else if(osl_strcmp(level, "ERROR") == 0)
    {
        loglevel = LOG_LEVEL_ERROR;
    }
    else if(osl_strcmp(level, "SILENT") == 0)
    {
        loglevel = LOG_LEVEL_SILENT;
    }
    else
    {
        loglevel = LOG_LEVEL;
    }
    s_logger->level = loglevel;
}

void logger_disable()
{
    if(s_logger->handler)
        logger_usr_enable();
    logger_set_level(LOG_LEVEL_SILENT);
}

void logger_color(int on)
{
    default_logger();
    if(s_logger->handler)
        logger_usr_enable();
    s_logger->enable_color = on > 0 ? 1 : 0;
}

void logger_set_handler(logger_handler handler)
{
    default_logger();
    s_logger->handler = handler;
}

int logger_print(int level, const char* file, int line, const char* func, const char* suffix, const char* fmt, ...)
{
    default_logger();

    char *buf = s_logger->buf;
    int bufsize = s_logger->bufsize;
    int len = 0;

    if(!s_logger->handler) {
        if (level < s_logger->level) return -10;

        int year, month, day, hour, min, sec, ms;

        time_get_date(&year, &month, &day, &hour, &min, &sec, &ms);

        char level_str[64]      = { 0 };
        char timestamp_str[128] = { 0 };
        switch (level) {
            case LOG_LEVEL_DEBUG:
                snprintf(level_str, sizeof(level_str), "%s", STYLE_DEBUG_L"DEBUG"STYLE_DEBUG_R);
                break;
            case LOG_LEVEL_INFO:
                snprintf(level_str, sizeof(level_str), "%s", STYLE_INFO_L"INFO"STYLE_INFO_R);
                break;
            case LOG_LEVEL_ERROR:
                snprintf(level_str, sizeof(level_str), "%s", STYLE_ERROR_L"ERROR"STYLE_ERROR_R);
                break;
            default:
                break;
        }

        snprintf(timestamp_str, sizeof(timestamp_str), "%04d-%02d-%02d %02d:%02d:%02d.%03d", year, month, day, hour, min, sec, ms);

        len += snprintf(buf + len, bufsize - len, STYLE_TIME_L"%s"STYLE_TIME_R" %s ", timestamp_str, level_str);
        len += snprintf(buf + len, bufsize - len, "[%s:%s:%d] ", file, func, line);

        va_list ap;
        va_start(ap, fmt);
        len += vsnprintf(buf + len, bufsize - len, fmt, ap);
        va_end(ap);

        printf("%s", buf);
        if(suffix) printf("%s", suffix);
    } else {
        va_list ap;
        va_start(ap, fmt);
        len += vsnprintf(buf + len, bufsize - len, fmt, ap);
        va_end(ap);
        s_logger->handler(level, file, line, func, buf, len);
    }
    return len;
}

/**************************************************************************************/
void logger_init(logger_t *logger)
{
    logger->handler = NULL;
    logger->bufsize = LOG_MAX_BUFSIZE;
    logger->level = LOG_LEVEL;
    logger->enable_color = 0;
}

logger_t *logger_create()
{
    logger_t *logger = (logger_t *)malloc(sizeof(logger_t));
    logger_init(logger);
    return logger;
}

void logger_destroy(logger_t *logger)
{
    if(logger)
    {
        free(logger);
        logger = NULL;
    }
}

logger_t *default_logger()
{
    if(!s_logger)
    {
        s_logger = logger_create();
        atexit(destroy_default_logger);
    }
    return s_logger;
}

void destroy_default_logger()
{
    logger_destroy(s_logger);
}

// UTFCode
// 0xxxxxxx：Single-byte encoding form，This is exactly the same as ASCII；
// 110xxxxx 10xxxxxx：Double-byte encoding form
// 1110xxxx 10xxxxxx 10xxxxxx：Three-byte encoded form
// 11110xxx 10xxxxxx 10xxxxxx 10xxxxxx：Four-byte encoding form
int logger_print_buf(int level, const char* file, int line, const char* func, const char* name, const char* buf,int len) {
    logger_print(level, file, line, func, NULL,"");
    int i=0;
    printf("%s(buffer,%p,%d): ",name,buf,len);
    // When buf is only one character，output HEX String
    if(len==1) {
        printf("[%02X]\n",(uint8_t)buf[0]);
        return 0;
    }

    char str[SDK_RECV_BUF_LEN]={0};
    int non_ascii_byte=0;
    int n=0;

    #ifndef SDK_USE_NBIOT
        int msg_header_len=13; //COAP
        int is_packet=0;
    #endif
    #ifdef SDK_USE_COAP
        if(len>msg_header_len) is_packet=1;
    #endif
    #ifdef SDK_USE_MQTTS
    if(len>2) {
        for( msg_header_len=2 ; msg_header_len<10 && msg_header_len<len-4 ; msg_header_len++ ) {
            if ( strncmp(buf+msg_header_len,"MQTT",4)==0 || strncmp(buf+msg_header_len,"$sys",4)==0 ) {
                is_packet=1;
                break;
            }
        }
    }
    #endif

    for(i=0;i<len;i++) {
        uint8_t c=buf[i];

        #ifndef SDK_USE_NBIOT
        //output Header in Hexadecimal format
        if(is_packet && i < msg_header_len) {
            printf("[%02X]",c);
            continue;
        }
        #endif

        if(non_ascii_byte) {
            non_ascii_byte--;
        }

        // 2Byte encoding and 4Byte encoding characters belonging to other languages，There is a high probability that it does not exist in the log，Here it is considered binary data，So give up
        if(non_ascii_byte==0) {
            if ( (c>>5)==0b110 ) { //2Byte encoding
                // non_ascii_byte=2;
            }
            else if ( (c>>4)==0b1110 ) { //3Byte encoding,Chinese characters
                #ifndef SDK_USE_COAP
                uint8_t c1=buf[i+1];
                uint8_t c2=buf[i+2];
                if ( (len>=3) && ((c1>>6)==0b10) && ((c2>>6)==0b10) ) non_ascii_byte=3;
                #endif
            }
            else if ( (c>>3)==0b11110 ) { //4Byte encoding
                // non_ascii_byte=4;
            }
        }
        // isprint Function，Check that the character belongs to letters, numbers, symbols, line breaks,TAB, spaces
        // isgraph Function，Check that the character belongs to letters, numbers, or symbols
        // Letters, numbers, symbols, spaces are output as they are，Other（Include line breaks，Display as[0D][0A]),display in HEX String
        if( !isgraph(c) && non_ascii_byte==0 && c!=' ') {
            n+=snprintf(&str[n],SDK_RECV_BUF_LEN-n,"[%02X]",c);
        }
        else {
            str[n]=c;
            n++;
        }
    }

    printf("%s\n",str);

    return 0;
}
