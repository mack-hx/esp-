/**
 * Copyright (c), 2012~2024 iot.10086.cn All Rights Reserved
 *
 * @file uart_linux.c
 * @date 2020/07/31
 * @brief UART API for linux
 */

/*****************************************************************************/
/* Includes                                                                  */
/*****************************************************************************/
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <sys/select.h>

#include "log.h"
#include "plat_uart.h"
#include "err_def.h"
#include "plat_time.h"
#include "utils.h"
/*****************************************************************************/
/* Local Definitions ( Constant and Macro )                                  */
/*****************************************************************************/
char* g_uart_list[]={
    "",
    "/dev/ttyUSB0",
    "/dev/ttyS17",
    "/dev/ttyAMA1",
};
int g_uart_num=3;

/*****************************************************************************/
/* Structures, Enum and Typedefs                                             */
/*****************************************************************************/

/*****************************************************************************/
/* Local Function Prototype                                                  */
/*****************************************************************************/

/*****************************************************************************/
/* Local Variables                                                           */
/*****************************************************************************/
static struct int_pair_t g_baudrate_pair[]={
    {.arr={200,B200}},
    {.arr={1800,B1800}},
    {.arr={2400,B2400}},
    {.arr={4800,B4800}},
    {.arr={9600,B9600}},
    {.arr={19200,B19200}},
    {.arr={38400,B38400}},
    {.arr={57600,B57600}},
    {.arr={115200,B115200}},
    {.arr={230400,B230400}},
    {.arr={460800,B460800}},
    {.arr={500000,B500000}},
    {.arr={576000,B576000}},
    {.arr={921600,B921600}},
};
static uint16_t g_baudrate_pair_cnt = sizeof(g_baudrate_pair) / sizeof(g_baudrate_pair[0]);

static struct int_pair_t g_dsize_pair[]={
    {.arr={PORT_DATABITS_5,CS5}},
    {.arr={PORT_DATABITS_6,CS6}},
    {.arr={PORT_DATABITS_7,CS7}},
    {.arr={PORT_DATABITS_8,CS8}},
};
static uint16_t g_dsize_pair_cnt = sizeof(g_dsize_pair) / sizeof(g_dsize_pair[0]);

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/

/*****************************************************************************/
/* Function Implementation                                                   */
/*****************************************************************************/
/**
 * @brief Set the baud rate
 *
 * @param tios Pointer to Serial Port Attribute Structure
 * @param br Baud rate
 *
 * @return
 */
static int32_t set_baudrate(struct termios *tios, int32_t br)
{
    int i = 0;

    int s_baudrate_flag=-1;
    int ret=int_pair_get(g_baudrate_pair,g_baudrate_pair_cnt,0,1,br,&s_baudrate_flag);
    if (ret != ERR_OK) return ERR_INVALID_PARAM;

    /* Use the Flag Value corresponding to baud rate*/
    cfsetispeed(tios, s_baudrate_flag);
    cfsetospeed(tios, s_baudrate_flag);

    return ERR_OK;
}

/**
 * @brief Set data bit width
 *
 * @param tios Pointer to Serial Port Attribute Structure
 * @param ds Data bit width
 *
 * @return
 */
static int32_t set_data_bits(struct termios *tios, int32_t ds)
{
    int dsize = 0;

    int ret=int_pair_get(g_dsize_pair,g_dsize_pair_cnt,0,1,ds,&dsize);
    if (ret != ERR_OK) return ERR_INVALID_PARAM;

    tios->c_cflag &= ~CSIZE;
    tios->c_cflag |= dsize;

    return ERR_OK;
}

/**
 * @brief Set parity
 *
 * @param tios Pointer to Serial Port Attribute Structure
 * @param p Check mode，See PORT_PARITY_XXX
 */
static void set_parity(struct termios *tios, int32_t p)
{
    switch (p)
    {
        case PORT_PARITY_NONE: /* Not verified*/
            tios->c_cflag &= ~PARENB;
            tios->c_iflag &= ~INPCK;
            break;
        case PORT_PARITY_ODD: /* Odd check*/
            tios->c_cflag |= PARENB;
            tios->c_cflag |= PARODD;
            tios->c_iflag |= (INPCK | ISTRIP);
            break;
        case PORT_PARITY_EVEN: /* Even check*/
            tios->c_cflag |= PARENB;
            tios->c_cflag &= ~PARODD;
            tios->c_iflag |= (INPCK | ISTRIP);
            break;
        default:
            break;
    }
}

/**
 * @brief Set stop bit
 *
 * @param tios Pointer to Serial Port Attribute Structure
 * @param s See PORT_STOPBITS_XXX
 */
static void set_stopbits(struct termios *tios, int32_t s)
{
    if (s)
        tios->c_cflag |= CSTOPB;
    else
        tios->c_cflag &= ~CSTOPB;
}

osl_handle_t uart_open(struct uart_config_t *port_config)
{
    int32_t        fd    = -1;
    int32_t        flags = 0;
    struct termios tios;
    int32_t        ret = ERR_OK;

    fd = open(port_config->port_name, O_RDWR | O_NOCTTY | O_NDELAY);
    if (0 > fd)
    {
        loge("open %s failed",port_config->port_name);
        goto exit;
    }

    tcgetattr(fd, &tios);

    tios.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tios.c_iflag &= ~(IXON | IXOFF | IXANY);
    /*Prevent entered carriage from being translated as line breaks*/
    tios.c_iflag &= ~ICRNL;

    tios.c_oflag &= ~OPOST;

    tios.c_cflag |= (CLOCAL | CREAD);

    /* Set the baud rate*/
    if (ERR_INVALID_PARAM == set_baudrate(&tios, port_config->baudrate))
    {
        goto exit1;
    }
    /* Set data bit width*/
    if (ERR_INVALID_PARAM == set_data_bits(&tios, port_config->data_bits))
    {
        goto exit1;
    }

    set_parity(&tios, port_config->parity);
    set_stopbits(&tios, port_config->stop_bits);

    /* Settings take effect immediately*/
    if (0 > tcsetattr(fd, TCSANOW, &tios))
    {
        goto exit1;
    }

    // fcntl(fd, F_SETFL, 0);

    /* Set to non-blocking mode*/
    flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0)
    {
        goto exit1;
    }
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    return (osl_handle_t)fd;

exit1:
    close(fd);
exit:
    return -1;
}

int32_t uart_send(osl_handle_t uart, uint8_t *data, uint32_t data_len, uint32_t timeout_ms)
{
    int32_t        fd        = (int32_t)uart;
    int32_t        data_left = data_len;
    int32_t        ret       = -1;
    struct timeval tv;
    fd_set         fs;
    osl_handle_t       cd_hdl = countdown_start(timeout_ms);

    if (data_len == 0)
    {
        return 0;
    }

    do
    {
        FD_ZERO(&fs);
        FD_SET(fd, &fs);
        tv.tv_usec = (countdown_left(cd_hdl) % 1000) * 1000;
        tv.tv_sec  = (countdown_left(cd_hdl) / 1000);

        ret = select(fd + 1, NULL, &fs, NULL, &tv);
        if (-1 == ret)
        {
            countdown_stop(cd_hdl);
            return ERR_IO;
        }
        else if (ret)
        {
            if (FD_ISSET(fd, &fs))
            {
                ret = write(fd, (data + data_len - data_left), data_left);
                if (ret > 0)
                {
                    data_left -= ret;
                }
                else if (EAGAIN != errno)
                {
                    countdown_stop(cd_hdl);
                    return ERR_IO;
                }
            }
        }
    } while ((0 != data_left) && (0 == countdown_is_expired(cd_hdl)));
    countdown_stop(cd_hdl);
    return (data_len - data_left);
}

int32_t uart_recv(osl_handle_t uart, uint8_t *buf, uint32_t buf_len, uint32_t timeout_ms)
{
    int32_t        fd       = (int32_t)uart;
    int32_t        ret      = -1;
    int32_t        recv_len = 0;
    struct timeval tv;
    fd_set         fs;
    osl_handle_t       cd_hdl = countdown_start(timeout_ms);

    do
    {
        FD_ZERO(&fs);
        FD_SET(fd, &fs);
        tv.tv_usec = (countdown_left(cd_hdl) % 1000) * 1000;
        tv.tv_sec  = (countdown_left(cd_hdl) / 1000);
        ret        = select(fd + 1, &fs, NULL, NULL, &tv);
        if (-1 == ret)
        {
            countdown_stop(cd_hdl);
            return ERR_IO;
        }
        else if (0 < ret)
        {
            if (FD_ISSET(fd, &fs))
            {
                ret = read(fd, buf + recv_len, buf_len - recv_len);
                if (ret > 0)
                {
                    recv_len += ret;
                }
                else if (EAGAIN != errno)
                {
                    countdown_stop(cd_hdl);
                    return ERR_IO;
                }
            }
        }
    } while ((recv_len != buf_len) && (0 == countdown_is_expired(cd_hdl)));

    countdown_stop(cd_hdl);
    return recv_len;
}

void uart_close(osl_handle_t uart)
{
    close((int32_t)uart);
}

int32_t uart_stdio_init()
{
    FILE* r=freopen(g_uart_list[UART_STDIO_FD],"r",stdin);
    FILE* w=freopen(g_uart_list[UART_STDIO_FD],"w",stdout);
    if(r && w) return ERR_OK;
    else return ERR_INITIALIZING;
}