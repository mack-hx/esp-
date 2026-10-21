/**
 * @file ringbuf.h
 * @brief Ring buffer
 */

#ifndef __RINGBUF_H__
#define __RINGBUF_H__

/*****************************************************************************/
/* Includes                                                                  */
/*****************************************************************************/
#include "data_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************/
/* External Definition ( Constant and Macro )                                */
/*****************************************************************************/

/*****************************************************************************/
/* External Structures, Enum and Typedefs                                    */
/*****************************************************************************/
struct ringbuf_t
{
    uint8_t *buf;
    uint32_t buf_len;
    int16_t  rd_offset;
    int16_t  wr_offset;
};

/*****************************************************************************/
/* External Variables and Functions                                          */
/*****************************************************************************/
/**
 * @brief Create a ring buffer
 *
 * @param buf_size：Buffer size（Bytes）
 * @return Succeed - Buffer structure pointer，Failed - NULL
 */
void *ringbuf_create(uint32_t buf_size);

/**
 * @brief Delete Ring Buffer
 *
 * @param rb：Buffer structure pointer
 * @return None
 */
void ringbuf_delete(void *rb);

/**
 * @brief Get specified byte length data from a buffer
 *
 * @param rb：Buffer structure pointer
 * @param buf：Get the storage address of the data
 * @param buf_len：Length of data to be obtained
 * @return >0 - Length of successfully acquired data，Other - Fetch failed
 */
int32_t ringbuf_get(void *rb, uint8_t *buf, uint32_t buf_len);

/**
 * @brief Store data of a specified byte length into a buffer
 *
 * @param rb：Buffer structure pointer
 * @param buf：Data to be stored
 * @param buf_len：Length of data to be stored
 * @return >0 - Length of data successfully stored，Other - Deposit failed
 */
int32_t ringbuf_put(void *rb, uint8_t *buf, uint32_t buf_len);

/**
 * @brief Clear the ring buffer
 *
 * @param rb：Buffer structure pointer
 */
void ringbuf_reset(void *rb);

#ifdef __cplusplus
}
#endif

#endif
