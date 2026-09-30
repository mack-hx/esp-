/*
 * @Author: chenxingxing 812150818@qq.com
 * @Date: 2025-12-18 17:22:23
 * @LastEditors: chenxingxing 812150818@qq.com
 * @LastEditTime: 2025-12-31 17:49:24
 * @FilePath: \ai-esp32s3\main\drivers\camera\camera.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "camera.h"
#include <usb_stream.h>
#include <jpeg_decoder.h>
#include "spilcd.h"
#include "esp_camera.h"

static const char *TAG = "uvc_camera";

static uint8_t *xfer_buffer_a = NULL;
static uint8_t *xfer_buffer_b = NULL;
static uint8_t *frame_buffer = NULL;

static uint16_t frame_width_;
static uint16_t frame_height_;

static uint8_t *img_camera_buf_ = NULL;
static camera_fb_t *camera_frame_ = NULL;

extern QueueHandle_t xQueueAIFrame;
static void camera_frame_cb(uvc_frame_t *frame, void *ptr)
{
    // ESP_LOGI(TAG, "uvc callback! frame_format = %d, seq = %"PRIu32", width = %"PRIu32", height = %"PRIu32", length = %u, ptr = %d",
    //          frame->frame_format, frame->sequence, frame->width, frame->height, frame->data_bytes, (int) ptr);
    if (img_camera_buf_ == NULL)
    {
        ESP_LOGE(TAG, "img_camera_buf_ is NULL");
        return;
    }
    if (camera_frame_ == NULL)
    {
        ESP_LOGE(TAG, "camera_frame_ is NULL");
        return;
    }
    switch (frame->frame_format)
    {
    case UVC_FRAME_FORMAT_MJPEG: //
        esp_jpeg_image_cfg_t jpeg_cfg = {
            .indata = frame->data,
            .indata_size = frame->data_bytes,
            .outbuf = (uint8_t *)img_camera_buf_,                                                         // 输出到缓冲区
            .outbuf_size = (uint32_t)(spilcddev.width) * (uint32_t)(spilcddev.height) * sizeof(uint16_t), // RGB565: 2 bytes per pixel
            .out_format = JPEG_IMAGE_FORMAT_RGB565,
            .out_scale = JPEG_IMAGE_SCALE_0, // 不缩放，保持原尺寸
            .flags.swap_color_bytes = 1,     // 交换颜色字节
        };
        esp_jpeg_image_output_t outimg;
        esp_err_t ret = esp_jpeg_decode(&jpeg_cfg, &outimg);
        if (ret == ESP_OK)
        {
            camera_frame_->width = outimg.width;
            camera_frame_->height = outimg.height;
            camera_frame_->len = outimg.output_len;
            camera_frame_->buf =img_camera_buf_;
            xQueueSend(xQueueAIFrame, &camera_frame_, portMAX_DELAY);
        }
        break;
    default:
        break;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
}
static void camera_stream_state_changed_cb(usb_stream_state_t event, void *arg)
{

    switch (event)
    {
    case STREAM_CONNECTED:
    {
        size_t frame_size = 0;
        size_t frame_index = 0;
        uvc_frame_size_list_get(NULL, &frame_size, &frame_index);
        if (frame_size)
        {
            ESP_LOGI(TAG, "UVC: get frame list size = %u, current = %u", frame_size, frame_index);
            uvc_frame_size_t *uvc_frame_list = (uvc_frame_size_t *)malloc(frame_size * sizeof(uvc_frame_size_t));
            uvc_frame_size_list_get(uvc_frame_list, NULL, NULL);
            for (size_t i = 0; i < frame_size; i++)
            {
                ESP_LOGI(TAG, "\tframe[%u] = %ux%u", i, uvc_frame_list[i].width, uvc_frame_list[i].height);
            }
            free(uvc_frame_list);
        }
        else
        {
            ESP_LOGW(TAG, "UVC: get frame list size = %u", frame_size);
        }
        ESP_LOGI(TAG, "Device connected");
        break;
    }
    case STREAM_DISCONNECTED:
        ESP_LOGI(TAG, "Device disconnected");
        break;
    default:
        ESP_LOGE(TAG, "Unknown event");
        break;
    }
}

static void camera_uvc_init(void)
{
    // 分配传输缓冲区
    xfer_buffer_a = (uint8_t *)malloc(DEMO_UVC_XFER_BUFFER_SIZE);
    assert(xfer_buffer_a != NULL);
    xfer_buffer_b = (uint8_t *)malloc(DEMO_UVC_XFER_BUFFER_SIZE);
    assert(xfer_buffer_b != NULL);
    // 分配帧缓冲区
    frame_buffer = (uint8_t *)malloc(DEMO_UVC_XFER_BUFFER_SIZE);
    assert(frame_buffer != NULL);
    // 配置UVC
    uvc_config_t uvc_config = {
        .frame_width = frame_width_,
        .frame_height = frame_height_,
        .frame_interval = FPS2INTERVAL(15),
        .xfer_buffer_size = DEMO_UVC_XFER_BUFFER_SIZE,
        .xfer_buffer_a = xfer_buffer_a,
        .xfer_buffer_b = xfer_buffer_b,
        .frame_buffer_size = DEMO_UVC_XFER_BUFFER_SIZE,
        .frame_buffer = frame_buffer,
        .frame_cb = &camera_frame_cb,
        .frame_cb_arg = NULL,
    };
    esp_err_t ret = uvc_streaming_config(&uvc_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "uvc streaming config failed");
        return;
    }
    ESP_LOGI(TAG, "UVC initialized successfully");

    return;
}
void camera_init(uint16_t frame_width, uint16_t frame_height)
{
    frame_width_ = frame_width;
    frame_height_ = frame_height;
    
    img_camera_buf_ = (uint8_t *)heap_caps_malloc(spilcddev.width * spilcddev.height * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
    assert(img_camera_buf_ != NULL);

    camera_frame_ = (camera_fb_t *)heap_caps_malloc(sizeof(camera_fb_t), MALLOC_CAP_SPIRAM);
    assert(camera_frame_ != NULL);

    camera_uvc_init();
    /* register the state callback to get connect/disconnect event
     * in the callback, we can get the frame list of current device*/
    ESP_ERROR_CHECK(usb_streaming_state_register(&camera_stream_state_changed_cb, NULL));
    /* start usb streaming, UVC and UAC MIC will start streaming because SUSPEND_AFTER_START flags not set */
    ESP_ERROR_CHECK(usb_streaming_start());
}

void camera_uvc_cleanup(void)
{
    if (xfer_buffer_a != NULL)
    {
        free(xfer_buffer_a);
        xfer_buffer_a = NULL;
    }

    if (xfer_buffer_b != NULL)
    {
        free(xfer_buffer_b);
        xfer_buffer_b = NULL;
    }

    if (frame_buffer != NULL)
    {
        free(frame_buffer);
        frame_buffer = NULL;
    }
    ESP_LOGI(TAG, "UVC resources cleaned up");
}
