#include "who_human_face_recognition.hpp"
#include "dl_image.hpp"
#include "human_face_detect_msr01.hpp"
#include "human_face_detect_mnp01.hpp"
#include "face_recognition_112_v1_s16.hpp"
#include "who_ai_utils.hpp"
#include "esp_camera.h"
#include "spilcd.h"
#include "fb_gfx.h"

QueueHandle_t xQueueAIFrame = NULL;
static QueueHandle_t xQueueLCDFrame = NULL; 


static QueueHandle_t xQueueKeyState = NULL;
static QueueHandle_t gpio_evt_queue = NULL;
static QueueHandle_t xQueueEventLogic = NULL;
SemaphoreHandle_t xMutex;

static recognizer_state_t gEvent = DETECT;
static face_info_t recognize_result;



/**
 * @brief       输出打印
 * @param       fb：按键标号
 * @param       color：颜色
 * @param       format：字符
 * @retval      无
 */
static void esp_rgb_print(camera_fb_t *fb, uint32_t color, const char *str)
{
    fb_gfx_print(fb, (fb->width - (strlen(str) * 14)) / 2, 10, color, str);
}

/**
 * @brief       输出打印
 * @param       fb：按键标号
 * @param       color：颜色
 * @param       format：字符
 * @retval      无
 */
static int esp_rgb_printf(camera_fb_t *fb, uint32_t color, const char *format, ...)
{
    char loc_buf[64];
    char *temp = loc_buf;
    int len;
    va_list arg;
    va_list copy;
    va_start(arg, format);
    va_copy(copy, arg);
    len = vsnprintf(loc_buf, sizeof(loc_buf), format, arg);
    va_end(copy);

    if (len >= sizeof(loc_buf))
    {
        temp = (char *)malloc(len + 1);
        if (temp == NULL)
        {
            return 0;
        }
    }

    vsnprintf(temp, len + 1, format, arg);
    va_end(arg);
    esp_rgb_print(fb, color, temp);

    if (len > 64)
    {
        free(temp);
    }

    return len;
}

// AI处理任务
static void task_process_ai(void *arg)
{
    camera_fb_t *frame = NULL;
    HumanFaceDetectMSR01 detector(0.3F, 0.3F, 10, 0.3F);
    HumanFaceDetectMNP01 detector2(0.4F, 0.3F, 10);
    FaceRecognition112V1S16 *recognizer = new FaceRecognition112V1S16();
    show_state_t frame_show_state = SHOW_STATE_IDLE;
    recognizer_state_t _gEvent;

    recognizer->set_partition(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage");
    recognizer->set_ids_from_flash();
    while(1)
    {
        xSemaphoreTake(xMutex, portMAX_DELAY);
        _gEvent = gEvent;
        gEvent = DETECT;
        xSemaphoreGive(xMutex);

        if (_gEvent)
        {
            bool is_detected = false;

            if (xQueueReceive(xQueueAIFrame, &frame, portMAX_DELAY))
            {
                std::list<dl::detect::result_t> &detect_candidates = detector.infer((uint16_t *)frame->buf, {(int)frame->height, (int)frame->width, 3});
                std::list<dl::detect::result_t> &detect_results = detector2.infer((uint16_t *)frame->buf, {(int)frame->height, (int)frame->width, 3}, detect_candidates);

                if (detect_results.size() == 1)
                    is_detected = true;

                if (is_detected)
                {
                    switch (_gEvent)
                    {
                        /* 注册 -长按 */
                        case ENROLL:
                            recognizer->enroll_id((uint16_t *)frame->buf, {(int)frame->height, (int)frame->width, 3}, detect_results.front().keypoint, "", true);
                            ESP_LOGW("ENROLL", "ID %d is enrolled", recognizer->get_enrolled_ids().back().id);
                            frame_show_state = SHOW_STATE_ENROLL;
                            break; 
                        /* 识别-短按 */
                        case RECOGNIZE:
                            recognize_result = recognizer->recognize((uint16_t *)frame->buf, {(int)frame->height, (int)frame->width, 3}, detect_results.front().keypoint);
                            print_detection_result(detect_results);
                            if (recognize_result.id > 0)
                                ESP_LOGI("RECOGNIZE", "Similarity: %f, Match ID: %d", recognize_result.similarity, recognize_result.id);
                            else
                                ESP_LOGE("RECOGNIZE", "Similarity: %f, Match ID: %d", recognize_result.similarity, recognize_result.id);
                            frame_show_state = SHOW_STATE_RECOGNIZE;
                            break;
                        /* 删除--双击 */
                        case DELETE:
                            vTaskDelay(10);
                            recognizer->delete_id(true);
                            ESP_LOGE("DELETE", "% d IDs left", recognizer->get_enrolled_id_num());
                            frame_show_state = SHOW_STATE_DELETE;
                            break;

                        default:
                            break;
                    }
                }

                if (frame_show_state != SHOW_STATE_IDLE)
                {
                    static int frame_count = 0;

                    switch (frame_show_state)
                    {
                        /* 删除图像 */
                        case SHOW_STATE_DELETE:
                            esp_rgb_printf(frame, RGB565_MASK_RED, "%d IDs left", recognizer->get_enrolled_id_num());
                            break;
                        /* 图像识别 */
                        case SHOW_STATE_RECOGNIZE:
                            if (recognize_result.id > 0)
                                esp_rgb_printf(frame, RGB565_MASK_GREEN, "ID %d", recognize_result.id);
                            else
                                esp_rgb_print(frame, RGB565_MASK_RED, "who ?");
                            break;
                        /* 图像注册 */
                        case SHOW_STATE_ENROLL:
                            esp_rgb_printf(frame, RGB565_MASK_BLUE, "Enroll: ID %d", recognizer->get_enrolled_ids().back().id);
                            break;

                        default:
                            break;
                    }

                    if (++frame_count > FRAME_DELAY_NUM)
                    {
                        frame_count = 0;
                        frame_show_state = SHOW_STATE_IDLE;
                    }
                }

                if (detect_results.size())
                {
                    draw_detection_result((uint16_t *)frame->buf, frame->height, frame->width, detect_results);
                }
            }

            if (xQueueLCDFrame)
            {
                xQueueSend(xQueueLCDFrame, &frame, portMAX_DELAY);
            }
            else
            {
                free(frame);
            }
        }
    }
}

// lcd处理任务
static void task_process_lcd(void *arg)
{
    camera_fb_t *frame = NULL;

    while (true)
    {
        if (xQueueReceive(xQueueLCDFrame, &frame, portMAX_DELAY))
        {
            lcd_draw_bitmap(0, 0, frame->width, frame->height, (uint16_t *)frame->buf);
        }
    }
}


/**
 * @brief       事件处理任务
 * @param       arg：未使用
 * @retval      无
 */
static void esp_task_event_handler(void *arg)
{
    arg = arg;
    recognizer_state_t _gEvent;

    while (1)
    {
        xQueueReceive(xQueueEventLogic, &(_gEvent), portMAX_DELAY);
        xSemaphoreTake(xMutex, portMAX_DELAY);
        gEvent = _gEvent;
        xSemaphoreGive(xMutex);
    }
}

/**
 * @brief       事件生成任务
 * @param       arg：未使用
 * @retval      无
 */
void esp_event_generate(void *arg)
{
    arg = arg;
    static key_state_t key_state;
    recognizer_state_t recognizer_state;

    while (1)
    {
        /* 接收状态 */
        xQueueReceive(xQueueKeyState, &key_state, portMAX_DELAY);
        /* 判断状态 */
        switch (key_state)
        {
            case KEY_SHORT_PRESS:   /* 短按状态 */
                recognizer_state = RECOGNIZE;
                break;

            case KEY_LONG_PRESS:    /* 长按状态 */
                recognizer_state = ENROLL;
                break;

            case KEY_DOUBLE_CLICK:  /* 双击状态 */
                recognizer_state = DELETE;
                break;

            default:
                recognizer_state = DETECT;
                break;
        }
        /* 发送状态 */
        xQueueSend(xQueueEventLogic, &recognizer_state, portMAX_DELAY);
    }
}

/**
 * @brief       按键扫描(判断短按、长按及双击状态)
 * @param       ticks_to_wait：等待时间
 * @retval      返回按键状态
 */
int esp_key_scan(TickType_t ticks_to_wait)
{
    gpio_num_t io_num;
    BaseType_t press_key = pdFALSE;
    BaseType_t lift_key = pdFALSE;
    int64_t backup_time = 0;
    int64_t interval_time = 0;
    static int64_t last_time = 0;

    while (1)
    {
        xQueueReceive(gpio_evt_queue, &io_num, ticks_to_wait);

        if (gpio_get_level(io_num) == 0)
        {
            press_key = pdTRUE;
            backup_time = esp_timer_get_time();
            interval_time = backup_time - last_time;
        }
        else if (press_key)
        {
            lift_key = pdTRUE;
            last_time = esp_timer_get_time();
            backup_time = last_time - backup_time;
        }

        if (press_key & lift_key)
        {
            press_key = pdFALSE;
            lift_key = pdFALSE;

            if (backup_time > LONG_PRESS_THRESH)
            {
                return KEY_LONG_PRESS;
            }
            else
            {
                if ((interval_time < DOUBLE_CLICK_THRESH) && (interval_time > 0))
                    return KEY_DOUBLE_CLICK;
                else
                    return KEY_SHORT_PRESS;
            }
        }
    }
}

/**
 * @brief       按键任务
 * @param       arg：未使用
 * @retval      无
 */
static void esp_key_trigger(void *arg)
{
    arg = arg;
    int ret = 0;

    while (1)
    {
        ret = esp_key_scan(portMAX_DELAY);
        xQueueOverwrite(xQueueKeyState, &ret);
    }

    vTaskDelete(NULL);
}

/**
 * @brief       按键中断回调函数
 * @param       arg：按键标号
 * @retval      无
 */
static void IRAM_ATTR esp_gpio_isr_handler_key(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

/**
 * @brief       BOOT按键初始化
 * @param       gpio_num：IO标号
 * @retval      无
 */
void esp_key_init(gpio_num_t gpio_num)
{
    gpio_config_t io_conf;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.pin_bit_mask = 1ull << gpio_num;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
    gpio_evt_queue = xQueueCreate(5, sizeof(uint32_t));
    gpio_install_isr_service(0);
    gpio_isr_handler_add(gpio_num, esp_gpio_isr_handler_key, (void *)gpio_num);
}

// 人脸检测
void human_face_recognition_start(void)
{
    xQueueLCDFrame = xQueueCreate(5, sizeof(camera_fb_t *));
    xQueueAIFrame = xQueueCreate(5, sizeof(camera_fb_t *));
    xQueueKeyState = xQueueCreate(1, sizeof(int *));
    xQueueEventLogic = xQueueCreate(1, sizeof(int *));
    xMutex = xSemaphoreCreateMutex();

    /* 初始化按键 */
    esp_key_init(GPIO_NUM_0);

    /* 创建任务 */
    xTaskCreatePinnedToCore(esp_key_trigger, "esp_key_scan_task", 2 * 1024, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(esp_event_generate, "event_logic_task", 2 * 1024, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(esp_task_event_handler, "esp_task_event_handler", 2 * 1024, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(task_process_lcd, "task_process_lcd", 4 * 1024, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(task_process_ai, "task_process_ai", 6 * 1024, NULL, 5, NULL, 0);    //不能是1
}
