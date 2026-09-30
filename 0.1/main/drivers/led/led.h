
#ifndef _LED_H_
#define _LED_H_

#include <stdio.h> 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

void led_init(void);
void led_on(void);
void led_off(void);

#endif