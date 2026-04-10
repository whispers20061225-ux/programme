/*
 * app_main.c
 *
 *  Created on: Apr 8, 2026
 *      Author: whispers
 */

#include "host_link.h"
#include "app_main.h"
#include "servo_bus.h"
#include "tactile_bus.h"
#include "tactile_api.h"
#include "usart.h"

void app_init(void)
{
    host_link_init();
    servo_bus_init(&huart1);
    tactile_bus_init(&huart2);
    tactile_api_init();
}

void app_poll(void)
{
    host_link_poll();
}
