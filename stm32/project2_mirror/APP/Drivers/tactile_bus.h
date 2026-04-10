/*
 * tactile_bus.h
 *
 *  Created on: Apr 8, 2026
 *      Author: whispers
 */

#ifndef APP_DRIVERS_TACTILE_BUS_H_
#define APP_DRIVERS_TACTILE_BUS_H_

#include "stm32f1xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
typedef struct
{
    uint32_t rx_bytes;
    uint32_t rx_overflow;
    uint32_t rx_pe;
    uint32_t rx_ne;
    uint32_t rx_fe;
    uint32_t rx_ore;
    uint32_t rx_rearm_fail;
} tactile_bus_stats_t;

/**
 * @brief 初始化触觉传感器 UART 总线
 * @param huart 绑定到触觉传感器的 UART 句柄（你这里是 huart2）
 */
void tactile_bus_init(UART_HandleTypeDef *huart);

/**
 * @brief 发送一帧数据到触觉传感器
 * @param data       发送缓冲区
 * @param len        长度
 * @param timeout_ms 超时（毫秒）
 * @return 0 成功；负数失败
 */
int tactile_bus_send(const uint8_t *data, uint16_t len, uint32_t timeout_ms);

/**
 * @brief 从 UART 缓冲中接收一整帧触觉传感器回包
 *
 * 当前实现思路：
 * 1. 先在接收流里找到帧头 AA 55
 * 2. 再读取长度字段（2字节，小端）
 * 3. 再根据长度读后续内容
 *
 * @param out        输出缓冲区
 * @param out_cap    输出缓冲区容量
 * @param timeout_ms 超时（毫秒）
 * @return >0 = 实际收到的整帧长度；<0 = 失败
 */
int tactile_bus_recv_frame(uint8_t *out, uint16_t out_cap, uint32_t timeout_ms);

int tactile_bus_recv_raw(uint8_t *out, uint16_t out_cap, uint32_t wait_ms);
void tactile_bus_get_stats(tactile_bus_stats_t *out);
void tactile_bus_reset_stats(void);
#ifdef __cplusplus
}
#endif

#endif /* APP_DRIVERS_TACTILE_BUS_H_ */
