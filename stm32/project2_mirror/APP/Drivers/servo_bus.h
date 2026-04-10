/*
 * servo_bus.h
 *
 *  Created on: Jan 10, 2026
 *      Author: whispers
 */

#ifndef DRIVERS_SERVO_BUS_H_
#define DRIVERS_SERVO_BUS_H_

#pragma once
#include <stdint.h>
#include <stddef.h>
#include "stm32f1xx_hal.h"

/*
 * 这里的设计目标：
 * 1) 把“USART 半双工收发 + 超时 + 找帧头 + 校验”封装起来
 * 2) 上层（servo_api）不关心字节流细节，只关心“成功/失败/参数”
 */

typedef struct {
    uint8_t id;           // 回包舵机ID
    uint8_t len;          // LEN字段
    uint8_t status;       // STATUS字段（0通常表示OK）
    uint8_t params[32];   // 参数区（READ回来的数据会在这里）
    uint8_t params_len;   // 参数长度
} ds_rsp_t;

void servo_bus_init(UART_HandleTypeDef* huart);

/*
 * 发送一帧并等待回包：
 * - 用于 PING / READ 这种“必须确认回包”的命令
 * - timeout_ms 给得稍大一点，避免总线负载高时误判
 */
int servo_bus_txrx(const uint8_t* tx, size_t txlen, ds_rsp_t* out, uint32_t timeout_ms);

/*
 * 只发送不等待回包：
 * - 用于 MOVE（写目标位置）这种“即使回包关闭也要能控制”的场景
 * - 这是为了让你最小闭环先动起来，后续再加“可选回包”
 */
int servo_bus_send(const uint8_t* tx, size_t txlen, uint32_t timeout_ms);

#endif /* DRIVERS_SERVO_BUS_H_ */
