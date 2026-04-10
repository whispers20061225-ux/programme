/*
 * paxini_uart_proto.h
 *
 *  Created on: Apr 8, 2026
 *      Author: whispers
 */

#ifndef PAXINI_UART_PROTO_H
#define PAXINI_UART_PROTO_H

#include <stdint.h>
#include <stddef.h>

#define PAXINI_UART_REQ_H1  0x55
#define PAXINI_UART_REQ_H2  0xAA
#define PAXINI_UART_RSP_H1  0xAA
#define PAXINI_UART_RSP_H2  0x55

#define PAXINI_FUNC_READ_APP   0xFB   // 0x80 + 0x7B
#define PAXINI_FUNC_WRITE_CFG  0x79

uint8_t paxini_lrc(const uint8_t *data, size_t len);

/* 组一条“读应用区”请求帧
 * dev_addr: 设备地址 = 模组号 + 1
 * start_addr: 起始地址（4字节小端）
 * read_len: 读取字节数（2字节小端）
 * out: 输出缓冲
 * return: 帧总长度
 */
size_t paxini_pack_read_app(uint8_t dev_addr,
                            uint32_t start_addr,
                            uint16_t read_len,
                            uint8_t *out,
                            size_t out_cap);

#endif
