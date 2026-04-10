/*
 * paxini_uart_proto.c
 *
 *  Created on: Apr 8, 2026
 *      Author: whispers
 */
#include "paxini_uart_proto.h"

uint8_t paxini_lrc(const uint8_t *data, size_t len)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return (uint8_t)(0 - sum);
}

size_t paxini_pack_read_app(uint8_t dev_addr,
                            uint32_t start_addr,
                            uint16_t read_len,
                            uint8_t *out,
                            size_t out_cap)
{
    // 固定读请求帧总长度 = 14
    if (out_cap < 14) return 0;

    out[0] = PAXINI_UART_REQ_H1;
    out[1] = PAXINI_UART_REQ_H2;

    // 帧长度：从 data[4] 到 data[13]，也就是 10 字节？
    // 但手册示例固定写成 0x0009，我们先严格照手册示例格式
    out[2] = 0x09;
    out[3] = 0x00;

    out[4] = dev_addr;
    out[5] = 0x00;                  // 预留
    out[6] = PAXINI_FUNC_READ_APP;  // 读应用区

    // 起始地址，小端
    out[7]  = (uint8_t)(start_addr & 0xFF);
    out[8]  = (uint8_t)((start_addr >> 8) & 0xFF);
    out[9]  = (uint8_t)((start_addr >> 16) & 0xFF);
    out[10] = (uint8_t)((start_addr >> 24) & 0xFF);

    // 读取长度，小端
    out[11] = (uint8_t)(read_len & 0xFF);
    out[12] = (uint8_t)((read_len >> 8) & 0xFF);

    out[13] = paxini_lrc(out, 13);

    return 14;
}

