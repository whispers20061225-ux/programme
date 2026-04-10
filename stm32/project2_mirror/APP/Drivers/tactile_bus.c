/*
 * tactile_bus.c
 *
 *  Created on: Apr 8, 2026
 *      Author: whispers
 *
 *  说明：
 *  1. 这版代码基于 USART2 中断单字节接收
 *  2. 不使用 DMA
 *  3. 按 Paxini UART 手册格式收帧：
 *       AA 55
 *       LEN_L LEN_H
 *       data[4] ... data[N+14]
 *
 *     手册定义：
 *       - 帧长度字段表示从 data[4] 开始的长度
 *       - 最后一个 LRC 在 data[N+14]
 *       - 所以总帧长 = 4 + frame_len + 1
 *
 *     但根据手册示例和字段表，frame_len 通常不包含最后 LRC，
 *     因此这里采用：
 *         剩余字节数 = frame_len + 1
 *     总帧长 = 4 + frame_len + 1
 *
 *     你前一版的问题主要不是这个公式本身，而是同步/读取过程不够稳。
 */

#include "tactile_bus.h"
#include "paxini_uart_proto.h"
#include "usbd_cdc_if.h"

#include <string.h>

/* ========== 调试开关 ==========
 * 0：关闭 UART2 原始字节镜像到 USB CDC
 * 1：开启（调试时可能污染 host_link 文本返回）
 */
#define TACTILE_MIRROR_TO_CDC   0

#define TACTILE_RX_RING_SIZE    512

static UART_HandleTypeDef *s_tactile_uart = NULL;

/* UART 中断单字节接收缓冲 */
static uint8_t s_rx_byte = 0;

/* 环形缓冲 */
static uint8_t  s_rx_ring[TACTILE_RX_RING_SIZE];
static volatile uint16_t s_rx_w = 0;
static volatile uint16_t s_rx_r = 0;
static volatile tactile_bus_stats_t s_stats = {0};

#if TACTILE_MIRROR_TO_CDC
static uint8_t s_cdc_mirror_byte[1];
#endif
static uint32_t tactile_enter_critical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void tactile_exit_critical(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

/* =========================
 * 环形缓冲工具函数
 * ========================= */
static void tactile_ring_clear(void)
{
    uint32_t primask = tactile_enter_critical();
    s_rx_r = 0;
    s_rx_w = 0;
    tactile_exit_critical(primask);
}

static void tactile_ring_push(uint8_t b)
{
    uint16_t next = (uint16_t)((s_rx_w + 1U) % TACTILE_RX_RING_SIZE);

    /* 满了就覆盖最老数据，避免卡死 */
    if (next == s_rx_r) {
        s_stats.rx_overflow++;
        s_rx_r = (uint16_t)((s_rx_r + 1U) % TACTILE_RX_RING_SIZE);
    }

    s_rx_ring[s_rx_w] = b;
    s_rx_w = next;
    s_stats.rx_bytes++;
}

static int tactile_ring_pop(uint8_t *b)
{
    if (s_rx_r == s_rx_w) return 0;

    *b = s_rx_ring[s_rx_r];
    s_rx_r = (uint16_t)((s_rx_r + 1U) % TACTILE_RX_RING_SIZE);
    return 1;
}

/* =========================
 * 对外接口
 * ========================= */
void tactile_bus_init(UART_HandleTypeDef *huart)
{
    s_tactile_uart = huart;
    tactile_bus_reset_stats();
    tactile_ring_clear();

    /* 启动 USART2 中断接收 */
    if (HAL_UART_Receive_IT(s_tactile_uart, &s_rx_byte, 1) != HAL_OK) {
        s_stats.rx_rearm_fail++;
    }
}

int tactile_bus_send(const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    if (s_tactile_uart == NULL) return -1;
    if (data == NULL || len == 0) return -2;

    /* 每次发新请求前，把旧接收缓存清掉，避免旧帧干扰 */
    tactile_ring_clear();

    if (HAL_UART_Transmit(s_tactile_uart, (uint8_t*)data, len, timeout_ms) != HAL_OK) {
        return -3;
    }

    return 0;
}

int tactile_bus_recv_frame(uint8_t *out, uint16_t out_cap, uint32_t timeout_ms)
{
    if (s_tactile_uart == NULL) return -1;
    if (out == NULL || out_cap < 16) return -2;

    uint8_t b = 0;
    uint32_t start = HAL_GetTick();

    /* --------------------------------------------------
     * 1) 同步帧头 AA 55
     * -------------------------------------------------- */
    while ((HAL_GetTick() - start) < timeout_ms) {
        if (tactile_ring_pop(&b)) {
            if (b == PAXINI_UART_RSP_H1) {   // 0xAA
                uint8_t b2 = 0;
                uint32_t t2 = HAL_GetTick();

                while ((HAL_GetTick() - t2) < timeout_ms) {
                    if (tactile_ring_pop(&b2)) {
                        if (b2 == PAXINI_UART_RSP_H2) { // 0x55
                            out[0] = PAXINI_UART_RSP_H1;
                            out[1] = PAXINI_UART_RSP_H2;
                            goto header_ok;
                        } else {
                            /* 第二字节不是 0x55，则继续从当前字节往后找 */
                            break;
                        }
                    }
                }
            }
        }
    }

    return -3;   // 超时没找到帧头

header_ok:

    /* --------------------------------------------------
     * 2) 读取长度字段 LEN_L LEN_H
     * -------------------------------------------------- */
    start = HAL_GetTick();
    while ((HAL_GetTick() - start) < timeout_ms) {
        if (tactile_ring_pop(&out[2])) break;
    }
    if ((HAL_GetTick() - start) >= timeout_ms) return -4;

    start = HAL_GetTick();
    while ((HAL_GetTick() - start) < timeout_ms) {
        if (tactile_ring_pop(&out[3])) break;
    }
    if ((HAL_GetTick() - start) >= timeout_ms) return -5;

    uint16_t frame_len = (uint16_t)out[2] | ((uint16_t)out[3] << 8);

    /* --------------------------------------------------
     * 3) 根据长度读取剩余部分
     *
     * 手册：最后 LRC 在 data[N+14]
     * 所以在 data[0..3] 之后，至少还要收：
     *   frame_len 字节主体 + 1 字节 LRC
     * -------------------------------------------------- */
    uint16_t remain = (uint16_t)(frame_len + 1U);

    if ((uint16_t)(4U + remain) > out_cap) {
        return -6;
    }

    for (uint16_t i = 0; i < remain; i++) {
        start = HAL_GetTick();
        while ((HAL_GetTick() - start) < timeout_ms) {
            if (tactile_ring_pop(&out[4 + i])) {
                break;
            }
        }
        if ((HAL_GetTick() - start) >= timeout_ms) {
            return -7;   // 后续内容没收够
        }
    }

    /* --------------------------------------------------
     * 4) 可选：校验 LRC
     * -------------------------------------------------- */
    uint16_t total_len = (uint16_t)(4U + remain);

    uint8_t lrc_calc = paxini_lrc(out, total_len - 1U);
    uint8_t lrc_recv = out[total_len - 1U];

    if (lrc_calc != lrc_recv) {
        return -8;   // LRC 错
    }

    return (int)total_len;
}



int tactile_bus_recv_raw(uint8_t *out, uint16_t out_cap, uint32_t wait_ms)
{
    if (s_tactile_uart == NULL) return -1;
    if (out == NULL || out_cap == 0) return -2;

    uint32_t start = HAL_GetTick();
    uint16_t n = 0;
    uint8_t b = 0;

    // 在指定时间窗口内，把环形缓冲里收到的所有字节都拿出来
    // 只要有新字节，就更新 start，让窗口往后延一点，尽量收全
    while ((HAL_GetTick() - start) < wait_ms && n < out_cap) {
        if (tactile_ring_pop(&b)) {
            out[n++] = b;
            start = HAL_GetTick();  // 有新字节，延长等待，尽量收完整一点
        }
    }

    return (int)n;   // 返回实际收到的字节数；0 表示完全没收到
}



void tactile_bus_get_stats(tactile_bus_stats_t *out)
{
    if (out == NULL) return;

    uint32_t primask = tactile_enter_critical();
    *out = (tactile_bus_stats_t){
        .rx_bytes = s_stats.rx_bytes,
        .rx_overflow = s_stats.rx_overflow,
        .rx_pe = s_stats.rx_pe,
        .rx_ne = s_stats.rx_ne,
        .rx_fe = s_stats.rx_fe,
        .rx_ore = s_stats.rx_ore,
        .rx_rearm_fail = s_stats.rx_rearm_fail,
    };
    tactile_exit_critical(primask);
}

void tactile_bus_reset_stats(void)
{
    uint32_t primask = tactile_enter_critical();
    s_stats = (tactile_bus_stats_t){0};
    tactile_exit_critical(primask);
}

/* =========================
 * USART 中断回调
 * ========================= */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (s_tactile_uart == NULL) return;

    if (huart == s_tactile_uart) {
        /* 推入环形缓冲 */
        tactile_ring_push(s_rx_byte);

#if TACTILE_MIRROR_TO_CDC
        /* 调试时可把原始字节直接透传给 USB CDC */
        s_cdc_mirror_byte[0] = s_rx_byte;
        (void)CDC_Transmit_FS(s_cdc_mirror_byte, 1);
#endif

        /* 继续挂下一字节接收 */
        if (HAL_UART_Receive_IT(s_tactile_uart, &s_rx_byte, 1) != HAL_OK) {
            s_stats.rx_rearm_fail++;
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (s_tactile_uart == NULL) return;

    if (huart == s_tactile_uart) {
        /* 清错误并重启接收 */
        uint32_t err = huart->ErrorCode;

        if (err & HAL_UART_ERROR_PE) {
            s_stats.rx_pe++;
        }
        if (err & HAL_UART_ERROR_NE) {
            s_stats.rx_ne++;
        }
        if (err & HAL_UART_ERROR_FE) {
            s_stats.rx_fe++;
        }
        if (err & HAL_UART_ERROR_ORE) {
            s_stats.rx_ore++;
        }

        __HAL_UART_CLEAR_PEFLAG(huart);
        if (HAL_UART_Receive_IT(s_tactile_uart, &s_rx_byte, 1) != HAL_OK) {
            s_stats.rx_rearm_fail++;
        }
    }
}
