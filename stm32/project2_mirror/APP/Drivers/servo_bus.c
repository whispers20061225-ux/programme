/*
 * servo_bus.c
 *
 *  Created on: Jan 10, 2026
 *      Author: whispers
 */

#include "servo_bus.h"
#include "desheng_v401.h"
#include <string.h>

/*
 * 注意：Half-Duplex 单线模式下，HAL_UART_Transmit / Receive 仍然可用，
 * HAL 底层会切换方向。但总线是一根线，多从机会回包，所以我们要：
 *  - 发完后立刻开始收
 *  - 收的时候要能“找帧头”，因为总线可能存在噪声/残留字节
 */

static UART_HandleTypeDef* s_hu = NULL;

void servo_bus_init(UART_HandleTypeDef* huart)
{
    s_hu = huart;
}

// 在字节流里同步到回包头：FF F5（如果你后续发现是 FF FF，我会教你改成双兼容）
static int sync_header(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    uint8_t b = 0, prev = 0;

    while ((HAL_GetTick() - t0) < timeout_ms) {
        // 每次只收1字节，用很短的超时（2ms）循环拼接
        if (HAL_UART_Receive(s_hu, &b, 1, 2) == HAL_OK) {
            if (prev == DS_RSP_H1 && b == DS_RSP_H2) {
                return 0; // 找到帧头
            }
            prev = b;
        }
    }
    return -1; // 超时没找到帧头
}

int servo_bus_send(const uint8_t* tx, size_t txlen, uint32_t timeout_ms)
{
    if (!s_hu) return -10;

    // 只发不收：MOVE 这种命令非常适合先这样做
    if (HAL_UART_Transmit(s_hu, (uint8_t*)tx, (uint16_t)txlen, timeout_ms) != HAL_OK) {
        return -11;
    }
    return 0;
}

int servo_bus_txrx(const uint8_t* tx, size_t txlen, ds_rsp_t* out, uint32_t timeout_ms)
{
    if (!s_hu || !out) return -10;

    // 1) 发送命令帧
    if (HAL_UART_Transmit(s_hu, (uint8_t*)tx, (uint16_t)txlen, timeout_ms) != HAL_OK) {
        return -11;
    }

    // 2) 同步到回包头（FF F5）
    if (sync_header(timeout_ms) != 0) {
        return -20; // 没回包 / 回包头不匹配 / 波特率不对 / 线没接好等都会到这里
    }

    // 3) 读取 ID 和 LEN
    uint8_t id = 0, len = 0;
    if (HAL_UART_Receive(s_hu, &id, 1, timeout_ms) != HAL_OK) return -21;
    if (HAL_UART_Receive(s_hu, &len, 1, timeout_ms) != HAL_OK) return -22;

    // 4) 根据 LEN 读取剩余 len 字节（STATUS + PARAMS + CHK）
    //    len 至少应为2（STATUS + CHK）
    uint8_t buf[64] = {0};
    if (len < 2 || len > sizeof(buf)) return -23;

    if (HAL_UART_Receive(s_hu, buf, len, timeout_ms) != HAL_OK) return -24;

    uint8_t status = buf[0];
    uint8_t chk_rx = buf[len - 1];
    uint8_t params_len = (uint8_t)(len - 2); // 去掉 STATUS(1) 和 CHK(1)

    // 5) 校验，确保不是误收/噪声帧
    uint8_t chk_calc = ds_calc_chk_rsp(id, len, status, &buf[1], params_len);
    if (chk_calc != chk_rx) {
        return -30; // 校验失败：常见原因是波特率不对、回包头不对、或收包偏移
    }

    // 6) 输出结构体
    out->id = id;
    out->len = len;
    out->status = status;
    out->params_len = params_len;

    if (params_len > sizeof(out->params)) params_len = sizeof(out->params);
    memcpy(out->params, &buf[1], params_len);

    return 0;
}
