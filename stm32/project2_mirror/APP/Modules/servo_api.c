/*
 * servo_api.c
 *
 *  Created on: Jan 10, 2026
 *      Author: whispers
 */
#include "servo_api.h"
#include "servo_bus.h"
#include "desheng_v401.h"

/*
 * 为什么这里要分三层？
 * - desheng_v401：只管协议打包/校验（纯算法，不依赖HAL）
 * - servo_bus：只管UART收发+找帧头+超时（硬件相关）
 * - servo_api：只管“舵机要做什么”（业务接口）
 *
 * 好处：后面你换 UART / 加 DMA / 换协议，都不会把代码搅成一团。
 */

static uint8_t txbuf[32];

int servo_ping(uint8_t id)
{
    ds_rsp_t rsp;

    // 打包PING
    size_t n = ds_pack_ping(id, txbuf, sizeof(txbuf));
    if (!n) return -1;

    // 发并等回包：如果舵机回包关闭，会失败（这很正常，后续我们可做“可选回包”）
    int rc = servo_bus_txrx(txbuf, n, &rsp, 60);
    if (rc != 0) return rc;

    // status==0 表示正常
    return (rsp.status == 0) ? 0 : -2;
}

int servo_read_pos(uint8_t id, uint16_t* pos_out)
{
    if (!pos_out) return -1;

    ds_rsp_t rsp;

    // 读当前位置寄存器 0x38，长度2字节
    size_t n = ds_pack_read(id, DS_ADDR_PRESENT_POS, 2, txbuf, sizeof(txbuf));
    if (!n) return -1;

    int rc = servo_bus_txrx(txbuf, n, &rsp, 80);
    if (rc != 0) return rc;

    // 回包必须有2字节参数，且状态为0
    if (rsp.status != 0 || rsp.params_len < 2) return -2;

    // 协议数据通常是“大端”（高字节在前）
    *pos_out = ((uint16_t)rsp.params[0] << 8) | (uint16_t)rsp.params[1];

    return 0;
}

int servo_move(uint8_t id, uint16_t pos, uint16_t ms)
{
    /*
     * 写目标位置+运行时间：地址0x2A，4字节：
     *   pos_hi pos_lo time_hi time_lo （大端）
     */
    uint8_t data[4] = {
        (uint8_t)(pos >> 8), (uint8_t)(pos & 0xFF),
        (uint8_t)(ms  >> 8), (uint8_t)(ms  & 0xFF),
    };

    size_t n = ds_pack_write(id, DS_ADDR_GOAL_POS_TIME, data, 4, txbuf, sizeof(txbuf));
    if (!n) return -1;

    /*
     * 为什么 MOVE 先用“只发不等回包”？
     * - 很多总线舵机支持“关闭回包”以减少总线占用（尤其多舵机）
     * - 你现在最关键是“先让它动起来”，避免卡在“回包没开”的坑里
     * - 后续我们再把 MOVE 做成可选等待回包（更工程化）
     */
    return servo_bus_send(txbuf, n, 50);
}

