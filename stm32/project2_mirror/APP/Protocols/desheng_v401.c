/*
 * desheng_v401.c
 *
 *  Created on: Jan 10, 2026
 *      Author: whispers
 */
#include "desheng_v401.h"

/*
 * 校验规则（常见写法）：
 *   CHK = ~(ID + LEN + INST/STATUS + PARAMS...)  (取反后保留低8位)
 */
static uint8_t sum_cmd(uint8_t id, uint8_t len, uint8_t inst, const uint8_t* p, size_t n)
{
    uint16_t s = (uint16_t)id + (uint16_t)len + (uint16_t)inst;
    for (size_t i = 0; i < n; i++) s += p[i];
    return (uint8_t)s;
}

static uint8_t sum_rsp(uint8_t id, uint8_t len, uint8_t status, const uint8_t* p, size_t n)
{
    uint16_t s = (uint16_t)id + (uint16_t)len + (uint16_t)status;
    for (size_t i = 0; i < n; i++) s += p[i];
    return (uint8_t)s;
}

uint8_t ds_calc_chk_cmd(uint8_t id, uint8_t len, uint8_t inst,
                        const uint8_t* params, size_t n)
{
    return (uint8_t)(~sum_cmd(id, len, inst, params, n));
}

uint8_t ds_calc_chk_rsp(uint8_t id, uint8_t len, uint8_t status,
                        const uint8_t* params, size_t n)
{
    return (uint8_t)(~sum_rsp(id, len, status, params, n));
}

size_t ds_pack_ping(uint8_t id, uint8_t* out, size_t cap)
{
    // PING 帧：FF FF ID 02 01 CHK
    if (cap < 6) return 0;

    uint8_t len = 0x02;  // 参数0字节，所以 LEN = 0 + 2 = 2

    out[0] = DS_CMD_H1;
    out[1] = DS_CMD_H2;
    out[2] = id;
    out[3] = len;
    out[4] = DS_INST_PING;
    out[5] = ds_calc_chk_cmd(id, len, DS_INST_PING, NULL, 0);

    return 6;
}

size_t ds_pack_read(uint8_t id, uint8_t addr, uint8_t nbytes, uint8_t* out, size_t cap)
{
    // READ 帧：FF FF ID 04 02 ADDR NBYTES CHK
    if (cap < 8) return 0;

    uint8_t params[2] = { addr, nbytes };
    uint8_t len = 0x04;  // 参数2字节，所以 LEN = 2 + 2 = 4

    out[0] = DS_CMD_H1;
    out[1] = DS_CMD_H2;
    out[2] = id;
    out[3] = len;
    out[4] = DS_INST_READ;
    out[5] = params[0];
    out[6] = params[1];
    out[7] = ds_calc_chk_cmd(id, len, DS_INST_READ, params, 2);

    return 8;
}

size_t ds_pack_write(uint8_t id, uint8_t addr, const uint8_t* data, uint8_t nbytes,
                     uint8_t* out, size_t cap)
{
    /*
     * WRITE 帧：FF FF ID (nbytes+3) 03 ADDR DATA... CHK
     * 参数是：ADDR + DATA[nbytes]  => 参数字节数 = 1 + nbytes
     * LEN = 参数字节数 + 2 = (1+nbytes)+2 = nbytes+3
     */
    if (cap < (size_t)(7 + nbytes)) return 0;

    uint8_t len = (uint8_t)(nbytes + 3);

    out[0] = DS_CMD_H1;
    out[1] = DS_CMD_H2;
    out[2] = id;
    out[3] = len;
    out[4] = DS_INST_WRITE;
    out[5] = addr;

    for (uint8_t i = 0; i < nbytes; i++) {
        out[6 + i] = data[i];
    }

    // 不额外分配 params 数组，直接累加算校验即可
    uint16_t s = (uint16_t)id + (uint16_t)len + (uint16_t)DS_INST_WRITE + (uint16_t)addr;
    for (uint8_t i = 0; i < nbytes; i++) s += data[i];

    out[6 + nbytes] = (uint8_t)(~(uint8_t)s);

    return (size_t)(7 + nbytes);
}
