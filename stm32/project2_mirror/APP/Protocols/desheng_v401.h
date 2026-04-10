/*
 * desheng_v401.h
 *
 *  Created on: Jan 10, 2026
 *      Author: whispers
 */

#ifndef PROTOCOLS_DESHENG_V401_H_
#define PROTOCOLS_DESHENG_V401_H_

#pragma once
#include <stdint.h>
#include <stddef.h>

/*
 * 德晟总线舵机（v4.01）协议常见形式：
 *   发送帧：FF FF ID LEN INST PARAM... CHK
 *   回包帧：FF F5 ID LEN STATUS PARAM... CHK   （有些型号回包头可能不同，后面可兼容）
 *
 * LEN 的含义：从 INST/STATUS 开始，到 CHK 结束的字节总数
 *  - 对命令：LEN = 参数字节数 + 2（INST + CHK）
 *  - 对回包：LEN = 参数字节数 + 2（STATUS + CHK）
 */

#define DS_CMD_H1  0xFF
#define DS_CMD_H2  0xFF

#define DS_RSP_H1  0xFF
#define DS_RSP_H2  0xF5   // 如果你发现收不到回包，我们再把这里做成“FF F5 或 FF FF”双兼容

// 指令码（常用子集）
#define DS_INST_PING   0x01
#define DS_INST_READ   0x02
#define DS_INST_WRITE  0x03

// 常用寄存器地址（这两个足够完成最小闭环）
#define DS_ADDR_GOAL_POS_TIME 0x2A   // 写入：目标位置pos(2B) + 运行时间time(2B)，共4字节
#define DS_ADDR_PRESENT_POS   0x38   // 读取：当前位置信息(2B)

// 校验计算（命令/回包略不同：命令用 INST，回包用 STATUS）
uint8_t ds_calc_chk_cmd(uint8_t id, uint8_t len, uint8_t inst,
                        const uint8_t* params, size_t n);

uint8_t ds_calc_chk_rsp(uint8_t id, uint8_t len, uint8_t status,
                        const uint8_t* params, size_t n);

// 打包：PING / READ / WRITE（最小闭环足够用）
size_t ds_pack_ping(uint8_t id, uint8_t* out, size_t cap);
size_t ds_pack_read(uint8_t id, uint8_t addr, uint8_t nbytes, uint8_t* out, size_t cap);
size_t ds_pack_write(uint8_t id, uint8_t addr, const uint8_t* data, uint8_t nbytes,
                     uint8_t* out, size_t cap);

#endif /* PROTOCOLS_DESHENG_V401_H_ */
