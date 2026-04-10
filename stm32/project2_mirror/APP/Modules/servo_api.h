#pragma once
#include <stdint.h>

/*
 * 这里提供“业务友好”的接口：
 * - servo_ping：检查是否在线
 * - servo_read_pos：读当前位置（验证读通）
 * - servo_move：写目标位置+时间（验证能动）
 */

int servo_ping(uint8_t id);
int servo_read_pos(uint8_t id, uint16_t* pos_out);
int servo_move(uint8_t id, uint16_t pos, uint16_t ms);
