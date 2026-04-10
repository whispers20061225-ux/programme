/*
 * tactile_api.h
 *
 *  Created on: Apr 8, 2026
 *      Author: whispers
 */

#ifndef APP_MODULES_TACTILE_API_H_
#define APP_MODULES_TACTILE_API_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void tactile_api_init(void);

int tactile_read_app_frame(uint8_t dev_addr,
                           uint32_t start_addr,
                           uint16_t read_len,
                           uint8_t *out,
                           uint16_t out_cap,
                           uint16_t *out_len);

int tactile_read_block_1038_32(uint8_t dev_addr,
                               uint8_t *out,
                               uint16_t out_cap,
                               uint16_t *out_len);

#ifdef __cplusplus
}
#endif

#endif /* APP_MODULES_TACTILE_API_H_ */
