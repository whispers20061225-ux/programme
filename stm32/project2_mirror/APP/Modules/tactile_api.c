/*
 * tactile_api.c
 *
 *  Created on: Apr 8, 2026
 *      Author: whispers
 */

#include "tactile_api.h"

#include "paxini_uart_proto.h"
#include "tactile_bus.h"

static uint8_t s_tx_buf[32];

void tactile_api_init(void)
{
}

int tactile_read_app_frame(uint8_t dev_addr,
                           uint32_t start_addr,
                           uint16_t read_len,
                           uint8_t *out,
                           uint16_t out_cap,
                           uint16_t *out_len)
{
    size_t tx_len = 0;
    int rc = 0;

    if (out == NULL || out_len == NULL || read_len == 0U) {
        return -1;
    }

    tx_len = paxini_pack_read_app(dev_addr, start_addr, read_len, s_tx_buf, sizeof(s_tx_buf));
    if (tx_len == 0U) {
        return -2;
    }

    rc = tactile_bus_send(s_tx_buf, (uint16_t)tx_len, 50U);
    if (rc != 0) {
        return rc;
    }

    rc = tactile_bus_recv_frame(out, out_cap, 100U);
    if (rc < 0) {
        return rc;
    }

    *out_len = (uint16_t)rc;
    return 0;
}

int tactile_read_block_1038_32(uint8_t dev_addr,
                               uint8_t *out,
                               uint16_t out_cap,
                               uint16_t *out_len)
{
    return tactile_read_app_frame(dev_addr, 1038U, 32U, out, out_cap, out_len);
}
