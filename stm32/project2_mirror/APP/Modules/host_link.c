/*
 * host_link.c
 *
 * Text command bridge between USB CDC and the STM32 application modules.
 */

#include "host_link.h"

#include "build_info.h"
#include "paxini_uart_proto.h"
#include "servo_api.h"
#include "tactile_api.h"
#include "tactile_bus.h"
#include "usbd_cdc_if.h"

#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define LINE_BUF_SIZE 256
#define TX_BUF_SIZE   256
#define TACTILE_MAX_READ_LEN 64U

static char s_line_buf[LINE_BUF_SIZE];
static size_t s_line_len = 0U;
static volatile uint8_t s_line_ready = 0U;

static uint8_t s_tx_buf[TX_BUF_SIZE];
static uint16_t s_tx_len = 0U;
static volatile uint8_t s_tx_pending = 0U;

static void trim_inplace(char *s)
{
    char *start = s;
    size_t len = 0U;

    if (s == NULL) {
        return;
    }

    while (*start != '\0' && isspace((unsigned char)*start)) {
        start++;
    }

    if (start != s) {
        memmove(s, start, strlen(start) + 1U);
    }

    len = strlen(s);
    while (len > 0U && isspace((unsigned char)s[len - 1U])) {
        s[len - 1U] = '\0';
        len--;
    }
}

static uint8_t tx_queue_bytes(const uint8_t *data, size_t len)
{
    if (data == NULL || len == 0U || s_tx_pending) {
        return 0U;
    }

    if (len >= TX_BUF_SIZE) {
        len = TX_BUF_SIZE - 1U;
    }

    memcpy(s_tx_buf, data, len);
    s_tx_len = (uint16_t)len;
    s_tx_pending = 1U;
    return 1U;
}

static uint8_t tx_send_line(const char *s)
{
    if (s == NULL) {
        return 0U;
    }

    return tx_queue_bytes((const uint8_t *)s, strlen(s));
}

static void try_flush_tx(void)
{
    if (!s_tx_pending) {
        return;
    }

    if (CDC_Transmit_FS(s_tx_buf, s_tx_len) == USBD_OK) {
        s_tx_pending = 0U;
        s_tx_len = 0U;
    }
}

static int parse_u32_token(const char **cursor, uint32_t *out_value)
{
    char *end = NULL;
    unsigned long value = 0UL;

    if (cursor == NULL || *cursor == NULL || out_value == NULL) {
        return -1;
    }

    while (**cursor != '\0' && isspace((unsigned char)**cursor)) {
        (*cursor)++;
    }

    if (**cursor == '\0') {
        return -1;
    }

    value = strtoul(*cursor, &end, 0);
    if (end == *cursor) {
        return -1;
    }

    *out_value = (uint32_t)value;
    *cursor = end;
    return 0;
}

static int parse_tactile_read_args(const char *args,
                                   uint8_t *dev_addr,
                                   uint32_t *start_addr,
                                   uint16_t *read_len)
{
    uint32_t dev = 0U;
    uint32_t addr = 0U;
    uint32_t len = 0U;
    const char *cursor = args;

    if (parse_u32_token(&cursor, &dev) != 0 ||
        parse_u32_token(&cursor, &addr) != 0 ||
        parse_u32_token(&cursor, &len) != 0) {
        return -1;
    }

    while (*cursor != '\0' && isspace((unsigned char)*cursor)) {
        cursor++;
    }

    if (*cursor != '\0') {
        return -1;
    }

    if (dev > 255U || len == 0U || len > TACTILE_MAX_READ_LEN) {
        return -2;
    }

    *dev_addr = (uint8_t)dev;
    *start_addr = addr;
    *read_len = (uint16_t)len;
    return 0;
}

static int tactile_send_request(uint8_t dev_addr,
                                uint32_t start_addr,
                                uint16_t read_len,
                                uint8_t *tx_buf,
                                size_t tx_cap)
{
    size_t tx_len = paxini_pack_read_app(dev_addr, start_addr, read_len, tx_buf, tx_cap);
    if (tx_len == 0U) {
        return -2;
    }

    return tactile_bus_send(tx_buf, (uint16_t)tx_len, 50U);
}

static void send_hex_payload_line(const char *prefix, const uint8_t *data, int data_len)
{
    char out[TX_BUF_SIZE];
    int pos = snprintf(out, sizeof(out), "%s n=%d ", prefix, data_len);

    if (pos < 0) {
        return;
    }

    for (int i = 0; i < data_len && pos < (int)sizeof(out) - 4; i++) {
        pos += snprintf(out + pos, sizeof(out) - (size_t)pos, "%02X ", data[i]);
    }

    if (pos < (int)sizeof(out) - 2) {
        out[pos++] = '\n';
        out[pos] = '\0';
    } else {
        out[sizeof(out) - 2] = '\n';
        out[sizeof(out) - 1] = '\0';
    }

    (void)tx_send_line(out);
}

static void handle_tactile_raw_read(const char *prefix,
                                    uint8_t dev_addr,
                                    uint32_t start_addr,
                                    uint16_t read_len)
{
    uint8_t tx[32];
    uint8_t raw[128];
    char out[TX_BUF_SIZE];
    int rc = tactile_send_request(dev_addr, start_addr, read_len, tx, sizeof(tx));

    if (rc != 0) {
        snprintf(out, sizeof(out), "%s FAIL send %d\n", prefix, rc);
        (void)tx_send_line(out);
        return;
    }

    rc = tactile_bus_recv_raw(raw, sizeof(raw), 100U);
    if (rc < 0) {
        snprintf(out, sizeof(out), "%s FAIL raw %d\n", prefix, rc);
        (void)tx_send_line(out);
        return;
    }

    send_hex_payload_line(prefix, raw, rc);
}

static void handle_tactile_frame_read(const char *prefix,
                                      uint8_t dev_addr,
                                      uint32_t start_addr,
                                      uint16_t read_len)
{
    uint8_t frame[128];
    uint16_t frame_len = 0U;
    char out[TX_BUF_SIZE];
    int rc = tactile_read_app_frame(dev_addr, start_addr, read_len, frame, sizeof(frame), &frame_len);

    if (rc != 0) {
        snprintf(out, sizeof(out), "%s FAIL %d\n", prefix, rc);
        (void)tx_send_line(out);
        return;
    }

    {
        int pos = snprintf(out, sizeof(out), "%s OK len=%u ", prefix, (unsigned)frame_len);
        for (uint16_t i = 0; i < frame_len && pos < (int)sizeof(out) - 4; i++) {
            pos += snprintf(out + pos, sizeof(out) - (size_t)pos, "%02X ", frame[i]);
        }

        if (pos < (int)sizeof(out) - 2) {
            out[pos++] = '\n';
            out[pos] = '\0';
        } else {
            out[sizeof(out) - 2] = '\n';
            out[sizeof(out) - 1] = '\0';
        }
    }

    (void)tx_send_line(out);
}

void host_link_init(void)
{
    s_line_len = 0U;
    s_line_buf[0] = '\0';
    s_line_ready = 0U;
    s_tx_len = 0U;
    s_tx_pending = 0U;
}

void host_link_send(const uint8_t *data, size_t len)
{
    (void)tx_queue_bytes(data, len);
}

void host_link_on_rx(const uint8_t *data, size_t len)
{
    if (data == NULL || len == 0U) {
        return;
    }

    for (size_t i = 0U; i < len; i++) {
        char c = (char)data[i];

        if (c == '\r') {
            continue;
        }

        if (c == '\n') {
            s_line_buf[s_line_len] = '\0';
            s_line_ready = 1U;
            s_line_len = 0U;
            return;
        }

        if (s_line_len < (LINE_BUF_SIZE - 1U)) {
            s_line_buf[s_line_len++] = c;
        } else {
            s_line_len = 0U;
        }
    }
}

static void handle_line(char *s)
{
    char out[TX_BUF_SIZE];

    if (s == NULL) {
        return;
    }

    trim_inplace(s);
    if (s[0] == '\0') {
        return;
    }

    if (strcmp(s, "PING") == 0) {
        (void)tx_send_line("PONG\n");
        return;
    }

    if (strcmp(s, "VER") == 0) {
        snprintf(out, sizeof(out), "VER %s %s %s %s\n", FW_NAME, FW_VERSION, __DATE__, __TIME__);
        (void)tx_send_line(out);
        return;
    }

    if (strncmp(s, "ECHO ", 5) == 0) {
        snprintf(out, sizeof(out), "ECHO %.*s\n", (int)(sizeof(out) - 7U), s + 5);
        (void)tx_send_line(out);
        return;
    }

    if (strcmp(s, "TSTAT RESET") == 0) {
        tactile_bus_reset_stats();
        (void)tx_send_line("TSTAT RESET OK\n");
        return;
    }

    if (strcmp(s, "TSTAT") == 0) {
        tactile_bus_stats_t st;
        tactile_bus_get_stats(&st);
        snprintf(out,
                 sizeof(out),
                 "TSTAT bytes=%lu ovf=%lu pe=%lu ne=%lu fe=%lu ore=%lu rearm=%lu\n",
                 (unsigned long)st.rx_bytes,
                 (unsigned long)st.rx_overflow,
                 (unsigned long)st.rx_pe,
                 (unsigned long)st.rx_ne,
                 (unsigned long)st.rx_fe,
                 (unsigned long)st.rx_ore,
                 (unsigned long)st.rx_rearm_fail);
        (void)tx_send_line(out);
        return;
    }

    if (strncmp(s, "TRAWX ", 6) == 0) {
        uint8_t dev_addr = 0U;
        uint32_t start_addr = 0U;
        uint16_t read_len = 0U;
        int rc = parse_tactile_read_args(s + 6, &dev_addr, &start_addr, &read_len);
        if (rc != 0) {
            (void)tx_send_line("ERR bad_args\n");
            return;
        }
        handle_tactile_raw_read("TRAWX", dev_addr, start_addr, read_len);
        return;
    }

    if (strncmp(s, "TREADX ", 7) == 0) {
        uint8_t dev_addr = 0U;
        uint32_t start_addr = 0U;
        uint16_t read_len = 0U;
        int rc = parse_tactile_read_args(s + 7, &dev_addr, &start_addr, &read_len);
        if (rc != 0) {
            (void)tx_send_line("ERR bad_args\n");
            return;
        }
        handle_tactile_frame_read("TREADX", dev_addr, start_addr, read_len);
        return;
    }

    if (strncmp(s, "TRAW ", 5) == 0) {
        int dev = atoi(s + 5);
        handle_tactile_raw_read("TRAW", (uint8_t)dev, 1038U, 32U);
        return;
    }

    if (strncmp(s, "TREAD ", 6) == 0) {
        int dev = atoi(s + 6);
        handle_tactile_frame_read("TREAD", (uint8_t)dev, 1038U, 32U);
        return;
    }

    if (strncmp(s, "SPING ", 6) == 0) {
        int id = atoi(s + 6);
        int rc = servo_ping((uint8_t)id);
        (void)tx_send_line((rc == 0) ? "OK\n" : "FAIL\n");
        return;
    }

    if (strncmp(s, "SREADPOS ", 9) == 0) {
        int id = atoi(s + 9);
        uint16_t pos = 0U;
        int rc = servo_read_pos((uint8_t)id, &pos);

        if (rc == 0) {
            snprintf(out, sizeof(out), "POS %d %u\n", id, (unsigned)pos);
        } else {
            snprintf(out, sizeof(out), "FAIL %d\n", id);
        }

        (void)tx_send_line(out);
        return;
    }

    if (strncmp(s, "SMOVE ", 6) == 0) {
        int id = 0;
        unsigned pos = 0U;
        unsigned ms = 0U;

        if (sscanf(s + 6, "%d %u %u", &id, &pos, &ms) == 3) {
            int rc = servo_move((uint8_t)id, (uint16_t)pos, (uint16_t)ms);
            (void)tx_send_line((rc == 0) ? "OK\n" : "FAIL\n");
        } else {
            (void)tx_send_line("ERR bad_args\n");
        }
        return;
    }

    (void)tx_send_line("ERR unknown_cmd\n");
}

void host_link_poll(void)
{
    try_flush_tx();

    if (!s_line_ready) {
        return;
    }

    s_line_ready = 0U;
    handle_line(s_line_buf);
}
