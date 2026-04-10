#ifndef APP_MODULES_HOST_LINK_H_
#define APP_MODULES_HOST_LINK_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void host_link_init(void);
void host_link_poll(void);
void host_link_on_rx(const uint8_t *data, size_t len);
void host_link_send(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* APP_MODULES_HOST_LINK_H_ */
