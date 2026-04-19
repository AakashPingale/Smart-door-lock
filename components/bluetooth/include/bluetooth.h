#ifndef BLUETOOTH_H
#define BLUETOOTH_H

typedef void (*bt_spp_cmd_cb_t)(int id);

void bluetooth_spp_init(bt_spp_cmd_cb_t cmd_callback);
void bluetooth_spp_send(const char *msg);

#endif
