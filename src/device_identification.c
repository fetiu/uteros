#include "device_identification.h"
#include <zephyr/posix/arpa/inet.h>
#include "protocol.h"

static ProtocolHandler id_setting_handler, id_confirm_handler;
static uint16_t login_id;

int setting(ProtocolDataUnit *pdu)
{
    if (pdu->command != id_setting_handler.command) {
        return -1;
    }
    login_id = NTOH_UINT16(pdu->params);
    return 0;
}

int confirm(ProtocolDataUnit *pdu)
{
    if (pdu->command != id_confirm_handler.command) {
        return -1;
    }
    uint16_t *response = (uint16_t *)pdu->params;
    if (*response) {
        // must be empty on received
        return -2;
    }
    *response = HTON_UINT16(&login_id);
    return 0;
}

void DeviceIdentification_init(void)
{
    id_setting_handler.command = 0x55AA;
    id_setting_handler.handler = setting;
    id_confirm_handler.command = 0xAA55;
    id_confirm_handler.handler = confirm;
    Protocol_set_handler(COMMAND_ID_SETTING, &id_setting_handler);
    Protocol_set_handler(COMMAND_ID_CONFIRM, &id_confirm_handler);
}