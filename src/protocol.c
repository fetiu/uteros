#include "protocol.h"
#include <zephyr/kernel.h>

static ProtocolHandler *handlers[MAX_COMMAND_TYPE];

ProtocolDataUnit *Protocol_new_pdu(uint8_t *data, size_t len)
{
	uint16_t command = NTOH_UINT16(data);
	for (int i = 0; i < MAX_COMMAND_TYPE; i++) {
		if (handlers[i]->command == command) {
            ProtocolDataUnit *pdu = k_malloc(sizeof(*pdu));
            if (!pdu) {
                break;
            }
            pdu->command = command;
            pdu->params = data + sizeof(command);
			return pdu;
		}
	}
	return NULL;
}

void Protocol_delete_pdu(ProtocolDataUnit *pdu)
{
	k_free(pdu);
}

void Protocol_set_handler(CommandType command_type, ProtocolHandler *handler)
{
	handlers[command_type] = handler;
}

int Protocol_handle(ProtocolDataUnit *pdu)
{
	for (int i = 0; i < MAX_COMMAND_TYPE; i++) {
		if (handlers[i]->command == pdu->command) {
			return handlers[i]->handler(pdu);
		}
	}
	return 0;
}
