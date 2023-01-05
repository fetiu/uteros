#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <zephyr/posix/arpa/inet.h>
#include "command.h"

#define NTOH_UINT16(data) ntohs(*(uint16_t *)data)
#define HTON_UINT16(data) htons(*(uint16_t *)data)

typedef struct {
    uint16_t command;
    uint8_t *params;
} ProtocolDataUnit;

typedef struct {
    uint16_t command;
    int (*handler)(ProtocolDataUnit *);
} ProtocolHandler;

ProtocolDataUnit *Protocol_new_pdu(uint8_t *data, size_t len);
void Protocol_delete_pdu(ProtocolDataUnit *pdu);
void Protocol_set_handler(CommandType command_type, ProtocolHandler *handler);
int Protocol_handle(ProtocolDataUnit *pdu);