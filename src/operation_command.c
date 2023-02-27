#include "device_identification.h"
#include <zephyr/posix/arpa/inet.h>
#include "protocol.h"

static ProtocolHandler start_handler, stop_handler;
static ProtocolHandler pause_resume_handler, power_off_handler;

bool is_paused = false;

extern void start_vibration(void);
extern void stop_vibration(void);
extern void start_heater(void);
extern void stop_heater(void);
extern void start_measure(void);
extern void stop_measure(void);

static void start_operation_mode(void)
{
    start_measure();
    start_vibration();
    start_heater();
}

static void stop_operation_mode(void)
{
    stop_measure();
    stop_vibration();
    stop_heater();
}

int start(ProtocolDataUnit *pdu)
{
    start_operation_mode();
    if (pdu->command != start_handler.command) {
        return -1;
    }
    return 0;
}

int stop(ProtocolDataUnit *pdu)
{
    stop_operation_mode();
    if (pdu->command != stop_handler.command) {
        return -1;
    }
    return 0;
}

int pause_resume(ProtocolDataUnit *pdu)
{
    if (pdu->command != pause_resume_handler.command) {
        return -1;
    }
    is_paused = !is_paused;
    if (is_paused) {
        stop_operation_mode();
    } else {
        start_operation_mode();
    }
    return 0;
}

int power_off(ProtocolDataUnit *pdu)
{
    if (pdu->command != power_off_handler.command) {
        return -1;
    }
    return 0;
}

void OperationCommand_init(void)
{
    start_handler.command = 0x01FE;
    start_handler.handler = start;
    stop_handler.command = 0x02FD;
    stop_handler.handler = stop;
    pause_resume_handler.command = 0x04FB;
    pause_resume_handler.handler = pause_resume;
    power_off_handler.command = 0x08F7;
    power_off_handler.handler = power_off;
    Protocol_set_handler(COMMAND_OP_START, &start_handler);
    Protocol_set_handler(COMMAND_OP_STOP, &stop_handler);
    Protocol_set_handler(COMMAND_OP_PAUSE_RESUME, &pause_resume_handler);
    Protocol_set_handler(COMMAND_OP_POWER_OFF, &power_off_handler);
    start_operation_mode();
}