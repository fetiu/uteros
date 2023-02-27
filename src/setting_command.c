#include "device_identification.h"
#include <zephyr/posix/arpa/inet.h>
#include <hal/nrf_gpio.h>
#include "protocol.h"

#define HEATER_PIN  23
#define MOTOR_PIN   14

static ProtocolHandler set_heater_handler, set_motor_handler;
static ProtocolHandler set_measure_handler, set_pattern_hander, set_level_hander;
static bool is_heating, is_vibrating, is_measuring, is_over_level = true;
static enum {PATTERN1, PATTERN2, PATTERN3, PATTERN4, PATTERN5} pattern;
static uint8_t vibration_level = 40;

static void limit(struct k_timer *dummy)
{
    extern float PressureSensor_temp(void);
    if (PressureSensor_temp() > 40) {
        nrf_gpio_pin_clear(HEATER_PIN);
    }

    if (PressureSensor_temp() < 30) {
        nrf_gpio_pin_set(HEATER_PIN);
    }
}

K_TIMER_DEFINE(heater_timer, limit, NULL);

void heater_mode(void)
{
    k_timer_start(&heater_timer, K_MSEC(0), K_MSEC(1000));
}

void heater_stop(void)
{
    nrf_gpio_pin_set(HEATER_PIN);
}

int set_heater(ProtocolDataUnit *pdu)
{
    if (pdu->command != set_heater_handler.command) {
        return -1;
    }
    uint16_t on_off = NTOH_UINT16(pdu->params);
    if (on_off > 0x0001) {
        return -2;
    }
    is_heating = (bool)on_off;
    if (is_heating) {
        heater_mode();
    } else {
	    nrf_gpio_pin_clear(HEATER_PIN);
    }
    return 0;
}

static void motor(struct k_timer *dummy)
{
    static uint32_t cnt = 0;

    if (cnt++ % 6 >= pattern) {
        nrf_gpio_pin_set(MOTOR_PIN);
    } else {
        nrf_gpio_pin_clear(MOTOR_PIN);
    }
}

K_TIMER_DEFINE(motor_timer, motor, NULL);

void MotorPattern_run(bool enable)
{
    if (!enable) {
        nrf_gpio_pin_clear(MOTOR_PIN);
        return;
    }
    
    k_timer_start(&heater_timer, K_MSEC(0), K_MSEC(1000));
}

void vibration_mode(void)
{
    extern uint8_t PressureSensor_read(void);
    uint8_t data = PressureSensor_read();
    
    if (is_over_level) {
        MotorPattern_run(data > vibration_level);
    } else {
        MotorPattern_run(data < vibration_level);
    }
} 

int set_motor(ProtocolDataUnit *pdu)
{
    if (pdu->command != set_motor_handler.command) {
        return -1;
    }
    uint16_t on_off = NTOH_UINT16(pdu->params);
    if (on_off > 0x0001) {
        return -2;
    }
    is_vibrating = (bool)on_off;
    if (is_vibrating) {
	    MotorPattern_run(true);
    } else {
	    MotorPattern_run(false);
    }
    return 0;
}

int set_measure(ProtocolDataUnit *pdu)
{
    if (pdu->command != set_measure_handler.command) {
        return -1;
    }
    uint16_t on_off = NTOH_UINT16(pdu->params);
    if (on_off > 0x0001) {
        return -2;
    }
    is_measuring = (bool)on_off;
    if (is_measuring) {
        extern void start_measure(void);
        start_measure();
    } else {
        extern void stop_measure(void);
        stop_measure();
    }
    return 0;
}

int set_pattern(ProtocolDataUnit *pdu)
{
    if (pdu->command != set_pattern_hander.command) {
        return -1;
    }
    uint16_t number = NTOH_UINT16(pdu->params);
    if (number > 0x0004) {
        return -2;
    }
    pattern = number;
    return 0;
}

int set_level(ProtocolDataUnit *pdu)
{
    if (pdu->command != set_level_hander.command) {
        return -1;
    }
    vibration_level = pdu->params[0];
    uint8_t direction = pdu->params[1];
    if (direction > 0x01) {
        return -2;
    }
    is_over_level = (bool)direction;
    return 0;
}

void SettingCommand_init(void)
{
    is_heating = is_vibrating = is_measuring = false;
    nrf_gpio_cfg_output(HEATER_PIN);
	nrf_gpio_cfg_output(MOTOR_PIN);
	nrf_gpio_pin_clear(HEATER_PIN);
	nrf_gpio_pin_clear(MOTOR_PIN);

    set_heater_handler.command = 0x10EF;
    set_heater_handler.handler = set_heater;
    set_motor_handler.command = 0x20DF;
    set_motor_handler.handler = set_motor;
    set_measure_handler.command = 0x40FB;
    set_measure_handler.handler = set_measure;
    set_pattern_hander.command = 0x807F;
    set_pattern_hander.handler = set_pattern;
    set_level_hander.command = 0x817E;
    set_level_hander.handler = set_level;
    Protocol_set_handler(COMMAND_SET_HEATER, &set_heater_handler);
    Protocol_set_handler(COMMAND_SET_MOTOR, &set_motor_handler);
    Protocol_set_handler(COMMAND_SET_MEASURE, &set_measure_handler);
    Protocol_set_handler(COMMAND_SET_PATTERN, &set_pattern_hander);
    Protocol_set_handler(COMMAND_SET_LEVEL, &set_level_hander);
}