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
static uint8_t vibration_level;

extern uint8_t PressureSensor_read(void);

void reserve(struct k_timer *timer, k_timeout_t timeout)
{
    k_timer_start(timer, timeout, K_NO_WAIT);
}

static void heater(struct k_timer *dummy)
{
    extern float PressureSensor_temp(void);
    if (PressureSensor_temp() < 36.5) {
        nrf_gpio_pin_set(HEATER_PIN);
    }
    if (PressureSensor_temp() > 40) {
        nrf_gpio_pin_clear(HEATER_PIN);
    }
}

K_TIMER_DEFINE(heater_timer, heater, NULL);

void stop_heater(void)
{
    is_heating = false;
    nrf_gpio_pin_set(HEATER_PIN);
    k_timer_stop(&heater_timer);
}

K_TIMER_DEFINE(heater_end_timer, (k_timer_expiry_t)stop_heater, NULL);

void start_heater(void)
{
    is_heating = true;
    k_timer_start(&heater_timer, K_MSEC(0), K_SECONDS(1));
    k_timer_stop(&heater_end_timer);
    if (!is_measuring && !is_vibrating) {
        reserve(&heater_end_timer, K_MINUTES(15));
    }
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
    if (on_off) {
        start_heater();
    } else {
        stop_heater();
    }
    return 0;
}

static void motor(struct k_timer *dummy)
{
    static uint32_t cnt = 0;

    uint8_t duty = cnt++ % 5;
    if (duty < pattern || duty == MIN(pattern + 1, PATTERN5)) {
        nrf_gpio_pin_set(MOTOR_PIN);
    } else {
        nrf_gpio_pin_clear(MOTOR_PIN);
    }
}

K_TIMER_DEFINE(motor_timer, motor, NULL);

void start_motor(void)
{
    k_timer_start(&motor_timer, K_MSEC(0), K_MSEC(25));
}

void stop_motor(void)
{
    nrf_gpio_pin_clear(MOTOR_PIN);
    k_timer_stop(&motor_timer);
}

K_TIMER_DEFINE(motor_end_timer, (k_timer_expiry_t)stop_motor, NULL);

static void vibration(struct k_timer *timer)
{
    uint8_t data = PressureSensor_read();

    if (is_over_level) {
        if (data >= vibration_level) {
            start_motor();
            reserve(&motor_end_timer, K_SECONDS(10));
        }
    } else {    
        if (data < vibration_level) {
            start_motor();
        } else {
            stop_motor();
            k_timer_start(timer, K_SECONDS(10), K_MSEC(500)); 
        }
    }
}

K_TIMER_DEFINE(vibration_timer, vibration, NULL);

void stop_vibration(void)
{
    is_vibrating = false;
    k_timer_stop(&vibration_timer);
    stop_motor();
}

K_TIMER_DEFINE(vibration_end_timer, (k_timer_expiry_t)stop_vibration, NULL);

void start_vibration(void)
{
    is_vibrating = true;
    if (vibration_level == 0) {
        vibration_level = PressureSensor_read() + 10;
    }
    if (is_measuring) {
        stop_motor();
        k_timer_start(&vibration_timer, K_MSEC(0), K_MSEC(500));
    } else {
        k_timer_stop(&vibration_timer);
        start_motor();
    }
    k_timer_stop(&motor_end_timer);
    k_timer_stop(&vibration_end_timer);
    if (!is_measuring && !is_heating) {
        reserve(&vibration_end_timer, K_MINUTES(5));
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
    if (on_off) {
        stop_measure(); // for UX
        start_vibration();
    } else {
	    stop_vibration();
    }
    return 0;
}

extern void measure(struct k_timer *dummy);

K_TIMER_DEFINE(measure_timer, measure, NULL);

void stop_measure(void)
{
    is_measuring = false;
    k_timer_stop(&measure_timer);
}

K_TIMER_DEFINE(measure_end_timer, (k_timer_expiry_t)stop_measure, NULL);

void start_measure(void)
{
    is_measuring = true;
    k_timer_start(&measure_timer, K_MSEC(0), K_MSEC(200));
    k_timer_stop(&measure_end_timer);
    if (!is_vibrating && !is_heating) {
        reserve(&measure_end_timer, K_MINUTES(15));
    }
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
    if (on_off) {
        start_measure();
    } else {
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