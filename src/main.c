/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Bridge Service (NUS) sample
 */
#include "uart_async_adapter.h"

#include <zephyr/types.h>
#include <zephyr.h>
#include <drivers/uart.h>
#include <usb/usb_device.h>

#include <device.h>
#include <soc.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/hci.h>

#include <bluetooth/services/nus.h>

#include <dk_buttons_and_leds.h>

#include <settings/settings.h>

#include <stdio.h>

#include <logging/log.h>

#include <hal/nrf_gpio.h>

#define LOG_MODULE_NAME peripheral_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 7

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED DK_LED1
#define RUN_LED_BLINK_INTERVAL 1000

#define CON_STATUS_LED DK_LED2

#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK

#define UART_BUF_SIZE CONFIG_BT_NUS_UART_BUFFER_SIZE
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_WAIT_FOR_RX CONFIG_BT_NUS_UART_RX_WAIT_TIME

static K_SEM_DEFINE(ble_init_ok, 0, 1);

static struct bt_conn *current_conn = NULL;
static struct bt_conn *auth_conn;

static const struct device *uart;
static struct k_work_delayable uart_work;

struct uart_data_t {
	void *fifo_reserved;
	uint8_t data[UART_BUF_SIZE];
	uint16_t len;
};

static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

#if CONFIG_BT_NUS_UART_ASYNC_ADAPTER
UART_ASYNC_ADAPTER_INST_DEFINE(async_adapter);
#else
static const struct device *const async_adapter;
#endif

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(dev);

	static uint8_t *current_buf;
	static size_t aborted_len;
	static bool buf_release;
	struct uart_data_t *buf;
	static uint8_t *aborted_buf;

	switch (evt->type) {
	case UART_TX_DONE:
		LOG_DBG("tx_done");
		if ((evt->data.tx.len == 0) ||
		    (!evt->data.tx.buf)) {
			return;
		}

		if (aborted_buf) {
			buf = CONTAINER_OF(aborted_buf, struct uart_data_t,
					   data);
			aborted_buf = NULL;
			aborted_len = 0;
		} else {
			buf = CONTAINER_OF(evt->data.tx.buf, struct uart_data_t,
					   data);
		}

		k_free(buf);

		buf = k_fifo_get(&fifo_uart_tx_data, K_NO_WAIT);
		if (!buf) {
			return;
		}

		if (uart_tx(uart, buf->data, buf->len, SYS_FOREVER_MS)) {
			LOG_WRN("Failed to send data over UART");
		}

		break;

	case UART_RX_RDY:
		LOG_DBG("rx_rdy");
		buf = CONTAINER_OF(evt->data.rx.buf, struct uart_data_t, data);
		buf->len += evt->data.rx.len;
		buf_release = false;

		if (buf->len == UART_BUF_SIZE) {
			k_fifo_put(&fifo_uart_rx_data, buf);
		} else if ((evt->data.rx.buf[buf->len - 1] == '\n') ||
			  (evt->data.rx.buf[buf->len - 1] == '\r')) {
			k_fifo_put(&fifo_uart_rx_data, buf);
			current_buf = evt->data.rx.buf;
			buf_release = true;
			uart_rx_disable(uart);
		}

		break;

	case UART_RX_DISABLED:
		LOG_DBG("rx_disabled");
		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0;
		} else {
			LOG_WRN("Not able to allocate UART receive buffer");
			k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
			return;
		}

		uart_rx_enable(uart, buf->data, sizeof(buf->data),
			       UART_WAIT_FOR_RX);

		break;

	case UART_RX_BUF_REQUEST:
		LOG_DBG("rx_buf_request");
		buf = k_malloc(sizeof(*buf));
		if (buf) {
			buf->len = 0;
			uart_rx_buf_rsp(uart, buf->data, sizeof(buf->data));
		} else {
			LOG_WRN("Not able to allocate UART receive buffer");
		}

		break;

	case UART_RX_BUF_RELEASED:
		LOG_DBG("rx_buf_released");
		buf = CONTAINER_OF(evt->data.rx_buf.buf, struct uart_data_t,
				   data);
		if (buf_release && (current_buf != evt->data.rx_buf.buf)) {
			k_free(buf);
			buf_release = false;
			current_buf = NULL;
		}

		break;

	case UART_TX_ABORTED:
			LOG_DBG("tx_aborted");
			if (!aborted_buf) {
				aborted_buf = (uint8_t *)evt->data.tx.buf;
			}

			aborted_len += evt->data.tx.len;
			buf = CONTAINER_OF(aborted_buf, struct uart_data_t,
					   data);

			uart_tx(uart, &buf->data[aborted_len],
				buf->len - aborted_len, SYS_FOREVER_MS);

		break;

	default:
		break;
	}
}

static void uart_work_handler(struct k_work *item)
{
	struct uart_data_t *buf;

	buf = k_malloc(sizeof(*buf));
	if (buf) {
		buf->len = 0;
	} else {
		LOG_WRN("Not able to allocate UART receive buffer");
		k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
		return;
	}

	uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_WAIT_FOR_RX);
}

static bool uart_test_async_api(const struct device *dev)
{
	const struct uart_driver_api *api =
			(const struct uart_driver_api *)dev->api;

	return (api->callback_set != NULL);
}

static int uart_init(void)
{
	int err;
	int pos;
	struct uart_data_t *rx;
	struct uart_data_t *tx;

	uart = device_get_binding(CONFIG_BT_NUS_UART_DEV);
	if (!uart) {
		return -ENXIO;
	}

	if (IS_ENABLED(CONFIG_USB_DEVICE_STACK)) {
		err = usb_enable(NULL);
		if (err) {
			LOG_ERR("Failed to enable USB");
			return err;
		}
	}

	rx = k_malloc(sizeof(*rx));
	if (rx) {
		rx->len = 0;
	} else {
		return -ENOMEM;
	}

	k_work_init_delayable(&uart_work, uart_work_handler);


	if (IS_ENABLED(CONFIG_BT_NUS_UART_ASYNC_ADAPTER) && !uart_test_async_api(uart)) {
		/* Implement API adapter */
		uart_async_adapter_init(async_adapter, uart);
		uart = async_adapter;
	}

	err = uart_callback_set(uart, uart_cb, NULL);
	if (err) {
		LOG_ERR("Cannot initialize UART callback");
		return err;
	}

	if (IS_ENABLED(CONFIG_UART_LINE_CTRL)) {
		LOG_INF("Wait for DTR");
		while (true) {
			uint32_t dtr = 0;

			uart_line_ctrl_get(uart, UART_LINE_CTRL_DTR, &dtr);
			if (dtr) {
				break;
			}
			/* Give CPU resources to low priority threads. */
			k_sleep(K_MSEC(100));
		}
		LOG_INF("DTR set");
		err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DCD, 1);
		if (err) {
			LOG_WRN("Failed to set DCD, ret code %d", err);
		}
		err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DSR, 1);
		if (err) {
			LOG_WRN("Failed to set DSR, ret code %d", err);
		}
	}

	tx = k_malloc(sizeof(*tx));

	if (tx) {
		pos = snprintf(tx->data, sizeof(tx->data),
			       "Starting Nordic UART service example\r\n");

		if ((pos < 0) || (pos >= sizeof(tx->data))) {
			k_free(tx);
			LOG_ERR("snprintf returned %d", pos);
			return -ENOMEM;
		}

		tx->len = pos;
	} else {
		return -ENOMEM;
	}

	err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
	if (err) {
		LOG_ERR("Cannot display welcome message (err: %d)", err);
		return err;
	}

	return uart_rx_enable(uart, rx->data, sizeof(rx->data), 50);
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		LOG_ERR("Connection failed (err %u)", err);
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected %s", log_strdup(addr));

	current_conn = bt_conn_ref(conn);

	//dk_set_led_on(CON_STATUS_LED);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason %u)", log_strdup(addr), reason);

	if (auth_conn) {
		bt_conn_unref(auth_conn);
		auth_conn = NULL;
	}

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
		//dk_set_led_off(CON_STATUS_LED);
	}
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		LOG_INF("Security changed: %s level %u", log_strdup(addr),
			level);
	} else {
		LOG_WRN("Security failed: %s level %u err %d", log_strdup(addr),
			level, err);
	}
}
#endif

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected    = connected,
	.disconnected = disconnected,
#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	.security_changed = security_changed,
#endif
};

#if defined(CONFIG_BT_NUS_SECURITY_ENABLED)
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", log_strdup(addr), passkey);
}

static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	auth_conn = bt_conn_ref(conn);

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", log_strdup(addr), passkey);
	LOG_INF("Press Button 1 to confirm, Button 2 to reject.");
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing cancelled: %s", log_strdup(addr));
}

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing completed: %s, bonded: %d", log_strdup(addr),
		bonded);
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing failed conn: %s, reason %d", log_strdup(addr),
		reason);
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.passkey_confirm = auth_passkey_confirm,
	.cancel = auth_cancel,
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};
#else
static struct bt_conn_auth_cb conn_auth_callbacks;
#endif

static uint16_t data2uint16(uint8_t *data)
{
	uint16_t val;

	val = data[0];
	val <<= 8;
	val |= data[1];

	return val;
}

static void motor_on(void)
{
	nrf_gpio_pin_set(14);
}

static void motor_off(void)
{
	nrf_gpio_pin_clear(14);
}

static void heater_on(void)
{
	nrf_gpio_pin_set(23);
}

static void heater_off(void)
{
	nrf_gpio_pin_clear(23);
}

static uint32_t m_pressure_out_flag = false;
static uint32_t m_state_time = 0;

static uint8_t  m_id[8];

#define MO_PATTERN_1_ON		(1)
#define MO_PATTERN_1_OFF	(1)

#define MO_PATTERN_2_ON		(2)
#define MO_PATTERN_2_OFF	(1)

#define MO_PATTERN_3_ON		(3)
#define MO_PATTERN_3_OFF	(1)

#define MO_PATTERN_4_ON		(4)
#define MO_PATTERN_4_OFF	(1)

static void set_motor_pattern(uint32_t on, uint32_t off);

typedef enum {
	MOTOR_ON = 0,
	MOTOR_OFF,
} en_motor_state_t;

static en_motor_state_t m_motor_state = MOTOR_OFF;

static uint32_t m_motor_pattern_on = 0;
static uint32_t m_motor_pattern_off = 0;

static uint32_t m_running_flag = false;

static void write_flash_data(uint8_t *data, int len);
static void read_flash_data(uint8_t *buff, int len);

static void start_motor(void);
static void stop_motor(void);

static uint8_t m_buff[32];
static uint8_t m_response[32];

static void send_response(uint8_t *data, uint16_t size)
{
	bt_nus_send(current_conn, data, size);
}

static void app_handle(uint8_t *data, uint16_t len)
{
	uint16_t code;
	uint16_t param;

	code = data2uint16(data);
	param = data2uint16(data + 2);

	switch (code)
	{
		/* ID Setting */
		case 0x55AA:
			m_buff[0] = data[2];
			m_buff[1] = data[3];
			m_buff[2] = data[4];
			m_buff[3] = data[5];
			write_flash_data(m_buff, 4);


			m_response[0] = 0xAA;
			m_response[1] = 0x55;
			m_response[2] = data[2];
			m_response[3] = data[3];
			m_response[4] = data[4];
			m_response[5] = data[5];
			send_response(m_response, 6);
			break;

		/* Start */
		case 0x01FE:
			//motor_on();
			start_motor();
			
			heater_on();
			break;
			
		/* Stop */
		case 0x02FD:
			//motor_off();
			stop_motor();
			
			heater_off();
			break;
			
		/* Pause and resume */
		case 0x04FB:
			break;
			
		/* Power off */
		case 0x08F7:
			//motor_off();
			stop_motor();
			
			heater_off();
			break;
			
		/* Heater on/off */
		case 0x10EF:				
			if (param == 0x0001) 
			{
				heater_on();
			}
			else if (param == 0x0000)
			{
				heater_off();
			}
			break;

		/* Motor on/off */
		case 0x20DF:				
			if (param == 0x0001) 
			{
				//motor_on();
				start_motor();
			}
			else if (param == 0x0000)
			{
				//motor_off();
				stop_motor();
			}
			break;

		/* Measure on/off */
		case 0x40FB:				
			if (param == 0x0001) 
			{
				m_pressure_out_flag = true;
			}
			else if (param == 0x0000)
			{
				m_pressure_out_flag = false;
			}
			break;

		/* Vibration Pattern [pattern number] */
		case 0x807F:				
			set_pattern(param);
			break;

		/* Vibration Level */
		case 0x817E:				
			break;

		default:
			break;
	}
}

static void bt_sent_cb(struct bt_conn *conn)
{
}

static void bt_send_enabled_cb(int status)
{
}

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data, uint16_t len)
{
	int err;
	char addr[BT_ADDR_LE_STR_LEN] = {0};

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

	for (uint16_t pos = 0; pos != len;) {
		struct uart_data_t *tx = k_malloc(sizeof(*tx));

		if (!tx) {
			LOG_WRN("Not able to allocate UART send data buffer");
			return;
		}

		/* Keep the last byte of TX buffer for potential LF char. */
		size_t tx_data_size = sizeof(tx->data) - 1;

		if ((len - pos) > tx_data_size) {
			tx->len = tx_data_size;
		} else {
			tx->len = (len - pos);
		}

		memcpy(tx->data, &data[pos], tx->len);

		pos += tx->len;

		/* Append the LF character when the CR character triggered
		 * transmission from the peer.
		 */
		if ((pos == len) && (data[len - 1] == '\r')) {
			tx->data[tx->len] = '\n';
			tx->len++;
		}

		err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
		if (err) {
			k_fifo_put(&fifo_uart_tx_data, tx);
		}

		app_handle(data, len);
	}
}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
	.sent = bt_sent_cb,
	.send_enabled = bt_send_enabled_cb,
};

void error(void)
{
	dk_set_leds_state(DK_ALL_LEDS_MSK, DK_NO_LEDS_MSK);

	while (true) {
		/* Spin for ever */
		k_sleep(K_MSEC(1000));
	}
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void num_comp_reply(bool accept)
{
	if (accept) {
		bt_conn_auth_passkey_confirm(auth_conn);
		LOG_INF("Numeric Match, conn %p", (void *)auth_conn);
	} else {
		bt_conn_auth_cancel(auth_conn);
		LOG_INF("Numeric Reject, conn %p", (void *)auth_conn);
	}

	bt_conn_unref(auth_conn);
	auth_conn = NULL;
}

void button_changed(uint32_t button_state, uint32_t has_changed)
{
	uint32_t buttons = button_state & has_changed;

	if (auth_conn) {
		if (buttons & KEY_PASSKEY_ACCEPT) {
			num_comp_reply(true);
		}

		if (buttons & KEY_PASSKEY_REJECT) {
			num_comp_reply(false);
		}
	}
}
#endif /* CONFIG_BT_NUS_SECURITY_ENABLED */

static void configure_gpio(void)
{
	int err;

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	err = dk_buttons_init(button_changed);
	if (err) {
		LOG_ERR("Cannot init buttons (err: %d)", err);
	}
#endif /* CONFIG_BT_NUS_SECURITY_ENABLED */

	err = dk_leds_init();
	if (err) {
		LOG_ERR("Cannot init LEDs (err: %d)", err);
	}
}

#include <hal/nrf_twi.h>
#include <hal/nrf_twim.h>
#include <drivers/i2c.h>

#define twi		DT_LABEL(DT_NODELABEL(i2c0))
#define twi1	DT_LABEL(DT_NODELABEL(i2c1))

struct device *dwi_dev;
struct device *dwi1_dev;

static uint8_t data_buff[32];

void sensor_delay(uint32_t delay)
{
    k_sleep(K_MSEC(delay));
}

int read_sensor_data(uint8_t dev, uint8_t reg, uint8_t *buff, uint8_t cnt)
{
    return i2c_burst_read(dwi_dev, dev, reg, buff, cnt);
}

int write_sensor_data(uint8_t dev, uint8_t reg, uint8_t *buff, uint8_t cnt)
{
    return i2c_burst_write(dwi_dev, dev, reg, buff, cnt);
}

static void write_flash_data(uint8_t *data, int len)
{
	i2c_burst_write(dwi1_dev, 0x57, 0x00, data, len);

	k_sleep(K_MSEC(4));
}

static void read_flash_data(uint8_t *buff, int len)
{
    i2c_burst_read(dwi1_dev, 0x57, 0x00, buff, len);
}

#include <drivers/pwm.h>

#define pwm	DT_LABEL(DT_NODELABEL(pwm1))

struct device *pwm_dev;

#define PWM_PERIOD_US		(USEC_PER_SEC / 50U)
#define PWM_PULSE_US		(1000U)

void set_pattern(int p_no)
{
	switch (p_no)
	{
		case 0:
			set_motor_pattern(0xffffffff, 0);
			break;

		case 1:
			set_motor_pattern(MO_PATTERN_1_ON, MO_PATTERN_1_OFF);
			break;

		case 2:
			set_motor_pattern(MO_PATTERN_2_ON, MO_PATTERN_2_OFF);
			break;

		case 3:
			set_motor_pattern(MO_PATTERN_3_ON, MO_PATTERN_3_OFF);
			break;

		case 4:
			set_motor_pattern(MO_PATTERN_4_ON, MO_PATTERN_4_OFF);
			break;
		
		default:
			break;
	}
}

static void set_motor_pattern(uint32_t on, uint32_t off)
{
	m_motor_pattern_on = on;
	
	m_motor_pattern_off = off;
}

static void start_motor(void)
{
	motor_on();
	
	m_running_flag = true;

	m_state_time = 0;
	
	m_motor_state = MOTOR_ON;
}

static void stop_motor(void)
{
	motor_off();
	
	m_running_flag = false;

	m_state_time = 0;
	
	m_motor_state = MOTOR_OFF;
}

static void motor_proc(void)
{
	if (m_running_flag == true)
	{
		if (m_motor_state == MOTOR_ON)
		{
			if (++m_state_time > m_motor_pattern_on)
			{
				motor_off();
				m_motor_state = MOTOR_OFF;
				m_state_time = 0;
			}
		}
		else
		{
			if (++m_state_time > m_motor_pattern_off)
			{
				motor_on();
				m_motor_state = MOTOR_ON;
				m_state_time = 0;
			}
		}
	}
}

void main(void)
{
	int blink_status = 0;
	int err = 0;
	int flag = 0;
	int pos;
	int value;
	uint8_t buff[32] = { 0x00, };
	uint8_t data_buff[32];
	int led_count = 0;

	nrf_gpio_cfg_output(38);
	
	nrf_gpio_cfg_output(34);

	nrf_gpio_cfg_output(14);
	
	nrf_gpio_pin_clear(14);
	
	stop_motor();

	dwi_dev = device_get_binding(twi);

	dwi1_dev = device_get_binding(twi1);

	pwm_dev = device_get_binding(pwm);

	err = uart_init();
	if (err) {
		error();
	}

	bmp180_data_readout_template();

	read_flash_data(m_id, 4);

	set_pattern(2);
	
	if (IS_ENABLED(CONFIG_BT_NUS_SECURITY_ENABLED)) {
		bt_conn_auth_cb_register(&conn_auth_callbacks);
	}

	err = bt_enable(NULL);
	if (err) {
		error();
	}

	LOG_INF("Bluetooth initialized");

	k_sem_give(&ble_init_ok);

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_nus_init(&nus_cb);
	if (err) {
		LOG_ERR("Failed to initialize UART service (err: %d)", err);
		return;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd,
			      ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return;
	}

	m_pressure_out_flag = true;
	
	for (;;) 
	{
        if (led_count++ >= 5) 
        {
			if (flag)
			{
				nrf_gpio_pin_clear(38);
				nrf_gpio_pin_clear(34);
				flag = 0;
			}
			else
			{
				nrf_gpio_pin_set(38);
				nrf_gpio_pin_set(34);
				flag = 1;
			}
            led_count = 0;
        }

        if (current_conn != NULL)
        {
			if (m_pressure_out_flag)
			{
				value = bmp180_data_readout();
				pos = 0;

				data_buff[pos++] = 0x11;
				data_buff[pos++] = 0xEE;
				
				data_buff[pos++] = (uint8_t)(value >> 3);
				data_buff[pos++] = (uint8_t)(value >> 2);
				data_buff[pos++] = (uint8_t)(value >> 1);
				data_buff[pos++] = (uint8_t)(value >> 0);

				value = ~value;
				data_buff[pos++] = (uint8_t)(value >> 3);
				data_buff[pos++] = (uint8_t)(value >> 2);
				data_buff[pos++] = (uint8_t)(value >> 1);
				data_buff[pos++] = (uint8_t)(value >> 0);
				
				bt_nus_send(current_conn, data_buff, pos);
			}
        }

		motor_proc();
		
		k_sleep(K_MSEC(100));
	}
}

void ble_write_thread(void)
{
	/* Don't go any further until BLE is initialized */
	k_sem_take(&ble_init_ok, K_FOREVER);

	for (;;) {
		/* Wait indefinitely for data to be sent over bluetooth */
		struct uart_data_t *buf = k_fifo_get(&fifo_uart_rx_data,
						     K_FOREVER);

		if (bt_nus_send(NULL, buf->data, buf->len)) {
			LOG_WRN("Failed to send data over BLE connection");
		}

		k_free(buf);
	}
}

K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL,
		NULL, PRIORITY, 0, 0);
