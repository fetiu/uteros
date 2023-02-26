#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <soc.h>

#include <hal/nrf_twi.h>
#include <zephyr/drivers/i2c.h>
#include <BMP180_driver/bmp180.h>

#define I2C_NAME DT_NODELABEL(i2c0)
#define I2C_CFG (I2C_SPEED_SET(I2C_SPEED_FAST) | I2C_MODE_CONTROLLER)
#define ASSERT(expr) if (expr) __BKPT(0)

static const struct device *i2c_dev;

void BMP180_delay_msek(uint32_t msek)
{
    k_sleep(K_MSEC(msek));
}

s8 BMP180_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    return i2c_burst_read(i2c_dev, dev_addr, reg_addr, reg_data, cnt);
}

s8 BMP180_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    return i2c_burst_write(i2c_dev, dev_addr, reg_addr, reg_data, cnt);
}

static struct bmp180_t bmp180 = {
    .bus_write = BMP180_I2C_bus_write,
    .bus_read = BMP180_I2C_bus_read,
    .dev_addr = BMP180_I2C_ADDR,
    .delay_msec = BMP180_delay_msek,
};

void PressureSensor_init(void)
{
    i2c_dev = DEVICE_DT_GET(I2C_NAME);
    while(!device_is_ready(i2c_dev));
    ASSERT(i2c_configure(i2c_dev, I2C_CFG));
    
    ASSERT(bmp180_init(&bmp180));
    ASSERT(bmp180_get_calib_param());
}

uint8_t PressureSensor_read(void)
{
    u32 v_uncomp_press_u32 = bmp180_get_uncomp_pressure();

    return (uint8_t) bmp180_get_pressure(v_uncomp_press_u32);
}