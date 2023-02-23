#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <soc.h>

#include <hal/nrf_twi.h>
#include <zephyr/drivers/i2c.h>
#include <BMP180_driver/bmp180.h>

#define I2C_NAME DT_LABEL(DT_NODELABEL(i2c0))

static struct device *i2c_dev;

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
   	i2c_dev = device_get_binding(I2C_NAME);
    
    s32 com_rslt = E_BMP_COMM_RES;

    com_rslt = bmp180_init(&bmp180);
    com_rslt += bmp180_get_calib_param();
    if (com_rslt) {
        printf("error\n");
    }
}

uint8_t PressureSensor_read(void)
{
    u32 v_uncomp_press_u32 = bmp180_get_uncomp_pressure();

    return (uint8_t) bmp180_get_pressure(v_uncomp_press_u32);
}