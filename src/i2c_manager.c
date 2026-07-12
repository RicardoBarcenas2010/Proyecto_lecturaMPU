#include "i2c_manager.h"
#include "system_config.h"

#include "driver/i2c_master.h"
#include "esp_log.h"

static const char *TAG = "I2C";

static i2c_master_bus_handle_t bus_handle = NULL;

esp_err_t i2c_manager_init(void)
{
    i2c_master_bus_config_t bus_config =
    {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT,
        .scl_io_num = I2C_SCL_PIN,
        .sda_io_num = I2C_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };

    esp_err_t ret = i2c_new_master_bus(&bus_config, &bus_handle);

    if(ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Bus I2C inicializado correctamente");
    }

    return ret;
}

static esp_err_t add_device(uint8_t address,
                            i2c_master_dev_handle_t *dev)
{
    i2c_device_config_t dev_cfg =
    {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = I2C_FREQUENCY
    };

    return i2c_master_bus_add_device(bus_handle,
                                     &dev_cfg,
                                     dev);
}

esp_err_t i2c_write_register(uint8_t device_addr,
                             uint8_t reg_addr,
                             uint8_t *data,
                             size_t len)
{
    i2c_master_dev_handle_t dev;

    ESP_ERROR_CHECK(add_device(device_addr, &dev));

    uint8_t buffer[len + 1];

    buffer[0] = reg_addr;

    for(size_t i = 0; i < len; i++)
        buffer[i + 1] = data[i];

    esp_err_t ret = i2c_master_transmit(dev,
                                        buffer,
                                        len + 1,
                                        -1);

    i2c_master_bus_rm_device(dev);

    return ret;
}

esp_err_t i2c_read_register(uint8_t device_addr,
                            uint8_t reg_addr,
                            uint8_t *data,
                            size_t len)
{
    i2c_master_dev_handle_t dev;

    ESP_ERROR_CHECK(add_device(device_addr, &dev));

    esp_err_t ret = i2c_master_transmit_receive(dev,
                                                &reg_addr,
                                                1,
                                                data,
                                                len,
                                                -1);

    i2c_master_bus_rm_device(dev);

    return ret;
}