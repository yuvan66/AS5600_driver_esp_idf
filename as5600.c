#include "as5600.h"

esp_err_t as_init(as5600_config_t *config, as5600_t *dev)
{
    if (dev == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_master_bus_config_t bus_config = {
        .i2c_port = config->i2c_port,
        .scl_io_num = config->scl_num,
        .sda_io_num = config->sda_num,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags.enable_internal_pullup = true,
        .glitch_ignore_cnt = 7,
    };

    esp_err_t ret = i2c_new_master_bus(&bus_config, &dev->bus_handle);
    if (ret != ESP_OK)(val2 << 8) | val1;
    {
        return ret;
    }

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BASE,
        .scl_speed_hz = config->scl_hz,
    };

    return i2c_master_bus_add_device(dev->bus_handle, &dev_config, &dev->dev_handle);

}

esp_err_t as_get_angle_r (as5600_t *dev, float *angle)
{
    if (dev == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t reg = STATUS;
    uint8_t val1, val2;  // val1 -> Lower   // val2 -> Higher
    uint16_t value;
    esp_err_t ret = i2c_master_transmit_receive(dev->dev_handle, &reg, BUFFER_SIZE_1, &val1, BUFFER_SIZE_1, pdMS_TO_TICKS(100));

    if (ret != ESP_OK)
    {
        return ret;
    }

    if (!((val1 >> 5) & 0x01)) {
        return AS5600_ERR_MAGNET_NOT_DETECTED;
    }
    if ((val1 >> 4) & 0x01) {
        return AS5600_ERR_MAGNET_TOO_WEAK;
    }
    if ((val1 >> 3) & 0x01) {
        return AS5600_ERR_MAGNET_TOO_STRONG;
    }

    reg = RAW_H;
    ret = i2c_master_transmit_receive(dev->dev_handle, &reg, BUFFER_SIZE_1, &val2, BUFFER_SIZE_1, pdMS_TO_TICKS(100));

    if (ret != ESP_OK)
    {
        return ret;
    }

    reg = RAW_L;
    ret = i2c_master_transmit_receive(dev->dev_handle, &reg, BUFFER_SIZE_1, &val1, BUFFER_SIZE_1, pdMS_TO_TICKS(100));

    if (ret != ESP_OK)
    {
        return ret;
    }

    value &= 0x0000;
    value = ((val2 & 0x0F) << 8) | val1;
    uint16_t adjusted = (value + 4096 - dev->zero_offset) % 4096;
    *angle = ((float)adjusted/4096.0f)*360.0f;

    return ESP_OK;

}

esp_err_t as_get_angle(as5600_t *dev, float *angle)
{
    if (dev == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t reg = STATUS;
    uint8_t val1, val2;  // val1 -> Lower   // val2 -> Higher
    uint16_t value;
    esp_err_t ret = i2c_master_transmit_receive(dev->dev_handle, &reg, BUFFER_SIZE_1, &val1, BUFFER_SIZE_1, pdMS_TO_TICKS(100));

    if (ret != ESP_OK)
    {
        return ret;
    }

    if (!((val1 >> 5) & 0x01)) {
        return AS5600_ERR_MAGNET_NOT_DETECTED;
    }
    if ((val1 >> 4) & 0x01) {
        return AS5600_ERR_MAGNET_TOO_WEAK;
    }
    if ((val1 >> 3) & 0x01) {
        return AS5600_ERR_MAGNET_TOO_STRONG;
    }

    reg = ANGLE_H;
    ret = i2c_master_transmit_receive(dev->dev_handle, &reg, BUFFER_SIZE_1, &val2, BUFFER_SIZE_1, pdMS_TO_TICKS(100));

    if (ret != ESP_OK)
    {
        return ret;
    }

    reg = ANGLE_L;
    ret = i2c_master_transmit_receive(dev->dev_handle, &reg, BUFFER_SIZE_1, &val1, BUFFER_SIZE_1, pdMS_TO_TICKS(100));

    if (ret != ESP_OK)
    {
        return ret;
    }

    value &= 0x0000;
    value = ((val2 & 0x0F) << 8) | val1;

    *angle = ((float)value/4096.0f)*360.0f;

    return ESP_OK;
}

esp_err_t as_get_mag(as5600_t *dev, uint16_t *mag)
{
    if (dev == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t reg = STATUS;
    uint8_t val1, val2;  // val1 -> Lower   // val2 -> Higher
    uint16_t value;
    esp_err_t ret = i2c_master_transmit_receive(dev->dev_handle, &reg, BUFFER_SIZE_1, &val1, BUFFER_SIZE_1, pdMS_TO_TICKS(100));

    if (ret != ESP_OK)
    {
        return ret;
    }

    if (!((val1 >> 5) & 0x01)) {
        return AS5600_ERR_MAGNET_NOT_DETECTED;
    }
    if ((val1 >> 4) & 0x01) {
        return AS5600_ERR_MAGNET_TOO_WEAK;
    }
    if ((val1 >> 3) & 0x01) {
        return AS5600_ERR_MAGNET_TOO_STRONG;
    }

    reg = MAG_H;
    ret = i2c_master_transmit_receive(dev->dev_handle, &reg, BUFFER_SIZE_1, &val2, BUFFER_SIZE_1, pdMS_TO_TICKS(100));

    if (ret != ESP_OK)
    {
        return ret;
    }

    reg = MAG_L;
    ret = i2c_master_transmit_receive(dev->dev_handle, &reg, BUFFER_SIZE_1, &val1, BUFFER_SIZE_1, pdMS_TO_TICKS(100));

    if (ret != ESP_OK)
    {
        return ret;
    }

    value &= 0x0000;
    value = ((val2 & 0x0F) << 8) | val1;

    *mag = value;

    return ESP_OK;   
}


/* Helper functions start */
esp_err_t write_12b_reg (as5600_t *dev, uint8_t reg_h, uint8_t reg_l, uint16_t value)
{
    if (dev == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t hi = (value >> 8) & 0xFF;
    uint8_t lo = value & 0x00FF;

    uint8_t buf_h[2] = {reg_h, hi};
    uint8_t buf_l[2] = {reg_l, lo};

    esp_err_t ret = i2c_master_transmit(dev->dev_handle, buf_h, BUFFER_SIZE_2, pdMS_TO_TICKS(100));

    if (ret != ESP_OK)
    {
        return ret;
    }
    
    return i2c_master_transmit(dev->dev_handle, buf_l, BUFFER_SIZE_2, pdMS_TO_TICKS(100));
}

esp_err_t read_12b_reg (as5600_t *dev, uint8_t reg_h, uint8_t reg_l, uint16_t *value)
{
    if (dev == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t hi;
    uint8_t lo;

    // uint8_t buf_h[2] = {reg_h, hi};
    // uint8_t buf_l[2] = {reg_l, lo};

    esp_err_t ret = i2c_master_transmit_receive(dev->dev_handle, &reg_h, BUFFER_SIZE_1, &hi, BUFFER_SIZE_1, pdMS_TO_TICKS(100));
    if (ret != ESP_OK)
    {
        return ret;
    }

    ret = i2c_master_transmit_receive(dev->dev_handle, &reg_l, BUFFER_SIZE_1, &lo, BUFFER_SIZE_1, pdMS_TO_TICKS(100));
    if (ret != ESP_OK)
    {
        return ret;
    }

    *value = ((hi & 0x0F) << 8) | lo;
    return ESP_OK;
}

esp_err_t write_14b_reg(as5600_t *dev, uint8_t reg_h, uint8_t reg_l, uint16_t value)
{
    if (dev == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t hi = (value >> 8) & 0x3F;
    uint8_t lo = value & 0x00FF;

    uint8_t buf_h[2] = {reg_h, hi};
    uint8_t buf_l[2] = {reg_l, lo};

    esp_err_t ret = i2c_master_transmit(dev->dev_handle, buf_h, BUFFER_SIZE_2, pdMS_TO_TICKS(100));

    if (ret != ESP_OK)
    {
        return ret;
    }
    
    return i2c_master_transmit(dev->dev_handle, buf_l, BUFFER_SIZE_2, pdMS_TO_TICKS(100));

}

/* Helper functions end*/

esp_err_t as_set_zero_position(as5600_t *dev, uint16_t raw_value)
{
    return write_12b_reg(dev, ZPOS_H, ZPOS_L, raw_value);
}

esp_err_t as_get_zero_position(as5600_t *dev, uint16_t *out)
{
    return read_12b_reg(dev, ZPOS_H, ZPOS_L, out);
}

esp_err_t as_set_max_position(as5600_t *dev, uint16_t raw_value)
{
    return write_12b_reg(dev, MPOS_H, MPOS_L, raw_value);
}

esp_err_t as_get_max_position(as5600_t *dev, uint16_t *out)
{
    return read_12b_reg(dev, MPOS_H, MPOS_L, out);
}

esp_err_t as_set_max_angle(as5600_t *dev, uint16_t raw_value)
{
    return write_12b_reg(dev, MANG_H, MANG_L, raw_value);
}

esp_err_t as_get_max_angle(as5600_t *dev, uint16_t *out)
{
    return read_12b_reg(dev, MANG_H, MANG_L, out);
}

esp_err_t as_set_conf(as5600_t *dev, const as5600_conf_reg_t *conf)
{
    if (dev == NULL || conf == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t value = 0x0000;
    value |= (conf->power_mode & 0x03);
    value |= (conf->hysteresis & 0x03) << 2;
    value |= (conf->output_stage & 0x03) << 4;
    value |= (conf->pwm_freq & 0x03) << 6;
    value |= (conf->slow_filter & 0x03) << 8;
    value |= (conf->fast_filter_threshold & 0x07) << 10;
    value |= (conf->watchdog_en ? 1 : 0) << 13;

    return write_14b_reg(dev, CONF_H, CONF_L, value);
}

esp_err_t as_get_conf(as5600_t *dev, as5600_conf_reg_t *conf)
{
    if (dev == NULL || conf == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t value;
    esp_err_t ret = read_12b_reg(dev, CONF_H, CONF_L, &value);
    if (ret != ESP_OK) return ret;

    /*
     * CONF Register Bit Mapping Breakdown:
     * -------------------------------------------------------------------------
     * Bit 13      [WD]        Watchdog (0 = OFF, 1 = ON) -> e.g., (1 << 13) enables WD
     * Bits 12:10  [FTH]       Fast Filter Threshold (000 = slow filter only, ..., 111 = 10 LSBs)
     * Bits 9:8    [SF]        Slow Filter (00 = 16x, 01 = 8x, 10 = 4x, 11 = 2x)
     * Bits 7:6    [PWMF]      PWM Frequency (00 = 115 Hz, 01 = 230 Hz, 10 = 460 Hz, 11 = 920 Hz)
     * Bits 5:4    [OUTS]      Output Stage (00 = analog full, 01 = analog reduced, 10 = digital PWM)
     * Bits 3:2    [HYST]      Hysteresis (00 = OFF, 01 = 1 LSB, 10 = 2 LSBs, 11 = 3 LSBs)
     * Bits 1:0    [PM]        Power Mode (00 = NOM, 01 = LPM1, 10 = LPM2, 11 = LPM3)
     
     TODO:
     REFER README, will be added soon

     */

    conf->power_mode          = value & 0x03;
    conf->hysteresis          = (value >> 2) & 0x03;
    conf->output_stage        = (value >> 4) & 0x03;
    conf->pwm_freq       = (value >> 6) & 0x03;
    conf->slow_filter         = (value >> 8) & 0x03;
    conf->fast_filter_threshold  = (value >> 10) & 0x07;
    conf->watchdog_en    = (value >> 13) & 0x01;

    return ESP_OK;
}

esp_err_t as_zero_here (as5600_t *dev)
{
    if (dev == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t reg = STATUS;
    uint8_t val1, val2;  // val1 -> Lower   // val2 -> Higher
    esp_err_t ret = i2c_master_transmit_receive(dev->dev_handle, &reg, BUFFER_SIZE_1, &val1, BUFFER_SIZE_1, pdMS_TO_TICKS(100));

    if (ret != ESP_OK)
    {
        return ret;
    }
    
    if (!((val1 >> 5) & 0x01))
    {
        return AS5600_ERR_MAGNET_NOT_DETECTED;
    }

    reg = RAW_H;
    ret = i2c_master_transmit_receive(dev->dev_handle, &reg, BUFFER_SIZE_1, &val2, BUFFER_SIZE_1, pdMS_TO_TICKS(100));

    if (ret != ESP_OK)
    {
        return ret;
    }

    reg = RAW_L;
    ret = i2c_master_transmit_receive(dev->dev_handle, &reg, BUFFER_SIZE_1, &val1, BUFFER_SIZE_1, pdMS_TO_TICKS(100));

    if (ret != ESP_OK)
    {
        return ret;
    }

    dev->zero_offset = (val2 << 8) | val1;

    return ESP_OK;

}