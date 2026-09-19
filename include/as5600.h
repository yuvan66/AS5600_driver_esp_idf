#include <stdio.h>
#include <stdint.h>

#include <esp_err.h>
#include <freertos/FreeRTOS.h>

#include <driver/i2c_master.h>

#define BASE             0x36
#define STATUS           0x0B
#define AGC              0x01A

#define RAW_H            0x0C   // R    //12bit 11:8
#define RAW_L            0x0D   // R    //12bit 7:0

#define ANGLE_H          0x0E   // R    //12bit 11:8
#define ANGLE_L          0x0F   // R    //12bit 7:0

#define ZMCO             0x00   // R    

#define ZPOS_H           0x01   // R/W/P    //12bit 11:8
#define ZPOS_L           0x02   // R/W/P    //12bit 7:0

#define MPOS_H           0x03   // R/W/P    //12bit 11:8
#define MPOS_L           0x04   // R/W/P    //12bit 7:0

#define MANG_H           0x05   // R/W/P    //12bit 11:8
#define MANG_L           0x06   // R/W/P    //12bit 7:0

#define CONF_H           0x07   // R/W/P    //14bit 13:8
#define CONF_L           0x08   // R/W/P    //12bit 7:0

#define MAG_H            0x1B   // R    //12bit 11:8
#define MAG_L            0x1C   // R     //12bit 7:0

#define BURN             0xFF   // W

#define AS5600_ERR_BASE                 0x9000
#define AS5600_ERR_MAGNET_NOT_DETECTED  (AS5600_ERR_BASE + 1)
#define AS5600_ERR_MAGNET_TOO_WEAK      (AS5600_ERR_BASE + 2)
#define AS5600_ERR_MAGNET_TOO_STRONG    (AS5600_ERR_BASE + 3)

#define BUFFER_SIZE_1 1u
#define BUFFER_SIZE_2 2u

typedef struct {
    i2c_port_num_t i2c_port;
    gpio_num_t sda_num;
    gpio_num_t scl_num;
    uint32_t scl_hz;
}as5600_config_t;

typedef struct {
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    uint16_t zero_offset;
}as5600_t;

typedef struct {
    uint8_t power_mode;
    uint8_t hysteresis;
    uint8_t output_stage;
    uint8_t pwm_freq;
    uint8_t slow_filter;
    uint8_t fast_filter_threshold;
    bool watchdog_en;
}as5600_conf_reg_t;

esp_err_t as_init(as5600_config_t *config, as5600_t *dev);

esp_err_t as_get_angle_r(as5600_t *dev, float *angle);

esp_err_t as_get_angle(as5600_t *dev, float *angle);

esp_err_t as_get_mag(as5600_t *dev, uint16_t *mag);

esp_err_t as_set_zero_position(as5600_t *dev, uint16_t raw_value);

esp_err_t as_get_zero_position(as5600_t *dev, uint16_t *out);

esp_err_t as_set_max_position(as5600_t *dev, uint16_t raw_value);

esp_err_t as_get_max_position(as5600_t *dev, uint16_t *out);

esp_err_t as_set_max_angle(as5600_t *dev, uint16_t raw_value);

esp_err_t as_get_max_angle(as5600_t *dev, uint16_t *out);

esp_err_t as_set_conf(as5600_t *dev, const as5600_conf_reg_t *conf);

esp_err_t as_get_conf(as5600_t *dev, as5600_conf_reg_t *conf);

esp_err_t as_zero_here(as5600_t *dev);

/* Helper functions */
esp_err_t write_12b_reg (as5600_t *dev, uint8_t reg_h, uint8_t reg_l, uint16_t value);
esp_err_t write_14b_reg (as5600_t *dev, uint8_t reg_h, uint8_t reg_l, uint16_t value);

esp_err_t read_12b_reg (as5600_t *dev, uint8_t reg_h, uint8_t reg_l, uint16_t *value);
// esp_err_t read_14b_reg (as5600_t *dev, uint8_t reg_h, uint8_t reg_l, uint16_t *value);


//butter worth software filter
//position
//adc - for rate of change for rpm (as rpm and position is the ultimate goal)