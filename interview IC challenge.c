#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <math.h>

#define TAG "MAX31865"

// SPI Config
#define MAX31865_HOST    SPI2_HOST
#define PIN_NUM_MISO     19
#define PIN_NUM_MOSI     23
#define PIN_NUM_CLK      18
#define PIN_NUM_CS       5

// MAX31865 Registers
#define REG_CONFIG       0x00
#define REG_RTD_MSB      0x01
#define REG_RTD_LSB      0x02

// MAX31865 Config Bits
#define CONFIG_BIAS              0x80
#define CONFIG_MODEAUTO          0x40
#define CONFIG_1SHOT             0x20
#define CONFIG_3WIRE             0x10
#define CONFIG_FAULT_DETECT1     0x08
#define CONFIG_FAULT_DETECT0     0x04
#define CONFIG_CLEAR_FAULT       0x02
#define CONFIG_50HZ              0x01

// Reference resistor (e.g. 430.0 Ω for Pt100)
#define R_REF       430.0

// Callendar–Van Dusen constants for Pt100
#define RTD_A       3.9083e-3
#define RTD_B      -5.775e-7

spi_device_handle_t max31865;

void max31865_write_register(uint8_t reg, uint8_t value) {
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = (uint8_t[]){reg | 0x80, value},
    };
    spi_device_transmit(max31865, &t);
}

uint16_t max31865_read_rtd() {
    uint8_t read_buf[2];
    spi_transaction_t t[2] = {
        {
            .length = 8,
            .tx_buffer = (uint8_t[]){REG_RTD_MSB & 0x7F}, // Read MSB
        },
        {
            .length = 16,
            .rxlength = 16,
            .rx_buffer = read_buf,
        }
    };
    spi_device_transmit(max31865, &t[0]);
    spi_device_transmit(max31865, &t[1]);

    uint16_t rtd = ((read_buf[0] << 8) | read_buf[1]) >> 1; // 15-bit value
    return rtd;
}

float max31865_temperature(uint16_t rtd_raw) {
    float Rt = rtd_raw * R_REF / 32768.0;
    float temp = (-RTD_A + sqrt(RTD_A * RTD_A - 4 * RTD_B * (1 - Rt / 100.0))) / (2 * RTD_B);
    return temp;
}

void max31865_init() {
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,
        .mode = 1,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 1,
    };

    spi_bus_initialize(MAX31865_HOST, &buscfg, SPI_DMA_DISABLED);
    spi_bus_add_device(MAX31865_HOST, &devcfg, &max31865);

    max31865_write_register(REG_CONFIG, CONFIG_BIAS | CONFIG_MODEAUTO | CONFIG_50HZ);
    vTaskDelay(pdMS_TO_TICKS(10));  // Allow bias to stabilize
}

void app_main(void) {
    max31865_init();

    while (1) {
        uint16_t rtd = max31865_read_rtd();
        float temp = max31865_temperature(rtd);
        ESP_LOGI(TAG, "RTD Raw: %u, Temperature: %.2f °C", rtd, temp);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
