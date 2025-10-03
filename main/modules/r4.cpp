#include "r4.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "esp_err.h"

extern "C" {
    #include <string.h>
    #include <stdio.h>
    #include <stdlib.h>
}

/* State variables */
bool sbc_data_received_flag = false;

uint8_t sbc_i2c1_receive_buffer[32];
uint8_t sbc_i2c1_send_buffer[32];

char sbc_uart0_receive_buffer[128];
char sbc_uart0_send_buffer[128];
size_t uart_received_data_length;

int _encoder_positions[4];
int _encoder_positions_previous[4];
int _motor_rpms[4];
int _motor_currents[4];
uint64_t _previous_timer_value;
uint8_t _motor_stall_status;
uint8_t _motor_disconnect_status;

uint8_t _motor_mode_register;
uint8_t _motor_direction_register;

uint8_t _raw_motor_speeds[4];
uint8_t _filtered_motor_speeds[4];
motor_direction_t _motor_directions[4];
motor_safety_mode_t _motor_safety_mode;
motor_speed_mode_t _motor_speed_mode;
bool _motor_output_enabled;

bot_direction_t _bot_direction;
uint8_t _common_speed;

int _battery_voltage;

/* Local handles */
i2c_master_dev_handle_t _PCA_I2C0_device_handle;
adc_oneshot_unit_handle_t _adc1_unit_handle;
adc_cali_handle_t _adc1_cali_handle;
adc_oneshot_unit_handle_t _adc2_unit_handle;
adc_cali_handle_t _adc2_cali_handle;

/* R4 init: PCA + UART + SBC callbacks */
esp_err_t r4_init(i2c_master_bus_handle_t bus_handle, uart_config_t uart_config, i2c_slave_dev_handle_t sbc_slave_handle) {
    esp_err_t status;

    status = PCA_init(bus_handle);
    if (status != ESP_OK) return status;

    status = sbc_uart_init(uart_config);
    if (status != ESP_OK) return status;

    status = sbc_i2c1_register_callback(sbc_slave_handle);
    if (status != ESP_OK) return status;

    gpio_set_level(PCA_MOTOR_ENABLE_PIN, 0);

    return ESP_OK;
}
