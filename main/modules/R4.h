#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "driver/gpio.h"

/* Motor direction/type enums */
typedef enum { IDLE=0, REVERSE=1, BRAKE=2, FORWARD=3 } motor_direction_t;
typedef enum { STOP=0, MAX=1, COMMAND=2, COMMON=3 } motor_speed_mode_t;
typedef enum { UNSAFE=0, PROTECT_DISCONNECT=1 } motor_safety_mode_t;
typedef enum { FRONT=0, BACK=1, ROTATE_CLOCKWISE=2, ROTATE_COUNTERCLOCKWISE=3,
               FRONT_LEFT=4, FRONT_RIGHT=5, BACK_LEFT=6, BACK_RIGHT=7 } bot_direction_t;

/* Motor structure */
typedef struct {
    uint8_t output_pin_1;
    uint8_t output_pin_2;
    uint8_t LED_pin;
} motor_t;

/* API functions */
esp_err_t PCA_init(i2c_master_bus_handle_t bus_handle);
esp_err_t PCA_command_motor_with_LED(motor_t motor, motor_direction_t direction, uint8_t PWM);
esp_err_t set_motor_mode_register(motor_safety_mode_t safety_mode, motor_speed_mode_t speed_mode, bool enable);
esp_err_t set_bot_direction(bot_direction_t direction);
esp_err_t publish_motor_command(motor_t* motors);
esp_err_t sbc_uart_init(uart_config_t uart_config);
esp_err_t sbc_i2c1_register_callback(i2c_slave_dev_handle_t device_handle);

/* Externs for pseudo-registers */
extern uint8_t _raw_motor_speeds[];
extern uint8_t _filtered_motor_speeds[];
extern motor_direction_t _motor_directions[];
extern motor_safety_mode_t _motor_safety_mode;
extern motor_speed_mode_t _motor_speed_mode;
extern bool _motor_output_enabled;
extern bot_direction_t _bot_direction;
extern uint8_t _common_speed;
extern int _encoder_positions[];
extern int _motor_rpms[];
extern int _motor_currents[];
extern uint8_t _motor_stall_status;
extern uint8_t _motor_disconnect_status;
extern int _battery_voltage;
