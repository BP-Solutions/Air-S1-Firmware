/***************************************************************************************************************************

    SenseAir S8 Library for Serial Modbus Communication

    Copyright (c) 2021 Josep Comas

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.

***************************************************************************************************************************/

#ifndef _S8_UART_H
#define _S8_UART_H

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include <cstdint>
#include <cstring>

#include "modbus_crc.h"
#include "utils.h"

#define S8_BAUDRATE 9600         // Device to S8 Serial baudrate (should not be changed)
#define S8_TIMEOUT  5000ul       // Timeout for communication in milliseconds
#define S8_LEN_BUF_MSG  20       // Max length of buffer for communication with the sensor
#define S8_LEN_FIRMVER  10       // Length of software version

// Modbus
#define MODBUS_ANY_ADDRESS                  0XFE    // S8 uses any address
#define MODBUS_FUNC_READ_HOLDING_REGISTERS  0X03    // Read holding registers (HR)
#define MODBUS_FUNC_READ_INPUT_REGISTERS    0x04    // Read input registers (IR)
#define MODBUS_FUNC_WRITE_SINGLE_REGISTER   0x06    // Write single register (SR)

// Input registers for S8
#define MODBUS_IR1             0x0000  // MeterStatus
#define MODBUS_IR2             0x0001  // AlarmStatus
#define MODBUS_IR3             0x0002  // OutputStatus
#define MODBUS_IR4             0x0003  // Space CO2
#define MODBUS_IR22            0x0015  // PWM Output
#define MODBUS_IR26            0x0019  // Sensor Type ID High
#define MODBUS_IR27            0x001A  // Sensor Type ID Low
#define MODBUS_IR28            0x001B  // Memory Map version
#define MODBUS_IR29            0x001C  // FW version Main.Sub
#define MODBUS_IR30            0x001D  // Sensor ID High
#define MODBUS_IR31            0x001E  // Sensor ID Low

// Holding registers for S8
#define MODBUS_HR1             0x0000  // Acknowledgement Register
#define MODBUS_HR2             0x0001  // Special Command Register
#define MODBUS_HR32            0x001F  // ABC Period

// Meter status
#define S8_MASK_METER_FATAL_ERROR                    0x0001   // Fatal error
#define S8_MASK_METER_OFFSET_REGULATION_ERROR        0x0002   // Offset regulation error
#define S8_MASK_METER_ALGORITHM_ERROR                0x0004   // Algorithm error
#define S8_MASK_METER_OUTPUT_ERROR                   0x0008   // Output error
#define S8_MASK_METER_SELF_DIAG_ERROR                0x0010   // Self diagnostics error
#define S8_MASK_METER_OUT_OF_RANGE                   0x0020   // Out of range
#define S8_MASK_METER_MEMORY_ERROR                   0x0040   // Memory error
#define S8_MASK_METER_ANY_ERROR                      0x007F   // Mask to detect the previous errors (fatal error ... memory error)

// Output status
#define S8_MASK_OUTPUT_ALARM                         0x0001   // Alarm output status (inverted due to Open Collector)
#define S8_MASK_OUTPUT_PWM                           0x0002   // PWM output status (=1 -> full output)

// Acknowledgement flags
#define S8_MASK_CO2_BACKGROUND_CALIBRATION   0x0020   // CO2 Background calibration performed = 1
#define S8_MASK_CO2_NITROGEN_CALIBRATION     0x0040   // CO2 Nitrogen calibration performed = 1

// Calibration definitions for special command
#define S8_CO2_BACKGROUND_CALIBRATION        0x7C06   // CO2 Background calibration
#define S8_CO2_ZERO_CALIBRATION              0x7C07   // CO2 Zero calibration

struct S8_sensor {
    char firm_version[S8_LEN_FIRMVER + 1];
    int16_t co2;
    int16_t abc_period;
    int16_t ack;
    int16_t meter_status;
    int16_t alarm_status;
    int16_t output_status;
    int16_t pwm_output;
    int32_t sensor_type_id;
    int32_t sensor_id;
    int16_t map_version;
};

class S8_UART {
public:
    S8_UART(uart_inst_t* uart, uint tx_pin, uint rx_pin);
    void get_firmware_version(char firmwver[]);
    int32_t get_sensor_type_ID();
    int32_t get_sensor_ID();
    int16_t get_memory_map_version();
    int16_t get_co2();
    int16_t get_PWM_output();
    int16_t get_ABC_period();
    bool set_ABC_period(int16_t period);
    bool manual_calibration_background();
    bool manual_calibration_zero();
    int16_t get_acknowledgement();
    bool clear_acknowledgement();
    int16_t get_meter_status();
    int16_t get_alarm_status();
    int16_t get_output_status();
    bool send_special_command(int16_t command);

private:
    uart_inst_t* uart;
    uint tx_pin;
    uint rx_pin;
    uint8_t buf_msg[S8_LEN_BUF_MSG];

    void serial_write_bytes(uint8_t size);
    uint8_t serial_read_bytes(uint8_t max_bytes, uint32_t timeout_ms);
    bool valid_response(uint8_t func, uint8_t nb);
    bool valid_response_len(uint8_t func, uint8_t nb, uint8_t len);
    void send_cmd(uint8_t func, uint16_t reg, uint16_t value);
};

S8_UART::S8_UART(uart_inst_t* uart, uint tx_pin, uint rx_pin)
        : uart(uart), tx_pin(tx_pin), rx_pin(rx_pin) {
    uart_init(uart, S8_BAUDRATE);
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
}

void S8_UART::get_firmware_version(char firmwver[]) {
    send_cmd(MODBUS_FUNC_READ_INPUT_REGISTERS, MODBUS_IR29, 0x0001);
    if (serial_read_bytes(7, S8_TIMEOUT) == 7 && valid_response(MODBUS_FUNC_READ_INPUT_REGISTERS, 0x02)) {
        strncpy(firmwver, (char*)&buf_msg[3], S8_LEN_FIRMVER);
    }
}

int32_t S8_UART::get_sensor_type_ID() {
    send_cmd(MODBUS_FUNC_READ_INPUT_REGISTERS, MODBUS_IR26, 0x0002);
    if (serial_read_bytes(9, S8_TIMEOUT) == 9 && valid_response(MODBUS_FUNC_READ_INPUT_REGISTERS, 0x04)) {
        return (buf_msg[3] << 24) | (buf_msg[4] << 16) | (buf_msg[5] << 8) | buf_msg[6];
    }
    return -1;
}

int32_t S8_UART::get_sensor_ID() {
    send_cmd(MODBUS_FUNC_READ_INPUT_REGISTERS, MODBUS_IR30, 0x0002);
    if (serial_read_bytes(9, S8_TIMEOUT) == 9 && valid_response(MODBUS_FUNC_READ_INPUT_REGISTERS, 0x04)) {
        return (buf_msg[3] << 24) | (buf_msg[4] << 16) | (buf_msg[5] << 8) | buf_msg[6];
    }
    return -1;
}

int16_t S8_UART::get_memory_map_version() {
    send_cmd(MODBUS_FUNC_READ_INPUT_REGISTERS, MODBUS_IR28, 0x0001);
    if (serial_read_bytes(7, S8_TIMEOUT) == 7 && valid_response(MODBUS_FUNC_READ_INPUT_REGISTERS, 0x02)) {
        return (buf_msg[3] << 8) | buf_msg[4];
    }
    return -1;
}

int16_t S8_UART::get_co2() {
    send_cmd(MODBUS_FUNC_READ_INPUT_REGISTERS, MODBUS_IR4, 0x0001);
    if (serial_read_bytes(7, S8_TIMEOUT) == 7 && valid_response(MODBUS_FUNC_READ_INPUT_REGISTERS, 0x02)) {
        return (buf_msg[3] << 8) | buf_msg[4];
    }
    return -1;
}

int16_t S8_UART::get_PWM_output() {
    send_cmd(MODBUS_FUNC_READ_INPUT_REGISTERS, MODBUS_IR22, 0x0001);
    if (serial_read_bytes(7, S8_TIMEOUT) == 7 && valid_response(MODBUS_FUNC_READ_INPUT_REGISTERS, 0x02)) {
        return (buf_msg[3] << 8) | buf_msg[4];
    }
    return -1;
}

int16_t S8_UART::get_ABC_period() {
    send_cmd(MODBUS_FUNC_READ_HOLDING_REGISTERS, MODBUS_HR32, 0x0001);
    if (serial_read_bytes(7, S8_TIMEOUT) == 7 && valid_response(MODBUS_FUNC_READ_HOLDING_REGISTERS, 0x02)) {
        return (buf_msg[3] << 8) | buf_msg[4];
    }
    return -1;
}

bool S8_UART::set_ABC_period(int16_t period) {
    send_cmd(MODBUS_FUNC_WRITE_SINGLE_REGISTER, MODBUS_HR32, period);
    return (serial_read_bytes(8, S8_TIMEOUT) == 8 && valid_response(MODBUS_FUNC_WRITE_SINGLE_REGISTER, 0x00));
}

bool S8_UART::manual_calibration_background() {
    return send_special_command(S8_CO2_BACKGROUND_CALIBRATION);
}

bool S8_UART::manual_calibration_zero() {
    return send_special_command(S8_CO2_ZERO_CALIBRATION);
}

int16_t S8_UART::get_acknowledgement() {
    send_cmd(MODBUS_FUNC_READ_HOLDING_REGISTERS, MODBUS_HR1, 0x0001);
    if (serial_read_bytes(7, S8_TIMEOUT) == 7 && valid_response(MODBUS_FUNC_READ_HOLDING_REGISTERS, 0x02)) {
        return (buf_msg[3] << 8) | buf_msg[4];
    }
    return -1;
}

bool S8_UART::clear_acknowledgement() {
    send_cmd(MODBUS_FUNC_WRITE_SINGLE_REGISTER, MODBUS_HR1, 0x0000);
    return (serial_read_bytes(8, S8_TIMEOUT) == 8 && valid_response(MODBUS_FUNC_WRITE_SINGLE_REGISTER, 0x00));
}

int16_t S8_UART::get_meter_status() {
    send_cmd(MODBUS_FUNC_READ_INPUT_REGISTERS, MODBUS_IR1, 0x0001);
    if (serial_read_bytes(7, S8_TIMEOUT) == 7 && valid_response(MODBUS_FUNC_READ_INPUT_REGISTERS, 0x02)) {
        return (buf_msg[3] << 8) | buf_msg[4];
    }
    return -1;
}

int16_t S8_UART::get_alarm_status() {
    send_cmd(MODBUS_FUNC_READ_INPUT_REGISTERS, MODBUS_IR2, 0x0001);
    if (serial_read_bytes(7, S8_TIMEOUT) == 7 && valid_response(MODBUS_FUNC_READ_INPUT_REGISTERS, 0x02)) {
        return (buf_msg[3] << 8) | buf_msg[4];
    }
    return -1;
}

int16_t S8_UART::get_output_status() {
    send_cmd(MODBUS_FUNC_READ_INPUT_REGISTERS, MODBUS_IR3, 0x0001);
    if (serial_read_bytes(7, S8_TIMEOUT) == 7 && valid_response(MODBUS_FUNC_READ_INPUT_REGISTERS, 0x02)) {
        return (buf_msg[3] << 8) | buf_msg[4];
    }
    return -1;
}

bool S8_UART::send_special_command(int16_t command) {
    send_cmd(MODBUS_FUNC_WRITE_SINGLE_REGISTER, MODBUS_HR2, command);
    return (serial_read_bytes(8, S8_TIMEOUT) == 8 && valid_response(MODBUS_FUNC_WRITE_SINGLE_REGISTER, 0x00));
}

void S8_UART::serial_write_bytes(uint8_t size) {
    uart_write_blocking(uart, buf_msg, size);
}

uint8_t S8_UART::serial_read_bytes(uint8_t max_bytes, uint32_t timeout_ms) {
    uint8_t count = 0;
    absolute_time_t timeout_time = make_timeout_time_ms(timeout_ms);
    while (count < max_bytes && !time_reached(timeout_time)) {
        if (uart_is_readable(uart)) {
            buf_msg[count++] = uart_getc(uart);
        }
    }
    return count;
}

void S8_UART::send_cmd(uint8_t func, uint16_t reg, uint16_t value) {
    buf_msg[0] = MODBUS_ANY_ADDRESS;
    buf_msg[1] = func;
    buf_msg[2] = (reg >> 8) & 0xFF;
    buf_msg[3] = reg & 0xFF;
    buf_msg[4] = (value >> 8) & 0xFF;
    buf_msg[5] = value & 0xFF;
    uint16_t crc = modbus_CRC16(buf_msg, 6);
    buf_msg[6] = crc & 0xFF;
    buf_msg[7] = (crc >> 8) & 0xFF;
    serial_write_bytes(8);
}

bool S8_UART::valid_response(uint8_t func, uint8_t nb) {
    return (buf_msg[0] == MODBUS_ANY_ADDRESS && buf_msg[1] == func && buf_msg[2] == nb);
}

bool S8_UART::valid_response_len(uint8_t func, uint8_t nb, uint8_t len) {
    return valid_response(func, nb) && (len == nb + 5);
}

#endif
