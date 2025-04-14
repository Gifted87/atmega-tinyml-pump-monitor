#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include <stdint.h>
#include <stdbool.h>

// --- Configuration ---
#define MODBUS_SERIAL_BAUD 19200
#define MODBUS_DEVICE_ADDRESS 0x01 // This device's slave address

// Define Modbus register addresses (example)
#define MODBUS_REG_STATUS        0x0000 // Read-only: Device status (e.g., 0=OK, 1=Fault)
#define MODBUS_REG_LAST_PREDICT  0x0001 // Read-only: Last prediction result (e.g., 0=Normal, 1=BearingFail)
#define MODBUS_REG_FAULT_COUNT   0x0002 // Read-only: Counter for detected faults
#define MODBUS_REG_VIBRATION_RMS 0x0003 // Read-only: RMS vibration level (example)
#define MODBUS_REG_ALERT_THR     0x0100 // Read-Write: Alert threshold (example, maybe not needed if using ML)

#define MODBUS_HOLDING_REG_COUNT 4 // Number of accessible holding registers for this example

// --- Functions ---

/**
 * @brief Initializes the UART peripheral for Modbus RTU communication.
 */
void modbus_uart_init(void);

/**
 * @brief Checks for incoming Modbus RTU requests and processes them.
 * Should be called periodically in the main loop.
 */
void modbus_check_request(void);

/**
 * @brief Updates the value of a Modbus holding register.
 * Application code calls this to make data available to the Modbus master.
 *
 * @param address The 16-bit register address.
 * @param value The 16-bit value to write.
 */
void modbus_update_register(uint16_t address, uint16_t value);

/**
 * @brief Reads the value of a Modbus holding register.
 *
 * @param address The 16-bit register address.
 * @return The 16-bit value of the register.
 */
uint16_t modbus_read_register(uint16_t address);


#endif // MODBUS_RTU_H