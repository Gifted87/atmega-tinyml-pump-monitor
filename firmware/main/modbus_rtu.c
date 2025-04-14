#include "modbus_rtu.h"
#include <avr/io.h>
#include <util/delay.h> // For inter-frame delay timing
#include <string.h> // For memcpy

// --- UART Defines (assuming UART0) ---
// Adjust if using a different UART peripheral
#define BAUD MODBUS_SERIAL_BAUD
#include <util/setbaud.h> // Calculates UBRR value

// Modbus timing constants (T1.5 and T3.5 character times)
// Calculated based on baud rate (e.g., 11 bits per char: 1 start, 8 data, 1 parity/none, 1 stop)
// Example for 19200 baud: Char time = 11 bits / 19200 bps = ~0.573 ms
// T1.5 = 1.5 * 0.573ms = ~0.86ms
// T3.5 = 3.5 * 0.573ms = ~2.0ms
// Use delays or timer for accurate inter-frame gaps.
#define MODBUS_T1_5_DELAY_US 860 // Example
#define MODBUS_T3_5_DELAY_US 2000 // Example

// Modbus buffer
#define MODBUS_BUFFER_SIZE 64 // Max typical Modbus RTU frame size is ~256, but limited by function/data
static uint8_t modbus_rx_buffer[MODBUS_BUFFER_SIZE];
static uint8_t modbus_rx_count = 0;
// TODO: Add timer-based detection of T3.5 gap for frame end detection.
// Simple implementation below uses polling with timeout, less robust.

// Modbus Holding Registers storage (example)
static uint16_t holding_registers[MODBUS_HOLDING_REG_COUNT];

// --- UART Functions (Basic Polling Implementation) ---
void modbus_uart_init(void) {
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;
#if USE_2X
    UCSR0A |= (1 << U2X0);
#else
    UCSR0A &= ~(1 << U2X0);
#endif
    // Enable receiver and transmitter
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    // Set frame format: 8 data, no parity, 1 stop bit (8N1)
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
    // No parity: (0 << UPM01) | (0 << UPM00)
    // 1 stop bit: (0 << USBS0)
}

static void modbus_uart_putchar(uint8_t data) {
    // Wait for empty transmit buffer
    while (!(UCSR0A & (1 << UDRE0)));
    // Put data into buffer, sends the data
    UDR0 = data;
}

// Non-blocking getchar with simple timeout mechanism (needs improvement with timer)
static bool modbus_uart_getchar_timeout(uint8_t* data, uint16_t timeout_us) {
    uint16_t timer = 0;
    while (!(UCSR0A & (1 << RXC0))) { // Wait for data to be received
        _delay_us(1); // Very crude delay
        timer++;
        if (timer >= timeout_us) {
            return false; // Timeout
        }
    }
    // Get and return received data from buffer
    *data = UDR0;
    return true;
}


// --- CRC16 Calculation ---
static uint16_t calculate_crc16(const uint8_t* buffer, uint8_t length) {
    uint16_t crc = 0xFFFF;
    for (uint8_t pos = 0; pos < length; pos++) {
        crc ^= (uint16_t)buffer[pos]; // XOR byte into least sig. byte of crc
        for (uint8_t i = 8; i != 0; i--) { // Loop over each bit
            if ((crc & 0x0001) != 0) { // If the LSB is set
                crc >>= 1; // Shift right and XOR 0xA001
                crc ^= 0xA001;
            } else { // Else LSB is not set
                crc >>= 1; // Just shift right
            }
        }
    }
    // Note: Modbus RTU CRC is LSB first, MSB second. Swap bytes if needed by hardware/master.
    // Standard CRC16 result might need byte swapping. Let's assume direct use first.
    return crc;
}

// --- Modbus Register Access ---
void modbus_update_register(uint16_t address, uint16_t value) {
    // Basic bounds check
    if (address < MODBUS_HOLDING_REG_COUNT) {
        holding_registers[address] = value;
    }
}

uint16_t modbus_read_register(uint16_t address) {
    if (address < MODBUS_HOLDING_REG_COUNT) {
        return holding_registers[address];
    }
    return 0; // Return 0 for invalid address
}


// --- Modbus Request Handling ---
static void send_modbus_response(uint8_t* response, uint8_t length) {
    uint16_t crc = calculate_crc16(response, length);
    for (uint8_t i = 0; i < length; i++) {
        modbus_uart_putchar(response[i]);
    }
    modbus_uart_putchar(crc & 0xFF); // Send CRC Low Byte
    modbus_uart_putchar(crc >> 8);   // Send CRC High Byte
}

static void send_modbus_exception(uint8_t function_code, uint8_t exception_code) {
    uint8_t response[5];
    response[0] = MODBUS_DEVICE_ADDRESS;
    response[1] = function_code | 0x80; // Set high bit for exception
    response[2] = exception_code;
    send_modbus_response(response, 3);
}

// Function to handle Read Holding Registers (0x03)
static void handle_read_holding_registers(void) {
    uint16_t start_address = ((uint16_t)modbus_rx_buffer[2] << 8) | modbus_rx_buffer[3];
    uint16_t num_registers = ((uint16_t)modbus_rx_buffer[4] << 8) | modbus_rx_buffer[5];
    uint8_t byte_count = num_registers * 2;

    // Validate request
    if (num_registers == 0 || num_registers > 125) { // Max registers per request limit
        send_modbus_exception(0x03, 0x03); // Illegal Data Value
        return;
    }
    if ((start_address + num_registers) > MODBUS_HOLDING_REG_COUNT) {
        send_modbus_exception(0x03, 0x02); // Illegal Data Address
        return;
    }

    // Prepare response
    uint8_t response[MODBUS_BUFFER_SIZE]; // Ensure buffer is large enough
    response[0] = MODBUS_DEVICE_ADDRESS;
    response[1] = 0x03;
    response[2] = byte_count;
    uint8_t resp_idx = 3;

    for (uint16_t i = 0; i < num_registers; ++i) {
        uint16_t reg_value = holding_registers[start_address + i];
        response[resp_idx++] = (reg_value >> 8) & 0xFF; // High byte first
        response[resp_idx++] = reg_value & 0xFF;        // Low byte second
    }

    send_modbus_response(response, resp_idx);
}

// Function to handle Write Single Register (0x06)
static void handle_write_single_register(void) {
    uint16_t reg_address = ((uint16_t)modbus_rx_buffer[2] << 8) | modbus_rx_buffer[3];
    uint16_t reg_value = ((uint16_t)modbus_rx_buffer[4] << 8) | modbus_rx_buffer[5];

    // Validate address (only allow writing to specific registers if needed)
    if (reg_address != MODBUS_REG_ALERT_THR) { // Example: Only allow writing threshold
         if (reg_address >= MODBUS_HOLDING_REG_COUNT) {
            send_modbus_exception(0x06, 0x02); // Illegal Data Address
            return;
        }
        // Optionally allow writing to other regs, or send exception 0x02 if read-only
        // For now, assume only THR is writable, others cause exception
        // send_modbus_exception(0x06, 0x02); // Or maybe 0x04 Slave Device Failure?
        // return;
    }
    // If address is valid but value is bad (optional check)
    // if (reg_value > MAX_THRESHOLD) {
    //     send_modbus_exception(0x06, 0x03); // Illegal Data Value
    //     return;
    // }

    // Write the value
    holding_registers[reg_address] = reg_value;

    // Send echo response (standard for Write Single Register)
    send_modbus_response(modbus_rx_buffer, 6); // Echo request back (excluding CRC)
}


// Main function to check for and process requests
void modbus_check_request(void) {
    // --- Receive Frame ---
    // This part needs robust T3.5 gap detection using a timer.
    // Simple polling implementation:
    uint8_t byte;
    // Wait for first byte with a longer timeout maybe? Or just process when available.
    if (modbus_uart_getchar_timeout(&byte, MODBUS_T3_5_DELAY_US * 2)) { // Wait briefly for start
        modbus_rx_buffer[0] = byte;
        modbus_rx_count = 1;
        // Receive subsequent bytes with T1.5 timeout between them
        while (modbus_rx_count < MODBUS_BUFFER_SIZE) {
            if (modbus_uart_getchar_timeout(&byte, MODBUS_T1_5_DELAY_US)) {
                modbus_rx_buffer[modbus_rx_count++] = byte;
            } else {
                // T1.5 timeout occurred - end of frame?
                break;
            }
        }

        // --- Process Frame ---
        if (modbus_rx_count < 4) { // Minimum frame size: Addr, Func, CRC(2)
            modbus_rx_count = 0; // Invalid frame
            return;
        }

        // Check Address
        if (modbus_rx_buffer[0] != MODBUS_DEVICE_ADDRESS && modbus_rx_buffer[0] != 0) { // Allow broadcast addr 0? Usually not for responses.
            modbus_rx_count = 0; // Not for us
            return;
        }

        // Check CRC
        uint16_t received_crc = ((uint16_t)modbus_rx_buffer[modbus_rx_count - 1] << 8) | modbus_rx_buffer[modbus_rx_count - 2];
        uint16_t calculated_crc = calculate_crc16(modbus_rx_buffer, modbus_rx_count - 2);

        if (received_crc != calculated_crc) {
            modbus_rx_count = 0; // CRC error
            // Modbus spec says slaves should not respond to CRC errors.
            return;
        }

        // --- Valid Request Received ---
        uint8_t function_code = modbus_rx_buffer[1];

        switch (function_code) {
            case 0x03: // Read Holding Registers
                if (modbus_rx_count == 8) { // Expected length: Addr(1)+Func(1)+StartAddr(2)+NumRegs(2)+CRC(2)
                    handle_read_holding_registers();
                } else {
                    send_modbus_exception(function_code, 0x03); // Illegal Data Value (bad length)
                }
                break;

            case 0x06: // Write Single Register
                if (modbus_rx_count == 8) { // Expected length: Addr(1)+Func(1)+RegAddr(2)+Value(2)+CRC(2)
                    handle_write_single_register();
                } else {
                    send_modbus_exception(function_code, 0x03); // Illegal Data Value (bad length)
                }
                break;

            // Add other function codes if needed (e.g., 0x10 Write Multiple Registers)

            default:
                send_modbus_exception(function_code, 0x01); // Illegal Function
                break;
        }

        modbus_rx_count = 0; // Ready for next frame

    } // end if(getchar_timeout)

    // TODO: Add proper timer management for T3.5 frame silence detection.
    // The polling method above is sensitive to timing variations.
}