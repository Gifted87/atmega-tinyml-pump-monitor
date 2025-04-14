#include "gsm_alert.h"
#include <avr/io.h>
#include <util/delay.h>
#include <string.h> // For strlen, strstr
#include <stdio.h>  // For sprintf

// --- UART/SoftwareSerial Abstraction ---
// IMPORTANT: ATmega328P has one hardware UART (USART0). If Modbus uses it,
// GSM needs SoftwareSerial or bit-banging. This implementation ASSUMES
// functions `gsm_uart_init`, `gsm_uart_putchar`, `gsm_uart_getchar_timeout`,
// `gsm_uart_available`, `gsm_uart_read_string` exist.
// You would need to implement these using a SoftwareSerial library or custom bit-bang code.

// Placeholder function prototypes (replace with actual implementation)
static void gsm_uart_init(uint32_t baud);
static void gsm_uart_putchar(char c);
static bool gsm_uart_getchar_timeout(char* data, uint16_t timeout_ms); // Gets one char
static uint16_t gsm_uart_available(void); // Returns number of chars available
static uint16_t gsm_uart_read_string(char* buffer, uint16_t max_len, uint16_t timeout_ms); // Reads until timeout or buffer full

#define GSM_UART_BAUD 9600 // Common default for SIM800L
#define GSM_RESPONSE_BUFFER_SIZE 128
#define GSM_CMD_TIMEOUT_MS 1000
#define GSM_SMS_TIMEOUT_MS 10000 // Sending SMS can take longer

static char gsm_response_buffer[GSM_RESPONSE_BUFFER_SIZE];

// --- Low-level UART Communication (PLACEHOLDER - Requires Implementation) ---

static void gsm_uart_init(uint32_t baud) {
    // Configure SoftwareSerial pins or second hardware UART if available
    // Set baud rate, frame format (usually 8N1)
    // --- Placeholder ---
    (void)baud; // Suppress unused parameter warning
}

static void gsm_uart_putchar(char c) {
    // Send character via SoftwareSerial or UART1
    // --- Placeholder ---
    (void)c; // Suppress unused parameter warning
     // Example using UART0 for testing if Modbus isn't active:
     // while (!(UCSR0A & (1 << UDRE0))); UDR0 = c;
}

static void gsm_uart_print(const char* str) {
    while (*str) {
        gsm_uart_putchar(*str++);
    }
}

static void gsm_uart_println(const char* str) {
    gsm_uart_print(str);
    gsm_uart_putchar('\r');
    gsm_uart_putchar('\n');
}

static bool gsm_uart_getchar_timeout(char* data, uint16_t timeout_ms) {
    // Receive character with timeout
    // --- Placeholder ---
    (void)data; (void)timeout_ms;
    // Example using UART0 for testing:
    // uint16_t timer = 0;
    // while (!(UCSR0A & (1 << RXC0))) {
    //     _delay_ms(1); timer++;
    //     if(timer > timeout_ms) return false;
    // }
    // *data = UDR0; return true;
    return false; // Placeholder returns false
}

static uint16_t gsm_uart_available() {
    // Return number of bytes in SoftwareSerial/UART buffer
    // --- Placeholder ---
    return 0;
}

static uint16_t gsm_uart_read_string(char* buffer, uint16_t max_len, uint16_t timeout_ms) {
    uint16_t count = 0;
    uint16_t time_start = 0; // Needs millis() or timer for proper timeout
    char c;

    while(count < max_len - 1) { // Leave space for null terminator
         // Need a millis() equivalent for timeout checking
         // if (millis() - time_start > timeout_ms) break;

         if (gsm_uart_getchar_timeout(&c, 50)) { // Read char with short timeout
             buffer[count++] = c;
             time_start = 0; // Reset timeout timer (needs millis())
         } else {
             // Check overall timeout
             // Placeholder: just return after first short timeout if no char
             if(count > 0) break; // Return if we got something
             _delay_ms(10); // Prevent busy-looping
             // Need proper timeout check here
         }
    }
    buffer[count] = '\0'; // Null terminate
    return count;
}


// --- High-level GSM Functions ---

// Sends an AT command and waits for a specific response (e.g., "OK")
static bool send_at_command(const char* command, const char* expected_response, uint16_t timeout_ms) {
    // Clear buffer / flush input? (Read available chars)
    while(gsm_uart_available() > 0) gsm_uart_getchar_timeout(NULL, 10);

    gsm_uart_println(command);

    // Read response
    uint16_t len = gsm_uart_read_string(gsm_response_buffer, GSM_RESPONSE_BUFFER_SIZE, timeout_ms);

    if (len > 0) {
        // Simple check: does the response contain the expected string?
        return (strstr(gsm_response_buffer, expected_response) != NULL);
    }
    return false;
}

bool gsm_init(void) {
    gsm_uart_init(GSM_UART_BAUD);
    _delay_ms(1000); // Wait for module to boot

    // Basic checks
    if (!send_at_command("AT", "OK", GSM_CMD_TIMEOUT_MS)) return false;
    _delay_ms(100);
    if (!send_at_command("ATE0", "OK", GSM_CMD_TIMEOUT_MS)) return false; // Disable echo
    _delay_ms(100);
    if (!send_at_command("AT+CPIN?", "READY", GSM_CMD_TIMEOUT_MS)) return false; // Check SIM status
    _delay_ms(100);
    if (!send_at_command("AT+CMGF=1", "OK", GSM_CMD_TIMEOUT_MS)) return false; // Set SMS text mode
    _delay_ms(100);

    return true;
}

bool gsm_send_alert(uint8_t fault_code) {
    char sms_command[64];
    char sms_message[100];

    // 1. Prepare and send AT+CMGS command
    sprintf(sms_command, "AT+CMGS=\"%s\"", GSM_TARGET_PHONE_NUMBER);
    gsm_uart_println(sms_command);

    // Wait for '>' prompt
    // Need robust check here, reading response until '>' or timeout/error
    _delay_ms(100); // Simple delay, might not be robust
    // TODO: Read response until '>' is received.

    // 2. Send the SMS message content
    sprintf(sms_message, "PUMP ALERT: Fault code %d detected on device %d.", fault_code, MODBUS_DEVICE_ADDRESS);
    gsm_uart_print(sms_message);

    // 3. Send Ctrl+Z (ASCII 26) to send the message
    gsm_uart_putchar(26);

    // 4. Wait for response ("OK" or "ERROR", or "+CMGS: <mr>")
    // Need robust response checking here.
    // Simple check for "OK" within a timeout:
    uint16_t len = gsm_uart_read_string(gsm_response_buffer, GSM_RESPONSE_BUFFER_SIZE, GSM_SMS_TIMEOUT_MS);
    if (len > 0) {
        return (strstr(gsm_response_buffer, "OK") != NULL || strstr(gsm_response_buffer, "+CMGS:") != NULL);
    }

    return false; // Timeout or unexpected response
}