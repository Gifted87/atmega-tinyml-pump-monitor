#ifndef GSM_ALERT_H
#define GSM_ALERT_H

#include <stdint.h>
#include <stdbool.h>

// --- Configuration ---
#define GSM_TARGET_PHONE_NUMBER "+1234567890" // Replace with actual target number

// --- Functions ---

/**
 * @brief Initializes the UART/SoftwareSerial for GSM module communication.
 * Also sends basic AT commands to check module readiness.
 *
 * @return true if initialization seems successful, false otherwise.
 */
bool gsm_init(void);

/**
 * @brief Sends an SMS alert message via the GSM module.
 *
 * @param fault_code A code indicating the type of fault detected.
 * @return true if SMS sending was initiated successfully, false on error.
 */
bool gsm_send_alert(uint8_t fault_code);

#endif // GSM_ALERT_H