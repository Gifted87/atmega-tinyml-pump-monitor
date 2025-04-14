#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>

// Include project headers
#include "sensor_reader.h"
#include "fft_256.h"
#include "fixed_math.h"
#include "modbus_rtu.h"
#include "gsm_alert.h"
#include "edge-impulse/model.h" // EI model definitions
#include "edge-impulse-sdk/classifier/ei_classifier_types.h" // EI result types

// --- Defines ---
#define FAULT_DETECTION_THRESHOLD 0.7f // Example threshold for 'BearingFail' prediction
#define GSM_ALERT_DEBOUNCE_MS 60000UL // Send GSM alert at most once per minute

// --- Global Variables ---
static int8_t processing_buffer[ADC_BUFFER_SIZE]; // Buffer for processing samples
static q8_24_t fft_power_spectrum[FFT_SIZE / 2];  // Buffer for FFT results
static uint32_t last_gsm_alert_ms = 0; // Timestamp for alert debounce (requires millis() implementation)
static uint16_t fault_counter = 0;

// --- Milliseconds Timer (Basic Implementation using Timer0) ---
// NOTE: This is a simple millis implementation. It might interfere with other timer usage.
// Consider using Timer2 if Timer0/Timer1 are occupied heavily.
volatile uint32_t milliseconds_counter = 0;

ISR(TIMER0_OVF_vect) {
    // Timer0 overflows roughly every 1.024ms with 16MHz clock and /64 prescaler
    milliseconds_counter++; // Increment roughly every ms
    // Need calibration factor based on prescaler and F_CPU for accurate millis
}

void millis_init(void) {
    // Timer0 setup for approx 1ms overflow interrupts
    TCCR0B |= (1 << CS01) | (1 << CS00); // Prescaler 64
    TIMSK0 |= (1 << TOIE0); // Enable Timer0 Overflow Interrupt
    TCNT0 = 0;
}

uint32_t millis(void) {
    uint32_t ms;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        ms = milliseconds_counter;
    }
    // Add calibration if needed: ms = (uint32_t)((double)ms * 1.024);
    return ms;
}


// --- Main Application ---
int main(void) {
    // --- Initialization ---
    millis_init();         // Initialize millisecond timer
    sensor_adc_init();     // Initialize ADC for vibration sensor
    sensor_timer_init();   // Initialize Timer1 for ADC sampling trigger
    modbus_uart_init();    // Initialize UART for Modbus RTU
    //gsm_init();          // Initialize GSM module (and its UART/SoftwareSerial) - Call gsm_uart_init inside

    bool gsm_ok = gsm_init(); // Try to initialize GSM
    // Update Modbus status register based on GSM init result?

    // Initialize Modbus registers
    modbus_update_register(MODBUS_REG_STATUS, gsm_ok ? 0 : 0xFFFF); // Example: 0=OK, FFFF=GSM Fail
    modbus_update_register(MODBUS_REG_LAST_PREDICT, 0);
    modbus_update_register(MODBUS_REG_FAULT_COUNT, 0);
    modbus_update_register(MODBUS_REG_VIBRATION_RMS, 0); // Needs calculation
    modbus_update_register(MODBUS_REG_ALERT_THR, (uint16_t)(FAULT_DETECTION_THRESHOLD * 100)); // Example: Store threshold*100

    // Edge Impulse Classifier Init
    ei_classifier_t classifier_ctx; // Context object might be needed depending on SDK version
    ei_impulse_result_t result = { 0 }; // Structure to hold inference results
    // Check return code?
    ei_classifier_init(&classifier_ctx); // Initialize classifier context if required by SDK

    sei(); // Enable global interrupts (ADC, Timer0, UART?)

    // --- Main Loop ---
    while (1) {
        // 1. Check if new sensor data is available
        if (sensor_get_samples(processing_buffer)) {
            // New data acquired!

            // --- Calculate RMS (Example for Modbus Register) ---
            // Simple RMS calculation (can be computationally intensive)
            // int32_t sum_sq = 0;
            // for(uint16_t i=0; i<ADC_BUFFER_SIZE; ++i) {
            //     sum_sq += (int32_t)processing_buffer[i] * processing_buffer[i];
            // }
            // uint16_t rms = sqrt(sum_sq / ADC_BUFFER_SIZE); // Requires sqrtf from math.h
            // modbus_update_register(MODBUS_REG_VIBRATION_RMS, rms);

            // 2. Perform FFT
            // NOTE: fft_256 modifies processing_buffer if it's truly in-place.
            // If not, copy processing_buffer before calling FFT if original needed later.
            // The current fft_256 implementation copies to internal buffers first.
            fft_256_calculate_power_spectrum(processing_buffer, fft_power_spectrum);
            // Now fft_power_spectrum holds the PSD values.

            // 3. Run Edge Impulse Inference
            EI_IMPULSE_ERROR res;

            // --- Create signal_t wrapper for the raw data ---
            // The SDK expects data via a callback function associated with signal_t.
            // Need to ensure the callback function (e.g., extract_features_get_data in ei_classifier.c)
            // can access the 'processing_buffer'. Making processing_buffer global helps.
            signal_t signal;
            signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
            // The get_data callback needs to be correctly implemented in the EI generated code.
            signal.get_data = &extract_features_get_data; // Link the callback


            // --- Run Classifier ---
            // Pass context if needed, signal wrapper, result struct pointer
            res = ei_run_classifier(&classifier_ctx, &signal, &result, false); // Run inference

            if (res != EI_IMPULSE_OK) {
                // Handle inference error - update Modbus status?
                modbus_update_register(MODBUS_REG_STATUS, 0xEEEE); // Example error code
            } else {
                // 4. Process Inference Results
                uint8_t prediction = 0; // Default to normal
                float fault_prob = 0.0f;

                // Find the probability of "BearingFail" (assuming label order)
                for (uint8_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
                    // IMPORTANT: Compare label string carefully. Assumes "BearingFail" is the target.
                    if (strcmp(result.classification[i].label, "BearingFail") == 0) {
                        fault_prob = result.classification[i].value;
                        break;
                    }
                }

                if (fault_prob >= FAULT_DETECTION_THRESHOLD) {
                    prediction = 1; // Bearing Failure detected
                    fault_counter++;
                    modbus_update_register(MODBUS_REG_FAULT_COUNT, fault_counter);

                    // Send GSM Alert (with debouncing)
                    uint32_t current_time = millis();
                    if (gsm_ok && (current_time - last_gsm_alert_ms > GSM_ALERT_DEBOUNCE_MS)) {
                        if (gsm_send_alert(prediction)) { // Send prediction code (1)
                            last_gsm_alert_ms = current_time;
                        } else {
                             // GSM send failed - update status?
                             modbus_update_register(MODBUS_REG_STATUS, 0xFFFF); // GSM Fail
                             gsm_ok = false; // Assume GSM failed until re-init?
                        }
                    }
                    // Update Modbus status register to indicate fault
                     modbus_update_register(MODBUS_REG_STATUS, 1); // Status = Fault

                } else {
                     // Update Modbus status register to indicate normal
                     modbus_update_register(MODBUS_REG_STATUS, 0); // Status = OK
                }

                // Update last prediction register
                modbus_update_register(MODBUS_REG_LAST_PREDICT, prediction);
            }
             // Re-enable ADC trigger if it was disabled in ISR
             // ADCSRA |= (1 << ADATE);
        }

        // 5. Handle Modbus Communication
        modbus_check_request();

        // 6. Other tasks (e.g., watchdog reset, status LED blink)
        // wdt_reset();
        // _delay_ms(1); // Small delay to prevent tight loop hogging CPU?

    } // end while(1)

    return 0; // Should never reach here
}