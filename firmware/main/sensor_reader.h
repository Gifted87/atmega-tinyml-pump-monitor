#ifndef SENSOR_READER_H
#define SENSOR_READER_H

#include <stdint.h>
#include <stdbool.h>
#include "fft_256.h" // For FFT_SIZE

// --- Configuration ---
#define ADC_CHANNEL 0 // ADC channel connected to the vibration sensor (e.g., ADC0)
#define SAMPLE_RATE_HZ 1000 // Target sample rate (Hz)
#define ADC_BUFFER_SIZE FFT_SIZE // Buffer size matches FFT size

// --- Global Variables ---
// Buffer to store ADC samples, filled by ISR
extern volatile int8_t adc_sample_buffer[ADC_BUFFER_SIZE];
// Flag indicating when the buffer is full and ready for processing
extern volatile bool adc_buffer_ready_flag;

// --- Functions ---

/**
 * @brief Initializes the ADC peripheral for sensor reading.
 */
void sensor_adc_init(void);

/**
 * @brief Initializes Timer1 to trigger ADC conversions at the specified SAMPLE_RATE_HZ.
 */
void sensor_timer_init(void);

/**
 * @brief Copies the completed sample buffer into a destination buffer.
 * Should be called after adc_buffer_ready_flag is true. Clears the flag.
 *
 * @param destination_buffer Pointer to the buffer where samples should be copied.
 * @return true if a buffer was ready and copied, false otherwise.
 */
bool sensor_get_samples(int8_t* destination_buffer);

#endif // SENSOR_READER_H