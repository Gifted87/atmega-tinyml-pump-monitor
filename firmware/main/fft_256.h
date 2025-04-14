#ifndef FFT_256_H
#define FFT_256_H

#include <stdint.h>
#include "fixed_math.h"

#define FFT_SIZE 256
#define FFT_LOG2_SIZE 8 // log2(256)

// Input: time_series - Array of 256 samples (int8_t, range -128 to 127)
// Output: freq_domain - Array of 128 complex points (represented as 256 q8_24_t values, Real/Imag interleaved)
//                     OR Array of 128 power spectral density values (q8_24_t) - matching prompt example
// Note: The implementation below follows the prompt's output format (power spectral density).

/**
 * @brief Computes the 256-point Fast Fourier Transform using fixed-point arithmetic.
 * Calculates the power spectral density for the first 128 frequency bins.
 *
 * @param time_series Pointer to the input array of 256 time-domain samples (int8_t).
 *                    This buffer WILL BE MODIFIED (in-place computation).
 * @param freq_domain Pointer to the output array of 128 power spectral density values (q8_24_t).
 */
void fft_256_calculate_power_spectrum(int8_t* time_series, q8_24_t* freq_domain);

#endif // FFT_256_H