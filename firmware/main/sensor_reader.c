#include "sensor_reader.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h> // For atomic access to volatile variables

// --- Global Variables Definition ---
volatile int8_t adc_sample_buffer[ADC_BUFFER_SIZE];
volatile bool adc_buffer_ready_flag = false;
volatile uint16_t adc_buffer_index = 0;

// --- ADC Initialization ---
void sensor_adc_init(void) {
    // ADMUX: Reference voltage (AVCC), Left Adjust Result (ADLAR=1 for 8-bit reads), Channel Selection
    ADMUX = (1 << REFS0) | // AVCC with external capacitor at AREF pin
            (1 << ADLAR) | // Left Adjust Result (reads ADCH for 8 bits)
            (ADC_CHANNEL & 0x0F); // Select ADC channel (mask to ensure valid channel)

    // ADCSRA: Enable ADC, Enable Auto Trigger, Enable Interrupt, Set Prescaler
    ADCSRA = (1 << ADEN) | // Enable ADC
             (1 << ADATE) | // Enable ADC Auto Trigger
             (1 << ADIE) | // Enable ADC Conversion Complete Interrupt
             (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Prescaler 128 (16MHz/128 = 125kHz ADC clock)
             // Choose prescaler for ADC clock between 50kHz and 200kHz

    // ADCSRB: Set Auto Trigger Source (Timer/Counter1 Compare Match A)
    ADCSRB = (1 << ADTS2) | (1 << ADTS0); // Timer/Counter1 Compare Match A Trigger Source (0b101)
}

// --- Timer1 Initialization ---
void sensor_timer_init(void) {
    // Configure Timer1 for CTC mode to trigger ADC at SAMPLE_RATE_HZ
    TCCR1A = 0; // No PWM, normal port operation
    TCCR1B = (1 << WGM12) | // CTC Mode (Clear Timer on Compare Match)
             (1 << CS11);   // Prescaler 8 (16MHz / 8 = 2MHz timer clock)
             // For 1kHz sample rate (1ms period): OCR1A = (2MHz / 1000Hz) - 1 = 1999

    OCR1A = (F_CPU / 8 / SAMPLE_RATE_HZ) - 1; // Calculate compare value

    // No need to enable Timer1 Compare A interrupt here, it only triggers the ADC
    // TIMSK1 |= (1 << OCIE1A); // Don't enable timer interrupt directly
}

// --- ADC Conversion Complete ISR ---
ISR(ADC_vect) {
    // This ISR is called when an ADC conversion finishes.
    // The Timer1 Compare Match triggers the conversion automatically (ADATE=1).

    if (!adc_buffer_ready_flag) {
        // Read the 8-bit result (ADLAR=1 makes result left-adjusted, so read ADCH)
        // ADC result is 0-255. Vibration sensor might be AC coupled, centered around 128.
        // Subtract offset (e.g., 128 if sensor biased mid-rail) to get signed int8_t (-128 to +127).
        adc_sample_buffer[adc_buffer_index] = (int8_t)(ADCH - 128); // Assuming sensor output is biased at VCC/2

        adc_buffer_index++;

        if (adc_buffer_index >= ADC_BUFFER_SIZE) {
            adc_buffer_ready_flag = true; // Signal that buffer is full
            adc_buffer_index = 0;
            // Optionally disable ADC auto trigger here until buffer is processed?
            // ADCSRA &= ~(1 << ADATE);
        }
    }
    // If buffer is ready, new samples might be discarded until processed by main loop.
    // Consider double buffering for high sample rates / slow processing.
}

// --- Get Samples Function ---
bool sensor_get_samples(int8_t* destination_buffer) {
    bool was_ready = false;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { // Ensure atomic access to the flag and buffer
        if (adc_buffer_ready_flag) {
            for (uint16_t i = 0; i < ADC_BUFFER_SIZE; ++i) {
                destination_buffer[i] = adc_sample_buffer[i];
            }
            adc_buffer_ready_flag = false; // Clear flag, ready for next batch
            was_ready = true;
            // Re-enable ADC auto trigger if it was disabled
            // ADCSRA |= (1 << ADATE);
        }
    }
    return was_ready;
}