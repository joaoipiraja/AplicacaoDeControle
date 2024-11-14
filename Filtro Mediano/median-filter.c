#include "16f874.h"
#device adc=8
#use delay(clock=1000000)
#fuses XT, NOWDT, NOPUT

#define NUM_SAMPLES 10

// Function to calculate the median of three values
int8 median(int8 a, int8 b, int8 c) {
    int8 temp;
    // Sort the values to find the median
    if (a > b) { temp = a; a = b; b = temp; }
    if (a > c) { temp = a; a = c; c = temp; }
    if (b > c) { temp = b; b = c; c = temp; }
    return b; // Return the median value
}

void main() {
    int8 i;
    int8 voltage[NUM_SAMPLES + 2]; // Array to store ADC readings with padding
    int8 median_value;

    setup_adc_ports(ALL_ANALOG);    // Set all ports as analog
    setup_adc(ADC_CLOCK_INTERNAL);  // Set ADC clock source
    set_adc_channel(0);             // Set ADC to read from channel 0
    delay_us(100);                  // Small delay for ADC stabilization
    output_high(PIN_C0);            // Set PIN_C0 high

    while(1) {
        // Read ADC samples into voltage[1..NUM_SAMPLES]
        for (i = 1; i <= NUM_SAMPLES; i++) {
            delay_ms(1);
            voltage[i] = read_adc();
        }
        // Pad the beginning and end of the array
        voltage[0] = voltage[1];
        voltage[NUM_SAMPLES + 1] = voltage[NUM_SAMPLES];

        // Apply median filtering and output results
        for (i = 0; i < NUM_SAMPLES; i++) {
            median_value = median(voltage[i], voltage[i + 1], voltage[i + 2]);
            output_d(median_value);       // Output the median value
            output_low(PIN_C0);           // Set PIN_C0 low
            delay_ms(10);
            output_high(PIN_C0);          // Set PIN_C0 high
        }
    }
}

