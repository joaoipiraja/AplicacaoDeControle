#include "16f874.h"
#device adc=8
#use delay(clock=1000000)
#fuses XT, NOWDT, NOPUT

void main() {
   int16 sum_voltage;          // Variable to hold the sum of voltages
   int8 avg_voltage;           // Variable for average voltage calculation
   int8 i;

   setup_adc_ports(ALL_ANALOG);    // Set all ports as analog
   setup_adc(ADC_CLOCK_INTERNAL);  // Set ADC clock source
   set_adc_channel(0);             // Set ADC to read from channel 0
   delay_us(100);                  // Small delay for ADC stabilization
   output_high(PIN_C0);            // Set PIN_C0 high

   while(1) {
      sum_voltage = 0;              // Initialize sum
      // Loop to read ADC values and sum them
      for(i = 0; i < 5; i++) {
         delay_ms(1);
         sum_voltage += read_adc();
      }
      
      // Calculate the average of the 5 ADC values
      avg_voltage = sum_voltage / 5;
      
      output_d(avg_voltage);      // Output the average to PORTD
      output_low(PIN_C0);         // Set PIN_C0 low
      delay_ms(10);               // Delay for output stabilization
      output_high(PIN_C0);        // Set PIN_C0 high
   }
}

