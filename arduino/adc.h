/* adc.h
 *
 * OS-aware functions for reading the ADC.
 *
 * Not quite sure what these should do in the context of the OS yet,
 *  but they should:
 * 1) provide general consensus about ADC settings
 * 2) prevent different threads from invalidating each other's ADC readings
 * 3) potentially allow other threads to execute while waiting for an ADC result
 * 4) potentially provide threads access to timed ADC interrupts
 *
 * Author: Austin Hendrix
 */

/* initialize the ADC */
void adc_init();

/* read from the specified ADC channel */
uint16_t adc_read(uint8_t channel);
