#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

/*
 * Arduino Uno / ATmega328P
 *
 * A0  -> analog input
 * D6  -> PWM output (OC0A), actuator LED
 * D7 -> 1 Hz square wave / built-in LED
 *
 * No Arduino libraries are used.
 */


/* ============================================================
 * ADC
 * ============================================================ */

void adc_init(void)
{
    /*
     * AVcc = ADC reference voltage (5 V on the Uno)
     * MUX = 0000 -> ADC0 -> Arduino A0
     */
    ADMUX = (1 << REFS0);

    /*
     * Enable ADC
     *
     * ADC clock = 16 MHz / 128 = 125 kHz
     * This is within the recommended ADC clock range.
     */
    ADCSRA =
        (1 << ADEN)  |
        (1 << ADPS2) |
        (1 << ADPS1) |
        (1 << ADPS0);
}


uint8_t adc_read_8bit(void)
{
    uint16_t result;

    /*
     * Start conversion
     */
    ADCSRA |= (1 << ADSC);

    /*
     * Wait until conversion is complete
     */
    while (ADCSRA & (1 << ADSC))
    {
        /* wait */
    }

    /*
     * ADC is 10 bits:
     *
     *     0 ... 1023
     *
     * Shift right by 2:
     *
     *     0 ... 255
     *
     * This gives the required 8-bit value.
     */
    result = ADC;

    return (uint8_t)(result >> 2);
}


/* ============================================================
 * PWM
 * ============================================================ */

void pwm_init(void)
{
    /*
     * D6 = PD6 = OC0A
     *
     * Set PD6 as output.
     */
    DDRD |= (1 << PD6);

    /*
     * Timer0 Fast PWM, 8-bit
     *
     * WGM00 = 1
     * WGM01 = 1
     *
     * Non-inverting PWM on OC0A (D6):
     *
     * COM0A1 = 1
     * COM0A0 = 0
     */
    TCCR0A =
        (1 << COM0A1) |
        (1 << WGM01)  |
        (1 << WGM00);

    /*
     * No prescaler
     *
     * 16 MHz / 256 = 62.5 kHz
     */
    TCCR0B =
        (1 << CS00);

    /*
     * Start with 0% duty cycle.
     */
    OCR0A = 0;
}


/* ============================================================
 * 1 Hz square wave
 * ============================================================ */

void timer2_init(void)
{
    /*
     * D7 = PD7
     *
     * Configure D7 as output.
     */
    DDRD |= (1 << PD7);

    /*
     * Timer2 in CTC mode.
     *
     * WGM21 = 1
     */
    TCCR2A = (1 << WGM21);

    /*
     * Timer2 clock:
     *
     * 16 MHz / 1024 = 15625 Hz
     *
     * We want an interrupt every 0.5 seconds.
     *
     * 15625 * 0.5 = 7812.5
     *
     * OCR2A is only 8 bits, so we cannot get 0.5 s
     * directly with Timer2.
     *
     * Instead we generate an interrupt every 16 ms
     * approximately and count interrupts in software.
     *
     * With OCR2A = 249:
     *
     * 16 MHz / 1024 / 250 = 62.5 Hz
     *
     * One interrupt = 16 ms.
     *
     * 31 interrupts ~= 496 ms.
     *
     * Toggling every 31 interrupts gives approximately
     * 1.008 Hz, close to 1 Hz.
     */

    OCR2A = 249;

    /*
     * Prescaler = 1024
     *
     * CS22 = 1
     * CS21 = 1
     * CS20 = 1
     */
    TCCR2B =
        (1 << CS22) |
        (1 << CS21) |
        (1 << CS20);

    /*
     * Enable Timer2 Compare Match A interrupt.
     */
    TIMSK2 = (1 << OCIE2A);
}


/*
 * Timer2 Compare Match A interrupt.
 *
 * Called approximately every 16 ms.
 */
ISR(TIMER2_COMPA_vect)
{
    static uint8_t count = 0;

    count++;

    if (count >= 31)
    {
        /*
         * Toggle D7.
         */
        PORTD ^= (1 << PD7);

        count = 0;
    }
}


/* ============================================================
 * Main
 * ============================================================ */

int main(void)
{
    uint8_t adc_value;

    /*
     * Initialize peripherals.
     */
    adc_init();
    pwm_init();
    timer2_init();

    /*
     * Enable global interrupts.
     */
    sei();

    /*
     * Main loop.
     */
    while (1)
    {
        /*
         * Read analog input and convert
         * 10-bit ADC result to 8 bits.
         */
        adc_value = adc_read_8bit();

        /*
         * Use the 8-bit value as PWM duty cycle.
         *
         * 0   -> 0%
         * 128 -> ~50%
         * 255 -> ~100%
         */
        OCR0A = adc_value;
    }

    return 0;
}
