#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

/* ============================================================
 * ADC
 * A0 -> 8-bit analog input
 * ============================================================ */

static void adc_init(void)
{
    /* AVcc reference, ADC0, left-adjust result */
    ADMUX = (1 << REFS0) | (1 << ADLAR);

    /* Enable ADC, prescaler = 128 */
    ADCSRA = (1 << ADEN)  |
             (1 << ADPS2) |
             (1 << ADPS1) |
             (1 << ADPS0);
}

static uint8_t adc_read_8bit(void)
{
    /* Start conversion */
    ADCSRA |= (1 << ADSC);

    /* Wait for conversion to complete */
    while (ADCSRA & (1 << ADSC))
    {
    }

    /* Return the upper 8 bits of the ADC result */
    return ADCH;
}


/* ============================================================
 * PWM
 * D6 (PD6 / OC0A) -> PWM output
 * Frequency = 62.5 kHz
 * ============================================================ */

static void pwm_init(void)
{
    /* Configure D6 as output */
    DDRD |= (1 << PD6);

    /* Fast PWM, non-inverting mode */
    TCCR0A = (1 << COM0A1) |
             (1 << WGM01)  |
             (1 << WGM00);

    /* Prescaler = 1 */
    TCCR0B = (1 << CS00);

    /* Initial duty cycle = 0 */
    OCR0A = 0;
}


/* ============================================================
 * TIMER1
 * D7 (PD7) -> Approximately 1 Hz square wave
 *
 * CPU frequency = 16 MHz
 * Prescaler = 1024
 * Compare value = 7812
 *
 * Interrupt interval:
 * (7812 + 1) * 1024 / 16000000
 * = 0.500032 seconds
 *
 * Output frequency:
 * 1 / (2 * 0.500032)
 * approximately 0.999936 Hz
 * ============================================================ */

static void timer1_init(void)
{
    /* Configure D7 as output */
    DDRD |= (1 << PD7);

    /* Initialize output LOW */
    PORTD &= ~(1 << PD7);

    /* Stop and configure Timer1 */
    TCCR1A = 0;
    TCCR1B = 0;

    /* Reset counter */
    TCNT1 = 0;

    /* Compare match interval */
    OCR1A = 7812;

    /* Clear pending compare-match flag */
    TIFR1 = (1 << OCF1A);

    /* Enable Timer1 Compare Match A interrupt */
    TIMSK1 = (1 << OCIE1A);

    /* CTC mode, prescaler = 1024 */
    TCCR1B = (1 << WGM12) |
             (1 << CS12)  |
             (1 << CS10);
}


/* ============================================================
 * TIMER1 INTERRUPT
 * Toggle D7 on each compare match
 * ============================================================ */

ISR(TIMER1_COMPA_vect)
{
    /* Toggle PD7 directly */
    PIND = (1 << PD7);
}


/* ============================================================
 * MAIN
 * ============================================================ */

int main(void)
{
    /* Initialize peripherals */
    adc_init();
    pwm_init();
    timer1_init();

    /* Enable global interrupts */
    sei();

    /* Continuously update PWM from analog input */
    while (1)
    {
        OCR0A = adc_read_8bit();
    }
}
