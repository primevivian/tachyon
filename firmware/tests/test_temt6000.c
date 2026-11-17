/*
 * ============================================================
 * TACHYON - TEMT6000 LIGHT SENSOR TEST
 *
 * Wiring: SIG -> A1, VCC -> 5V, GND -> GND
 *
 * Sequence:
 *   1. Ambient light (2 s): level, stability
 *   2. Cover the sensor -> reading must drop
 *   3. 10 s with NO start light -> no false starts
 *      (move around, wave a hand over it, switch room lights)
 *   4. Flash the start light -> detected? how fast? how strong?
 *      -> suggested LIGHT_DELTA
 *   5. Live bar graph
 *
 * Steps 2-4 wait for a button press first.
 *
 * Detection is the same as in main.c: the reading must rise
 * LIGHT_DELTA above the learned ambient level for
 * LIGHT_CONFIRM_SAMPLES samples in a row (2 ms apart).
 * ============================================================
 */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>


/* ============================================================
 * MINIMUM PROJECT
 *
 * A0 -> 8-bit value -> PWM duty cycle on D6 (Timer0, 62.5 kHz)
 * D7 -> 1 Hz square wave (Timer2 CTC, 1 ms tick, toggle / 500)
 * ============================================================ */

static volatile uint32_t ms_ticks = 0;

static void adc_init(void)
{
    /* AVcc reference */
    ADMUX = (1 << REFS0);

    /* Enable ADC, prescaler 128 -> 125 kHz */
    ADCSRA = (1 << ADEN)  |
             (1 << ADPS2) |
             (1 << ADPS1) |
             (1 << ADPS0);

    DIDR0 = (1 << ADC0D) | (1 << ADC1D);
}

static uint16_t adc_read(uint8_t channel)
{
    ADMUX = (1 << REFS0) | (channel & 0x07);

    ADCSRA |= (1 << ADSC);

    while (ADCSRA & (1 << ADSC))
    {
        /* wait */
    }

    return ADC;
}

static uint8_t adc_read_8bit(void)
{
    return (uint8_t)(adc_read(0) >> 2);
}

static void pwm_init(void)
{
    DDRD |= (1 << PD6);

    TCCR0A = (1 << COM0A1) |
             (1 << WGM01)  |
             (1 << WGM00);

    TCCR0B = (1 << CS00);

    OCR0A = 0;
}

static void timer2_init(void)
{
    DDRD |= (1 << PD7);

    TCCR2A = (1 << WGM21);
    OCR2A = 249;
    TCCR2B = (1 << CS22);
    TIMSK2 = (1 << OCIE2A);
}

ISR(TIMER2_COMPA_vect)
{
    static uint16_t count = 0;

    ms_ticks++;

    count++;

    if (count >= 500)
    {
        PIND = (1 << PD7);
        count = 0;
    }
}

static void minimum_project_init(void)
{
    adc_init();
    pwm_init();
    timer2_init();
}

/* Call as often as possible: A0 sets the D6 duty cycle. */
static void minimum_project_update(void)
{
    OCR0A = adc_read_8bit();
}

static uint32_t millis(void)
{
    uint32_t t;
    uint8_t sreg = SREG;

    cli();
    t = ms_ticks;
    SREG = sreg;

    return t;
}

/* Waits while keeping the minimum project running. */
static void delay_ms(uint32_t ms)
{
    uint32_t start = millis();

    while ((uint32_t)(millis() - start) < ms)
    {
        minimum_project_update();
    }
}


/* ============================================================
 * SERIAL OUTPUT
 * ============================================================ */

static void uart_init(void)
{
    /* 16 MHz / (8 * (16 + 1)) = 117647 baud (+2.1%) */
    UCSR0A = (1 << U2X0);
    UBRR0 = 16;
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

static void uart_putc(char c)
{
    while (!(UCSR0A & (1 << UDRE0)))
    {
        /* wait */
    }

    UDR0 = c;
}

static void print_str(const char *s)
{
    while (*s)
    {
        uart_putc(*s++);
    }
}

static void print_u32(uint32_t v)
{
    char buf[10];
    uint8_t i = 0;

    do
    {
        buf[i++] = (char)('0' + (v % 10));
        v /= 10;
    } while (v);

    while (i)
    {
        uart_putc(buf[--i]);
    }
}

static void print_header(const char *title)
{
    print_str("\r\n==============================\r\n ");
    print_str(title);
    print_str("\r\n==============================\r\n");
}

static uint8_t tests_passed = 0;
static uint8_t tests_failed = 0;

/* Prints "[PASS] name" or "[FAIL] name" and keeps a tally. */
static uint8_t check(const char *name, uint8_t pass)
{
    print_str(pass ? "[PASS] " : "[FAIL] ");
    print_str(name);
    print_str("\r\n");

    if (pass)
    {
        tests_passed++;
    }
    else
    {
        tests_failed++;
    }

    return pass;
}

static void print_tally(void)
{
    print_str("\r\nRESULT: ");
    print_u32(tests_passed);
    print_str(" passed, ");
    print_u32(tests_failed);
    print_str(" failed\r\n");
}


/* ============================================================
 * ARM BUTTON (D4 to GND, internal pull-up)
 * ============================================================ */

static void button_init(void)
{
    DDRD &= ~(1 << PD4);
    PORTD |= (1 << PD4);
}

static uint8_t button_down(void)
{
    return !(PIND & (1 << PD4));
}

/* Waits for a debounced press AND release. */
static void button_wait(void)
{
    uint8_t stable = 0;

    while (stable < 20)
    {
        delay_ms(1);
        stable = button_down() ? (uint8_t)(stable + 1) : 0;
    }

    stable = 0;

    while (stable < 20)
    {
        delay_ms(1);
        stable = button_down() ? 0 : (uint8_t)(stable + 1);
    }
}


/* Start-light settings (keep in sync with config.h) */
#define LIGHT_CHANNEL           1
#define LIGHT_DELTA             150
#define LIGHT_CONFIRM_SAMPLES   2
#define LIGHT_BASELINE_SHIFT    6

#define SAMPLE_MS               2


/* ============================================================
 * START-LIGHT DETECTOR (same algorithm as main.c)
 * ============================================================ */

static uint32_t baseline_x64 = 0;
static uint8_t confirm = 0;

static uint16_t light_read(void)
{
    return adc_read(LIGHT_CHANNEL);
}

static uint16_t baseline(void)
{
    return (uint16_t)(baseline_x64 >> LIGHT_BASELINE_SHIFT);
}

static void detector_reset(uint16_t ambient)
{
    baseline_x64 = (uint32_t)ambient << LIGHT_BASELINE_SHIFT;
    confirm = 0;
}

/* Feed one sample; returns 1 when the start light is detected. */
static uint8_t detector_update(uint16_t light)
{
    if (light > baseline() + LIGHT_DELTA)
    {
        if (++confirm >= LIGHT_CONFIRM_SAMPLES)
        {
            return 1;
        }
    }
    else
    {
        confirm = 0;
        baseline_x64 += light;
        baseline_x64 -= baseline_x64 >> LIGHT_BASELINE_SHIFT;
    }

    return 0;
}

/* Average over `ms` milliseconds. */
static uint16_t light_average(uint16_t ms)
{
    uint32_t sum = 0;
    uint16_t i;

    for (i = 0; i < ms; i++)
    {
        delay_ms(1);
        sum += light_read();
    }

    return (uint16_t)(sum / ms);
}


/* ============================================================
 * TESTS
 * ============================================================ */

static uint16_t ambient = 0;

static void test_ambient(void)
{
    uint32_t sum = 0;
    uint16_t lo = 1023;
    uint16_t hi = 0;
    uint16_t i;
    uint16_t v;

    print_header("1. AMBIENT LIGHT");
    print_str("Normal lighting, don't touch the sensor...\r\n");

    delay_ms(500);

    for (i = 0; i < 2000; i++)
    {
        delay_ms(1);
        v = light_read();
        sum += v;

        if (v < lo) lo = v;
        if (v > hi) hi = v;
    }

    ambient = (uint16_t)(sum / 2000);

    print_str("  mean ");
    print_u32(ambient);
    print_str("  min ");
    print_u32(lo);
    print_str("  max ");
    print_u32(hi);
    print_str("  (0-1023)\r\n");

    check("reading above 0 (sensor powered, SIG on A1)", ambient > 2);
    check("not saturated (< 1020)", hi < 1020);

    /*
     * LED/fluorescent room lights flicker at 100/120 Hz, which
     * shows up as noise here. That is fine as long as it stays
     * well below LIGHT_DELTA.
     */
    check("stable: noise well below LIGHT_DELTA",
          (uint16_t)(hi - lo) < LIGHT_DELTA / 3);
}

static void test_cover(void)
{
    uint32_t start;
    uint16_t lowest = 1023;
    uint16_t v;

    print_header("2. COVER THE SENSOR");
    print_str("Press the button, then cover the sensor with a finger\r\n"
              "(within 5 s).\r\n");

    button_wait();
    print_str("  go!\r\n");

    start = millis();

    while (millis() - start < 5000)
    {
        delay_ms(SAMPLE_MS);
        v = light_read();

        if (v < lowest)
        {
            lowest = v;
        }

        if (v < ambient / 2)
        {
            break;
        }
    }

    print_str("  lowest reading: ");
    print_u32(lowest);
    print_str("\r\n");

    check("reading drops when covered", lowest < ambient / 2);

    delay_ms(1000);
}

static void test_no_false_start(void)
{
    uint32_t start;
    uint16_t highest = 0;
    uint16_t v;
    uint8_t triggered = 0;

    print_header("3. NO FALSE STARTS (10 s)");
    print_str("Press the button. For 10 s do NOT flash the start light:\r\n"
              "move around, wave a hand over it, switch room lights.\r\n");

    button_wait();
    print_str("  watching...\r\n");

    detector_reset(light_average(200));
    start = millis();

    while (millis() - start < 10000)
    {
        delay_ms(SAMPLE_MS);
        v = light_read();

        if (v > highest)
        {
            highest = v;
        }

        if (detector_update(v))
        {
            triggered = 1;
            print_str("  FALSE START at ");
            print_u32(millis() - start);
            print_str(" ms, reading ");
            print_u32(v);
            print_str(", baseline ");
            print_u32(baseline());
            print_str("\r\n");
            break;
        }
    }

    print_str("  highest reading: ");
    print_u32(highest);
    print_str("\r\n");

    check("no false start", !triggered);
}

static void test_flash(void)
{
    uint32_t start;
    uint32_t t_trigger = 0;
    uint32_t t_peak = 0;
    uint16_t base;
    uint16_t peak = 0;
    uint16_t v;
    uint8_t detected = 0;

    print_header("4. START LIGHT");
    print_str("Put the robot at the start line. Press the button, then\r\n"
              "flash the start light (or a phone flashlight) within 30 s.\r\n");

    button_wait();
    print_str("  waiting for the flash...\r\n");

    detector_reset(light_average(200));
    start = millis();

    while (millis() - start < 30000)
    {
        delay_ms(SAMPLE_MS);

        if (detector_update(light_read()))
        {
            detected = 1;
            t_trigger = millis();
            break;
        }
    }

    if (!check("start light detected", detected))
    {
        print_str("  -> lower LIGHT_DELTA or move the sensor closer\r\n");
        return;
    }

    /* Keep watching to see how bright the flash gets */
    base = baseline();

    while (millis() - t_trigger < 500)
    {
        delay_ms(SAMPLE_MS);
        v = light_read();

        if (v > peak)
        {
            peak = v;
            t_peak = millis();
        }
    }

    print_str("  ambient ");
    print_u32(base);
    print_str("  peak ");
    print_u32(peak);
    print_str("  rise ");
    print_u32(peak - base);
    print_str("\r\n  detected ");
    print_u32((uint32_t)LIGHT_CONFIRM_SAMPLES * SAMPLE_MS);
    print_str(" ms after crossing the threshold; peak ");
    print_u32(t_peak - t_trigger);
    print_str(" ms after detection\r\n");

    check("flash is at least 2x LIGHT_DELTA (safety margin)",
          (uint16_t)(peak - base) >= 2 * LIGHT_DELTA);

    print_str("  suggested LIGHT_DELTA: ");
    print_u32((uint32_t)(peak - base) / 2);
    print_str(" (half the rise)\r\n");
}

static void live(void)
{
    uint16_t v;
    uint8_t i;

    print_header("5. LIVE");

    while (1)
    {
        delay_ms(100);
        v = light_read();

        print_u32(v);
        print_str(v < 10 ? "    " : v < 100 ? "   " : v < 1000 ? "  " : " ");

        for (i = 0; i < (uint8_t)(v / 16); i++)
        {
            uart_putc('#');
        }

        print_str("\r\n");
    }
}


/* ============================================================
 * MAIN
 * ============================================================ */

int main(void)
{
    minimum_project_init();
    uart_init();
    button_init();

    sei();

    print_header("TACHYON TEMT6000 TEST");

    test_ambient();
    test_cover();
    test_no_false_start();
    test_flash();

    print_tally();

    live();

    return 0;
}
