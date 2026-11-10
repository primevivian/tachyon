/*
 * ============================================================
 * TACHYON - MOTOR + ENCODER TEST (L298N, JGA25-371)
 *
 * !! LIFT THE WHEELS OFF THE GROUND BEFORE STARTING !!
 *
 * Wiring (same as main.c):
 *   D9  ENA   D5 IN1   A2 IN2      left motor
 *   D10 ENB   D8 IN3   A3 IN4      right motor
 *   D2  left encoder A    D3 right encoder A
 *   D4  button
 *
 * Sequence (press the button to start):
 *   For each motor:
 *     1. 50% then 100% forward: encoder counts, PWM controls
 *        speed, correct encoder belongs to the correct motor
 *     2. Coast-down time vs brake time: braking works
 *     3. Slowest PWM that still turns the wheel (-> CRAWL_PWM)
 *   Both motors at 100%: left/right speed mismatch
 *   Live encoder mode: turn a wheel 10 turns by hand to check
 *   ENCODER_COUNTS_PER_REV (button resets the counts).
 *
 * Watch the wheels: each should spin FORWARD when the
 * message says so. If one spins backwards, set
 * LEFT/RIGHT_MOTOR_REVERSED in config.h.
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

/* Float with 3 decimals. */
static void print_f(float v)
{
    uint32_t scaled;
    uint16_t frac;

    if (v < 0.0f)
    {
        uart_putc('-');
        v = -v;
    }

    scaled = (uint32_t)(v * 1000.0f + 0.5f);
    frac = (uint16_t)(scaled % 1000);

    print_u32(scaled / 1000);
    uart_putc('.');
    uart_putc((char)('0' + frac / 100));
    uart_putc((char)('0' + (frac / 10) % 10));
    uart_putc((char)('0' + frac % 10));
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

/* Non-blocking: 1 once per press. Call every ~1 ms. */
static uint8_t button_pressed(void)
{
    static uint8_t stable = 0;
    static uint8_t count = 0;
    uint8_t raw = button_down();

    if (raw != stable)
    {
        if (++count >= 20)
        {
            stable = raw;
            count = 0;

            return stable;
        }
    }
    else
    {
        count = 0;
    }

    return 0;
}


/* Robot settings (keep in sync with config.h) */
#define WHEEL_DIAMETER_MM       60.0f
#define ENCODER_PPR             11.0f
#define GEAR_RATIO              4.0f
#define ENCODER_COUNTS_PER_REV  (ENCODER_PPR * GEAR_RATIO)
#define LEFT_MOTOR_REVERSED     0
#define RIGHT_MOTOR_REVERSED    0


#define METERS_PER_COUNT \
    ((3.14159265f * WHEEL_DIAMETER_MM / 1000.0f) / ENCODER_COUNTS_PER_REV)

#define LEFT    0
#define RIGHT   1

/* Wheel counts as stopped after this long without an edge. */
#define STOP_GAP_MS     100


/* ============================================================
 * ENCODERS
 * ============================================================ */

static volatile uint32_t enc_count[2];
static volatile uint32_t enc_last_ms[2];

ISR(INT0_vect)
{
    enc_count[LEFT]++;
    enc_last_ms[LEFT] = ms_ticks;
}

ISR(INT1_vect)
{
    enc_count[RIGHT]++;
    enc_last_ms[RIGHT] = ms_ticks;
}

static void encoders_init(void)
{
    DDRD &= ~((1 << PD2) | (1 << PD3));
    PORTD |= (1 << PD2) | (1 << PD3);

    EICRA = (1 << ISC01) | (1 << ISC00) |
            (1 << ISC11) | (1 << ISC10);

    EIFR = (1 << INTF0) | (1 << INTF1);
    EIMSK = (1 << INT0) | (1 << INT1);
}

static uint32_t enc_get(uint8_t side)
{
    uint32_t n;
    uint8_t sreg = SREG;

    cli();
    n = enc_count[side];
    SREG = sreg;

    return n;
}

static uint32_t enc_last(uint8_t side)
{
    uint32_t t;
    uint8_t sreg = SREG;

    cli();
    t = enc_last_ms[side];
    SREG = sreg;

    return t;
}

static void enc_reset(void)
{
    uint8_t sreg = SREG;

    cli();
    enc_count[LEFT] = 0;
    enc_count[RIGHT] = 0;
    SREG = sreg;
}


/* ============================================================
 * MOTORS
 * ============================================================ */

static void motors_init(void)
{
    DDRB |= (1 << PB1) | (1 << PB2) | (1 << PB0);
    DDRD |= (1 << PD5);
    DDRC |= (1 << PC2) | (1 << PC3);

    OCR1A = 0;
    OCR1B = 0;

    /* 8-bit phase-correct PWM, 31.4 kHz (same as main.c) */
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
    TCCR1B = (1 << CS10);
}

static void set_dir(uint8_t side, uint8_t a, uint8_t b)
{
    if (side == LEFT)
    {
        if (a) PORTD |= (1 << PD5); else PORTD &= ~(1 << PD5);
        if (b) PORTC |= (1 << PC2); else PORTC &= ~(1 << PC2);
    }
    else
    {
        if (a) PORTB |= (1 << PB0); else PORTB &= ~(1 << PB0);
        if (b) PORTC |= (1 << PC3); else PORTC &= ~(1 << PC3);
    }
}

static void set_pwm(uint8_t side, uint8_t pwm)
{
    if (side == LEFT) OCR1A = pwm; else OCR1B = pwm;
}

static void motor_forward(uint8_t side, uint8_t pwm)
{
    uint8_t rev = (side == LEFT) ? LEFT_MOTOR_REVERSED : RIGHT_MOTOR_REVERSED;

    set_dir(side, !rev, rev);
    set_pwm(side, pwm);
}

static void motor_brake(uint8_t side)
{
    set_dir(side, 0, 0);
    set_pwm(side, 255);
}

static void motor_coast(uint8_t side)
{
    set_pwm(side, 0);
    set_dir(side, 0, 0);
}


/* ============================================================
 * MEASUREMENTS
 * ============================================================ */

/* Counts per second over `ms` (motor must already be running). */
static float rate(uint8_t side, uint32_t ms)
{
    uint32_t c0 = enc_get(side);

    delay_ms(ms);

    return (float)(enc_get(side) - c0) * 1000.0f / (float)ms;
}

/* Time from now until the wheel stops; counts travelled meanwhile. */
static uint32_t time_to_stop(uint8_t side, uint32_t *counts)
{
    uint32_t start = millis();
    uint32_t c0 = enc_get(side);

    while (millis() - start < 3000)
    {
        minimum_project_update();

        if (millis() - enc_last(side) >= STOP_GAP_MS)
        {
            break;
        }
    }

    *counts = enc_get(side) - c0;

    /* The wheel really stopped at its last edge */
    if (enc_last(side) > start)
    {
        return enc_last(side) - start;
    }

    return 0;
}

static void print_speed(const char *label, float cps)
{
    print_str(label);
    print_f(cps);
    print_str(" counts/s = ");
    print_f(cps * METERS_PER_COUNT);
    print_str(" m/s\r\n");
}

static uint8_t min_start_pwm = 0;

static void test_one_motor(uint8_t side)
{
    const char *name = (side == LEFT) ? "LEFT" : "RIGHT";
    uint8_t other = (side == LEFT) ? RIGHT : LEFT;
    float r50, r100;
    uint32_t other0;
    uint32_t coast_ms, brake_ms, coast_counts, brake_counts;
    uint16_t pwm;

    print_header(side == LEFT ? "LEFT MOTOR" : "RIGHT MOTOR");
    print_str(name);
    print_str(" wheel should spin FORWARD now.\r\n");

    other0 = enc_get(other);

    /* --- Speed control --- */
    motor_forward(side, 128);
    delay_ms(700);
    r50 = rate(side, 1000);
    print_speed("  50%:  ", r50);

    motor_forward(side, 255);
    delay_ms(700);
    r100 = rate(side, 1000);
    print_speed("  100%: ", r100);

    check("encoder counts while motor runs", r50 > 0.0f);
    check("more PWM = more speed", r100 > r50 * 1.1f);
    check("other encoder stays still (not swapped/crossed)",
          (enc_get(other) - other0) < 5);

    /* --- Coast vs brake --- */
    motor_coast(side);
    coast_ms = time_to_stop(side, &coast_counts);

    motor_forward(side, 255);
    delay_ms(1000);
    motor_brake(side);
    brake_ms = time_to_stop(side, &brake_counts);
    delay_ms(300);
    motor_coast(side);

    print_str("  coast to stop: ");
    print_u32(coast_ms);
    print_str(" ms, ");
    print_u32(coast_counts);
    print_str(" counts\r\n  brake to stop: ");
    print_u32(brake_ms);
    print_str(" ms, ");
    print_u32(brake_counts);
    print_str(" counts\r\n");

    check("brake stops faster than coasting", brake_ms < coast_ms);

    /* --- Minimum PWM that turns the wheel --- */
    delay_ms(500);

    for (pwm = 0; pwm <= 255; pwm += 5)
    {
        uint32_t c0 = enc_get(side);

        motor_forward(side, (uint8_t)pwm);
        delay_ms(150);

        if (enc_get(side) - c0 >= 3)
        {
            break;
        }
    }

    motor_coast(side);

    if (pwm <= 255)
    {
        print_str("  wheel starts turning at PWM ");
        print_u32(pwm);
        print_str("\r\n");

        if (pwm > min_start_pwm)
        {
            min_start_pwm = (uint8_t)pwm;
        }
    }

    check("wheel starts below full power", pwm <= 255);

    delay_ms(500);
}

static void test_both(void)
{
    uint32_t l0, r0;
    float lr, rr, diff;

    print_header("BOTH MOTORS 100%");

    motor_forward(LEFT, 255);
    motor_forward(RIGHT, 255);
    delay_ms(1000);

    l0 = enc_get(LEFT);
    r0 = enc_get(RIGHT);
    delay_ms(2000);
    lr = (float)(enc_get(LEFT) - l0) / 2.0f;
    rr = (float)(enc_get(RIGHT) - r0) / 2.0f;

    motor_brake(LEFT);
    motor_brake(RIGHT);
    delay_ms(1000);
    motor_coast(LEFT);
    motor_coast(RIGHT);

    print_speed("  left:  ", lr);
    print_speed("  right: ", rr);

    diff = (lr > rr) ? (lr - rr) : (rr - lr);
    diff = (lr + rr > 0.0f) ? diff * 200.0f / (lr + rr) : 100.0f;

    print_str("  mismatch: ");
    print_f(diff);
    print_str(" %  (the gyro heading hold corrects this)\r\n");

    check("motors within 15% of each other", diff < 15.0f);
}

static void live_encoders(void)
{
    uint32_t last_print = 0;

    print_header("LIVE ENCODERS");
    print_str("Turn one wheel exactly 10 turns by hand.\r\n");
    print_str("Expected: ");
    print_f(ENCODER_COUNTS_PER_REV * 10.0f);
    print_str(" counts (config.h). Button = reset.\r\n");

    enc_reset();

    while (1)
    {
        delay_ms(1);

        if (button_pressed())
        {
            enc_reset();
            print_str("-- reset --\r\n");
        }

        if (millis() - last_print >= 250)
        {
            last_print = millis();

            print_str("encL ");
            print_u32(enc_get(LEFT));
            print_str("  encR ");
            print_u32(enc_get(RIGHT));
            print_str("\r\n");
        }
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
    motors_init();
    encoders_init();

    sei();

    print_header("TACHYON MOTOR + ENCODER TEST");
    print_str("Lift the wheels off the ground, then press the button.\r\n");

    button_wait();

    test_one_motor(LEFT);
    test_one_motor(RIGHT);
    test_both();

    if (min_start_pwm > 0)
    {
        print_str("\r\nSuggested CRAWL_PWM: ");
        print_u32(min_start_pwm + 20U > 255U ? 255U : min_start_pwm + 20U);
        print_str(" (on the ground it may need more)\r\n");
    }

    print_tally();

    live_encoders();

    return 0;
}
