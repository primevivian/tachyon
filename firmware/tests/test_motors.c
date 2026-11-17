/*
 * ============================================================
 * TACHYON - MOTOR + ENCODER TEST (L298N, 2x JGA25-371)
 *
 * !! LIFT THE WHEELS OFF THE GROUND BEFORE STARTING !!
 * !! Press the button at any time while a motor runs to STOP !!
 *
 * Board: Arduino Uno (ATmega328P, 16 MHz), flashed as plain C.
 * Output: Arduino Serial Monitor, 115200 baud.
 *
 * Power: 3S LiPo (11.1 V nominal, 12.6 V full) -> L298N +12V
 *        and a 7805 -> 5 V for the encoders and L298N logic.
 *
 * Wiring
 *   Encoder cable M+ / M- -> L298N OUT1 / OUT2   (motor 1, M1)
 *   Encoder cable M+ / M- -> L298N OUT3 / OUT4   (motor 2, M2)
 *   Encoder VCC / GND     -> 7805 5 V / GND
 *
 *   L298N ENA -> D9  (OC1A PWM)   IN1 -> D5   IN2 -> A2
 *   L298N ENB -> D10 (OC1B PWM)   IN3 -> D8   IN4 -> A3
 *   L298N +5V -> 7805 5 V. With the 5V-EN jumper removed, the
 *   on-board regulator is off and this pin is the logic SUPPLY
 *   INPUT: left open, the L298N does nothing.
 *   All grounds common: LiPo -, 7805, L298N, Uno.
 *
 *   M1 encoder C1 (A) -> D2 (INT0)   C2 (B) -> A4
 *   M2 encoder C1 (A) -> D3 (INT1)   C2 (B) -> A5
 *
 *   Button: D4 to GND (internal pull-up)
 *   D7: 1 Hz blinking LED (project requirement)
 *   The PWM-driven actuator required by the project is the
 *   motors themselves (D9 / D10).
 *
 * Encoder maths (JGA25-371 Hall encoder)
 *   11 pulses per MOTOR-shaft revolution, per channel.
 *   Wheel revolution = 11 x GEAR_RATIO pulses (channel A
 *   rising edges, which is what this test counts).
 *   The gear ratio depends on the exact motor variant, so
 *   the live mode at the end measures it by hand.
 *
 * Dragster use: the motors only ever drive FORWARD and are
 * never braked or reversed (the gearboxes should not take
 * reverse shock loads). Every stop in this test is a coast.
 *
 * Sequence (press the button to start)
 *   0. Encoder noise with motors off
 *   For each motor:
 *     1. Spin-up guard: no encoder pulses -> stop, skip motor
 *     2. PWM sweep 25/50/75/100%: speed follows PWM
 *     3. Speed stability at 100% (+ channel B quadrature)
 *     4. Signal quality: rise/fall balance, glitch count
 *     5. Other encoder stays still (swapped / crossed wires)
 *     6. Coast-down from 100%: time and distance to stop
 *     7. Lowest PWM that starts the wheel
 *   Both motors at 100%: each motor's speed, mismatch and the
 *   PWM trim that would make the faster one match
 *   Live encoder mode: turn a wheel exactly 10 turns by hand,
 *   press the button -> counts per wheel turn + gear ratio.
 *
 * Watch the wheels: both must spin FORWARD. If one spins
 * backwards, swap its two motor wires or set M1_REVERSED /
 * M2_REVERSED below.
 * ============================================================
 */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdint.h>


/* ============================================================
 * SETTINGS (keep in sync with the main firmware)
 * ============================================================ */

#define WHEEL_DIAMETER_MM   60.0f

/* Channel A rising edges per MOTOR-shaft revolution. */
#define ENC_PPR             11.0f

/* Gearbox ratio (motor turns per wheel turn). Read it from the
 * motor label / listing. 0 = unknown: the test estimates it
 * from the measured speed and MOTOR_NOMINAL_RPM. */
#define GEAR_RATIO          0.0f

/* Gearbox output speed, no load, at MOTOR_RATED_V. Only used
 * for the rough gear-ratio estimate. */
#define MOTOR_NOMINAL_RPM   500.0f
#define MOTOR_RATED_V       12.0f
#define BATTERY_V           11.1f   /* 3S LiPo nominal */
#define L298N_DROP_V        2.0f    /* typical, grows with current */

/* 1 = this motor spins backwards with the default direction. */
#define M1_REVERSED         0
#define M2_REVERSED         0

/* Channel B (direction). 1 = wired to the pins below. */
#define ENC_B_ENABLED       1
#define ENC1_B_PIN          PINC
#define ENC1_B_PORT         PORTC
#define ENC1_B_DDR          DDRC
#define ENC1_B_BIT          PC4
#define ENC2_B_PIN          PINC
#define ENC2_B_PORT         PORTC
#define ENC2_B_DDR          DDRC
#define ENC2_B_BIT          PC5

/* 1 = forward shows as negative position: flip it. */
#define ENC1_B_INVERT       0
#define ENC2_B_INVERT       0

/* Two edges closer than this are counted as a glitch. A real
 * edge at full speed is several hundred us apart. */
#define ENC_GLITCH_US       50UL

/* A wheel counts as stopped after this long without an edge. */
#define STOP_GAP_MS         100UL

/* Longest wait for a wheel to coast to a stop. */
#define STOP_WAIT_MS        3000UL

/* Stall guard: a motor driven at >= STALL_PWM that shows no
 * encoder edge for STALL_MS is stopped (jammed wheel, dead
 * encoder). Must be longer than the 500 ms spin-up check. */
#define STALL_PWM           192
#define STALL_MS            1000UL

/* Motor PWM (Timer1, 8-bit phase correct).
 *   (1 << CS11) -> 16 MHz / 8 / 510 = 3.9 kHz (default)
 *   (1 << CS10) -> 16 MHz / 1 / 510 = 31.4 kHz (silent)
 * The L298N switches slowly (bipolar), so at 31 kHz a large
 * part of each period is lost in switching: more heat, less
 * torque at low duty. 3.9 kHz whines, but drives better. */
#define MOTOR_PWM_CLOCK     (1 << CS11)

#define M1      0
#define M2      1

#define PI_F    3.14159265f


/* ============================================================
 * 1 MS TICK + 1 HZ LED
 *
 * Timer2 CTC, 1 ms tick. D7 toggles every 500 ms -> 1 Hz blink
 * (project requirement). Runs in the ISR, so it keeps blinking
 * whatever the test is doing, even after a halt.
 * ============================================================ */

static volatile uint32_t ms_ticks = 0;

static void timer2_init(void)
{
    DDRD |= (1 << PD7);

    /* CTC, 16 MHz / 64 / 250 = 1 kHz; TCNT2 steps are 4 us */
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

static uint32_t millis(void)
{
    uint32_t t;
    uint8_t sreg = SREG;

    cli();
    t = ms_ticks;
    SREG = sreg;

    return t;
}

/* Microseconds, 4 us resolution. Interrupts must be OFF. */
static uint32_t micros_noint(void)
{
    uint32_t ms = ms_ticks;
    uint8_t t = TCNT2;

    /* Compare match happened but its ISR has not run yet */
    if ((TIFR2 & (1 << OCF2A)) && t < 125)
    {
        ms++;
    }

    return ms * 1000UL + (uint32_t)t * 4UL;
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

/* Prints a string stored in flash (PSTR / PGM_P). Keeping the
 * text out of RAM matters: the 328P only has 2 KB of it. */
static void print_P(PGM_P s)
{
    char c;

    while ((c = (char)pgm_read_byte(s++)) != '\0')
    {
        uart_putc(c);
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

#if ENC_B_ENABLED
static void print_i32(int32_t v)
{
    if (v < 0)
    {
        uart_putc('-');
        print_u32((uint32_t)(-(v + 1)) + 1U);
    }
    else
    {
        print_u32((uint32_t)v);
    }
}
#endif

/* Float with 2 decimals. */
static void print_f(float v)
{
    uint32_t scaled;
    uint8_t frac;

    if (v < 0.0f)
    {
        uart_putc('-');
        v = -v;
    }

    scaled = (uint32_t)(v * 100.0f + 0.5f);
    frac = (uint8_t)(scaled % 100);

    print_u32(scaled / 100);
    uart_putc('.');
    uart_putc((char)('0' + frac / 10));
    uart_putc((char)('0' + frac % 10));
}

static void print_header(PGM_P title)
{
    print_P(PSTR("\r\n==============================\r\n "));
    print_P(title);
    print_P(PSTR("\r\n==============================\r\n"));
}

static uint8_t tests_passed = 0;
static uint8_t tests_failed = 0;

/* Prints "[PASS] name" or "[FAIL] name" and keeps a tally. */
static uint8_t check(PGM_P name, uint8_t pass)
{
    print_P(pass ? PSTR("[PASS] ") : PSTR("[FAIL] "));
    print_P(name);
    print_P(PSTR("\r\n"));

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
    print_P(PSTR("\r\nRESULT: "));
    print_u32(tests_passed);
    print_P(PSTR(" passed, "));
    print_u32(tests_failed);
    print_P(PSTR(" failed\r\n"));
}


/* ============================================================
 * BUTTON (D4 to GND, internal pull-up)
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


/* ============================================================
 * MOTORS
 * ============================================================ */

static uint8_t motor_live[2];   /* EN is driving (PWM > 0) */
static uint8_t motor_run[2];    /* driven at >= STALL_PWM */
static uint32_t run_since[2];   /* millis() when motor_run was set */

static void motors_init(void)
{
    /* IN pins low first, then outputs: no twitch at power up */
    PORTD &= ~(1 << PD5);
    PORTB &= ~(1 << PB0);
    PORTC &= ~((1 << PC2) | (1 << PC3));

    DDRB |= (1 << PB1) | (1 << PB2) | (1 << PB0);
    DDRD |= (1 << PD5);
    DDRC |= (1 << PC2) | (1 << PC3);

    OCR1A = 0;
    OCR1B = 0;

    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10);
    TCCR1B = MOTOR_PWM_CLOCK;
}

static void set_in(uint8_t m, uint8_t a, uint8_t b)
{
    if (m == M1)
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

static void set_en(uint8_t m, uint8_t pwm)
{
    if (m == M1) OCR1A = pwm; else OCR1B = pwm;

    motor_live[m] = (pwm != 0);
}

/* Forward only: the dragster never reverses. */
static void motor_drive(uint8_t m, uint8_t pwm)
{
    uint8_t a = (m == M1) ? !M1_REVERSED : !M2_REVERSED;

    set_in(m, a, !a);
    set_en(m, pwm);

    /* Stall guard timer starts when the drive reaches STALL_PWM */
    if (pwm < STALL_PWM)
    {
        motor_run[m] = 0;
    }
    else if (!motor_run[m])
    {
        motor_run[m] = 1;
        run_since[m] = millis();
    }
}

/* L298N: EN low -> free running. The L298N "fast stop" (EN
 * high, IN1 = IN2) is deliberately not used: it shorts the
 * motor and shock-loads the gearbox.
 * The IN pins are left at forward on purpose: in phase correct
 * PWM, OCR1x only takes the new value at TOP, so EN can stay
 * high for up to one PWM period (255 us) after set_en(m, 0).
 * Pulling IN low here would brake the motor for that time. */
static void motor_coast(uint8_t m)
{
    motor_run[m] = 0;
    set_en(m, 0);
}

static void motors_off(void)
{
    motor_coast(M1);
    motor_coast(M2);
}


/* ============================================================
 * SAFETY + WAITING
 * ============================================================ */

/* Stops the motors and parks here forever. */
static void halt(PGM_P why)
{
    motors_off();
    print_P(PSTR("\r\n!! STOPPED: "));
    print_P(why);
    print_P(PSTR(" - reset the board to run again.\r\n"));

    while (1)
    {
        /* parked; the 1 Hz LED keeps blinking from the ISR */
    }
}

static uint8_t motor_stalled(uint8_t m);   /* ENCODERS section */

/* Called from every wait loop. While a motor is live, a 10 ms
 * button hold is an emergency stop; a stalled motor is stopped
 * as well. */
static void service(void)
{
    static uint32_t last = 0;
    static uint8_t held = 0;
    uint32_t now;

    now = millis();

    if (now != last)
    {
        last = now;

        if (!button_down())
        {
            held = 0;
        }
        else if (held < 255)
        {
            held++;
        }

        if (held >= 10 && (motor_live[M1] || motor_live[M2]))
        {
            halt(PSTR("button pressed"));
        }

        if (motor_stalled(M1))
        {
            halt(PSTR("M1 stalled (driven, no encoder edges)"));
        }

        if (motor_stalled(M2))
        {
            halt(PSTR("M2 stalled (driven, no encoder edges)"));
        }
    }
}

static void delay_ms(uint32_t ms)
{
    uint32_t start = millis();

    while ((uint32_t)(millis() - start) < ms)
    {
        service();
    }
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


/* ============================================================
 * ENCODERS
 *
 * INT0 / INT1 fire on BOTH edges of channel A:
 *   rises    - the count used for distance (11 per motor rev)
 *   falls    - must track rises (signal is clean)
 *   pos      - quadrature: +1 per A edge when A != B, else -1.
 *              Forward with a working B: pos = rises + falls.
 *              B inverted: pos = -(rises + falls).
 *              B stuck / not wired: pos stays near 0.
 *   glitches - edges closer than ENC_GLITCH_US (noise)
 * ============================================================ */

typedef struct
{
    uint32_t rises;
    uint32_t falls;
    int32_t  pos;
    uint32_t glitches;
    uint32_t last_us;
    uint32_t last_ms;
} enc_t;

static volatile enc_t enc[2];

static inline void enc_edge(volatile enc_t *e, uint8_t a_high, uint8_t b_high)
{
    uint32_t now = micros_noint();

    if (now - e->last_us < ENC_GLITCH_US)
    {
        e->glitches++;
    }

    e->last_us = now;
    e->last_ms = ms_ticks;

    e->pos += (a_high != b_high) ? 1 : -1;

    if (a_high)
    {
        e->rises++;
    }
    else
    {
        e->falls++;
    }
}

ISR(INT0_vect)
{
    uint8_t a = (PIND >> PD2) & 1;
#if ENC_B_ENABLED
    uint8_t b = ((ENC1_B_PIN >> ENC1_B_BIT) & 1) ^ ENC1_B_INVERT;
#else
    uint8_t b = 0;
#endif

    enc_edge(&enc[M1], a, b);
}

ISR(INT1_vect)
{
    uint8_t a = (PIND >> PD3) & 1;
#if ENC_B_ENABLED
    uint8_t b = ((ENC2_B_PIN >> ENC2_B_BIT) & 1) ^ ENC2_B_INVERT;
#else
    uint8_t b = 0;
#endif

    enc_edge(&enc[M2], a, b);
}

static void encoders_init(void)
{
    DDRD &= ~((1 << PD2) | (1 << PD3));
    PORTD |= (1 << PD2) | (1 << PD3);

#if ENC_B_ENABLED
    ENC1_B_DDR &= ~(1 << ENC1_B_BIT);
    ENC1_B_PORT |= (1 << ENC1_B_BIT);
    ENC2_B_DDR &= ~(1 << ENC2_B_BIT);
    ENC2_B_PORT |= (1 << ENC2_B_BIT);
#endif

    /* Any logical change on INT0 and INT1 */
    EICRA = (1 << ISC00) | (1 << ISC10);

    EIFR = (1 << INTF0) | (1 << INTF1);
    EIMSK = (1 << INT0) | (1 << INT1);
}

/* Atomic copy of one encoder's counters. */
static void enc_snap(uint8_t m, enc_t *out)
{
    uint8_t sreg = SREG;

    cli();
    out->rises    = enc[m].rises;
    out->falls    = enc[m].falls;
    out->pos      = enc[m].pos;
    out->glitches = enc[m].glitches;
    out->last_us  = enc[m].last_us;
    out->last_ms  = enc[m].last_ms;
    SREG = sreg;
}

static uint32_t enc_rises(uint8_t m)
{
    enc_t s;

    enc_snap(m, &s);

    return s.rises;
}

static void enc_reset(void)
{
    uint8_t i;
    uint8_t sreg = SREG;

    cli();

    for (i = 0; i < 2; i++)
    {
        enc[i].rises = 0;
        enc[i].falls = 0;
        enc[i].pos = 0;
        enc[i].glitches = 0;
    }

    SREG = sreg;
}

/* Driven at >= STALL_PWM but no edge for STALL_MS, counted from
 * the last edge or from the start of the drive (later wins). */
static uint8_t motor_stalled(uint8_t m)
{
    uint32_t last;
    uint8_t sreg;

    if (!motor_run[m])
    {
        return 0;
    }

    sreg = SREG;
    cli();
    last = enc[m].last_ms;
    SREG = sreg;

    if ((int32_t)(run_since[m] - last) > 0)
    {
        last = run_since[m];
    }

    return (uint32_t)(millis() - last) > STALL_MS;
}


/* ============================================================
 * MEASUREMENTS
 * ============================================================ */

static float gear_ratio = GEAR_RATIO;
static uint8_t ratio_estimated = 0;
static uint8_t start_pwm[2];    /* lowest PWM that turns it */
static float full_cps[2];       /* solo speed at PWM 255 */
static float cps_192[2];        /* solo speed at PWM 192 */
static uint32_t coast_cnt[2];   /* counts to coast to a stop */

/* Rising edges per second over `ms` (motor already running). */
static float rate(uint8_t m, uint32_t ms)
{
    uint32_t c0 = enc_rises(m);

    delay_ms(ms);

    return (float)(enc_rises(m) - c0) * 1000.0f / (float)ms;
}

/* Waits until the wheel stops (or STOP_WAIT_MS). Returns the ms
 * from now to the last edge, or STOP_WAIT_MS if the wheel was
 * still turning; *counts = rises travelled meanwhile. */
static uint32_t time_to_stop(uint8_t m, uint32_t *counts)
{
    uint32_t start = millis();
    uint32_t c0 = enc_rises(m);
    enc_t s;

    do
    {
        service();
        enc_snap(m, &s);
    } while ((uint32_t)(millis() - s.last_ms) < STOP_GAP_MS &&
             (uint32_t)(millis() - start) < STOP_WAIT_MS);

    *counts = s.rises - c0;

    /* Timed out: the last edge is always just before the
     * timeout, so returning it would look like a stop */
    if ((uint32_t)(millis() - s.last_ms) < STOP_GAP_MS)
    {
        return STOP_WAIT_MS;
    }

    if ((int32_t)(s.last_ms - start) > 0)
    {
        return s.last_ms - start;
    }

    return 0;
}

static void print_speed(PGM_P label, float cps)
{
    float motor_rpm = cps * 60.0f / ENC_PPR;

    print_P(label);
    print_f(cps);
    print_P(PSTR(" cnt/s  motor "));
    print_f(motor_rpm);
    print_P(PSTR(" rpm"));

    if (gear_ratio > 0.0f)
    {
        float wheel_rpm = motor_rpm / gear_ratio;

        print_P(PSTR("  wheel "));
        print_f(wheel_rpm);
        print_P(PSTR(" rpm  "));
        print_f(wheel_rpm * PI_F * WHEEL_DIAMETER_MM / 60000.0f);
        print_P(PSTR(" m/s"));

        if (ratio_estimated)
        {
            print_P(PSTR(" (est.)"));
        }
    }

    print_P(PSTR("\r\n"));
}

/* Rising-edge counts -> metres of wheel travel (0 if the gear
 * ratio is not known yet). */
static float counts_to_m(uint32_t counts)
{
    if (gear_ratio <= 0.0f)
    {
        return 0.0f;
    }

    return (float)counts * (PI_F * WHEEL_DIAMETER_MM / 1000.0f) /
           (ENC_PPR * gear_ratio);
}

static uint8_t within_pct(float a, float b, float pct)
{
    float diff = (a > b) ? (a - b) : (b - a);
    float mean = (a + b) * 0.5f;

    return mean > 0.0f && diff * 100.0f <= pct * mean;
}

/* 0. Nothing should count while both motors are off. */
static void test_idle_noise(void)
{
    enc_t a, b;

    print_header(PSTR("ENCODER NOISE (motors off)"));
    print_P(PSTR("Do not touch the wheels for 2 s.\r\n"));

    motors_off();
    delay_ms(300);
    enc_reset();
    delay_ms(2000);

    enc_snap(M1, &a);
    enc_snap(M2, &b);

    print_P(PSTR("  M1 edges: "));
    print_u32(a.rises + a.falls);
    print_P(PSTR("   M2 edges: "));
    print_u32(b.rises + b.falls);
    print_P(PSTR("\r\n"));

    check(PSTR("M1 encoder silent with motors off"), a.rises + a.falls == 0);
    check(PSTR("M2 encoder silent with motors off"), b.rises + b.falls == 0);
}

static void test_one_motor(uint8_t m)
{
    static const uint8_t sweep[4] = { 64, 128, 192, 255 };
    uint8_t other = (m == M1) ? M2 : M1;
    float r[4], win, wmin, wmax, wsum;
    enc_t s0, s1, o0, o1;
    uint32_t coast_ms, coast_counts;
    uint32_t rises, falls, imbalance;
    uint16_t pwm;
    uint8_t i, ok;

    print_header(m == M1 ? PSTR("MOTOR 1 (OUT1/OUT2, ENA IN1 IN2)")
                         : PSTR("MOTOR 2 (OUT3/OUT4, ENB IN3 IN4)"));
    print_P(m == M1 ? PSTR("M1") : PSTR("M2"));
    print_P(PSTR(" wheel should spin FORWARD now.\r\n"));

    enc_snap(m, &s0);
    enc_snap(other, &o0);

    /* --- 1. Spin-up guard: never sit in a stall --- */
    motor_drive(m, 255);
    delay_ms(500);

    if (!check(PSTR("encoder counts when the motor is driven"),
               enc_rises(m) - s0.rises >= 10))
    {
        motors_off();
        print_P(PSTR("  No pulses: check motor power (L298N +12V and +5V logic,\r\n"
                  "  ENA/ENB jumpers removed, common GND) and encoder VCC/GND/A.\r\n"
                  "  Skipping the rest of this motor.\r\n"));
        delay_ms(500);
        return;
    }

    /* --- 2. PWM sweep --- */
    for (i = 0; i < 4; i++)
    {
        motor_drive(m, sweep[i]);
        delay_ms(600);
        r[i] = rate(m, 1000);

        print_P(PSTR("  PWM "));
        print_u32(sweep[i]);
        print_speed(i == 0 ? PSTR(":  ") : PSTR(": "), r[i]);
    }

    full_cps[m] = r[3];
    cps_192[m] = r[2];

    check(PSTR("motor turns at 50% PWM"), r[1] > 0.0f);
    check(PSTR("speed never drops as PWM rises"),
          r[1] >= r[0] && r[2] >= r[1] * 0.98f && r[3] >= r[2] * 0.98f);
    check(PSTR("100% clearly faster than 50%"), r[3] > r[1] * 1.1f);

    if (gear_ratio <= 0.0f && r[3] > 0.0f)
    {
        /* No-load speed scales with the voltage the motor sees */
        const float expect_rpm = MOTOR_NOMINAL_RPM *
            (BATTERY_V - L298N_DROP_V) / MOTOR_RATED_V;

        gear_ratio = (r[3] * 60.0f / ENC_PPR) / expect_rpm;
        ratio_estimated = 1;

        print_P(PSTR("  Rough gear ratio estimate: 1:"));
        print_f(gear_ratio);
        print_P(PSTR(" (assumes "));
        print_f(expect_rpm);
        print_P(PSTR(" wheel rpm at this voltage;\r\n"
                  "  measure it in live mode or read it off the label)\r\n"));
    }

    /* --- 3. Stability at 100%: 10 windows of 100 ms --- */
    wmin = 1e9f;
    wmax = 0.0f;
    wsum = 0.0f;

    enc_snap(m, &s1);

    for (i = 0; i < 10; i++)
    {
        win = rate(m, 100);
        wsum += win;
        if (win < wmin) wmin = win;
        if (win > wmax) wmax = win;
    }

    print_P(PSTR("  100 ms windows: min "));
    print_f(wmin);
    print_P(PSTR("  max "));
    print_f(wmax);
    print_P(PSTR(" cnt/s\r\n"));

    check(PSTR("steady speed at 100% (spread < 15%)"),
          wsum > 0.0f && (wmax - wmin) * 10.0f * 100.0f <= 15.0f * wsum);

#if ENC_B_ENABLED
    {
        /* Quadrature: going forward, every A edge should add +1.
         * |pos| ~ edges: B works. pos ~ 0: B never changes. */
        enc_t s2;
        int32_t dpos;
        uint32_t edges, mag;

        enc_snap(m, &s2);
        dpos = s2.pos - s1.pos;
        edges = (s2.rises - s1.rises) + (s2.falls - s1.falls);
        mag = (dpos < 0) ? (uint32_t)(-dpos) : (uint32_t)dpos;

        print_P(PSTR("  channel B: pos "));
        print_i32(dpos);
        print_P(PSTR(" over "));
        print_u32(edges);
        print_P(PSTR(" A edges\r\n"));

        check(PSTR("channel B: quadrature says FORWARD"),
              edges > 0 && dpos > 0 && mag * 10UL >= edges * 9UL);

        if (edges > 0 && mag * 10UL >= edges * 9UL && dpos < 0)
        {
            print_P(m == M1 ? PSTR("  -> B works but is inverted: set ENC1_B_INVERT 1\r\n")
                            : PSTR("  -> B works but is inverted: set ENC2_B_INVERT 1\r\n"));
        }
        else if (mag * 10UL < edges * 9UL)
        {
            print_P(m == M1 ? PSTR("  -> B not toggling / erratic: check C2 -> A4\r\n")
                            : PSTR("  -> B not toggling / erratic: check C2 -> A5\r\n"));
        }
    }
#endif

    /* --- 4. Signal quality over everything so far --- */
    enc_snap(m, &s1);
    rises = s1.rises - s0.rises;
    falls = s1.falls - s0.falls;
    imbalance = (rises > falls) ? rises - falls : falls - rises;

    print_P(PSTR("  rises "));
    print_u32(rises);
    print_P(PSTR("  falls "));
    print_u32(falls);
    print_P(PSTR("  glitches "));
    print_u32(s1.glitches - s0.glitches);
    print_P(PSTR("\r\n"));

    check(PSTR("rises and falls match (clean signal)"), imbalance <= 2);
    check(PSTR("no glitches (PWM / brush noise on encoder wire)"),
          s1.glitches - s0.glitches == 0);

    /* --- 5. Other encoder must not see this motor --- */
    enc_snap(other, &o1);

    print_P(PSTR("  other encoder edges meanwhile: "));
    print_u32((o1.rises + o1.falls) - (o0.rises + o0.falls));
    print_P(PSTR("\r\n"));

    check(PSTR("other encoder stays still (not swapped / crossed)"),
          (o1.rises + o1.falls) - (o0.rises + o0.falls) < 5);

    /* --- 6. Coast-down from 100% (no braking, ever) --- */
    motor_coast(m);
    coast_ms = time_to_stop(m, &coast_counts);
    coast_cnt[m] = coast_counts;

    print_P(PSTR("  coast to stop: "));
    if (coast_ms >= STOP_WAIT_MS)
    {
        print_P(PSTR("still turning after "));
    }

    print_u32(coast_ms);
    print_P(PSTR(" ms, "));
    print_u32(coast_counts);
    print_P(PSTR(" counts"));

    if (gear_ratio > 0.0f)
    {
        print_P(PSTR(" = "));
        print_f(counts_to_m(coast_counts));
        print_P(ratio_estimated ? PSTR(" m (est.)") : PSTR(" m"));
    }

    print_P(PSTR("\r\n  (wheels in the air: the real car coasts much further;\r\n"
              "  this compares gearbox drag between the two motors)\r\n"));

    check(PSTR("wheel coasts to a stop within 3 s"),
          coast_counts > 0 && coast_ms < STOP_WAIT_MS);

    /* --- 7. Lowest PWM that starts the wheel --- */
    delay_ms(300);
    ok = 0;

    for (pwm = 0; pwm <= 255; pwm += 5)
    {
        uint32_t c0 = enc_rises(m);

        motor_drive(m, (uint8_t)pwm);
        delay_ms(150);

        if (enc_rises(m) - c0 >= 3)
        {
            ok = 1;
            break;
        }
    }

    motor_coast(m);

    if (ok)
    {
        start_pwm[m] = (uint8_t)pwm;
        print_P(PSTR("  wheel starts turning at PWM "));
        print_u32(pwm);
        print_P(PSTR("\r\n"));
    }

    check(PSTR("wheel starts below full power"), ok);

    time_to_stop(m, &coast_counts);
    delay_ms(300);
}

static void test_both(void)
{
    uint32_t l0, r0;
    float lr, rr;

    print_header(PSTR("BOTH MOTORS 100%"));

    /* Never power a motor that failed its spin-up guard */
    if (full_cps[M1] <= 0.0f || full_cps[M2] <= 0.0f)
    {
        print_P(PSTR("  Skipped: a motor failed its own test.\r\n"));
        return;
    }

    motor_drive(M1, 255);
    motor_drive(M2, 255);
    delay_ms(1000);

    l0 = enc_rises(M1);
    r0 = enc_rises(M2);
    delay_ms(3000);
    lr = (float)(enc_rises(M1) - l0) / 3.0f;
    rr = (float)(enc_rises(M2) - r0) / 3.0f;

    motors_off();

    print_P(PSTR("  Each motor at max drive, both running (3 s average):\r\n"));
    print_speed(PSTR("  M1: "), lr);
    print_speed(PSTR("  M2: "), rr);

    print_P(PSTR("  vs. running alone: M1 "));
    print_f(full_cps[M1] > 0.0f ? lr * 100.0f / full_cps[M1] : 0.0f);
    print_P(PSTR(" %,  M2 "));
    print_f(full_cps[M2] > 0.0f ? rr * 100.0f / full_cps[M2] : 0.0f);
    print_P(PSTR(" %\r\n"));

    if (lr > 0.0f && rr > 0.0f)
    {
        uint8_t fast = (lr > rr) ? M1 : M2;
        float hi = (lr > rr) ? lr : rr;
        float lo = (lr > rr) ? rr : lr;
        /* Solo speed the faster motor must drop to */
        float target = full_cps[fast] * lo / hi;
        float span = full_cps[fast] - cps_192[fast];

        print_P(PSTR("  mismatch: "));
        print_f((hi - lo) * 200.0f / (hi + lo));
        print_P(PSTR(" %\r\n"));

        /* Linear between the PWM 192 and 255 sweep points */
        print_P(fast == M1 ? PSTR("  straight-line trim: M1 at PWM ")
                           : PSTR("  straight-line trim: M2 at PWM "));

        if (span > 0.0f && target >= cps_192[fast])
        {
            print_u32((uint32_t)(192.0f + 63.0f * (target - cps_192[fast]) / span + 0.5f));
            print_P(PSTR(", the other at 255 (no-load estimate;\r\n"
                      "  the on-track heading correction does this live)\r\n"));
        }
        else if (span <= 0.0f)
        {
            print_P(PSTR("? (no speed gain from PWM 192 to 255)\r\n"));
        }
        else
        {
            print_P(PSTR("below 192 - motors differ a lot, check them\r\n"));
        }
    }

    /* Wheel-in-air speed: an upper bound for the real run */
    if (gear_ratio > 0.0f && lr + rr > 0.0f)
    {
        /* Metres travelled in one second = m/s */
        float v = counts_to_m((uint32_t)((lr + rr) * 0.5f + 0.5f));

        print_P(PSTR("  no-load top speed "));
        print_f(v);
        print_P(PSTR(" m/s -> 10 m in no less than "));
        print_f(v > 0.0f ? 10.0f / v : 0.0f);
        print_P(ratio_estimated ? PSTR(" s (est. gear ratio)\r\n")
                                : PSTR(" s\r\n"));
    }

    check(PSTR("both run together (no supply sag)"),
          lr > full_cps[M1] * 0.8f && rr > full_cps[M2] * 0.8f);
    check(PSTR("motors within 15% of each other"), within_pct(lr, rr, 15.0f));

    time_to_stop(M1, &l0);
    time_to_stop(M2, &r0);
}

static void live_encoders(void)
{
    uint32_t last_print = 0;
    enc_t a, b;

    print_header(PSTR("LIVE ENCODERS"));
    print_P(PSTR("Mark a wheel, turn it EXACTLY 10 turns by hand,\r\n"
              "then press the button. Expected per wheel turn: 11 x\r\n"
              "gear ratio counts.\r\n"));

    enc_reset();

    while (1)
    {
        delay_ms(1);

        enc_snap(M1, &a);
        enc_snap(M2, &b);

        if (button_pressed())
        {
            uint8_t i;

            for (i = 0; i < 2; i++)
            {
                uint32_t n = (i == M1) ? a.rises : b.rises;

                if (n < 20)
                {
                    continue;
                }

                print_P(i == M1 ? PSTR("-> M1: ") : PSTR("-> M2: "));
                print_f((float)n / 10.0f);
                print_P(PSTR(" counts per wheel turn = gear ratio 1:"));
                print_f((float)n / (10.0f * ENC_PPR));
                print_P(PSTR("\r\n"));
            }

            enc_reset();
            print_P(PSTR("-- reset --\r\n"));
        }

        if ((uint32_t)(millis() - last_print) >= 250)
        {
            last_print = millis();

            print_P(PSTR("M1 "));
            print_u32(a.rises);
#if ENC_B_ENABLED
            print_P(PSTR(" (pos "));
            print_i32(a.pos);
            print_P(PSTR(")"));
#endif
            print_P(PSTR("   M2 "));
            print_u32(b.rises);
#if ENC_B_ENABLED
            print_P(PSTR(" (pos "));
            print_i32(b.pos);
            print_P(PSTR(")"));
#endif
            print_P(PSTR("\r\n"));
        }
    }
}


/* ============================================================
 * MAIN
 * ============================================================ */

int main(void)
{
    motors_init();
    timer2_init();
    uart_init();
    button_init();
    encoders_init();

    sei();

    print_header(PSTR("TACHYON MOTOR + ENCODER TEST"));
    print_P(PSTR("Lift the wheels off the ground, then press the button.\r\n"
              "While a motor runs, press the button to STOP.\r\n"));

    button_wait();

    test_idle_noise();
    test_one_motor(M1);
    test_one_motor(M2);
    test_both();

    print_header(PSTR("SUMMARY"));

    /* 0 = not measured (motor skipped or failed step 7) */
    print_P(PSTR("Lowest PWM that turns the wheel: M1 "));
    print_u32(start_pwm[M1]);
    print_P(PSTR(", M2 "));
    print_u32(start_pwm[M2]);
    print_P(PSTR(" (0 = not measured; no load,\r\n"
              "heading correction should never drop a motor below this)\r\n"));

    print_P(PSTR("Coast-down counts: M1 "));
    print_u32(coast_cnt[M1]);
    print_P(PSTR(", M2 "));
    print_u32(coast_cnt[M2]);
    print_P(PSTR(" (a big difference = one gearbox drags more)\r\n"));

    print_tally();

    live_encoders();

    return 0;
}
