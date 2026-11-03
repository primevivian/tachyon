// EDIT values after encoder test

/*
 * ============================================================
 * DRAGSTER ROBOT - ATmega328P / Arduino Uno
 * ============================================================
 *
 * FEATURES
 *   - TEMT6000 light sensor starts the run.
 *   - Two DC motors controlled by an L298N.
 *   - Two encoders measure wheel movement.
 *   - Differential correction helps the car travel straight.
 *   - Gradual acceleration and distance-based deceleration.
 *   - Stops after approximately 10 metres.
 *
 * HARDWARE
 *   CPU:             ATmega328P, 16 MHz
 *   Motor driver:    L298N
 *   Light sensor:    TEMT6000 on A0
 *   Left encoder A:  D2 / INT0
 *   Right encoder A: D3 / INT1
 *   Left motor PWM:  D5 / OC0B
 *   Right motor PWM: D6 / OC0A
 *
 * ENCODER ASSUMPTION
 *   44 counts per wheel revolution, measured using rising
 *   edges on encoder channel A.
 *
 * IMPORTANT
 *   Confirm that the 44-count measurement corresponds to one
 *   complete revolution of the actual wheel, including any
 *   gearbox between the encoder and the wheel.
 *
 *   This program counts forward encoder pulses. Channel B is
 *   not used for direction detection.
 *
 * ============================================================
 */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <stdint.h>


/* ============================================================
 * 1. USER-CONFIGURABLE PARAMETERS
 *
 * Change these values when your hardware changes.
 * ============================================================
 */

/* Physical wheel diameter in millimetres. */
#define WHEEL_DIAMETER_MM       60UL

/*
 * Measured rising-edge counts per complete wheel revolution.
 *
 * Change this if you change the encoder or decoding method.
 * Do not multiply by four unless the measured count requires
 * x4 quadrature decoding.
 */
#define ENCODER_COUNTS_PER_REV  44UL

/* Desired distance in millimetres. */
#define TARGET_DISTANCE_MM      10000UL

/*
 * TEMT6000 ADC threshold.
 *
 * ADC range: 0-1023.
 *
 * This is only an initial value. Measure the actual ADC
 * readings under ambient light and under the start signal.
 */
#define START_THRESHOLD         600U

/*
 * Number of consecutive samples above the light threshold
 * required to start.
 *
 * This rejects brief spikes and some sensor noise.
 */
#define START_CONFIRM_SAMPLES  5U


/* ============================================================
 * 2. MOTOR CONTROL PARAMETERS
 *
 * PWM range: 0-255.
 *
 * These values require tuning for your motors, battery,
 * chassis, and L298N module.
 * ============================================================
 */

/* Initial cruising PWM target. */
#define CRUISE_PWM              180

/* Minimum PWM used during the deceleration phase. */
#define MIN_DRIVE_PWM           95

/* PWM increase per control-loop update during acceleration. */
#define ACCEL_STEP              2

/*
 * Begin deceleration when this much distance remains.
 *
 * 1500 mm = 1.5 metres.
 *
 * This is an initial estimate, NOT a guaranteed braking
 * distance. Measure the actual stopping distance and adjust.
 */
#define DECEL_DISTANCE_MM       1500UL

/*
 * Proportional straight-line correction.
 *
 * Correction = wheel-count difference * BALANCE_KP
 *
 * A higher value corrects differences more aggressively,
 * but excessive correction can cause oscillation.
 */
#define BALANCE_KP              8

/* Maximum correction applied to either motor's PWM. */
#define MAX_CORRECTION          60


/* ============================================================
 * 3. CONTROL LOOP TIMING
 *
 * Timer1 generates an interrupt every 20 ms.
 *
 * CPU clock:       16 MHz
 * Prescaler:       64
 * Timer frequency: 16 MHz / 64 = 250 kHz
 *
 * Timer counts needed for 20 ms:
 *
 * 250000 * 0.020 = 5000 counts
 *
 * In CTC mode, the counter counts from 0 through OCR1A:
 *
 * OCR1A = 5000 - 1 = 4999
 *
 * Control frequency = 1 / 0.020 = 50 Hz.
 * ============================================================
 */

#define CONTROL_PERIOD_MS      20U

#define TIMER1_COMPARE_VALUE   4999U


/* ============================================================
 * 4. DISTANCE CALCULATIONS
 *
 * Wheel circumference:
 *
 * C = pi * D
 *
 * Distance per encoder count:
 *
 * distance/count = pi * D / counts_per_revolution
 *
 * Target counts:
 *
 * target_counts =
 *     target_distance / distance_per_count
 *
 * With D = 60 mm and 44 counts/revolution:
 *
 * circumference = pi * 60 = 188.50 mm
 *
 * target counts = 10000 * 44 / 188.50
 *               approximately 2334 counts
 *
 * We use integer arithmetic to avoid floating-point
 * calculations during motor control.
 *
 * The value below is the rounded-up circumference in
 * micrometres:
 *
 * circumference_um = pi * diameter_mm * 1000
 *
 * This provides sufficient precision for this application.
 * ============================================================
 */

#define PI_SCALED               3141593UL

#define WHEEL_CIRCUMFERENCE_UM  \
    ((PI_SCALED * WHEEL_DIAMETER_MM) / 1000UL)

/*
 * Distance per count, represented in micrometres:
 *
 * distance_um = circumference_um / counts_per_revolution
 *
 * Target count calculation:
 *
 * target_counts =
 *   target_distance_mm * 1000 * counts_per_revolution
 *   / circumference_um
 *
 * The addition of circumference/2 implements integer
 * rounding to the nearest count.
 */
#define TARGET_COUNTS           \
    (((TARGET_DISTANCE_MM * 1000UL * ENCODER_COUNTS_PER_REV) \
      + (WHEEL_CIRCUMFERENCE_UM / 2UL)) \
      / WHEEL_CIRCUMFERENCE_UM)


/* ============================================================
 * 5. PIN DEFINITIONS
 * ============================================================
 */

/*
 * D5 = PD5 = OC0B
 * D6 = PD6 = OC0A
 *
 * These two pins provide hardware PWM using Timer0.
 */
#define LEFT_PWM_PIN            PD5
#define RIGHT_PWM_PIN           PD6

/*
 * Left motor direction:
 * D7 = PD7
 * D8 = PB0
 *
 * Right motor direction:
 * D9  = PB1
 * D10 = PB2
 */
#define LEFT_DIR1               PD7
#define LEFT_DIR2               PB0

#define RIGHT_DIR1              PB1
#define RIGHT_DIR2              PB2

/*
 * Encoder inputs:
 *
 * D2 = PD2 = INT0
 * D3 = PD3 = INT1
 */
#define LEFT_ENCODER_PIN        PD2
#define RIGHT_ENCODER_PIN       PD3


/* ============================================================
 * 6. GLOBAL VARIABLES
 * ============================================================
 */

/*
 * These counters are modified inside interrupt handlers.
 *
 * volatile prevents the compiler from assuming their values
 * remain unchanged outside the main program.
 *
 * 32-bit counters allow the car to accumulate counts without
 * overflowing after only a few wheel revolutions.
 */
static volatile uint32_t left_encoder_count = 0;
static volatile uint32_t right_encoder_count = 0;

/*
 * Timer1 sets this flag every 20 ms.
 *
 * The main loop performs the control calculations.
 *
 * Keeping the timer ISR short reduces interrupt overhead.
 */
static volatile uint8_t control_tick = 0;


/* ============================================================
 * 7. ROBOT STATES
 * ============================================================
 */

typedef enum
{
    STATE_WAIT_FOR_LIGHT,
    STATE_ACCELERATE,
    STATE_CRUISE,
    STATE_DECELERATE,
    STATE_STOPPED
} robot_state_t;

static robot_state_t state = STATE_WAIT_FOR_LIGHT;


/* ============================================================
 * 8. MOTOR OUTPUT CONTROL
 * ============================================================
 */

/*
 * Set the left motor direction.
 *
 * If the motor turns backward when commanded forward, reverse
 * the motor wires or change these direction signals.
 */
static void left_motor_forward(void)
{
    PORTD |= (1 << LEFT_DIR1);
    PORTB &= ~(1 << LEFT_DIR2);
}


/*
 * Set the right motor direction.
 */
static void right_motor_forward(void)
{
    PORTB |= (1 << RIGHT_DIR1);
    PORTB &= ~(1 << RIGHT_DIR2);
}


/*
 * Configure both motors to travel forward.
 */
static void motors_forward(void)
{
    left_motor_forward();
    right_motor_forward();
}


/*
 * Set the motor PWM values.
 *
 * The input values are limited to 0-255.
 *
 * Timer0 generates the PWM in hardware, so the CPU does not
 * need to generate individual PWM pulses in software.
 */
static void motors_set_pwm(int16_t left, int16_t right)
{
    if (left < 0)
        left = 0;

    if (left > 255)
        left = 255;

    if (right < 0)
        right = 0;

    if (right > 255)
        right = 255;

    OCR0B = (uint8_t)left;
    OCR0A = (uint8_t)right;
}


/*
 * Stop motor drive.
 *
 * Setting both PWM values to zero disables drive through
 * the L298N enable inputs.
 *
 * This is a COAST-style stop, not active electrical braking.
 * The car may continue rolling due to momentum.
 */
static void motors_stop(void)
{
    OCR0A = 0;
    OCR0B = 0;
}


/* ============================================================
 * 9. ADC INITIALIZATION
 * ============================================================
 */

static void adc_init(void)
{
    /*
     * AVcc reference voltage.
     * ADC0 = A0.
     * Right-adjusted result, since the light detector needs
     * the complete 10-bit ADC value.
     */
    ADMUX = (1 << REFS0);

    /*
     * Enable ADC.
     *
     * ADC clock = 16 MHz / 128 = 125 kHz.
     *
     * This is within the usual recommended ADC clock range
     * for full-resolution measurements.
     */
    ADCSRA = (1 << ADEN)  |
             (1 << ADPS2) |
             (1 << ADPS1) |
             (1 << ADPS0);
}


/*
 * Read the TEMT6000 output.
 *
 * The conversion is blocking, but it takes only about
 * 104 microseconds at a 125 kHz ADC clock.
 *
 * The 20 ms control period leaves ample time for this read.
 */
static uint16_t adc_read_light(void)
{
    ADCSRA |= (1 << ADSC);

    while (ADCSRA & (1 << ADSC))
    {
        /* Wait for ADC conversion. */
    }

    return ADC;
}


/* ============================================================
 * 10. PWM INITIALIZATION
 * ============================================================
 */

static void pwm_init(void)
{
    /*
     * Configure D5 and D6 as outputs.
     */
    DDRD |= (1 << LEFT_PWM_PIN) |
            (1 << RIGHT_PWM_PIN);

    /*
     * Fast PWM mode on Timer0.
     *
     * Non-inverting PWM on OC0A and OC0B.
     *
     * COM0A1 = 1: non-inverting PWM on D6.
     * COM0B1 = 1: non-inverting PWM on D5.
     *
     * WGM01 = 1 and WGM00 = 1: Fast PWM, TOP = 255.
     */
    TCCR0A = (1 << COM0A1) |
             (1 << COM0B1) |
             (1 << WGM01)  |
             (1 << WGM00);

    /*
     * Prescaler = 64.
     *
     * PWM frequency:
     *
     * 16 MHz / (64 * 256) = 976.5625 Hz.
     *
     * This is a reasonable starting frequency for the L298N.
     *
     * We use Timer0 for PWM and Timer1 for the control loop,
     * keeping the two timer functions independent.
     */
    TCCR0B = (1 << CS01) | (1 << CS00);

    motors_stop();
}


/* ============================================================
 * 11. MOTOR DIRECTION INITIALIZATION
 * ============================================================
 */

static void motor_direction_init(void)
{
    /*
     * D7 is PD7.
     * D8, D9 and D10 are PB0, PB1 and PB2.
     */
    DDRD |= (1 << LEFT_DIR1);

    DDRB |= (1 << LEFT_DIR2)  |
            (1 << RIGHT_DIR1) |
            (1 << RIGHT_DIR2);

    motors_forward();
}


/* ============================================================
 * 12. ENCODER INITIALIZATION
 * ============================================================
 */

static void encoder_init(void)
{
    /*
     * D2 and D3 as inputs.
     */
    DDRD &= ~((1 << LEFT_ENCODER_PIN) |
              (1 << RIGHT_ENCODER_PIN));

    /*
     * Enable internal pull-ups.
     *
     * Appropriate for some open-collector encoder outputs.
     * Verify the output circuit before relying on these.
     */
    PORTD |= (1 << LEFT_ENCODER_PIN) |
             (1 << RIGHT_ENCODER_PIN);

    /*
     * Trigger INT0 and INT1 on rising edges.
     *
     * This matches the 44-count measurement obtained with
     * the earlier rising-edge encoder test.
     */
    EICRA = (1 << ISC01) | (1 << ISC00) |
            (1 << ISC11) | (1 << ISC10);

    /*
     * Clear pending interrupt flags.
     *
     * AVR interrupt flags are cleared by writing a one.
     */
    EIFR = (1 << INTF0) | (1 << INTF1);

    /*
     * Enable both encoder interrupts.
     */
    EIMSK = (1 << INT0) | (1 << INT1);
}


/* ============================================================
 * 13. TIMER1 CONTROL LOOP
 * ============================================================
 */

static void timer1_init(void)
{
    /*
     * Stop Timer1 while configuring it.
     */
    TCCR1A = 0;
    TCCR1B = 0;

    /*
     * CTC mode, TOP = OCR1A.
     */
    TCCR1B = (1 << WGM12);

    /*
     * 20 ms compare interval.
     */
    OCR1A = TIMER1_COMPARE_VALUE;

    /*
     * Start counter from zero.
     */
    TCNT1 = 0;

    /*
     * Clear pending compare-match flag.
     */
    TIFR1 = (1 << OCF1A);

    /*
     * Enable Compare Match A interrupt.
     */
    TIMSK1 = (1 << OCIE1A);

    /*
     * Start Timer1 with prescaler 64.
     *
     * CS11 = 1 and CS10 = 1.
     */
    TCCR1B |= (1 << CS11) | (1 << CS10);
}


/*
 * Timer1 interrupt: runs every 20 ms.
 *
 * The ISR only sets a flag. Keeping the interrupt short
 * reduces latency for the encoder interrupts.
 */
ISR(TIMER1_COMPA_vect)
{
    control_tick = 1;
}


/* ============================================================
 * 14. ENCODER INTERRUPTS
 * ============================================================
 */

/*
 * Count rising edges on the left encoder A channel.
 *
 * Direction is assumed to be forward.
 */
ISR(INT0_vect)
{
    left_encoder_count++;
}


/*
 * Count rising edges on the right encoder A channel.
 *
 * Direction is assumed to be forward.
 */
ISR(INT1_vect)
{
    right_encoder_count++;
}


/* ============================================================
 * 15. SAFE ENCODER SNAPSHOT
 * ============================================================
 */

/*
 * Copy the 32-bit counters atomically.
 *
 * The ATmega328P is an 8-bit CPU. Without an atomic section,
 * an interrupt could modify a counter between the individual
 * byte reads.
 *
 * Atomic reads are important for reliable distance control.
 */
static void encoder_snapshot(uint32_t *left, uint32_t *right)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        *left = left_encoder_count;
        *right = right_encoder_count;
    }
}


/* ============================================================
 * 16. MAIN CONTROL LOGIC
 * ============================================================
 */

int main(void)
{
    uint16_t light_value = 0;
    uint8_t light_confirm = 0;

    uint32_t left_count = 0;
    uint32_t right_count = 0;

    uint32_t previous_left = 0;
    uint32_t previous_right = 0;

    uint32_t distance_count = 0;
    uint32_t remaining_count = TARGET_COUNTS;

    uint32_t left_delta = 0;
    uint32_t right_delta = 0;

    int32_t speed_error = 0;
    int16_t correction = 0;

    int16_t base_pwm = 0;
    int16_t left_pwm = 0;
    int16_t right_pwm = 0;

    uint32_t decel_start_counts =
        ((DECEL_DISTANCE_MM * ENCODER_COUNTS_PER_REV * 1000UL)
         + (WHEEL_CIRCUMFERENCE_UM / 2UL))
        / WHEEL_CIRCUMFERENCE_UM;

    /*
     * Initialize all peripherals.
     */
    adc_init();
    pwm_init();
    motor_direction_init();
    encoder_init();
    timer1_init();

    /*
     * Motors remain stopped until the start signal.
     */
    motors_stop();

    /*
     * Enable global interrupts.
     */
    sei();


    /* ========================================================
     * MAIN LOOP
     *
     * Control calculations run every 20 ms.
     * Encoder interrupts remain active between updates.
     * ========================================================
     */

    while (1)
    {
        /*
         * Wait for the next Timer1 control tick.
         *
         * The flag is one byte, so its read and write are
         * atomic on the ATmega328P.
         */
        if (!control_tick)
        {
            continue;
        }

        control_tick = 0;


        /* ====================================================
         * STATE 1: WAIT FOR LIGHT
         * ====================================================
         */

        if (state == STATE_WAIT_FOR_LIGHT)
        {
            /*
             * Read TEMT6000.
             *
             * The threshold must be calibrated for the
             * actual sensor module and ambient lighting.
             */
            light_value = adc_read_light();

            if (light_value >= START_THRESHOLD)
            {
                if (light_confirm < START_CONFIRM_SAMPLES)
                {
                    light_confirm++;
                }
            }
            else
            {
                light_confirm = 0;
            }

            /*
             * Require several consecutive samples above
             * the threshold to reject short spikes.
             */
            if (light_confirm >= START_CONFIRM_SAMPLES)
            {
                /*
                 * Snapshot initial encoder counts.
                 */
                encoder_snapshot(&left_count, &right_count);

                previous_left = left_count;
                previous_right = right_count;

                base_pwm = MIN_DRIVE_PWM;

                /*
                 * Begin acceleration.
                 */
                state = STATE_ACCELERATE;
            }

            continue;
        }


        /* ====================================================
         * GET ENCODER COUNTS
         * ====================================================
         */

        encoder_snapshot(&left_count, &right_count);


        /* ====================================================
         * CALCULATE MOVEMENT DURING THE LAST 20 ms
         *
         * Delta counts provide a simple estimate of each
         * wheel's speed.
         *
         * speed is proportional to:
         *
         * delta_counts / control_period
         *
         * Since both wheels use the same period and encoder
         * resolution, comparing delta counts is sufficient
         * for initial straight-line correction.
         * ====================================================
         */

        left_delta = left_count - previous_left;
        right_delta = right_count - previous_right;

        previous_left = left_count;
        previous_right = right_count;


        /*
         * Use the larger wheel count as the distance estimate.
         *
         * This conservative choice helps avoid travelling
         * beyond the target if one wheel moves faster.
         *
         * For accurate straight-line distance, calibrate both
         * wheels and investigate wheel slip.
         */
        distance_count = (left_count > right_count)
                       ? left_count
                       : right_count;


        if (distance_count >= TARGET_COUNTS)
        {
            distance_count = TARGET_COUNTS;
        }

        remaining_count = TARGET_COUNTS - distance_count;


        /* ====================================================
         * STATE 2: ACCELERATE
         * ====================================================
         */

        if (state == STATE_ACCELERATE)
        {
            /*
             * Ramp PWM gradually.
             *
             * This reduces abrupt torque changes and may
             * reduce wheel slip during launch.
             */
            base_pwm += ACCEL_STEP;

            if (base_pwm >= CRUISE_PWM)
            {
                base_pwm = CRUISE_PWM;
                state = STATE_CRUISE;
            }

            /*
             * Switch to deceleration when the remaining
             * distance reaches the configured threshold.
             */
            if (remaining_count <= decel_start_counts)
            {
                state = STATE_DECELERATE;
            }
        }


        /* ====================================================
         * STATE 3: CRUISE
         * ====================================================
         */

        else if (state == STATE_CRUISE)
        {
            base_pwm = CRUISE_PWM;

            if (remaining_count <= decel_start_counts)
            {
                state = STATE_DECELERATE;
            }
        }


        /* ====================================================
         * STATE 4: DECELERATE
         * ====================================================
         */

        if (state == STATE_DECELERATE)
        {
            /*
             * Reduce PWM approximately linearly as the
             * remaining distance decreases.
             *
             * base_pwm =
             * MIN_DRIVE_PWM
             * + (CRUISE_PWM - MIN_DRIVE_PWM)
             *   * remaining_count / decel_start_counts
             *
             * This is a simple distance-based ramp, not a
             * physical braking model.
             *
             * The actual stopping point must be calibrated
             * because motor inertia, speed, battery voltage,
             * and load affect the stopping distance.
             */

            if (decel_start_counts > 0)
            {
                base_pwm =
                    MIN_DRIVE_PWM +
                    ((CRUISE_PWM - MIN_DRIVE_PWM)
                     * (int32_t)remaining_count)
                    / (int32_t)decel_start_counts;
            }

            if (base_pwm < MIN_DRIVE_PWM)
            {
                base_pwm = MIN_DRIVE_PWM;
            }

            if (base_pwm > CRUISE_PWM)
            {
                base_pwm = CRUISE_PWM;
            }
        }


        /* ====================================================
         * STRAIGHT-LINE CORRECTION
         * ====================================================
         */

        /*
         * Positive error means the left wheel advanced more
         * than the right wheel during this control interval.
         *
         * Correction = error * BALANCE_KP
         *
         * If the left wheel is faster:
         *   reduce left PWM;
         *   increase right PWM.
         *
         * The signs assume both encoders count forward motion
         * positively and both motors are wired for forward
         * motion. Verify this on the actual robot.
         */
        speed_error = (int32_t)left_delta -
                      (int32_t)right_delta;

        correction = speed_error * BALANCE_KP;


        /*
         * Limit the correction to avoid excessive changes
         * in motor commands.
         */
        if (correction > MAX_CORRECTION)
        {
            correction = MAX_CORRECTION;
        }

        if (correction < -MAX_CORRECTION)
        {
            correction = -MAX_CORRECTION;
        }


        /* ====================================================
         * APPLY MOTOR PWM
         * ====================================================
         */

        left_pwm = base_pwm - correction;
        right_pwm = base_pwm + correction;

        motors_set_pwm(left_pwm, right_pwm);


        /* ====================================================
         * STATE 5: STOP
         * ====================================================
         */

        if (distance_count >= TARGET_COUNTS)
        {
            motors_stop();

            state = STATE_STOPPED;
        }


        /*
         * STATE_STOPPED intentionally does not restart.
         *
         * Reset the controller to prepare for another run.
         */
        if (state == STATE_STOPPED)
        {
            motors_stop();
        }
    }

    return 0;
}
