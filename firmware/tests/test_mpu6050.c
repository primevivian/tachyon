/*
 * TACHYON - MPU6050 GYRO TEST
 *
 * Prints the turn rate (deg/s) and the heading (deg turned since
 * reset) 10 times a second. Arduino Uno, Serial Monitor 115200.
 *
 * Wiring: VCC -> 5V, GND -> GND, SDA -> A4, SCL -> A5.
 * D7: 1 Hz blinking LED.
 *
 * Keep the car STILL for 1 s after reset (gyro calibration).
 * Turn it 90 deg left: heading ~ +90. Right: negative.
 * Reset button = zero the heading and calibrate again.
 */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>

/* ============================================================
 * SETTINGS
 * ============================================================ */

#define MPU_ADDR     0x68       /* 0x69 if AD0 is tied high */
#define YAW_AXIS     2          /* axis pointing up: 0 = X, 1 = Y, 2 = Z (flat) */
#define GYRO_RANGE   0          /* 0 = 250, 1 = 500, 2 = 1000, 3 = 2000 deg/s */
#define GYRO_FILTER  3          /* low-pass: 0 = off, 3 = 44 Hz, 6 = 5 Hz */
#define I2C_HZ       100000UL   /* I2C clock */

#define SAMPLE_MS    5          /* one gyro reading every 5 ms */
#define PRINT_MS     100        /* one serial line every 100 ms */
#define SETTLE_MS    100        /* wait after wake-up before calibrating */
#define CAL_MS       1000       /* calibration time, car still */
#define LED_MS       500        /* LED toggle time: 500 ms = 1 Hz blink */

/* MPU6050 registers */
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_GYRO_XOUT_H  0x43
#define REG_PWR_MGMT_1   0x6B

/* Worked out from the settings above */
#define LSB_PER_DPS  (131.0f / (1 << GYRO_RANGE))
#define DT           (SAMPLE_MS / 1000.0f)


/* ---- Timer2: 1 ms interrupt, sample flag, 1 Hz LED ---- */

static volatile uint8_t sample_due = 0;

ISR(TIMER2_COMPA_vect)
{
    static uint8_t sample_ms = 0;
    static uint16_t led_ms = 0;

    if (++sample_ms == SAMPLE_MS)
    {
        sample_ms = 0;
        sample_due = 1;
    }

    if (++led_ms == LED_MS)
    {
        led_ms = 0;
        PIND = (1 << PD7);      /* toggle LED */
    }
}

static void wait_sample(void)
{
    while (!sample_due);
    sample_due = 0;
}


/* ---- Serial: printf() goes to the UART ---- */

static int uart_putchar(char c, FILE *f)
{
    (void)f;
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
    return 0;
}

static FILE uart_out = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);


/* ---- I2C ---- */

/* Do one I2C step and wait for it. Returns the status code. */
static uint8_t twi(uint8_t flags)
{
    TWCR = flags | (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    return TWSR & 0xF8;
}

static uint8_t twi_send(uint8_t byte)
{
    TWDR = byte;
    return twi(0);
}

static void twi_stop(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    while (TWCR & (1 << TWSTO));
}

static void mpu_write(uint8_t reg, uint8_t value)
{
    twi(1 << TWSTA);
    twi_send(MPU_ADDR << 1);
    twi_send(reg);
    twi_send(value);
    twi_stop();
}

/* Raw yaw rate from the gyro */
static int16_t gyro_read(void)
{
    uint8_t hi, lo;

    twi(1 << TWSTA);
    twi_send(MPU_ADDR << 1);            /* write: register number */
    twi_send(REG_GYRO_XOUT_H + 2 * YAW_AXIS);
    twi(1 << TWSTA);
    twi_send((MPU_ADDR << 1) | 1);      /* read: 2 bytes */
    twi(1 << TWEA);
    hi = TWDR;
    twi(0);
    lo = TWDR;
    twi_stop();

    return (int16_t)(((uint16_t)hi << 8) | lo);
}


int main(void)
{
    float offset = 0.0f, rate, heading = 0.0f;
    uint16_t i;

    /* LED + Timer2: 16 MHz / 64 / 250 = 1 kHz */
    DDRD |= (1 << PD7);
    TCCR2A = (1 << WGM21);
    OCR2A = 249;
    TCCR2B = (1 << CS22);
    TIMSK2 = (1 << OCIE2A);

    /* UART 115200 (U2X, UBRR 16) */
    UCSR0A = (1 << U2X0);
    UBRR0 = 16;
    UCSR0B = (1 << TXEN0);
    stdout = &uart_out;

    /* I2C. Pull-ups on A4/A5 so a missing module can't hang it. */
    PORTC |= (1 << PC4) | (1 << PC5);
    TWBR = (F_CPU / I2C_HZ - 16) / 2;

    sei();
    printf("\r\n=== TACHYON MPU6050 GYRO TEST ===\r\n");

    /* Is the module there? (0x18 = it answered its address) */
    twi(1 << TWSTA);
    i = twi_send(MPU_ADDR << 1);
    twi_stop();
    if (i != 0x18)
    {
        printf("MPU6050 not found: check VCC, GND, SDA->A4, SCL->A5\r\n");
        while (1);
    }

    mpu_write(REG_PWR_MGMT_1, 0x01);                /* wake up, gyro clock */
    mpu_write(REG_CONFIG, GYRO_FILTER);             /* vibration filter */
    mpu_write(REG_GYRO_CONFIG, GYRO_RANGE << 3);    /* range */

    /* Calibration: the average reading at rest is the zero */
    printf("Calibrating, keep still...\r\n");
    for (i = 0; i < SETTLE_MS / SAMPLE_MS; i++)
        wait_sample();      /* let the gyro settle after wake-up */
    for (i = 0; i < CAL_MS / SAMPLE_MS; i++)
    {
        wait_sample();
        offset += gyro_read();
    }
    offset /= CAL_MS / SAMPLE_MS;

    printf("rate deg/s   heading deg\r\n");

    i = 0;
    while (1)
    {
        wait_sample();

        rate = (gyro_read() - offset) / LSB_PER_DPS;
        heading += rate * DT;           /* add up the turning */

        if (++i == PRINT_MS / SAMPLE_MS)
        {
            i = 0;
            printf("%6d       %6d\r\n", (int)rate, (int)heading);
        }
    }
}
