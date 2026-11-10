/*
 * Tachyon - SD card test
 *
 * MOSI D11, MISO D12, SCK D13, CS A3
 *
 * Writes to the card, the original sector is restored after.
 * Set SD_WRITE_TEST to 0 for read only.
 * Serial: 115200 baud
 */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>


/* ============================================================
 * MINIMUM PROJECT
 * A0 -> D6 PWM, D7 1 Hz
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

/* A0 -> D6 duty cycle */
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

/* Delay that keeps the minimum project running */
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

static void print_hex8(uint8_t v)
{
    const char *digits = "0123456789ABCDEF";

    uart_putc(digits[v >> 4]);
    uart_putc(digits[v & 0x0F]);
}

static void print_header(const char *title)
{
    print_str("\r\n==============================\r\n ");
    print_str(title);
    print_str("\r\n==============================\r\n");
}

static uint8_t tests_passed = 0;
static uint8_t tests_failed = 0;

/* Print PASS/FAIL and count it */
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


#define SD_WRITE_TEST       1

/* Sector for the write test */
#define TEST_SECTOR         1UL

/* Writes for the timing test */
#define WRITE_TIMING_COUNT  20

/* SD commands */
#define CMD0    0       /* GO_IDLE_STATE           */
#define CMD8    8       /* SEND_IF_COND            */
#define CMD9    9       /* SEND_CSD                */
#define CMD16   16      /* SET_BLOCKLEN            */
#define CMD17   17      /* READ_SINGLE_BLOCK       */
#define CMD24   24      /* WRITE_BLOCK             */
#define CMD55   55      /* APP_CMD                 */
#define CMD58   58      /* READ_OCR                */
#define ACMD41  41      /* SD_SEND_OP_COND         */

#define TOKEN_START_BLOCK   0xFE


/* ============================================================
 * SPI
 * ============================================================ */

static void spi_init(void)
{
    /* MOSI, SCK, SS out, MISO in */
    DDRB |= (1 << PB3) | (1 << PB5) | (1 << PB2);
    DDRB &= ~(1 << PB4);

    /* SS (D10) is ENB on the robot, keep it low */
    PORTB &= ~(1 << PB2);

    /* CS on A3, idle high */
    DDRC |= (1 << PC3);
    PORTC |= (1 << PC3);
}

/* 125 kHz for init */
static void spi_slow(void)
{
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR1) | (1 << SPR0);
    SPSR = 0;
}

/* 4 MHz after init */
static void spi_fast(void)
{
    SPCR = (1 << SPE) | (1 << MSTR);
    SPSR = 0;
}

static uint8_t spi(uint8_t out)
{
    SPDR = out;

    while (!(SPSR & (1 << SPIF)))
    {
        /* wait */
    }

    return SPDR;
}


/* ============================================================
 * SD CARD
 * ============================================================ */

static uint8_t sd_v2 = 0;           /* SD version 2 or later   */
static uint8_t sd_block_addr = 0;   /* SDHC/SDXC: sector addresses */
static uint8_t sector[512];

static void sd_select(void)
{
    PORTC &= ~(1 << PC3);
}

static void sd_deselect(void)
{
    PORTC |= (1 << PC3);

    /* Extra clock so the card releases MISO */
    spi(0xFF);
}

/* Wait while the card is busy */
static uint8_t sd_wait_ready(uint16_t timeout_ms)
{
    uint32_t start = millis();

    while (spi(0xFF) != 0xFF)
    {
        if (millis() - start > timeout_ms)
        {
            return 0;
        }
    }

    return 1;
}

/* Returns R1, leaves CS low */
static uint8_t sd_command(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    uint8_t r = 0xFF;
    uint8_t i;

    sd_deselect();
    sd_select();

    if (cmd != CMD0)
    {
        sd_wait_ready(500);
    }

    spi((uint8_t)(0x40 | cmd));
    spi((uint8_t)(arg >> 24));
    spi((uint8_t)(arg >> 16));
    spi((uint8_t)(arg >> 8));
    spi((uint8_t)arg);
    spi(crc);

    /* R1 arrives within 8 bytes; its top bit is 0 */
    for (i = 0; i < 10; i++)
    {
        r = spi(0xFF);

        if (!(r & 0x80))
        {
            break;
        }
    }

    return r;
}

static uint8_t sd_app_command(uint8_t cmd, uint32_t arg)
{
    sd_command(CMD55, 0, 0x01);

    return sd_command(cmd, arg, 0x01);
}

static uint8_t sd_wait_token(void)
{
    uint32_t start = millis();
    uint8_t t;

    do
    {
        t = spi(0xFF);

        if (t != 0xFF)
        {
            return t;
        }
    } while (millis() - start < 300);

    return 0xFF;
}

static uint8_t sd_init(void)
{
    uint8_t r = 0xFF;
    uint8_t ocr[4];
    uint8_t i;
    uint32_t start;

    spi_slow();

    /* At least 74 clocks with CS high to enter SPI mode */
    PORTC |= (1 << PC3);

    for (i = 0; i < 10; i++)
    {
        spi(0xFF);
    }

    /* CMD0: reset into idle state (R1 = 0x01) */
    for (i = 0; i < 10 && r != 0x01; i++)
    {
        r = sd_command(CMD0, 0, 0x95);
    }

    print_str("  CMD0 -> 0x");
    print_hex8(r);
    print_str("\r\n");

    if (!check("card responds to reset (CMD0)", r == 0x01))
    {
        sd_deselect();
        print_str("  -> check wiring, CS on A3, card inserted, 5V power\r\n");
        return 0;
    }

    /* CMD8: only SD v2+ cards understand it */
    r = sd_command(CMD8, 0x1AA, 0x87);

    if (r == 0x01)
    {
        for (i = 0; i < 4; i++)
        {
            ocr[i] = spi(0xFF);
        }

        if (ocr[2] != 0x01 || ocr[3] != 0xAA)
        {
            sd_deselect();
            check("card accepts 3.3 V (CMD8 echo)", 0);
            return 0;
        }

        sd_v2 = 1;
    }

    /* ACMD41: start initialisation, repeat until ready */
    start = millis();

    do
    {
        r = sd_app_command(ACMD41, sd_v2 ? 0x40000000UL : 0);
    } while (r != 0x00 && millis() - start < 1500);

    if (!check("card leaves idle state (ACMD41)", r == 0x00))
    {
        sd_deselect();
        return 0;
    }

    if (sd_v2)
    {
        /* CMD58: OCR bit 30 (CCS) = high capacity, sector addressing */
        if (sd_command(CMD58, 0, 0x01) == 0x00)
        {
            for (i = 0; i < 4; i++)
            {
                ocr[i] = spi(0xFF);
            }

            sd_block_addr = (ocr[0] & 0x40) ? 1 : 0;
        }
    }

    if (!sd_block_addr)
    {
        /* SDSC: 512 byte blocks */
        sd_command(CMD16, 512, 0x01);
    }

    sd_deselect();
    spi_fast();

    print_str("  type: ");
    print_str(sd_v2 ? "SD v2, " : "SD v1, ");
    print_str(sd_block_addr ? "SDHC/SDXC\r\n" : "SDSC\r\n");

    return 1;
}

static uint32_t sd_address(uint32_t sector_number)
{
    return sd_block_addr ? sector_number : sector_number * 512UL;
}

static uint8_t sd_read(uint32_t sector_number, uint8_t *buf)
{
    uint16_t i;

    if (sd_command(CMD17, sd_address(sector_number), 0x01) != 0x00 ||
        sd_wait_token() != TOKEN_START_BLOCK)
    {
        sd_deselect();
        return 0;
    }

    for (i = 0; i < 512; i++)
    {
        buf[i] = spi(0xFF);
    }

    /* Discard CRC */
    spi(0xFF);
    spi(0xFF);

    sd_deselect();

    return 1;
}

/* Test pattern */
static uint8_t pattern(uint16_t i, uint8_t seed)
{
    return (uint8_t)((i ^ (i >> 8)) + seed * 37U + 0x5A);
}

/* buf = 0 writes the test pattern */
static uint8_t sd_write(uint32_t sector_number, const uint8_t *buf, uint8_t seed)
{
    uint16_t i;
    uint8_t response;

    if (sd_command(CMD24, sd_address(sector_number), 0x01) != 0x00)
    {
        sd_deselect();
        return 0;
    }

    spi(0xFF);
    spi(TOKEN_START_BLOCK);

    for (i = 0; i < 512; i++)
    {
        spi(buf ? buf[i] : pattern(i, seed));
    }

    /* Dummy CRC */
    spi(0xFF);
    spi(0xFF);

    /* Data response: xxx00101 = accepted */
    response = spi(0xFF) & 0x1F;

    /* Wait for the card to finish programming */
    if (response != 0x05 || !sd_wait_ready(1000))
    {
        sd_deselect();
        return 0;
    }

    sd_deselect();

    return 1;
}

/* Count bytes that don't match the pattern */
static uint16_t sd_verify_pattern(uint32_t sector_number, uint8_t seed)
{
    uint16_t i;
    uint16_t errors = 0;

    if (sd_command(CMD17, sd_address(sector_number), 0x01) != 0x00 ||
        sd_wait_token() != TOKEN_START_BLOCK)
    {
        sd_deselect();
        return 512;
    }

    for (i = 0; i < 512; i++)
    {
        if (spi(0xFF) != pattern(i, seed))
        {
            errors++;
        }
    }

    spi(0xFF);
    spi(0xFF);

    sd_deselect();

    return errors;
}


/* ============================================================
 * TESTS
 * ============================================================ */

static void test_info(void)
{
    uint8_t csd[16];
    uint8_t i;
    uint8_t ok;
    uint32_t mb = 0;

    print_header("2. CARD INFO");

    ok = sd_command(CMD9, 0, 0x01) == 0x00 &&
         sd_wait_token() == TOKEN_START_BLOCK;

    if (ok)
    {
        for (i = 0; i < 16; i++)
        {
            csd[i] = spi(0xFF);
        }

        spi(0xFF);
        spi(0xFF);
    }

    sd_deselect();

    if (!check("CSD register readable", ok))
    {
        return;
    }

    if ((csd[0] >> 6) == 1)
    {
        /* CSD v2: capacity = (C_SIZE + 1) * 512 KiB */
        uint32_t c_size = ((uint32_t)(csd[7] & 0x3F) << 16) |
                          ((uint32_t)csd[8] << 8) | csd[9];

        mb = (c_size + 1) / 2;
    }
    else
    {
        /* CSD v1: (C_SIZE + 1) * 2^(C_SIZE_MULT + 2) * 2^READ_BL_LEN */
        uint32_t c_size = ((uint32_t)(csd[6] & 0x03) << 10) |
                          ((uint32_t)csd[7] << 2) | (csd[8] >> 6);
        uint8_t mult = (uint8_t)(((csd[9] & 0x03) << 1) | (csd[10] >> 7));
        uint8_t bl_len = csd[5] & 0x0F;

        mb = ((c_size + 1) << (mult + 2 + bl_len)) >> 20;
    }

    print_str("  capacity: ");
    print_u32(mb);
    print_str(" MB\r\n");

    check("capacity looks valid", mb > 0);
}

static void test_read(void)
{
    uint8_t type;

    print_header("3. READ SECTOR 0");

    if (!check("sector 0 read", sd_read(0, sector)))
    {
        return;
    }

    check("boot signature 55 AA present (card is formatted)",
          sector[510] == 0x55 && sector[511] == 0xAA);

    /* First partition entry type byte (MBR offset 0x1C2) */
    type = sector[0x1C2];

    print_str("  first partition type 0x");
    print_hex8(type);

    if (type == 0x0B || type == 0x0C)      print_str(" (FAT32)");
    else if (type == 0x06 || type == 0x0E) print_str(" (FAT16)");
    else if (type == 0x07)                 print_str(" (exFAT/NTFS)");
    else if (type == 0x00)                 print_str(" (none - no partition table?)");

    print_str("\r\n");
}

static void test_write(void)
{
    uint16_t errors;
    uint8_t ok;

    print_header("4. WRITE / VERIFY / RESTORE");

    print_str("  sector ");
    print_u32(TEST_SECTOR);
    print_str("\r\n");

    if (!check("original sector saved", sd_read(TEST_SECTOR, sector)))
    {
        return;
    }

    ok = sd_write(TEST_SECTOR, 0, 1);
    check("test pattern written", ok);

    errors = sd_verify_pattern(TEST_SECTOR, 1);

    print_str("  ");
    print_u32(errors);
    print_str(" bytes differ\r\n");

    check("data read back matches", ok && errors == 0);

    if (!check("original sector restored", sd_write(TEST_SECTOR, sector, 0)))
    {
        print_str("  !! RESTORE FAILED: sector ");
        print_u32(TEST_SECTOR);
        print_str(" may be corrupted - reformat the card !!\r\n");
    }
}

static void test_timing(void)
{
    uint32_t start;
    uint32_t t;
    uint32_t worst = 0;
    uint32_t total = 0;
    uint8_t i;
    uint8_t ok = 1;

    print_header("5. TIMING");

    /* 100 sectors = 50 KB */
    start = millis();

    for (i = 0; i < 100 && ok; i++)
    {
        ok = sd_read(i, sector);
    }

    t = millis() - start;

    print_str("  read: ");
    print_u32(t > 0 ? 50000UL / t : 0);
    print_str(" KB/s\r\n");

    check("100 sectors read", ok);

#if SD_WRITE_TEST
    /* Write time */
    if (!sd_read(TEST_SECTOR, sector))
    {
        return;
    }

    for (i = 0; i < WRITE_TIMING_COUNT && ok; i++)
    {
        start = millis();
        ok = sd_write(TEST_SECTOR, 0, (uint8_t)(i + 2));
        t = millis() - start;

        total += t;

        if (t > worst)
        {
            worst = t;
        }
    }

    if (!sd_write(TEST_SECTOR, sector, 0))
    {
        print_str("  !! RESTORE FAILED: reformat the card !!\r\n");
    }

    check("repeated writes succeed", ok);

    print_str("  write: average ");
    print_u32(total / WRITE_TIMING_COUNT);
    print_str(" ms, worst ");
    print_u32(worst);
    print_str(" ms per 512-byte sector\r\n");

    /* Slow writes can't go in the 2 ms race loop */
    if (worst > 2)
    {
        print_str("  note: worst case > 2 ms control period, so log to\r\n"
                  "  RAM during the run and write to the card afterwards.\r\n");
    }
#else
    (void)worst;
    (void)total;
#endif
}


/* ============================================================
 * MAIN
 * ============================================================ */

int main(void)
{
    minimum_project_init();
    uart_init();
    spi_init();

    sei();

    delay_ms(100);

    print_header("TACHYON SD CARD TEST");

    print_header("1. INITIALISE");

    if (sd_init())
    {
        test_info();
        test_read();

#if SD_WRITE_TEST
        test_write();
#else
        print_str("\r\n(write test disabled: SD_WRITE_TEST = 0)\r\n");
#endif

        test_timing();
    }

    print_tally();

    /* Keep the minimum project running */
    while (1)
    {
        minimum_project_update();
    }

    return 0;
}
