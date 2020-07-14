/*
 * PillboxP4.c
 *
 * Created: 07.10.2016 14:08:21
 */

#define LED2_SHOWS_MCU_RUN (0) // Use LED2 to show when the MCU thinks it is in "RUN" mode

#define F_CPU (32768UL) // When running on ULP oscillator

#define F_XOSC32K (32768UL)
#define DESIRED_RTC_TICK_S (3)                        // Desired duration of an RTC tick in seconds
                                                      // 3 seconds is 3*32768 = 98304 cycles of the XOSC32K, so
                                                      // we can measure the RTC tick duration with about
                                                      // (1/98304) or 10 ppm resolution
#define RTC_PER_VALUE (DESIRED_RTC_TICK_S * 1024 - 1) // RTC will run on an (approximate) 1024 Hz clock,
                                                      // so this will give a tick roughly every 3 seconds
#define N_TICKS_BETWEEN_MEAS (300)                    // Number of ticks between measurements
// With 3 seconds per tick, this is 3*300 = 900 seconds or 15 minutes
#define N_TICKS_FOR_MEAS (2)          // Number of ticks to use for a measurement
#define N_31US_PER_DAY (2831155200UL) // Number of 32768 Hz clock cycles per day
#define N_MINS_PER_DAY (1440)
#define N_MINS_PER_HOUR (60)
#define N_SECS_PER_MIN (60)
#define SHIFT_32768 (15) // Shift that is equivalent to mul or div by 32768
#define N_ALARMS (4)     // Number of alarms

#define B1_BP (4)      // Least significant button (B1) is connected to PA4
#define B2_BP (3)      // Next button (B2) is connected to PA3
#define B3_BP (8 + 1)  // Next button (B3) is connected to PB1
#define B4_BP (5)      // Most significant button (B4) is connected to PA5
#define SW_BP (7)      // Slide switch is connected to PA7
#define OLEDRST_BP (6) // OLED reset pin is connected to PA6

#define ALARM_INC_MASK (1 << B1_BP)
#define HOUR_INC_MASK (1 << B2_BP)
#define MIN_INC_MASK (1 << B3_BP)
#define ALARM_ONOFF_MASK (1 << B4_BP)
#define V_OLED_OFF_MASK (1 << SW_BP)

#define BYTES_PER_CHAR (6) // Number of bytes in the lookup table for a character to be displayed

#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>

volatile uint32_t n_31us        = 943718400UL; // Should correspond to 08:00 as default start time
volatile uint32_t secs          = 28800;       // Should correspond to 08:00 as default start time
volatile uint16_t totmins       = 480;         // Should correspond to 08:00 as default start time
volatile uint16_t old_mins      = 0;
volatile uint8_t  disp_mins     = 0; // minutes to be displayed
volatile uint8_t  tdh           = 0; // tens digit hours
volatile uint8_t  tdm           = 0; // tens digit minutes
volatile uint8_t  disp_hours    = 0; // hours to be displayed
volatile uint8_t  setting_index = 0;
volatile uint16_t old_button_status;
volatile uint16_t new_button_status;
volatile uint16_t button_pressed;

volatile uint16_t alarm_times_totmins[] = {482, (N_MINS_PER_DAY + 1), (N_MINS_PER_DAY + 1), (N_MINS_PER_DAY + 1)};
// alarm at 8:02 with other alarms disabled
volatile uint8_t  alarm_times_hours[] = {8, 0, 0, 0};
volatile uint8_t  alarm_times_mins[]  = {2, 0, 0, 0};
volatile uint16_t alarms_active       = 0; // Bit n high indicates that alarm n should be or is active

volatile uint8_t  now_in_adjust_mode   = 0;
volatile uint8_t  now_in_meas_mode     = 0;
volatile uint16_t cnt_in_meas_mode     = 0;
volatile uint16_t cnt_not_in_meas_mode = 0;
volatile uint32_t meas_value           = (F_XOSC32K * DESIRED_RTC_TICK_S); // Start with estimated measurement

volatile uint16_t adc_value = 0;

const uint8_t PROGMEM oled_inita[] = {
    0x00, 0xAE, // Write command, display off,
    0xD5, 0x80, //set display clock divide ratio/oscillator frequency,
    0xA8, 0x3F, //set multiplex ratio, (ultrasonic uses 0x1F as second byte)
    0xD3, 0x00, // Set display offset,
    0x40,       //set display start line,
    0xA1,       //set segment re-map,
    0xC8,       //set COM output scan direction,
    0xDA, 0x12, // Set COM pins hardware configuration, (ultrasonic uses 0x02 as second byte, OLED data sheet says 0x12)
    0x81, 0xCF, // set contrast control, (ultrasonic uses 0x8F as second byte)
    0xD9, 0xF1, // set pre-charge period,
    0xDB, 0x30, //set VCOMH deselect level, (ultrasonic uses 0x40 as second byte)
    0xA4,       // Set entire display on/off,
    0xA6,       //set normal/inverse display,
    0x8D, 0x14, //set charge pump,
    0xAF,       // set display on
    0x20, 0x01}; //set memory addressing mode to vertical addressing mode

// const uint8_t PROGMEM oled_initb[] = {0x00,0xA7}; // inverse display
// const uint8_t PROGMEM oled_initc[] = {0x00,0xA6}; // normal display

// const uint8_t PROGMEM oled_test[] = {0x40,0x00,0x7e,0x09,0x09,0x09,0x7e,
//	0x00,0x7f,0x40,0x40,0x40,0x40,
//	0x00,0x7e,0x09,0x09,0x09,0x7e,
//	0x00,0x7f,0x09,0x19,0x29,0x46,
//	0x00,0x7f,0x02,0x0c,0x02,0x7f
//	}; // Try to generate a small ALARM

/* const uint8_t PROGMEM all_char[] = {
    0x00,0x00,0x00,0x00,0x00,0x00,   //   0x20 32
    0x00,0x00,0x00,0x6f,0x00,0x00,   // ! 0x21 33
    0x00,0x00,0x07,0x00,0x07,0x00,   // " 0x22 34
    0x00,0x14,0x7f,0x14,0x7f,0x14,   // # 0x23 35
    0x00,0x00,0x07,0x04,0x1e,0x00,   // $ 0x24 36
    0x00,0x23,0x13,0x08,0x64,0x62,   // % 0x25 37
    0x00,0x36,0x49,0x56,0x20,0x50,   // & 0x26 38
    0x00,0x00,0x00,0x07,0x00,0x00,   // ' 0x27 39
    0x00,0x00,0x1c,0x22,0x41,0x00,   // ( 0x28 40
    0x00,0x00,0x41,0x22,0x1c,0x00,   // ) 0x29 41
    0x00,0x14,0x08,0x3e,0x08,0x14,   // * 0x2a 42
    0x00,0x08,0x08,0x3e,0x08,0x08,   // + 0x2b 43
    0x00,0x00,0x50,0x30,0x00,0x00,   // , 0x2c 44
    0x00,0x08,0x08,0x08,0x08,0x08,   // - 0x2d 45
    0x00,0x00,0x60,0x60,0x00,0x00,   // . 0x2e 46
    0x00,0x20,0x10,0x08,0x04,0x02,   // / 0x2f 47
    0x00,0x3e,0x51,0x49,0x45,0x3e,   // 0 0x30 48
    0x00,0x00,0x42,0x7f,0x40,0x00,   // 1 0x31 49
    0x00,0x42,0x61,0x51,0x49,0x46,   // 2 0x32 50
    0x00,0x21,0x41,0x45,0x4b,0x31,   // 3 0x33 51
    0x00,0x18,0x14,0x12,0x7f,0x10,   // 4 0x34 52
    0x00,0x27,0x45,0x45,0x45,0x39,   // 5 0x35 53
    0x00,0x3c,0x4a,0x49,0x49,0x30,   // 6 0x36 54
    0x00,0x01,0x71,0x09,0x05,0x03,   // 7 0x37 55
    0x00,0x36,0x49,0x49,0x49,0x36,   // 8 0x38 56
    0x00,0x06,0x49,0x49,0x29,0x1e,   // 9 0x39 57
    0x00,0x00,0x36,0x36,0x00,0x00,   // : 0x3a 58
    0x00,0x00,0x56,0x36,0x00,0x00,   // ; 0x3b 59
    0x00,0x08,0x14,0x22,0x41,0x00,   // < 0x3c 60
    0x00,0x14,0x14,0x14,0x14,0x14,   // = 0x3d 61
    0x00,0x00,0x41,0x22,0x14,0x08,   // > 0x3e 62
    0x00,0x02,0x01,0x51,0x09,0x06,   // ? 0x3f 63
    0x00,0x3e,0x41,0x5d,0x49,0x4e,   // @ 0x40 64
    0x00,0x7e,0x09,0x09,0x09,0x7e,   // A 0x41 65
    0x00,0x7f,0x49,0x49,0x49,0x36,   // B 0x42 66
    0x00,0x3e,0x41,0x41,0x41,0x22,   // C 0x43 67
    0x00,0x7f,0x41,0x41,0x41,0x3e,   // D 0x44 68
    0x00,0x7f,0x49,0x49,0x49,0x41,   // E 0x45 69
    0x00,0x7f,0x09,0x09,0x09,0x01,   // F 0x46 70
    0x00,0x3e,0x41,0x49,0x49,0x7a,   // G 0x47 71
    0x00,0x7f,0x08,0x08,0x08,0x7f,   // H 0x48 72
    0x00,0x00,0x41,0x7f,0x41,0x00,   // I 0x49 73
    0x00,0x20,0x40,0x41,0x3f,0x01,   // J 0x4a 74
    0x00,0x7f,0x08,0x14,0x22,0x41,   // K 0x4b 75
    0x00,0x7f,0x40,0x40,0x40,0x40,   // L 0x4c 76
    0x00,0x7f,0x02,0x0c,0x02,0x7f,   // M 0x4d 77
    0x00,0x7f,0x04,0x08,0x10,0x7f,   // N 0x4e 78
    0x00,0x3e,0x41,0x41,0x41,0x3e,   // O 0x4f 79
    0x00,0x7f,0x09,0x09,0x09,0x06,   // P 0x50 80
    0x00,0x3e,0x41,0x51,0x21,0x5e,   // Q 0x51 81
    0x00,0x7f,0x09,0x19,0x29,0x46,   // R 0x52 82
    0x00,0x46,0x49,0x49,0x49,0x31,   // S 0x53 83
    0x00,0x01,0x01,0x7f,0x01,0x01,   // T 0x54 84
    0x00,0x3f,0x40,0x40,0x40,0x3f,   // U 0x55 85
    0x00,0x0f,0x30,0x40,0x30,0x0f,   // V 0x56 86
    0x00,0x3f,0x40,0x30,0x40,0x3f,   // W 0x57 87
    0x00,0x63,0x14,0x08,0x14,0x63,   // X 0x58 88
    0x00,0x07,0x08,0x70,0x08,0x07,   // Y 0x59 89
    0x00,0x61,0x51,0x49,0x45,0x43,   // Z 0x5a 90
    0x00,0x3c,0x4a,0x49,0x29,0x1e,   // [ 0x5b 91
    0x00,0x02,0x04,0x08,0x10,0x20,   // \ 0x5c 92
    0x00,0x00,0x41,0x7f,0x00,0x00,   // ] 0x5d 93
    0x00,0x04,0x02,0x01,0x02,0x04,   // ^ 0x5e 94
    0x00,0x40,0x40,0x40,0x40,0x40   // _ 0x5f 95
};
*/

const uint8_t PROGMEM text_digits[] = {
    0x00, 0x3e, 0x51, 0x49, 0x45, 0x3e, // 0 0x30 48
    0x00, 0x00, 0x42, 0x7f, 0x40, 0x00, // 1 0x31 49
    0x00, 0x42, 0x61, 0x51, 0x49, 0x46, // 2 0x32 50
    0x00, 0x21, 0x41, 0x45, 0x4b, 0x31, // 3 0x33 51
    0x00, 0x18, 0x14, 0x12, 0x7f, 0x10, // 4 0x34 52
    0x00, 0x27, 0x45, 0x45, 0x45, 0x39, // 5 0x35 53
    0x00, 0x3c, 0x4a, 0x49, 0x49, 0x30, // 6 0x36 54
    0x00, 0x01, 0x71, 0x09, 0x05, 0x03, // 7 0x37 55
    0x00, 0x36, 0x49, 0x49, 0x49, 0x36, // 8 0x38 56
    0x00, 0x06, 0x49, 0x49, 0x29, 0x1e  // 9 0x39 57
};
const uint8_t PROGMEM text_alarm[] = {
    0x00, 0x7e, 0x09, 0x09, 0x09, 0x7e, // A 0x41 65
    0x00, 0x7f, 0x40, 0x40, 0x40, 0x40, // L 0x4c 76
    0x00, 0x7e, 0x09, 0x09, 0x09, 0x7e, // A 0x41 65
    0x00, 0x7f, 0x09, 0x19, 0x29, 0x46, // R 0x52 82
    0x00, 0x7f, 0x02, 0x0c, 0x02, 0x7f  // M 0x4d 77
};
const uint8_t PROGMEM text_time[] = {
    0x00, 0x01, 0x01, 0x7f, 0x01, 0x01, // T 0x54 84
    0x00, 0x00, 0x41, 0x7f, 0x41, 0x00, // I 0x49 73
    0x00, 0x7f, 0x02, 0x0c, 0x02, 0x7f, // M 0x4d 77
    0x00, 0x7f, 0x49, 0x49, 0x49, 0x41, // E 0x45 69
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // blank
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00  // blank
};
const uint8_t PROGMEM text_off[] = {
    0x00,
    0x3e,
    0x41,
    0x41,
    0x41,
    0x3e, // O 0x4f 79
    0x00,
    0x7f,
    0x09,
    0x09,
    0x09,
    0x01, // F 0x46 70
    0x00,
    0x7f,
    0x09,
    0x09,
    0x09,
    0x01 // F 0x46 70
};
const uint8_t PROGMEM text_on[] = {
    0x00,
    0x3e,
    0x41,
    0x41,
    0x41,
    0x3e, // O 0x4f 79
    0x00,
    0x7f,
    0x04,
    0x08,
    0x10,
    0x7f, // N 0x4e 78
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00 // space
};
const uint8_t PROGMEM text_space[] = {
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00, // space
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00, // space
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00 // space
};

const uint8_t PROGMEM text_b1arrow[] = {0x00,
                                        0x7f,
                                        0x49,
                                        0x49,
                                        0x49,
                                        0x36, // B 0x42 66
                                        0x00,
                                        0x00,
                                        0x42,
                                        0x7f,
                                        0x40,
                                        0x00, // 1 0x31 49
                                        0x00,
                                        0x24,
                                        0x22,
                                        0x3F,
                                        0x02,
                                        0x04};
const uint8_t PROGMEM text_b2arrow[] = {0x00, 0x7f, 0x49, 0x49, 0x49, 0x36, // B 0x42 66
                                        0x00, 0x42, 0x61, 0x51, 0x49, 0x46, // 2 0x32 50
                                        0x00, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x24, 0x22, 0x3F, 0x02, 0x04};
const uint8_t PROGMEM text_arrowb3[] = {
    0x00, 0x04, 0x02, 0x3F, 0x22, 0x24, 0x20, 0x20, 0x20,
    0x20, 0x20, 0x20, 0x00, 0x7f, 0x49, 0x49, 0x49, 0x36, // B 0x42 66
    0x00, 0x21, 0x41, 0x45, 0x4b, 0x31,                   // 3 0x33 51
};
const uint8_t PROGMEM text_arrowb4[] = {
    0x00,
    0x04,
    0x02,
    0x3F,
    0x22,
    0x24,
    0x00,
    0x7f,
    0x49,
    0x49,
    0x49,
    0x36, // B 0x42 66
    0x00,
    0x18,
    0x14,
    0x12,
    0x7f,
    0x10, // 4 0x34 52
};
const uint8_t PROGMEM text_colon[] = {
    0x00, 0x00, 0x36, 0x36, 0x00, 0x00 // : 0x3a 58
};
const uint8_t PROGMEM text_asterisk[] = {
    0x00,
    0x14,
    0x08,
    0x3e,
    0x08,
    0x14, // * 0x2a 42 need
    0x00,
    0x14,
    0x08,
    0x3e,
    0x08,
    0x14 // * 0x2a 42 need
};
const uint8_t PROGMEM text_uparrow[] = {0x00, 0x04, 0x02, 0x7F, 0x02, 0x04};

uint16_t button_status()
{
	uint16_t temp_a;
	temp_a = (PORTB.IN << 8) | PORTA.IN;
	return (~temp_a); // Invert all the bits, so pressing a button (grounding the I/O line) gives logic 1
}

void flash_led(uint16_t pbp) // Used for flashing a specified LED
{
	PORTB.DIRSET = pbp >> 8; // Set PBn as output
	PORTB.OUTCLR = pbp >> 8; // Turn ON PBn Alarm LED by grounding output
	PORTA.DIRSET = pbp;      // Set PAn as output
	PORTA.OUTCLR = pbp;      // Turn ON PAn Alarm LED by grounding output
	_delay_ms(50);           // 50 millisecond flash
	PORTB.DIRCLR = pbp >> 8; // Set PBn as input
	PORTA.DIRCLR = pbp;      // Set PAn as input
}

void buzzer_enable()
{
	PORTB.DIRSET      = (1 << 0);                    // Set PB0 as output
	TCA0.SINGLE.CTRLA = (1 << TCA_SINGLE_ENABLE_bp); // Enable buzzer
}

void buzzer_disable()
{
	TCA0.SINGLE.CTRLA = (0 << TCA_SINGLE_ENABLE_bp); // Disable buzzer
	PORTB.DIRCLR      = (1 << 0);                    // Set PB0 as input so we don't waste power
}

void twi_enable()
{
	TWI0.CTRLA = TWI_SDAHOLD_500NS_gc | TWI_SDASETUP_8CYC_gc; // Use max SDA setup and hold times to be safe
	TWI0.MBAUD = 12;                                          // 0 should give 3277 Hz I2C freq with 32768 Hz main clock
	                 // 45 should give 33 kHz I2C freq with 3.33 MHz main clock
	                 // 12 should give 98 kHz I2C freq with 3.33 MHz main clock
	TWI0.MCTRLA  = (1 << TWI_ENABLE_bp);
	TWI0.MSTATUS = 0x01; // Force state to IDLE
}

void twi_disable()
{
	TWI0.MCTRLA = (0 << TWI_ENABLE_bp);
}

void wait_clkhold()
{
	while (((TWI0.MSTATUS & (1 << TWI_CLKHOLD_bp)) == 0) && ((button_status() & V_OLED_OFF_MASK) == 0))
		;
	// Wait for clock hold while V_OLED is ON
}

void twi_send_addr()
{
	TWI0.MADDR = 0x78; // slave addr of 0111100 with 0 signifying write (for OLED display)
	wait_clkhold();
}

void twi_send_byte(uint8_t adat)
{
	TWI0.MDATA = adat;
	wait_clkhold();
}

void twi_send_stop()
{
	TWI0.MCTRLB = 0x3; // Send stop
	while (((TWI0.MSTATUS & TWI_BUSSTATE_gm) != TWI_BUSSTATE_IDLE_gc) && ((button_status() & V_OLED_OFF_MASK) == 0))
		;
	// Wait for idle state while V_OLED is ON
}

void twi_send_packet(const uint8_t *p, int n)
{
	twi_send_addr();
	while (n > 0) {
		TWI0.MDATA = *p++;
		n--;
		wait_clkhold();
	}
	twi_send_stop(); // Send stop
}

void twi_send_packet_p(const uint8_t *p, int n)
{ //_p indicates data is stored in progmem (flash)
	twi_send_addr();
	while (n > 0) {
		TWI0.MDATA = pgm_read_byte(p++); //*p++;
		n--;
		wait_clkhold();
	}
	twi_send_stop(); // Send stop
}

uint16_t stretch_byte(uint8_t byte_in)
{
	uint16_t outval;
	uint8_t  n;
	outval = 0;
	for (n = 0; n < 8; n++) {
		outval = outval | ((byte_in & (1 << n)) << n);
	}
	outval = outval | (outval << 1);
	return outval;
}

void clear_oled()
{
	uint16_t i;
	twi_send_addr();
	twi_send_byte(0x00); // Control byte indicates that commands follow
	twi_send_byte(0x21);
	twi_send_byte(0);   // Display column start
	twi_send_byte(127); // Display column end
	twi_send_byte(0x22);
	twi_send_byte(0); // Display page start
	twi_send_byte(7); // Display page end
	twi_send_stop();
	// Now that we have configured the starting point on the display, prepare data to be transferred

	twi_send_addr();
	twi_send_byte(0x40); // Control byte indicates that following bytes are data
	for (i = 0; i < 1024; i++) {
		twi_send_byte(0x00);
	}
	twi_send_stop();
}

void send_text_to_oled_p(const uint8_t *p, uint8_t n, uint8_t row, uint8_t col)
{
	// First we have to send the commands to the OLED to set up the row and column boundaries
	// page_min = row*2, page_max=page_min+1
	// disp_col = 12*col
	uint8_t  disp_col_start;
	uint8_t  disp_col_end;
	uint8_t  disp_page_start;
	uint8_t  disp_page_end;
	uint16_t double_byte;
	uint8_t  low_byte;
	uint8_t  high_byte;
	uint16_t i;

	if (button_status() & V_OLED_OFF_MASK) {
		return; // No need to send anything, the display is off
	}

	disp_page_start = row * 2;
	disp_page_end   = disp_page_start + 1;
	disp_col_start  = (col * 12) + 4; // Shift right by 4 pixels so display is centered
	disp_col_end    = 127;

	twi_send_addr();
	twi_send_byte(0x00); // Control byte indicates that commands follow
	twi_send_byte(0x21);
	twi_send_byte(disp_col_start);
	twi_send_byte(disp_col_end);
	twi_send_byte(0x22);
	twi_send_byte(disp_page_start);
	twi_send_byte(disp_page_end);
	twi_send_stop();
	// Now that we have configured the starting point on the display, prepare character to be transferred

	twi_send_addr();
	twi_send_byte(0x40);                         // Control byte indicates that following bytes are data
	for (i = 0; i < (BYTES_PER_CHAR * n); i++) { // i is the index for each byte (6 of them) within a character
		double_byte = stretch_byte(pgm_read_byte(p++));
		low_byte    = ((uint8_t)double_byte);
		high_byte   = ((uint8_t)(double_byte >> 8));
		// need to send, low byte, high byte, low byte, high byte from double_byte
		twi_send_byte(low_byte);
		twi_send_byte(high_byte);
		twi_send_byte(low_byte);
		twi_send_byte(high_byte);
	}
	twi_send_stop();
}

ISR(PORTA_PORT_vect)
{
	PORTA.INTFLAGS = (1 << SW_BP); // try to clear interrupt on slide switch
}

ISR(RTC_PIT_vect)
{
	RTC.PITINTFLAGS = (1 << RTC_PI_bp); // Clear the periodic timer interrupt
}

ISR(RTC_CNT_vect)
{
	uint8_t n;

	RTC.INTFLAGS = (1 << RTC_OVF_bp); // Clear the interrupt
	if (now_in_adjust_mode) {         // If in alarm+time adjust mode, reset counters and return
		cnt_in_meas_mode     = 0;
		cnt_not_in_meas_mode = 0;
		alarms_active        = 0;
		return;
	}

	if (!now_in_meas_mode) {
		cnt_in_meas_mode = 0;
		cnt_not_in_meas_mode++;
	} else {
		cnt_in_meas_mode++;
		cnt_not_in_meas_mode = 0;
	}

	if (cnt_in_meas_mode == N_TICKS_FOR_MEAS) {
		meas_value = (TCB0.CCMP) + 1; // Get count that was saved by TCB, add 1 since a value of 0 means 1 clock cycle
		meas_value
		    = meas_value
		      + 65536; // The TCB0 will overflow once during a 3-second duration, so we add 2^16 to get the true count
	}
	n_31us = n_31us + meas_value; // Update the (1/32768) second counter by adding the measured value
	if (n_31us >= N_31US_PER_DAY) {
		n_31us = n_31us - N_31US_PER_DAY; // We just crossed into a new day
	}
	secs     = n_31us >> SHIFT_32768; // Determine the integer number of seconds
	old_mins = totmins;
	totmins  = ((uint16_t)(secs / N_SECS_PER_MIN)); // Determine the integer number of minutes
	if (totmins != old_mins) {
		// Since minute has changed, we also need to update alarm status
		if (alarms_active != 0) {
			// Alarms have been going for about a minute, so shut them off
			alarms_active = 0;
			buzzer_disable();
		}
		for (n = 0; n < N_ALARMS; n++) {
			// Now determine if it's time to turn on alarms
			if ((alarm_times_totmins[n]) == totmins) {
				alarms_active = alarms_active | (1 << n); // Flip the nth bit high
			}
		}
		if (alarms_active == 0) // If alarms are inactive, flash LED briefly as an every minute heartbeat
		{
			flash_led((1 << B4_BP));
		}
	}
	if (alarms_active == 0) // If alarms are not active, check battery voltage
	{
		ADC0.CTRLA   = 1 << ADC_ENABLE_bp;   // Enable ADC
		ADC0.COMMAND = (1 << ADC_STCONV_bp); // Start conversion
		while (ADC0.COMMAND & (1 << ADC_STCONV_bp))
			; // Wait for conversion complete
		adc_value = ADC0.RES;
		// Vbat = (1024/adc_value)*(1.5V)
		// If adc_value is 768, Vbat is 2.0 V
		if (adc_value > 768) // Check if Vbat is below 2.0 V
		{
			buzzer_enable();
			_delay_ms(5); // Run buzzer for 5 milliseconds as low bat indicator to make a chirp
			buzzer_disable();
		}
		ADC0.CTRLA = 0 << ADC_ENABLE_bp; // Disable ADC to save power
	}
}

void switch_main_clk_to_ulp32k()
{
	_PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_OSCULP32K_gc); // Select ULP
	while (CLKCTRL.MCLKSTATUS & (1 << CLKCTRL_SOSC_bp)) {
		; // Wait for System Oscillator changing bit to go low
	}
	// The prescaler might still be enabled at this point from the previous
	// clock setting, which could result in a very slow clock (32768Hz / 6 = 5461 Hz, for example)
	// Thus, it is very important to disable the prescaler here,
	_PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, 0x00); // in case it was enabled previously
}

void switch_main_clk_to_xosc32k()
{
	_PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_XOSC32K_gc); // Select XOSC
	while (CLKCTRL.MCLKSTATUS & (1 << CLKCTRL_SOSC_bp)) {
		; // Wait for System Oscillator changing bit to go low
	}
	// The prescaler might still be enabled at this point from the previous
	// clock setting, which could result in a very slow clock (32768Hz / 6 = 5461 Hz, for example)
	// Thus, it is very important to disable the prescaler here,
	_PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, 0x00); // in case it was enabled previously
}

void switch_main_clk_to_osc20m()
{
	_PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, (CLKCTRL_PDIV_6X_gc | (1 << CLKCTRL_PEN_bp))); // Enable prescaler
	_PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_OSC20M_gc);                     // Select 20MHz osc
	while (CLKCTRL.MCLKSTATUS & (1 << CLKCTRL_SOSC_bp)) {
		; // Wait for System Oscillator changing bit to go low
	}
}

void handle_alarm()
{
	uint16_t port_bit_position; // The upper 8 bits represent position in PORTB

	buzzer_enable();
	port_bit_position = 0;

	if (alarms_active & (1 << 3))
		port_bit_position = (1 << B4_BP);
	if (alarms_active & (1 << 2))
		port_bit_position = (1 << B3_BP);
	if (alarms_active & (1 << 1))
		port_bit_position = (1 << B2_BP);
	if (alarms_active & (1 << 0))
		port_bit_position = (1 << B1_BP);

	flash_led(port_bit_position);

	buzzer_disable();

	_delay_ms(50); // Allow 50 milliseconds for recovery from LEDs on
	               // before checking if buttons are pressed

	if (button_status() & port_bit_position) { // Check if button is pressed
		alarms_active = 0;
	}
}

void send_static_text_to_oled()
{
	twi_send_packet_p(oled_inita, 26); // for OLED initialization
	// twi_send_packet_p(oled_initb, 2); // reverse display
	clear_oled();
	// twi_send_packet_p(oled_initc, 2); // normal display
	send_text_to_oled_p(text_b1arrow, 3, 1, 0); // B1^
	send_text_to_oled_p(text_arrowb4, 3, 1, 7); //^B4
	send_text_to_oled_p(text_colon, 1, 2, 4);   //:
	send_text_to_oled_p(text_b2arrow, 4, 3, 0); // B2-^
	send_text_to_oled_p(text_arrowb3, 4, 3, 6); //^-B3
}

void send_changeable_text_to_oled()
{

	if (setting_index < N_ALARMS) {
		// An alarm is currently being set
		send_text_to_oled_p(text_alarm, 5, 0, 0);
		send_text_to_oled_p(
		    text_digits + (BYTES_PER_CHAR * (1 + setting_index)), 1, 0, 5); // Send digit after ALARM text
		if (alarm_times_totmins[setting_index] > N_MINS_PER_DAY)
			send_text_to_oled_p(text_off, 3, 0, 7); // Show ALARMn OFF
		else
			send_text_to_oled_p(text_on, 3, 0, 7); // Show ALARMn ON
		disp_hours = alarm_times_hours[setting_index];
		disp_mins  = alarm_times_mins[setting_index];
	} else {
		// The time is currently being set
		send_text_to_oled_p(text_time, 6, 0, 0);
		send_text_to_oled_p(text_space, 3, 0, 7);              // Need to also blank out ON/OFF field
		disp_hours = totmins / N_MINS_PER_HOUR;                // should be current time (hours)
		disp_mins  = totmins - (disp_hours * N_MINS_PER_HOUR); // should be current time (minutes)
	}
	tdh = disp_hours / 10;
	tdm = disp_mins / 10;
	if ((setting_index < N_ALARMS) & (alarm_times_totmins[setting_index] > N_MINS_PER_DAY)) {
		// This handles the case of ALARM OFF to show **:** instead of digits
		send_text_to_oled_p(text_asterisk, 2, 2, 2);
		send_text_to_oled_p(text_asterisk, 2, 2, 5);
	} else {
		send_text_to_oled_p((text_digits + BYTES_PER_CHAR * tdh), 1, 2, 2);
		send_text_to_oled_p((text_digits + BYTES_PER_CHAR * (disp_hours - 10 * tdh)), 1, 2, 3);
		send_text_to_oled_p((text_digits + BYTES_PER_CHAR * tdm), 1, 2, 5);
		send_text_to_oled_p((text_digits + BYTES_PER_CHAR * (disp_mins - 10 * tdm)), 1, 2, 6);
	}
}

void update_from_button_pressed()
{
	if (button_pressed & ALARM_INC_MASK) {
		if (setting_index >= N_ALARMS)
			setting_index = 0;
		else
			setting_index++;
	}
	// The alarm on/off button has no effect when setting time (setting_index == N_ALARMS)
	else if ((button_pressed & ALARM_ONOFF_MASK) && (setting_index < N_ALARMS)) {
		if (alarm_times_totmins[setting_index] > N_MINS_PER_DAY)
			alarm_times_totmins[setting_index] = 0;
		else
			alarm_times_totmins[setting_index] = (N_MINS_PER_DAY + 1);
	} else if (button_pressed & HOUR_INC_MASK) {
		if (setting_index < N_ALARMS) {
			if (alarm_times_hours[setting_index] >= 23)
				alarm_times_hours[setting_index] = 0;
			else
				(alarm_times_hours[setting_index])++;
		} else {
			totmins = totmins + N_MINS_PER_HOUR; // Need to increment hours of current time
			if (totmins >= N_MINS_PER_DAY)
				totmins = totmins - N_MINS_PER_DAY;
		}
	} else if (button_pressed & MIN_INC_MASK) {
		if (setting_index < N_ALARMS) {
			if (alarm_times_mins[setting_index] >= 59)
				alarm_times_mins[setting_index] = 0;
			else
				(alarm_times_mins[setting_index])++;
		} else {
			totmins++; // Need to increment minutes of current time
			if (totmins >= N_MINS_PER_DAY)
				totmins = totmins - N_MINS_PER_DAY;
		}
	}

	if (setting_index == N_ALARMS) {
		// Update n_31us and secs based on totmins
		secs   = ((uint32_t)totmins) * N_SECS_PER_MIN;
		n_31us = secs << SHIFT_32768;
	} else if (alarm_times_totmins[setting_index] < N_MINS_PER_DAY) {
		// Update total mins with the correct number of minutes
		alarm_times_totmins[setting_index]
		    = N_MINS_PER_HOUR * alarm_times_hours[setting_index] + alarm_times_mins[setting_index];
	}
}

int main(void)
{
	// PA6 will be used for resetting the OLED display
	PORTA.OUTCLR = (1 << OLEDRST_BP); // For starters, hold the OLED reset line low
	PORTA.DIRSET = (1 << OLEDRST_BP); // Make it an output

	// PA7 will be used for detecting that the slide switch is ON
	PORTA.DIRCLR   = (1 << SW_BP);          // Make PA7 an input
	PORTA.PIN7CTRL = PORT_ISC_BOTHEDGES_gc; // sense both edges for interrupt

	// Select pins for TWI interface to communicate with display
	PORTMUX.CTRLB = PORTMUX_TWI0_ALTERNATE_gc; // Select alternate pins for TWI0 (PA1=SDA and PA2=SCL)

	// Setup needed to use ADC for monitoring battery voltage
	VREF.CTRLA  = VREF_ADC0REFSEL_1V5_gc;                                    // Select 1.5V to be generated by VREF
	ADC0.CTRLC  = (0 << ADC_SAMPCAP_bp) | (0x01 << 4) | (ADC_PRESC_DIV2_gc); // Use battery voltage as reference voltage
	ADC0.MUXPOS = 0x1d; // Input to ADC will be 1.5V from VREF peripheral

	_PROTECTED_WRITE(CLKCTRL.OSC32KCTRLA, (1 << CLKCTRL_RUNSTDBY_bp)); // Enable the 32kHz internal ULP oscillator
	                                                                   // to run during STANDBY sleep mode
	switch_main_clk_to_ulp32k();

	// At this point, the main clock is approximately 32 kHz from ULP oscillator
	// Flash each LED for about 0.05 seconds just to check that they are working
	flash_led((1 << B1_BP));
	flash_led((1 << B2_BP));
	flash_led((1 << B3_BP));
	flash_led((1 << B4_BP));

	// Now we must set up the RTC
	RTC.CLKSEL = RTC_CLKSEL_INT1K_gc; // Gives 1024 Hz from OSCULP32K
	RTC.CNT    = 0;
	RTC.PER    = RTC_PER_VALUE;
	RTC.CMP    = 0xFFFF;
	RTC.CTRLA  = (1 << RTC_RTCEN_bp) | (1 << RTC_RUNSTDBY_bp); // Enable RTC to run during STANDBY sleep mode
	// Enable overflow interrupt LATER
	// At this point the RTC is running, but not yet generating interrupts

	sei(); // Enable global interrupts

	// Now we want to set up the TCB to measure the time between successive
	// ticks of the RTC
	EVSYS.ASYNCCH0 = EVSYS_ASYNCCH0_RTC_OVF_gc; // Real Time Counter overflow
	// EVSYS.ASYNCUSER0 = EVSYS_ASYNCUSER0_ASYNCCH0_gc; // ASYNCUSER0 is TCB0 select
	_SFR_MEM8(0x0192) = (0x03 << 0); // HACK to get around missing definitions that cause previous line to fail

	// Now the RTC event generator should be connected to TCB0, the event user
	TCB0.CTRLB  = TCB_CNTMODE_FRQ_gc;   // input frequency measurement mode
	TCB0.EVCTRL = (1 << TCB_CAPTEI_bp); // capture event input enable
	TCB0.CTRLA  = (1 << TCB_ENABLE_bp); // enable

	// Prepare TCA for generating buzzer alarm
	TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_FRQ_gc | (1 << TCA_SINGLE_CMP0EN_bp); // Frequency waveform generation on WO0
	TCA0.SINGLE.PER   = 0;                                                      // 16-bit top value
	TCA0.SINGLE.CMP0  = 3; // 16-bit compare value of 3 should give fout = 32768Hz/(2*(3+1)) = 4096 Hz
	                      // value of 31 should give fout = 32768Hz/(2*(31+1)) = 512 Hz
	buzzer_enable(); // Enable the buzzer for 5 ms to test it at startup
	_delay_ms(5);
	buzzer_disable();

	// Enable the 32768 Hz crystal oscillator with 64k cycle startup time (about 2 seconds for good stabilization)
	_PROTECTED_WRITE(CLKCTRL.XOSC32KCTRLA, (CLKCTRL_CSUT_64K_gc | (1 << CLKCTRL_ENABLE_bp)));
	// However, the RUNSTDBY bit is not used because we do NOT want it running when in STANDBY sleep mode

	SLPCTRL.CTRLA = SLEEP_MODE_STANDBY | (1 << SLPCTRL_SEN_bp); // Sleep mode will be STANDBY whenever we sleep

	if (LED2_SHOWS_MCU_RUN) {
		PORTA.DIRSET = (1 << B2_BP); // Set PAn as output
		PORTA.OUTCLR = (1 << B2_BP); // Turn ON PAn Alarm LED by grounding output
	}

	if (button_status() & V_OLED_OFF_MASK) { // If slide switch is in "RUN" position
		sleep_cpu();                         // Sleep until the slide switch is in the "SET" position
	}

	while (1) {
		// At this point, the slide switch has just been put in the SET position, but we are still running from the ULP
		// oscillator
		if (LED2_SHOWS_MCU_RUN) {
			PORTA.DIRCLR = (1 << B2_BP); // Set PAn as input
		}

		PORTA.OUTCLR = (1 << OLEDRST_BP); // For starters, hold the OLED reset line low
		PORTA.DIRSET = (1 << OLEDRST_BP); // Make it an output

		PORTA.PIN7CTRL = 0;                 // Turn off PA7 interrupt (slide switch)
		RTC.INTCTRL    = (0 << RTC_OVF_bp); // Disable RTC overflow interrupt

		// First, allow some time for slide switch to finish bouncing, the OLED DC/DC converters to get started and
		// stabilize, etc.
		_delay_ms(250); // For now, wait 0.25 second

		PORTA.DIRCLR
		    = (1 << OLEDRST_BP); // Now bring the OLED reset line high by making the MCU pin an input
		                         // and let the pullup on the PCB work
		                         // This is better than trying to drive the line when the OLED is powered off and
		                         // the reset line may have a low impedance.
		_delay_ms(100);          // Allow 0.1 second for the RC circuit to get the OLED out of reset

		switch_main_clk_to_osc20m(); // We need to run from a fast (3.33 MHz) clock in order to update the display
		                             // quickly enough
		now_in_adjust_mode = 1;

		// Alarm and time adjustment procedure
		RTC.CNT         = 0;                                          // Reset RTC counter
		RTC.PITCTRLA    = RTC_PERIOD_CYC256_gc | (1 << RTC_PITEN_bp); // Set up RTC PIT so we get
		RTC.PITINTFLAGS = (1 << RTC_PI_bp); // (clear the periodic timer interrupt in case it was set earlier)
		RTC.PITINTCTRL  = (1 << RTC_PI_bp); // interrupted every 256 ms, approximately

		setting_index = 0; // 0 = Alarm1, 1 = Alarm2, 2 = Alarm3, 3 = Alarm4, 4 = Time

		// Verify that slide switch is still in the "SET" position before we try to init the display
		if (button_status() & V_OLED_OFF_MASK) {
			now_in_adjust_mode = 0;
		} else {
			twi_enable(); // We must enable TWI before we can send anything to OLED
			send_static_text_to_oled();
		}

		while (now_in_adjust_mode) {
			// Update display with information that may change
			send_changeable_text_to_oled();
			sleep_cpu(); // sleep until we get a PIT interrupt

			new_button_status = button_status();
			button_pressed = new_button_status & old_button_status; // button is considered pressed if two consecutive
			old_button_status = new_button_status & (~(button_pressed)); // reset if button was pressed

			if (button_pressed & V_OLED_OFF_MASK) {
				now_in_adjust_mode = 0; // We are no longer in adjust mode
			} else {
				update_from_button_pressed();
			}
		}
		// If we get here we are no longer in adjust mode
		if (LED2_SHOWS_MCU_RUN) {
			PORTA.DIRSET = (1 << B2_BP); // Set PAn as output
			PORTA.OUTCLR = (1 << B2_BP); // Turn ON PAn Alarm LED by grounding output
		}
		twi_disable(); // Disable the TWI interface to OLED

		RTC.PITINTCTRL = (0 << RTC_PI_bp); // Disable the PIT interrupt that was used for periodic button checking

		RTC.CNT        = 0;                     // Reset RTC count value to zero
		RTC.INTFLAGS   = (1 << RTC_OVF_bp);     // Clear any RTC interrupt flag that was set earlier
		RTC.INTCTRL    = (1 << RTC_OVF_bp);     // Enable overflow interrupt
		PORTA.PIN7CTRL = PORT_ISC_BOTHEDGES_gc; // sense both edges on PA7 for interrupt

		while (!now_in_adjust_mode) {

			// Switch the main clk to XOSC32K so we can use the TCB to make an accurate
			// measurement of the duration of two consecutive RTC ticks
			switch_main_clk_to_xosc32k();

			// At this point the RTC is running from OSC32KULP divided by 32,
			// while CPU and everything else is running from XOSC32K
			cnt_in_meas_mode = 0; // Reset measurement mode interrupt counter
			now_in_meas_mode = 1; // Inform ISR that we are in measurement mode

			// Wait for ISR to make measurement
			while (cnt_in_meas_mode < N_TICKS_FOR_MEAS) {
				;
			}

			cnt_not_in_meas_mode = 0; // Reset non-measurement mode counter
			now_in_meas_mode     = 0; // Inform ISR that we are out of measurement mode

			switch_main_clk_to_ulp32k(); // Switch main clock back to ULP32K
			while ((!now_in_adjust_mode) && (cnt_not_in_meas_mode < N_TICKS_BETWEEN_MEAS)) {
				while (alarms_active != 0) {
					handle_alarm();
				}
				sleep_cpu(); // Sleep while not in measurement mode

				// Here we need to check why we came out of sleep -- is it because of
				// an RTC overflow interrupt or is it because the slide switch has
				// been put in the set position?
				if ((button_status() & V_OLED_OFF_MASK) == 0) {
					now_in_adjust_mode = 1;
				}
			}

		} // end of while(!now_in_adjust_mode)
	}     // end of while(1)

	// Now run RTC from XOSC
	if (0) {
		RTC.CLKSEL  = RTC_CLKSEL_TOSC32K_gc; // Gives
		RTC.CNT     = 0;
		RTC.PER     = 16384 - 1; // was (3*1024-1);
		RTC.CMP     = 0xFFFF;
		RTC.INTCTRL = (1 << RTC_OVF_bp);                            // Enable overflow interrupt
		RTC.CTRLA   = (1 << RTC_RTCEN_bp) | (1 << RTC_RUNSTDBY_bp); // enable RTC, even in standby sleep mode
	}
}
