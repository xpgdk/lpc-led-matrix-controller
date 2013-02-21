#include "led_matrix.h"
#include "led_matrix_config.h"

#include "font.h"
#include <string.h>

uint8_t msg_mode;
volatile fb_color_t fb[LED_MATRIX_ROWS][LED_MATRIX_COLS];

static void shift_latch(void);
//static void shift_out(uint8_t b);
inline static void shift_out_data(const fb_color_t b[8], uint8_t shift, uint8_t threshold);
static void shift_out_row(uint8_t row, const fb_color_t data[LED_MATRIX_COLS], uint8_t threshold);
static void displayScrollTickCb(void);
static uint8_t hexCharToInt(char c);
static void row_reset(void);
static void row_tick(void);
static void row_first_tick(void);
static void row_latch(void);

#define delayMs(ms) (SysCtlDelay(((SysCtlClockGet() / 3) / 1000)*ms))

volatile char msg[50];
uint8_t msg_len;
uint8_t next_char = 0;
uint8_t off = 0;
static int current_intensity = 0;
static int current_row = 0;
//static int current_color = 0;
static display_anim_callback_t display_anim_cb;
static uint8_t display_interval = 0;
static volatile uint8_t display_counter;
static fb_color_t char_color;

static const char hexValues[] = {'0','1','2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

void displayInit(void) 
{
	msg_mode = MODE_STATIC;
	memset(fb, 0, FB_SIZE);
	display_interval = 2;
	display_counter = 0;
	char_color = COLOR(0, 0xF, 0);
	row_reset();
}

static void
shift_latch(void) {
	//P1OUT |= LATCH_OUT;
	FAST_GPIOPinWrite(LATCH_PORT, LATCH_PIN, LATCH_PIN);
	//delayMs(1);
	FAST_GPIOPinWrite(LATCH_PORT, LATCH_PIN, 0);
	//P1OUT &= ~LATCH_OUT;
}


static void
row_reset(void) {
	for(int i=0; i<LED_MATRIX_ROWS; i++) {
		FAST_GPIOPinWrite(ROW_SER_OUT_PORT, ROW_SER_OUT_PIN, ROW_SER_OUT_PIN);

		FAST_GPIOPinWrite(ROW_CLK_OUT_PORT, ROW_CLK_OUT_PIN, ROW_CLK_OUT_PIN);
		FAST_GPIOPinWrite(ROW_CLK_OUT_PORT, ROW_CLK_OUT_PIN, 0);
	}
}

static void
row_tick(void) {
	FAST_GPIOPinWrite(ROW_SER_OUT_PORT, ROW_SER_OUT_PIN, ROW_SER_OUT_PIN);

	FAST_GPIOPinWrite(ROW_CLK_OUT_PORT, ROW_CLK_OUT_PIN, ROW_CLK_OUT_PIN);
	FAST_GPIOPinWrite(ROW_CLK_OUT_PORT, ROW_CLK_OUT_PIN, 0);
}

static void
row_first_tick(void) {
	FAST_GPIOPinWrite(ROW_SER_OUT_PORT, ROW_SER_OUT_PIN, 0);

	FAST_GPIOPinWrite(ROW_CLK_OUT_PORT, ROW_CLK_OUT_PIN, ROW_CLK_OUT_PIN);
	FAST_GPIOPinWrite(ROW_CLK_OUT_PORT, ROW_CLK_OUT_PIN, 0);
}

static void
row_latch(void)
{
	FAST_GPIOPinWrite(ROW_LATCH_PORT, ROW_LATCH_PIN, ROW_LATCH_PIN);
	FAST_GPIOPinWrite(ROW_LATCH_PORT, ROW_LATCH_PIN, 0);
}

#if 0
static void
shift_out(uint8_t b) {
	for(int i=0;i<8; i++) {
		if( b & 0x1) {
			FAST_GPIOPinWrite(SER_OUT_PORT, SER_OUT_PIN, SER_OUT_PIN);
			//P1OUT |= SER_OUT;
		} else {
			//P1OUT &= ~SER_OUT;
			FAST_GPIOPinWrite(SER_OUT_PORT, SER_OUT_PIN, 0);
		}
		b = b >> 1;
		//P1OUT |= CLK_OUT;
		FAST_GPIOPinWrite(CLK_OUT_PORT, CLK_OUT_PIN, CLK_OUT_PIN);
		//delayMs(10);
		//__delay_cycles(100);
		//P1OUT &= ~CLK_OUT;
		FAST_GPIOPinWrite(CLK_OUT_PORT, CLK_OUT_PIN, 0);
	}
}
#endif

inline void shift_out_data(const fb_color_t b[8], uint8_t shift, uint8_t threshold) {
	for(int i=0;i<8; i++) {
		if( ((b[8-1-i] >> shift) & COLOR_MASK) > threshold) {
			FAST_GPIOPinWrite(SER_OUT_PORT, SER_OUT_PIN, SER_OUT_PIN);
		} else {
			FAST_GPIOPinWrite(SER_OUT_PORT, SER_OUT_PIN, 0);
		}
		//b = b >> 1;
		FAST_GPIOPinWrite(CLK_OUT_PORT, CLK_OUT_PIN, CLK_OUT_PIN);
		//delayMs(10);
		FAST_GPIOPinWrite(CLK_OUT_PORT, CLK_OUT_PIN, 0);
	}
}

void shift_out_row(uint8_t row, const fb_color_t data[LED_MATRIX_COLS], uint8_t threshold)
{
#if 0
	// Shift out left-most display first
	//shift_out_data(data, 8, current_color == 2 ? threshold : 15);
	shift_out_data(data, 4, current_color == 1 ? threshold : 15);
	shift_out_data(data, 0, current_color == 0 ? threshold : 15);

	// Shift out right-most display next
	//shift_out_data(data, 8, current_color == 2 ? threshold : 15);
	shift_out_data(data + 8, 4, current_color == 1 ? threshold : 15);
	shift_out_data(data + 8, 0, current_color == 0 ? threshold : 15);
#else
	// Shift out red
	shift_out_data(data, RED_SHIFT, threshold);
	shift_out_data(data + 8, RED_SHIFT, threshold);

	// Shift out green
	shift_out_data(data, GREEN_SHIFT, threshold);
	shift_out_data(data + 8, GREEN_SHIFT, threshold);
#endif

	//shift_out(row);

	FAST_GPIOPinWrite(ROW_ENABLE_PORT, ROW_ENABLE_PIN, ROW_ENABLE_PIN);
	if( row == 0 ) {
		//row_reset();
		row_first_tick();
	} else {
		row_tick();
	}

	row_latch();
	shift_latch();
	FAST_GPIOPinWrite(ROW_ENABLE_PORT, ROW_ENABLE_PIN, 0);
}

void set_char(char c, fb_color_t color) {
	msg_mode = MODE_STATIC;
	for(int i=0; i<LED_MATRIX_ROWS; i++) {
		for(int l=0; l<LED_MATRIX_COLS; l++) {
			fb[i][l] = ((font[c-32][i] >> (LED_MATRIX_COLS-1-l)) & 0x1) * color;
		}
	}
}

void set_message(char *buf, uint16_t len) {
	set_char(' ', 0x00);
	displaySetAnim(displayScrollTickCb, 2);
	strncpy((char*)msg, buf, len);
	msg_len = len;
	next_char = 0;
	off = 0;

	char_color = COLOR(0xFF, 0, 0);

	if( msg[next_char] == '#' ) {
			uint8_t r = (hexCharToInt(msg[next_char+1]) << 4) +
				    hexCharToInt(msg[next_char+2]);
			uint8_t g = (hexCharToInt(msg[next_char+3]) << 4) +
				    hexCharToInt(msg[next_char+4]);
			//uint8_t b = hexCharToInt(msg[next_char+3]);
			char_color = COLOR(r,g,0);
			next_char += 5;
	}
}

void
append_message(char *buf, uint16_t len)
{
	strncpy((char*)msg+msg_len, buf, len);
	msg_len += len;
}

void displayScrollTickSetMessage(char *buf, uint16_t len)
{
	set_char(' ', 0x00);
	strncpy((char*)msg, buf, len);
	msg_len = len;
	next_char = 0;
	off = 0;
}

void clearDisplay(fb_color_t v[LED_MATRIX_ROWS][LED_MATRIX_COLS]) 
{
	for(int i=0; i<LED_MATRIX_ROWS; i++) {
		for(int l=0; l<LED_MATRIX_COLS; l++) {
			v[i][l] = 0x00;
		}
	}
}

bool displayTick(void)
{
	shift_out_row(current_row, (const fb_color_t*)fb[current_row], current_intensity);
	current_row++;
	if( current_row >= LED_MATRIX_ROWS ) {
		current_intensity++;
		current_row = 0;

		if( current_intensity > (64-1) ) {
			current_intensity = 0;
		}
	}
	if( current_row == 0 && current_intensity == 0/* && current_color == 0*/ ) {
		return true;
	} else {
		return false;
	}
}

void displayScrollTickCb(void)
{
	displayScrollTick();
}

bool displayScrollTick(void)
{
	bool done = false;

	for(int i=0; i<LED_MATRIX_ROWS; i++) {
		for(int l=0; l<LED_MATRIX_COLS-1; l++) {
			fb[i][l] = fb[i][l+1];
		}
		fb[i][LED_MATRIX_COLS-1] = ((font[msg[next_char]-32][i] >> (7-off)) & 0x1) * char_color; //msg_color[next_char];
	}
	off++;
	if( off >= getCharEnd(msg[next_char]) ) {
		next_char++;
		if( msg[next_char] == '#' ) {
			uint8_t r = (hexCharToInt(msg[next_char+1]) << 4) +
				    hexCharToInt(msg[next_char+2]);
			uint8_t g = (hexCharToInt(msg[next_char+3]) << 4) +
				    hexCharToInt(msg[next_char+4]);
			//uint8_t b = hexCharToInt(msg[next_char+3]);
			char_color = COLOR(r,g,0);
			next_char += 5;
		}
		if( next_char >= msg_len) {
			done = true;
			next_char = 0;
		}
		if( msg[next_char] == '#' ) {
			uint8_t r = (hexCharToInt(msg[next_char+1]) << 4) +
				    hexCharToInt(msg[next_char+2]);
			uint8_t g = (hexCharToInt(msg[next_char+3]) << 4) +
				    hexCharToInt(msg[next_char+4]);
			//uint8_t b = hexCharToInt(msg[next_char+3]);
			char_color = COLOR(r,g,0);
			next_char += 5;
		}
		off = getCharStart(msg[next_char]);
	}

	return done;
}

void
displaySetAnim(display_anim_callback_t cb, uint8_t interval)
{
	msg_mode = MODE_ANIM;
	display_anim_cb = cb;
	display_interval = interval;
	display_counter = 0;
}

uint8_t
displayGetInterval(void)
{
	return display_interval;
}

bool displayCheckUpdate(void)
{
	display_counter++;
	if( display_counter >= display_interval ) {
		display_counter = 0;
		return true;
	}
	return false;
}

void
displayAnimTick(void)
{
	display_anim_cb();
}

void
displayFillColor(fb_color_t color)
{
	msg_mode = MODE_STATIC;
	for(int i=0; i<LED_MATRIX_ROWS; i++) {
		for(int l=0; l<LED_MATRIX_COLS; l++) {
			fb[i][l] = color;
		}
	}
}

int
getCharEnd(char c)
{
	const uint8_t *fontChar = font[c-32];

	if( c == ' ' ) {
		return 6;
	}

	int end = 0;

	for(int b=0;b<8;b++) {
		for(int r=0;r<8;r++) {
			if( (fontChar[r] >> (7-b)) & 0x1 ) {
				end = b;
				break;
			}
		}
	}

	end =  end < 8 ? end+1 : end;

	return end;
}

int
getCharStart(char c)
{
	const uint8_t *fontChar = font[c-32];

	if( c == ' ' ) {
		return 2;
	}

	int offset = 0;

	for(int b=0;b<8;b++) {
		bool hasBits = false;
		for(int r=0;r<8;r++) {
			if( (fontChar[r] >> (7-b)) & 0x1 ) {
				hasBits = true;
				break;
			}
		}
		if( !hasBits ) {
			offset = b;	
		} else {
			break;
		}
	}
	return offset;
}

static uint8_t
hexCharToInt(char c)
{
	if( c >= 0x30 && c <= 0x39 ) {
		return c-0x30;
	} else if( c >= 0x41 && c <= 0x46 ) {
		return 10 + (c-0x41);
	}
	return 0;
}
