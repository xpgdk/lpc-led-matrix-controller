#ifndef _LED_MATRIX_H
#define _LED_MATRIX_H

#include <stdint.h>
#include <stdbool.h>

//#define ROW(n) 			(~(1<<n))
#define ROW(n) 			(n==6 ? 0 : (1<<n) | 0x40 )

#define RED_SHIFT               0
#define GREEN_SHIFT     	4

#define COLOR(r,g,b) 		((r & 0xFF)+((g & 0xFF)<<GREEN_SHIFT))

#define LED_MATRIX_ROWS		8
#define LED_MATRIX_COLS		16

#define FB_SIZE			(LED_MATRIX_ROWS*LED_MATRIX_COLS*2)

extern uint8_t msg_mode;
extern volatile uint16_t fb[LED_MATRIX_ROWS][LED_MATRIX_COLS];

#define MODE_STATIC	0
#define MODE_ANIM	1

typedef void (*display_anim_callback_t)(void);

void set_message(char *buf, uint16_t len);
void append_message(char *buf, uint16_t len);
void clearDisplay(uint16_t v[LED_MATRIX_ROWS][LED_MATRIX_COLS]);
void displayFillColor(uint16_t color);
bool displayTick(void);
void displayScrollTickSetMessage(char *buf, uint16_t len);
bool displayScrollTick(void);
void displayInit(void);
void displaySetAnim(display_anim_callback_t cb, uint8_t interval);
uint8_t displayGetInterval(void);
bool displayCheckUpdate(void);
void displayAnimTick(void);
int getCharStart(char c);
int getCharEnd(char c);

#endif
