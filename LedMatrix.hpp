#ifndef _LED_MATRIX_HPP
#define _LED_MATRIX_HPP

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "utils.h"

extern "C" {
#include "uart.h"
#include "led_matrix_config.h"
}

template <class T, unsigned int R, unsigned int C> class LedMatrix;
template <unsigned R, unsigned C> class LedMatrixFrameBuffer;

struct LedMatrixColor
{
public:
	LedMatrixColor() 
		: color(0) {
	}

	LedMatrixColor(LedMatrixColor &color) {
		this->color = color.color;
	}

	LedMatrixColor(uint8_t red, uint8_t green, uint8_t blue) {
		NOT_USED(blue);
		color = (red & 0xFF) | ((green & 0xFF)<<8);
	}

	inline uint16_t getValue() {
		return color;
	}
private:
	uint16_t color;
};

typedef	uint8_t FontChar[8];

class LedMatrixFont
{
public:
	virtual const FontChar*	getFontData() = 0;
	virtual uint8_t		getFontWidth() = 0;
	virtual uint8_t		getFontHeight() = 0;

	int 			getStart(char c);
	int			getEnd(char c);
};

template<unsigned int R, unsigned int C>
class LedMatrixAnimation
{
public:
	virtual void update(LedMatrixFrameBuffer<R,C> &fb) = 0;
};

template<unsigned int R, unsigned int C>
class LedMatrixScrollAnimation : public LedMatrixAnimation<R,C>
{
public:
	LedMatrixScrollAnimation(LedMatrixFont &font) 
		: font(font) {
	}

	void update(LedMatrixFrameBuffer<R,C> &fb) {
		for(unsigned int i=0; i<R; i++) {
			for(unsigned int l=0; l<C-1; l++) {
				fb.fb[i][l] = fb.fb[i][l+1];
			}
			fb.fb[i][C-1] = ((font.getFontData()[msgBuffer[nextChar]-32][i] >> (7-offset)) & 0x1) * currentColor.getValue();
		}
		offset++;
		if( offset >= font.getEnd(msgBuffer[nextChar]) ) {
			nextChar++;
			parseColor();

			if( nextChar >= msgLen ) {
				nextChar = 0;
				parseColor();
			}
			offset = font.getStart(msgBuffer[nextChar]);
		}
	}

	void setMessage(char *msg, uint16_t len) {
		strncpy(msgBuffer, msg, len);
		msgLen = len;
		offset = 0;
		nextChar = 0;
		currentColor = LedMatrixColor(0x3F, 0, 0);
		parseColor();
	}

	void appendMessage(char *msg, uint16_t len) {
		strncpy(msgBuffer+msgLen, msg, len);
		msgLen += len;
	}

private:
	void parseColor() {
		if( msgBuffer[nextChar] == '#' ) {
			uint8_t r = (hexCharToInt(msgBuffer[nextChar+1]) << 4) +
				    hexCharToInt(msgBuffer[nextChar+2]);
			uint8_t g = (hexCharToInt(msgBuffer[nextChar+3]) << 4) +
				    hexCharToInt(msgBuffer[nextChar+4]);
			//uint8_t b = hexCharToInt(msg[next_char+3]);
			currentColor = LedMatrixColor(r, g, 0);
			nextChar += 5;
		}
	}

	uint8_t hexCharToInt(char c) {
		if( c >= 0x30 && c <= 0x39 ) {
			return c-0x30;
		} else if( c >= 0x41 && c <= 0x46 ) {
			return 10 + (c-0x41);
		}
		return 0;
	}

private:
	char 			msgBuffer[60];
	uint16_t 		msgLen;
	uint16_t		nextChar;
	uint16_t		offset;
	LedMatrixColor		currentColor;
	LedMatrixFont		&font;
};

template <unsigned int R, unsigned int C>
class LedMatrixFrameBuffer
{
public:
	LedMatrixFrameBuffer() {
		currentRow = 0;
		currentIntensity = 0;
		rowReset();
	}

	void setChar(char c, LedMatrixColor &color, LedMatrixFont &font) {
		printf("Entering\r\n");
		putHex16(color.getValue());
		printf("\r\n");
		putHex32((uint32_t)font.getFontData()[32]);
		printf("\r\nGot data\r\n");
		for(unsigned int i=0; i<R; i++) {
			for(unsigned int l=0; l<C; l++) {
				fb[i][l] = ((font.getFontData()[c-32][i] >> (C-1-l)) & 0x1) * color.getValue();
			}
		}
	}

	bool tick() {
		const uint16_t *dots = fb[currentRow];

		shiftOut(dots, 0, currentIntensity);
		shiftOut(dots + 8, 0, currentIntensity);

		shiftOut(dots, 8, currentIntensity);
		shiftOut(dots + 8, 8, currentIntensity);

		FAST_GPIOPinWrite(ROW_ENABLE_PORT, ROW_ENABLE_PIN, ROW_ENABLE_PIN);
		if( currentRow == 0 ) {
			rowFirstTick();
		} else {
			rowTick();
		}

		rowLatch();
		colLatch();
		FAST_GPIOPinWrite(ROW_ENABLE_PORT, ROW_ENABLE_PIN, 0);

		currentRow++;
		if( currentRow >= R ) {
			currentIntensity++;
			currentRow = 0;

			if( currentIntensity > (64-1) ) {
				currentIntensity = 0;
			}
		}
		if( currentRow == 0 && currentIntensity == 0) {
			return true;
		} else {
			return false;
		}
	}

	void clear(LedMatrixColor &color) {
		for(unsigned int i=0; i<R; i++) {
			for(unsigned int l=0; l<C; l++) {
				fb[i][l] = color.getValue();
			}
		}
	}

private:
	void shiftOut(const uint16_t b[8], uint8_t shift, uint8_t threshold) {
		for(unsigned int i=0;i<8; i++) {
			if( ((b[8-1-i] >> shift) & 0xFF) > threshold) {
				FAST_GPIOPinWrite(SER_OUT_PORT, SER_OUT_PIN, SER_OUT_PIN);
			} else {
				FAST_GPIOPinWrite(SER_OUT_PORT, SER_OUT_PIN, 0);
			}
			FAST_GPIOPinWrite(CLK_OUT_PORT, CLK_OUT_PIN, CLK_OUT_PIN);
			FAST_GPIOPinWrite(CLK_OUT_PORT, CLK_OUT_PIN, 0);
		}
	}

	void rowReset(void) {
		for(unsigned int i=0; i<R; i++) {
			FAST_GPIOPinWrite(ROW_SER_OUT_PORT, ROW_SER_OUT_PIN, ROW_SER_OUT_PIN);

			FAST_GPIOPinWrite(ROW_CLK_OUT_PORT, ROW_CLK_OUT_PIN, ROW_CLK_OUT_PIN);
			FAST_GPIOPinWrite(ROW_CLK_OUT_PORT, ROW_CLK_OUT_PIN, 0);
		}
	}

	void rowTick(void) {
		FAST_GPIOPinWrite(ROW_SER_OUT_PORT, ROW_SER_OUT_PIN, ROW_SER_OUT_PIN);

		FAST_GPIOPinWrite(ROW_CLK_OUT_PORT, ROW_CLK_OUT_PIN, ROW_CLK_OUT_PIN);
		FAST_GPIOPinWrite(ROW_CLK_OUT_PORT, ROW_CLK_OUT_PIN, 0);
	}

	void rowFirstTick(void) {
		FAST_GPIOPinWrite(ROW_SER_OUT_PORT, ROW_SER_OUT_PIN, 0);

		FAST_GPIOPinWrite(ROW_CLK_OUT_PORT, ROW_CLK_OUT_PIN, ROW_CLK_OUT_PIN);
		FAST_GPIOPinWrite(ROW_CLK_OUT_PORT, ROW_CLK_OUT_PIN, 0);
	}

	void rowLatch(void)
	{
		FAST_GPIOPinWrite(ROW_LATCH_PORT, ROW_LATCH_PIN, ROW_LATCH_PIN);
		FAST_GPIOPinWrite(ROW_LATCH_PORT, ROW_LATCH_PIN, 0);
	}

	void colLatch(void) {
		FAST_GPIOPinWrite(LATCH_PORT, LATCH_PIN, LATCH_PIN);
		FAST_GPIOPinWrite(LATCH_PORT, LATCH_PIN, 0);
	}

private:
	uint8_t		currentRow;
	uint8_t		currentIntensity;

public:
	uint16_t	fb[R][C]; // Holds actual framebuffer data
};


template <class T, unsigned int R, unsigned int C>
class LedMatrix
{
public:
	LedMatrix() 
		: animation(NULL),
		  scrollAnim(defaultFont)
	{
	}

	inline void setChar(char c, LedMatrixColor &color) {
		frameBuffer.setChar(c, color, defaultFont);
	}

	void setMessage(char *str, uint16_t len) {
		clear();
		scrollAnim.setMessage(str, len);
		setAnimation(&scrollAnim, 5);
	}

	void appendMessage(char *str, uint16_t len) {
		scrollAnim.appendMessage(str, len);
	}

	void clear() {
		LedMatrixColor blank(0,0,0);
		clear(blank);
	}
	void clear(LedMatrixColor &color) {
		frameBuffer.clear(color);
	}

	bool update() {
		bool frameDone = frameBuffer.tick();
		if( frameDone && animation != NULL ) {
			animCountdown--;
			if( animCountdown == 0 ) {
				animTick();
				animCountdown = animInterval;
			}
		}
		return frameDone;
	}

	void animTick() {
		if( animation != NULL ) {
			animation->update(frameBuffer);
		}
	}

	void setAnimation(LedMatrixAnimation<R,C> *animation, uint8_t interval) {
		animInterval = interval;
		if( animInterval == 0 ) {
			animInterval = 1;
		}
		animCountdown = animInterval;
		this->animation = animation;
	}
	uint8_t getAnimationInterval();

private:
	T 				defaultFont;
	LedMatrixAnimation<R,C>		*animation;
	LedMatrixFrameBuffer<R,C>	frameBuffer; 
	LedMatrixScrollAnimation<R,C>	scrollAnim;
	uint8_t				animInterval;
	uint8_t				animCountdown;

};

#endif
