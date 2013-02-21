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

template <unsigned R, unsigned C> class LedMatrixFrameBuffer;
class AbstractLedMatrixFrameBuffer;

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

class AbstractLedMatrixFrameBuffer
{
public:
	virtual uint16_t getRowCount() = 0;
	virtual uint16_t getColCount() = 0;
	virtual void setChar(char c, LedMatrixColor &color, LedMatrixFont &font) = 0;
	virtual bool tick() = 0;
	virtual void clear(LedMatrixColor &color) = 0;
	virtual void clear() = 0;
	virtual uint16_t* operator [](int index) = 0;
	virtual void fillRow(uint16_t row, LedMatrixColor &color) = 0;
};

template <unsigned int R, unsigned int C>
class LedMatrixFrameBuffer : public AbstractLedMatrixFrameBuffer
{
public:
	LedMatrixFrameBuffer() {
		currentRow = 0;
		currentIntensity = 0;
		rowReset();
	}

	uint16_t getRowCount() {
		return R;
	}

	uint16_t getColCount() {
		return C;
	}

	void setChar(char c, LedMatrixColor &color, LedMatrixFont &font) {
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

	void clear() {
		LedMatrixColor blank(0,0,0);
		clear(blank);
	}

	void fillRow(uint16_t row, LedMatrixColor &color) {
		for(uint16_t c = 0; c<getColCount(); c++) {
			fb[row][c] = color.getValue();
		}
	}

	uint16_t* operator [](int index) {
		return (uint16_t*)fb[index];
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

class LedMatrixAnimation
{
public:
	virtual void reset() = 0;
	virtual bool update(AbstractLedMatrixFrameBuffer &fb) = 0;
};

class LedMatrixScrollAnimation : public LedMatrixAnimation
{
public:
	LedMatrixScrollAnimation(LedMatrixFont &font) 
		: msgLen(0), font(font) {
	}

	void reset() {
		offset = 0;
		nextChar = 0;
		currentColor = LedMatrixColor(0x3F, 0, 0);
		parseColor();
	}

	bool update(AbstractLedMatrixFrameBuffer &fb) {
		bool restarted = false;
		for(uint16_t i=0; i<fb.getRowCount(); i++) {
			for(uint16_t l=0; l<fb.getColCount()-1; l++) {
				fb[i][l] = fb[i][l+1];
			}
			fb[i][fb.getColCount()-1] = ((font.getFontData()[msgBuffer[nextChar]-32][i] >> (7-offset)) & 0x1) * currentColor.getValue();
		}
		offset++;
		if( offset >= font.getEnd(msgBuffer[nextChar]) ) {
			nextChar++;
			parseColor();

			if( nextChar >= msgLen ) {
				nextChar = 0;
				restarted = true;
				parseColor();
			}
			offset = font.getStart(msgBuffer[nextChar]);
		}
		return restarted;
	}

	void setMessage(char *msg, uint16_t len) {
		strncpy(msgBuffer, msg, len);
		msgLen = len;
		reset();
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



class LedMatrix
{
public:
	LedMatrix(AbstractLedMatrixFrameBuffer &fb, LedMatrixFont &font) 
		: defaultFont(font), frameBuffer(fb), animation(NULL)
	{
	}

	inline void setChar(char c, LedMatrixColor &color) {
		frameBuffer.setChar(c, color, defaultFont);
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

	void setAnimation(LedMatrixAnimation *animation, uint8_t interval) {
		animInterval = interval;
		if( animInterval == 0 ) {
			animInterval = 1;
		}
		animCountdown = animInterval;
		this->animation = animation;
	}

	void setAnimationInterval(uint8_t interval) {
		animInterval = interval;
		if( animInterval == 0 ) {
			animInterval = 1;
		}
		animCountdown = animInterval;
	}

	uint8_t getAnimationInterval();

private:
	LedMatrixFont 			&defaultFont;
	AbstractLedMatrixFrameBuffer	&frameBuffer; 
	LedMatrixAnimation		*animation;
	uint8_t				animInterval;
	uint8_t				animCountdown;

};

#endif
