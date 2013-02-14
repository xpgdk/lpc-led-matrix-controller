#ifndef _LED_MATRIX_HPP
#define _LED_MATRIX_HPP

#include <stdint.h>

template <class T> class LedMatrix;
class LedMatrixFrameBuffer;

struct LedMatrixColor
{
public:
	LedMatrixColor(uint8_t red, uint8_t green, uint8_t blue) {
		blue = blue;
		color = (red & 0xFF) | ((green & 0xFF)<<4);
	}

	const inline uint16_t getData() {
		return color;
	}
private:
	uint16_t color;
};

class LedMatrixAnimation
{
public:
	virtual void update(LedMatrixFrameBuffer &fb) = 0;
};

class LedMatrixScrollAnimation : LedMatrixAnimation
{
public:
	void update(LedMatrixFrameBuffer &fb);
};

class LedMatrixFrameBuffer
{
};

class LedMatrixFont
{
public:
	virtual const uint8_t* 	getFontData() = 0;
	virtual uint8_t		getFontWidth() = 0;
	virtual uint8_t		getFontHeight() = 0;
};

template <class T>
class LedMatrix
{
public:
	void setMessage(char *str, uint16_t len);
	void clear();
	void clear(LedMatrixColor color);
	void update();

private:
	T defaultFont;
};

#endif
