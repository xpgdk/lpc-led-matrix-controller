#ifndef _LED_MATRIX_SIMPLE_FONT_HPP
#define _LED_MATRIX_SIMPLE_FONT_HPP

#include "LedMatrix.hpp"

class LedMatrixSimpleFont : public LedMatrixFont
{
public:
	const FontChar* 	getFontData();

	uint8_t		getFontWidth() {
		return 8;
	}

	uint8_t		getFontHeight() {
		return 8;
	}

private:
};

#endif
