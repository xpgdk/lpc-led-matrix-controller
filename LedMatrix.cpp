#include "LedMatrix.hpp"

int
LedMatrixFont::getStart(char c) 
{
	const uint8_t *fontChar = getFontData()[c-32];

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

int
LedMatrixFont::getEnd(char c)
{
	const uint8_t *fontChar = getFontData()[c-32];

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
