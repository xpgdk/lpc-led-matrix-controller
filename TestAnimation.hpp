#ifndef _TEST_ANIMATION_HPP
#define _TEST_ANIMATION_HPP

#include "LedMatrix.hpp"

class LedMatrixTestAnimation : public LedMatrixAnimation {
public:
	LedMatrixTestAnimation(LedMatrix &matrix, LedMatrixAnimation &anim1)
		: matrix(matrix), animation1(anim1),
	 	  mode(CLEAR),
		  yellow(4, 32,0) {
			  matrix.setAnimationInterval(1);
	}

	bool update(AbstractLedMatrixFrameBuffer &fb) {
		bool done = false;
		switch(mode) {
			case CLEAR:
				counter++;

				{
					LedMatrixColor color(3, counter, 0x00);
					fb.clear(color);
				}

				if( counter > 32 ) {
					matrix.setAnimationInterval(5);
					mode = ANIM1;
					counter = 0;
				}
			break;
			case ANIM1:
				if( animation1.update(fb) ) {
					matrix.setAnimationInterval(1);
					mode = FILL;
					fb.clear();
					counter = 0;
				}
			break;
			case FILL:
				fb[fillY][fillX] = yellow.getValue();
				fillX++;
				if( fillX >= fb.getColCount() ) {
					fillX = 0;
					fillY++;
				}
				if( fillY >= fb.getRowCount() ) {
					fillY = 0;
					mode = SCAN;
					matrix.setAnimationInterval(4);
				}
			break;
			case SCAN:
				counter++;
				fb.clear();
				fb.fillRow(fillY, yellow);
				
				if( fillX == 0) {
					fillY++;
					if( fillY >= fb.getRowCount() ) {
						fillX = 1;
						fillY -= 2;
					}
				} else {
					if( fillY == 0 ) {
						fillY++;
						fillX = 0;
					} else {
						fillY--;
					}
				}
				if( counter > 5 * fb.getRowCount() ) {
					counter = 0;
					fillX = fillY = 0;
					mode = CLEAR;
					matrix.setAnimationInterval(1);
					done = true;
				}
			break;
		}

		return done;
	}

	void reset() {
		counter = 0;
		fillX = fillY = 0;
		mode = CLEAR;
	}


private:
	enum Mode {CLEAR, ANIM1, FILL, SCAN};

	LedMatrix		&matrix;
	LedMatrixAnimation	&animation1;
	Mode		   	mode;
	int		   	counter;
	int			fillX, fillY;
	LedMatrixColor		yellow;
};

#endif
