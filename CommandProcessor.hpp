#ifndef _COMMAND_PROCESSOR_HPP
#define _COMMAND_PROCESSOR_HPP

/*
struct CommandProcessorConfig {
public:
	typedef XXXX 		FrameBuffer;
	static const int	ScreenOffsetX = 0;
	static const int	ScreenOffsetY = 0;
	static const bool	ScreenRotate = false;
	static const bool	ScreenFlipY = false;
};
*/

template<typename Config>
class CommandProcessor {
public:
	typedef typename Config::FrameBuffer 	FrameBuffer;
	typedef typename Config::Matrix		Matrix;

	CommandProcessor(FrameBuffer *frameBuffer, Matrix &matrix) 
		: matrix(matrix), state(Idle), frameBuffer(frameBuffer) {
		currentFrameBuffer = 0;
	}

	void reset() {
		state = Idle;
	}

	void handleByte(uint8_t data) {
		switch(state) {
			case Idle:
				if( data == PutRect) {
					state = MsgPutRectArgs;
					current_data_count = 0;
				} else if( data == Flip) {
					//frameBuffer.flipBuffers();
					currentFrameBuffer = !currentFrameBuffer;
					matrix.changeFrameBuffer(&frameBuffer[currentFrameBuffer]);
					matrix.clearAnimation();
				} else {
					/*putHex8(data);
					printf("\r\n");*/
					state = Idle;
				}
				break;
			case MsgPutRectArgs:
				data_buffer[current_data_count] = data;
				current_data_count++;
				if( current_data_count >= 8 ) {
					startX = (data_buffer[0] << 8) | data_buffer[1];
					startY = (data_buffer[2] << 8) | data_buffer[3];
					endX = (data_buffer[4] << 8) | data_buffer[5];
					endY = (data_buffer[6] << 8) | data_buffer[7];
					data_count = (endX-startX)*(endY-startY)*2;
					current_data_count = 0;
#ifdef DEBUG
					printf("Drawing from (");
					putHex16(startX);
					printf(",");
					putHex16(startY);
					printf(") to (");
					putHex16(endX);
					printf(",");
					putHex16(endY);
					printf(")\r\n");
					printf("Expecting ");
					putHex16(data_count);
					printf(" bytes\r\n");
#endif
					state = MsgPutRectData;
					currentX = startX;
					currentY = startY;
				}
				break;
			case MsgPutRectData:
				data_buffer[current_data_count%2] = data;
				if( current_data_count % 2 == 1 ) {
					uint16_t r = data_buffer[0];
					uint16_t g = data_buffer[1];
					//LedMatrixColor color(r,g, 0);
					uint16_t color = LedMatrixColor::getValue(r, g, 0);
					/*uint16_t y = ((current_data_count/2)/(endX-startX)) + startY;
					  uint16_t x = ((current_data_count/2)%(endX-startX)) + startX;*/
					uint16_t x = currentX;
					uint16_t y = currentY;
					x -= Config::ScreenOffsetX;
					y -= Config::ScreenOffsetY;
					if( Config::ScreenRotate ) {
						uint16_t tmp = x;
						x = FrameBuffer::getColCount() - y - 1;
						y = tmp;
					}
					if( Config::ScreenFlipY ) {
						y = FrameBuffer::getRowCount() - y - 1;
						x = FrameBuffer::getColCount() - x - 1;
					}
#ifdef DEBUG
					printf("Drawing pixel at ");
					putHex16(x);
					printf(",");
					putHex16(y);
					printf("\r\n");
#endif
#if 1
					if( x < FrameBuffer::getColCount() &&
							y < FrameBuffer::getRowCount() ) {
						frameBuffer[!currentFrameBuffer].putPixel(x,y,color);
					}
#endif
					currentX++;
					if( currentX >= endX ) {
						currentX = startX;
						currentY++;
					}
				}
				current_data_count++;
				if( current_data_count >= data_count ) {
					state = Idle;
				}
				break;
			case Invalid:
				printf("Ignoring data in invalid state: ");
				putHex8(data);
				printf("\r\n");
				break;
		}
	}

	typedef enum {
		Idle = 0,
		Invalid,
		MsgPutRectArgs,
		MsgPutRectData
	} State;

	typedef enum {
		Append 	= 0x02,
		PutRect = 0x04,
		Flip	= 0x05
	} Command;

private:
	Matrix		&matrix;
	State		state;
	FrameBuffer	*frameBuffer;
	uint8_t 	data_buffer[100];
	uint16_t 	data_count;
	uint16_t 	current_data_count;

	uint16_t 	startX, startY, endX, endY;
	uint16_t 	currentX, currentY;
	uint8_t		currentFrameBuffer;
};

#endif
