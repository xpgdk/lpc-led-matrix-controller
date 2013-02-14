#include "LedMatrix.hpp"

void
LedMatrixScrollAnimation::update(LedMatrixFrameBuffer &fb)
{
}

template <class T>
void
LedMatrix<T>::setMessage(char *str, uint16_t len)
{
	str = str;
	len = len;
}

template <class T>
void
LedMatrix<T>::clear()
{
	clear(LedMatrixColor(0,0,0));
}

template <class T>
void
LedMatrix<T>::clear(LedMatrixColor color) {
}

template <class T>
void
LedMatrix<T>::update()
{
}
