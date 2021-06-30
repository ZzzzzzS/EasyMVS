#include "Coordinate_Q.h"


bool Coordinate_Q::convertFrom_P(BasicData p)
{
	return false;
}

bool Coordinate_Q::convertFrom_X(BasicData x)
{
	this->a.i = x.i + 2;
	return false;
}
