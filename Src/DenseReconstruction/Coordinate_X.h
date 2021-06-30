#pragma once
#include "BasicData.h"
#include "Coordinate_P.h"
#include "Coordinate_Q.h"

class Coordinate_X :
	public BasicData
{
public:

	Coordinate_X()
	{

	}
	BasicData a;
	int b = 2;
public:
	bool convertFrom_Q(BasicData);
	bool convertFrom_P(BasicData);

};

