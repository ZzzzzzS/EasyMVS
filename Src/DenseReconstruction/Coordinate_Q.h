#pragma once
#include "BasicData.h"
#include "Coordinate_X.h"
#include "Coordinate_P.h"

class Coordinate_Q :
	public BasicData
{
public:

	Coordinate_Q()
	{
		 
	}

	BasicData a;
public:
	bool convertFrom_P(BasicData);
	bool convertFrom_X(BasicData);
};

