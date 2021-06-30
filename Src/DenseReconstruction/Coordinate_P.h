#pragma once
#include "BasicData.h"
#include "Coordinate_Q.h"
#include "Coordinate_X.h"

class Coordinate_P :
	public BasicData
{
public:

	Coordinate_P()
	{

	}
	BasicData a;

public:
	bool convertFrom_Q(BasicData);
	bool convertFrom_X(BasicData);
};

