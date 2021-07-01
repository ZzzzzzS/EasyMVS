#pragma once
#include "BasicData.h"
#include "Coordinate_X.h"
#include "Coordinate_P.h"
#include "CoordinateConvertParams.h"

class Coordinate_Q :
	public BasicData
{
public:

	Coordinate_Q()
	{
		 
	}

	BasicData a;
public:
	Coordinate_Q(CoordinateConvertParams* ConstantParameter);

	bool convertQFrom_P(BasicData);
	bool convertQFrom_X(BasicData);

private:
	CoordinateConvertParams* ConstantParameter;
};

