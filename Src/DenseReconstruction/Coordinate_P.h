#pragma once
#include "BasicData.h"
#include "Coordinate_Q.h"
#include "Coordinate_X.h"
#include "CoordinateConvertParams.h"

class Coordinate_P :
	public BasicData
{
public:

	Coordinate_P()
	{

	}


public:
	Coordinate_P(CoordinateConvertParams* ConstantParameter);

	bool convertPFrom_Q(BasicData);
	bool convertPFrom_X(BasicData);

private:
	CoordinateConvertParams* ConstantParameter;
};

