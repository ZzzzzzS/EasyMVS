#pragma once
#include "BasicData.h"
#include "Coordinate_P.h"
#include "Coordinate_Q.h"
#include "CoordinateConvertParams.h"

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
	
	Coordinate_X(CoordinateConvertParams* ConstantParameter);

	bool convertFrom_Q(BasicData& Q);
	bool convertFrom_P(BasicData& P);

private:
	CoordinateConvertParams* ConstantParameter;

};

