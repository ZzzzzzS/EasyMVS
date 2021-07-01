#include "Coordinate_X.h"

Coordinate_X::Coordinate_X(CoordinateConvertParams * ConstantParameter)
	:ConstantParameter(ConstantParameter)
{
}

bool Coordinate_X::convertXFrom_P(BasicData& P)
{
	this->Points.clear();
	CoordinateConvertParams* Para = this->ConstantParameter;
	for (auto i : P.Points)
	{
		float x1, x2, x3;

		//calculate x1 with three parts*********************************************
		float x1_Interval1 = 
			(Para->A9*i.p1*i.p2)
			+ (Para->A10*i.p2)
			+ (Para->A11*i.p1)
			+ (Para->A12);

		float x1_Interval2 = 
			(Para->B1*i.p1*i.p2*i.p3)
			+ (Para->B2*i.p1*i.p2)
			+ (Para->B3*i.p1*i.p3)
			+ (Para->B4*i.p2*i.p3)
			+ (Para->B5*i.p1)
			+ (Para->B6*i.p2)
			+ (Para->B7*i.p3)
			+ (Para->B8);

		float x1_Interval3 = (Para->B9) + (Para->B10*i.p3);

		x1 = x1_Interval1 * x1_Interval2 / x1_Interval3; //combine three parts
		
		//calculate x2*************************************************
		float x2_Interval1 = 
			(Para->A13*i.p1*i.p2)
			+ (Para->A14*i.p2)
			+ (Para->A15*i.p1)
			+ (Para->A16);

		float x2_Interval2 =
			(Para->B11*i.p1*i.p2*i.p3)
			+ (Para->B12*i.p1*i.p2)
			+ (Para->B13*i.p1*i.p3)
			+ (Para->B14*i.p2*i.p3)
			+ (Para->B15*i.p1)
			+ (Para->B16*i.p2)
			+ (Para->B17*i.p3)
			+ (Para->B18);

		float x2_Interval3 = (Para->B9) + (Para->B10*i.p3);

		x2 = x2_Interval1 * x2_Interval2 / x2_Interval3;


		//calculate x3**************************************************
		float x3_Interval1 = 
			(Para->B11*i.p1*i.p2*i.p3) 
			+ (Para->B12*i.p1*i.p2) 
			+ (Para->B13*i.p1*i.p3) 
			+ (Para->B14*i.p2*i.p3) 
			+ (Para->B15*i.p1) 
			+ (Para->B16*i.p2) 
			+ (Para->B17*i.p3) 
			+ (Para->B18);
		
		float x3_Interval2 = (Para->B9) + (Para->B10*i.p3);


		x3 = x3_Interval1 / x3_Interval2;

		//开始赋能
		BasicPoint temp = i;
		temp.x = x1;
		temp.y = x2;
		temp.z = x3;
		this->Points.push_back(temp);
	}
	return true;
}



bool Coordinate_X::convertXFrom_Q(BasicData& Q)
{
	this->Points.clear();
	CoordinateConvertParams* Para = this->ConstantParameter;

	for (auto i : Q.Points)
	{
		//*********************************calculate x1
		float x1_Interval1 = ((Para->e14) - (Para->e34*i.q3))*
			((Para->hu_l*i.q1) + (Para->hs_l*i.q2) + (Para->X0_l));

		float x1_Interval2 =
			(Para->e31*i.q1*i.q2)
			+ (Para->e32*i.q2*i.q3)
			+ (Para->e33*i.q3)
			- (Para->e11*i.q1)
			- (Para->e12*i.q2)
			- (Para->e13);

		float x1 = x1_Interval1 / x1_Interval2;


		//***********************calculate x2
		float x2_Interval1 = ((Para->hv_l*i.q1) + (Para->Y0_l))*((Para->e14) - (Para->e34*i.q3));

		float x2_Interval2 = 
			(Para->e31*i.q1*i.q3) 
			+ (Para->e32*i.q2*i.q3) 
			+ (Para->e33*i.q3) 
			- (Para->e11*i.q1) 
			- (Para->e12*i.q2) 
			- (Para->e13);

		float x2 = x2_Interval1 / x2_Interval2;


		//**********************calculate x3
		float x3_Interval1 = (Para->e14) - (Para->e34*i.q3);
		float x3_Interval2 = 
			(Para->e31*i.q1*i.q3) 
			+ (Para->e32*i.q2*i.q3) 
			+ (Para->e33*i.q3) 
			- (Para->e11*i.q1) 
			- (Para->e12*i.q2) 
			- (Para->e13);

		float x3 = x3_Interval1 / x3_Interval2;

		//开始赋能
		BasicPoint temp = i;
		temp.x = x1;
		temp.y = x2;
		temp.z = x3;
		this->Points.push_back(temp);
	}
	return true;
}
