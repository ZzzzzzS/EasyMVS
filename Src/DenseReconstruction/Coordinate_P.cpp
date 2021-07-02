#include "Coordinate_P.h"
Coordinate_P::Coordinate_P(CoordinateConvertParams * ConstantParameter)
	:ConstantParameter(ConstantParameter)
{
}  

bool Coordinate_P::convertPFrom_Q(BasicData Q)
{
	for (auto i : Q.Points)
	{
		BasicPoint P_point = i;
		P_point.p1 = (i.v_L - ConstantParameter->C8* i.u_L)/(ConstantParameter->C7*i.u_L+ 1);
		P_point.p2 = i.u_L;
		P_point.p3 = i.u_R;
		//约束未加

		 this->Points.push_back(P_point);
	}
	return false;
}

bool Coordinate_P::convertPFrom_X(BasicData X)
{
	for (auto i : X.Points)
	{
		CoordinateConvertParams *param = ConstantParameter;
		BasicPoint P_point = i;
		P_point.p1 = (param->C27*i.x + param->C28*i.y + param->C29*i.z) / (param->C30*i.x + param->C31*i.y + param->C32*i.z);
		P_point.p2 = param->kx_l*i.x/i.z + param->ks_l*i.y / i.z + param->u0_l;
		P_point.p3 = (param->C19*i.x + param->C20*i.y + param->C21*i.z + param->C22) / (param->nz*i.x + param->oz*i.y + param->az*i.z + param->tz);

		this->Points.push_back(P_point);
	}
	return false;
}
