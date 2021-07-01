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
		P_point.p1 = i.u_L;
		P_point.p2 = (i.v_L - i.u_L)/(ConstantParameter->C7*i.u_L+ ConstantParameter->C8);
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
#define param ConstantParameter
		BasicPoint P_point = i;
		P_point.p1 = (param->kx_l*i.x + param->ks_l*i.y + param->u0_l*i.z) / i.z;
		P_point.p2 = ((-1)*param->kx_l*i.x + param->C27*i.y + param->C28*i.z) / (param->C29*i.x + param->C30*i.y + param->C31*i.z + param->C8*i.z);
		P_point.p3 = (param->C19*i.x + param->C20*i.y + param->C21*i.z + param->C22) / (param->nz*i.x + param->oz*i.y + param->az*i.z + param->tz);;

		this->Points.push_back(P_point);
	}
	return false;
}





std::vector<int> a;

