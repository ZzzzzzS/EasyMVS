#include "Coordinate_Q.h"
Coordinate_Q::Coordinate_Q(CoordinateConvertParams * ConstantParameter)
	:ConstantParameter(ConstantParameter)
{
}

bool Coordinate_Q::convertQFrom_P(BasicData P)
{
	for (auto i : P.Points)
	{
		CoordinateConvertParams *param = ConstantParameter;
		BasicPoint Q_point = i;
		Q_point.u_L = i.p2;
		Q_point.v_L = param->C7*i.p1*i.p2+param->C8*i.p2+i.p1;
		Q_point.u_R	= i.p3;
		Q_point.v_R = param->C13*i.p1*i.p2*i.p3 + param->C14*i.p2*i.p3 + param->C15*i.p1*i.p2 + param->C2*i.p1*i.p3 + param->C11*i.p1 + param->C16*i.p2 + param->C3*i.p3 + param->C12;
		Q_point.v_R /= param->C17 * i.p1*i.p2 + param->C5*i.p1 + param->C18*i.p2 + param->C6;

		this->Points.push_back(Q_point);
	}
	return false;
}

bool Coordinate_Q::convertQFrom_X(BasicData X)
{
	for (auto i : X.Points)
	{
		CoordinateConvertParams *param = ConstantParameter;
		BasicPoint Q_point = i;
		Q_point.u_L = param->kx_l*i.x / i.z + param->ks_l*i.y / i.z + param->u0_l;
		Q_point.v_L = param->ky_l*i.y / i.z + param->v0_l;
		Q_point.u_R = param->C19*i.x + param->C20*i.y + param->C21*i.z + param->C22;
		Q_point.u_R /= param->nz*i.x + param->oz*i.y + param->az*i.z + param->tz;
		Q_point.v_R = param->C23*i.x + param->C24*i.y + param->C25*i.z + param->C26;
		Q_point.v_R /= param->nz*i.x + param->oz*i.y + param->az*i.z + param->tz;

		this->Points.push_back(Q_point);
	}
	return false;
}
