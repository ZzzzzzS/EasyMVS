﻿#include "CoordinateConvertParams.h"

bool CoordinateConvertParams::load_Min(std::string &filename)
{
	//cv::Mat Min_l(3, 3, CV_32FC1);
	//cv::Mat Min_r(3, 3, CV_32FC1);

	////cv::Mat	stereoR(3, 3, CV_32FC1);
	////cv::Mat	stereoT(3, 1, CV_32FC1);
	//////cv::Mat T_r2l(4, 4, CV_32FC1, cv::Scalar(0.0));

	////cv::FileStorage fs;
	////if (filename.empty())
	////	return false;
	////fs.open(filename, cv::FileStorage::READ);
	////fs["camIntrinsics"] >> Min_l;
	////fs["camDistCoeffs"] >> Min_r;
	////fs["stereoR"] >> stereoR;
	////fs["stereoT"] >> stereoT;
	////fs.release();

	////Min_l.convertTo(Min_l, CV_32FC1);//将cv::Mat中只有的8bit,16bit 的uchar 转换成32float
	////Min_r.convertTo(Min_r, CV_32FC1);
	////stereoR.convertTo(stereoR, CV_32FC1);
	////stereoT.convertTo(stereoT, CV_32FC1);

	////for (int i = 0; i < 3; i++)
	////	for (int j = 0; j < 3; j++)
	////		T_r2l.at<float>(i, j) = stereoR.at<float>(i, j);
	////for (int i = 0; i < 3; i++)
	////	T_r2l.at<float>(i, 3) = stereoT.at<float>(i, 0);
	////T_r2l.at<float>(3, 3) = 1.0;

	Min_l = (cv::Mat_<float>(3, 3) <<
		7000, 0,960,
		0, 7000, 600,
		0, 0, 1);
		Min_r = (cv::Mat_<float>(3, 3) <<
			7000, 0, 960,
			0, 7000, 600,
			0, 0, 1);
		T_r2l = (cv::Mat_<float>(4, 4) <<
			0.866, -0.5, 0, 150,
			0.5, 0.866, 0, 0,
			0, 0, 1, 10,
			0, 0, 0, 1);
	cv::invert(Min_l, Min_l_t);
	cv::invert(Min_r, Min_r_t);

	kx_r = Min_r.at<float>(0, 0);
	ky_r = Min_r.at<float>(1, 1);
	ks_r = Min_r.at<float>(0, 1);
	u0_r = Min_r.at<float>(0,2);
	v0_r = Min_r.at<float>(1, 2);

	kx_l = Min_l.at<float>(0, 0);
	ky_l = Min_l.at<float>(1, 1);
	ks_l = Min_l.at<float>(0, 1);
	u0_l = Min_l.at<float>(0, 2);
	v0_l = Min_l.at<float>(1, 2);

	hu_r = Min_r_t.at<float>(0, 0);
	hv_r = Min_r_t.at<float>(1, 1);
	hs_r = Min_r_t.at<float>(0, 1);
	X0_r = Min_r_t.at<float>(0, 2);
	Y0_r = Min_r_t.at<float>(1, 2);

	hu_l = Min_l_t.at<float>(0, 0);
	hv_l = Min_l_t.at<float>(1, 1);
	hs_l = Min_l_t.at<float>(0, 1);
	X0_l = Min_l_t.at<float>(0, 2);
	Y0_l = Min_l_t.at<float>(1, 2);

	nx = T_r2l.at<float>(0, 0);
	ny = T_r2l.at<float>(1, 0);
	nz = T_r2l.at<float>(2, 0);
	ox = T_r2l.at<float>(0, 1);
	oy = T_r2l.at<float>(1, 1);
	oz = T_r2l.at<float>(2, 1);
	ax = T_r2l.at<float>(0, 2);
	ay = T_r2l.at<float>(1, 2);
	az = T_r2l.at<float>(2, 2);
	tx = T_r2l.at<float>(0, 3);
	ty = T_r2l.at<float>(1, 3);
	tz = T_r2l.at<float>(2, 3);
	return false;
}

bool CoordinateConvertParams::calibrate_e()
{
	cv::Mat T_Min_r =cv::Mat(4,4,CV_32FC1);
	cv::Mat T_Min_l_t = cv::Mat(4, 4, CV_32FC1);

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			T_Min_r.at<float>(i, j) = Min_r.at<float>(i, j);
	for (int i = 0; i < 3; i++)
	{
		T_Min_r.at<float>(i, 3) = 0;
		T_Min_r.at<float>(3, i) = 0;
	}
	T_Min_r.at<float>(3, 3) = 1.0;

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			T_Min_l_t.at<float>(i, j) = Min_l_t.at<float>(i, j);
	for (int i = 0; i < 3; i++)
	{
		T_Min_l_t.at<float>(i, 3) = 0;
		T_Min_l_t.at<float>(3, i) = 0;
	}
	T_Min_l_t.at<float>(3, 3) = 1.0;

	cv::Mat MTM = cv::Mat(4, 4, CV_32FC1);
	MTM = T_Min_r * T_r2l*T_Min_l_t;

	e11 = MTM.at<float>(0,0);
	e12 = MTM.at<float>(0, 1);
	e13 = MTM.at<float>(0, 2);
	e14 = MTM.at<float>(0, 3);
	e21 = MTM.at<float>(1, 0);
	e22 = MTM.at<float>(1, 1);
	e23 = MTM.at<float>(1, 2);
	e24 = MTM.at<float>(1, 3);
	e31 = MTM.at<float>(2, 0);
	e32 = MTM.at<float>(2, 1);
	e33 = MTM.at<float>(2, 2);
	e34 = MTM.at<float>(2, 3);

	return false;
}

bool CoordinateConvertParams::calibrate_A()
{
	if (e34 == 0)
	{
		A1 = e14 * e32 - e12 * e34;
		A2 = e14 * e33 - e13 * e34;
		A3 = e14 * e32 * C7 - e12 * e34*C7;
		A4 = e14 * e32*C8 - e12 * e34*C8 - e11 * e34 + e14 * e31;
		A5 = 0;
		A6 = 0;
		A7 = 0;
		A8 = 0;
		A9 = hs_l * C7;
		A10 = hu_l + hs_l * C8;
		A11 = hs_l;
		A12 = X0_l;
		A13 = hv_l * C7;
		A14 = hv_l * C8;
		A15 = hv_l;
		A16 = Y0_l;
	}
	else
	{
		A1 = e14 * e32 - e12 * e34;
		A2 = e14 * e33 - e13 * e34;
		A3 = e14 * e32 * C7 - e12 * e34*C7;
		A4 = e14 * e32*C8 - e12 * e34*C8 - e11 * e34 + e14 * e31;
		A5 = (-1)*((e32*C7) / e34);
		A6 = (-1)*((e32*C8 + e31) / e34);
		A7 = (-1)*(e32 / e34);
		A8 = (-1)*(e33 / e34);
		A9 = hs_l * C7;
		A10 = hu_l + hs_l * C8;
		A11 = hs_l;
		A12 = X0_l;
		A13 = hv_l * C7;
		A14 = hv_l * C8;
		A15 = hv_l;
		A16 = Y0_l;
	}

	return false;
}

bool CoordinateConvertParams::calibrate_B()
{
	B1 = (-1)*(e34*e34*A5);
	B2 = A1 + e14 * e34*A5;
	B3 = (-1)*(e34*e34*A7);
	B4 = (-1)*(e34*e34*A6);
	B5 = e14 * e34*A7 + A3;
	B6 = e14 * e34*A6 + A2;
	B7 = (-1)*(e34*e34*A8);
	B8 = e14 * e34*A8 + A4;
	B9 = e14 * e34;
	B10 = (-1)*e34*e34;

	return false;
}

bool CoordinateConvertParams::calibrate_C()
{
	if (e34 == 0)
	{
		C1 = e34 * e21 - e24 * e31;
		C2 = e34 * e22 - e24 * e32;
		C3 = e34 * e23 - e24 * e33;
		C4 = e34 * e11 - e14 * e31;
		C5 = e34 * e12 - e14 * e32;
		C6 = e34 * e13 - e14 * e33;
		C7 = (e11*e22*e34 - e11 * e24*e32 + e12 * e24*e31 - e12 * e21*e34 - e14 * e22*e31 + e14 * e21*e32) /
			(e12*e24*e33 - e12 * e23*e34 + e13 * e22*e34 - e13 * e24*e32 + e14 * e23*e32 - e14 * e22*e33);
		C8 = (e11*e23*e34 - e11 * e24*e33 + e13 * e24*e31 - e13 * e21*e34 - e14 * e21*e33 + e14 * e23*e31) /
			(e12*e24*e33 - e12 * e23*e34 + e13 * e22*e34 - e13 * e24*e32 + e14 * e23*e32 - e14 * e22*e33);
		C9 = 0;//
		C10 =0;
		C11 = 0;
		C12 = 0;
		C13 = C2 * C7;
		C14 = C1 + C2 * C8;
		C15 = C7 * C11;
		C16 = C10 + C8 * C11;
		C17 = C5 * C7;
		C18 = C4 + C5 * C8;
		C19 = kx_r * nx + ks_r * ny + u0_r * nz;
		C20 = kx_r * ox + ks_r * oy + u0_r * oz;
		C21 = kx_r * ax + ks_r * ay + u0_r * az;
		C22 = kx_r * tx + ks_r * ty + u0_r * tz;
		C23 = ky_r * ny + v0_r * nz;
		C24 = ky_r * oy + v0_r * oz;
		C25 = ky_r * ay + v0_r * az;
		C26 = ky_r * ty + v0_r * tz;

		C27 = (-1)*C8*kx_l;
		C28 = (-1)*C8*ks_l + ky_l;
		C29 = v0_l - C8 * u0_l;
		C30 = C7 * kx_l;
		C31 = C7 * ks_l;
		C32 = C7 * u0_l + 1;
	}
	else
	{
		C1 = e34 * e21 - e24 * e31;
		C2 = e34 * e22 - e24 * e32;
		C3 = e34 * e23 - e24 * e33;
		C4 = e34 * e11 - e14 * e31;
		C5 = e34 * e12 - e14 * e32;
		C6 = e34 * e13 - e14 * e33;
		C7 = (e11*e22*e34 - e11 * e24*e32 + e12 * e24*e31 - e12 * e21*e34 - e14 * e22*e31 + e14 * e21*e32) /
			(e12*e24*e33 - e12 * e23*e34 + e13 * e22*e34 - e13 * e24*e32 + e14 * e23*e32 - e14 * e22*e33);
		C8 = (e11*e23*e34 - e11 * e24*e33 + e13 * e24*e31 - e13 * e21*e34 - e14 * e21*e33 + e14 * e23*e31) /
			(e12*e24*e33 - e12 * e23*e34 + e13 * e22*e34 - e13 * e24*e32 + e14 * e23*e32 - e14 * e22*e33);
		C9 = 0;//
		C10 = (C4*e24 - C1 * e14) / e34;
		C11 = (C5*e24 - C2 * e14) / e34;
		C12 = (C6*e24 - C3 * e14) / e34;
		C13 = C2 * C7;
		C14 = C1 + C2 * C8;
		C15 = C7 * C11;
		C16 = C10 + C8 * C11;
		C17 = C5 * C7;
		C18 = C4 + C5 * C8;
		C19 = kx_r * nx + ks_r * ny + u0_r * nz;
		C20 = kx_r * ox + ks_r * oy + u0_r * oz;
		C21 = kx_r * ax + ks_r * ay + u0_r * az;
		C22 = kx_r * tx + ks_r * ty + u0_r * tz;
		C23 = ky_r * ny + v0_r * nz;
		C24 = ky_r * oy + v0_r * oz;
		C25 = ky_r * ay + v0_r * az;
		C26 = ky_r * ty + v0_r * tz;

		C27 = (-1)*C8*kx_l;
		C28 = (-1)*C8*ks_l + ky_l;
		C29 = v0_l - C8 * u0_l;
		C30 = C7 * kx_l;
		C31 = C7 * ks_l;
		C32 = C7 * u0_l + 1;
	}

	return false;
}
