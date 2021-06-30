#include "CoordinateConvertParams.h"

bool CoordinateConvertParams::load_Min(std::string &filename)
{
	cv::Mat camK(3, 3, CV_32FC1);
	cv::Mat	camDistCoeffs(1, 5, CV_32FC1);
	cv::Mat	stereoR(3, 3, CV_32FC1);
	cv::Mat	stereoT(3, 1, CV_32FC1);
	cv::Mat T_c2p(4, 4, CV_32FC1, cv::Scalar(0.0));

	cv::FileStorage fs;
	if (filename.empty())
		return false;
	fs.open(filename, cv::FileStorage::READ);
	fs["camIntrinsics"] >> camK;
	fs["camDistCoeffs"] >> camDistCoeffs;
	fs["stereoR"] >> stereoR;
	fs["stereoT"] >> stereoT;
	fs.release();

	camK.convertTo(camK, CV_32FC1);//将cv::Mat中只有的8bit,16bit 的uchar 转换成32float
	camDistCoeffs.convertTo(camDistCoeffs, CV_32FC1);
	stereoR.convertTo(stereoR, CV_32FC1);
	stereoT.convertTo(stereoT, CV_32FC1);

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			T_c2p.at<float>(i, j) = stereoR.at<float>(i, j);
	for (int i = 0; i < 3; i++)
		T_c2p.at<float>(i, 3) = stereoT.at<float>(i, 0);
	T_c2p.at<float>(3, 3) = 1.0;

	return false;
}

bool CoordinateConvertParams::calibrate_e()
{
	
	return false;
}

bool CoordinateConvertParams::calibrate_A()
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
	C18 = C4 + C5*C8;
	C19 = kx_r * nx + ks_r * ny + u0_r * nz;
	C20 = kx_r *ox + ks_r * oy + u0_r * oz;
	C21 = kx_r * ax + ks_r * ay + u0_r * az;
	C22 = kx_r * tx + ks_r * ty + u0_r * tz;
	C23 = ky_r * ny + v0_r * nz;
	C24 = ky_r * oy + v0_r * oz;
	C25 = ky_r * ay + v0_r * az;
	C26 = ky_r * ty + v0_r * tz;
	C27 = (-1)*ks_l + ky_l;
	C28 = v0_l - u0_l;
	C29 = C7 * kx_l;
	C30 = C7 * ks_l;
	C31 = C7 * u0_l;

	return false;
}
