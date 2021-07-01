#pragma once
#include <opencv2/opencv.hpp>
#include <string>

class CoordinateConvertParams
{
public:
	CoordinateConvertParams()
	{
		std::string path = ".//params//param.xml";
		load_Min(path);
		calibrate_e();
		calibrate_C();
		calibrate_A();
		calibrate_B();
	}
	//内参矩阵
	cv::Mat Min_l = cv::Mat(3, 3, CV_32FC1);
	cv::Mat Min_r = cv::Mat(3, 3, CV_32FC1);

	cv::Mat T_r2l = cv::Mat(4, 4, CV_32FC1, cv::Scalar(0.0));

	//内参
	float kx_r = 0;
	float ky_r = 0;
	float ks_r = 0;
	float u0_r = 0;
	float v0_r = 0;

	float kx_l = 0;
	float ky_l = 0;
	float ks_l = 0;
	float u0_l = 0;
	float v0_l = 0;

	//内参的逆
	float hu_r = 0;
	float hv_r = 0;
	float hs_r = 0;
	float X0_r = 0;
	float Y0_r = 0;

	float hu_l = 0;
	float hv_l = 0;
	float hs_l = 0;
	float X0_l = 0;
	float Y0_l = 0;

	//外参
	float nx = 0;
	float ny = 0;
	float nz = 0;
	float ox = 0;
	float oy = 0;
	float oz = 0;
	float ax = 0;
	float ay = 0;
	float az = 0;
	float tx = 0;
	float ty = 0;
	float tz = 0;

	//e11~e34
	float e11 = 0;
	float e12 = 0;
	float e13 = 0;
	float e14 = 0;
	float e21 = 0;
	float e22 = 0;
	float e23 = 0;
	float e24 = 0;
	float e31 = 0;
	float e32 = 0;
	float e33 = 0;
	float e34 = 0;

	//A1~A16
	float A1 = 0;
	float A2 = 0;
	float A3 = 0;
	float A4 = 0;
	float A5 = 0;
	float A6 = 0; 
	float A7 = 0;
	float A8 = 0;
	float A9 = 0;
	float A10 = 0;
	float A11 = 0;
	float A12 = 0;
	float A13 = 0;
	float A14 = 0;
	float A15 = 0;
	float A16 = 0;

	//B1~B16
	float B1 = 0;
	float B2 = 0;
	float B3 = 0;
	float B4 = 0;
	float B5 = 0;
	float B6 = 0;
	float B7 = 0;
	float B8 = 0;
	float B9 = 0;
	float B10 = 0;

	//C1~C16
	float C1 = 0;
	float C2 = 0;
	float C3 = 0;
	float C4 = 0;
	float C5 = 0;
	float C6 = 0;
	float C7 = 0;
	float C8 = 0;
	float C9 = 0;
	float C10 = 0;
	float C11 = 0;
	float C12 = 0;
	float C13 = 0;
	float C14 = 0;
	float C15 = 0;
	float C16 = 0;
	float C17 = 0;
	float C18 = 0;
	float C19 = 0;
	float C20 = 0;
	float C21 = 0;
	float C22 = 0;
	float C23 = 0;
	float C24 = 0;
	float C25 = 0;
	float C26 = 0;
	float C27 = 0;
	float C28 = 0;
	float C29 = 0;
	float C30 = 0;
	float C31 = 0;

private:
	bool load_Min(std::string &filename);
	bool calibrate_e();
	bool calibrate_A();
	bool calibrate_B();
	bool calibrate_C();
};

