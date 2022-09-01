#include <iostream>
#include <opencv2/opencv.hpp>
#include "MVSObject.h"
#include "JsonSaver.hpp"

/**
 * @brief 
 * 
 * @return int 
 */
int main()
{
	std::cout << cv::getBuildInformation();
	
	cv::Mat a(3, 3, CV_8UC1);
	json j = a;
	std::cout << j;

	Eigen::Matrix4d b = Eigen::Matrix4d::Identity();
	j = b;
	std::cout << j;
	
	return 0;
}
