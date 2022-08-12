#include <iostream>
#include <opencv2/opencv.hpp>
#include "MVSObject.h"

/**
 * @brief 
 * 
 * @return int 
 */
int main()
{
	std::cout << cv::getBuildInformation();

	MVSObject test;
	std::cout << test.getTestString();

	return 0;
}
